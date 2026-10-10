/*
MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

/**
 * @file ExtraEffects.hpp
 * @brief Reverb, chorus, pitch shifter and distortion for FX Wizard Plus.
 *
 * These classes are deliberately free of any Pico SDK / Kastle 2 dependencies (plain <cstdint> only),
 * so they can be compiled and tested on a PC. See tools/fxplus_test.cpp.
 *
 * Memory: none of the effects allocate. The reverb, chorus and pitch shifter work in buffers
 * handed to them by the caller; the app lends them the (already allocated) long delay lines,
 * which are unused while one of these modes is active. The RP2040 has almost no spare RAM.
 *
 * Audio format is 16-bit signed (q15). No floating point is used per sample (the RP2040 has no FPU);
 * floats appear only in the Set*() functions, which are meant to be called from the UI loop.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>

namespace kastle2::fxplus
{

inline int32_t Sat16(int32_t v)
{
    return v > 32767 ? 32767 : (v < -32768 ? -32768 : v);
}

/**
 * @brief Multiplies by a q15 coefficient, rounding toward zero. Not suitable for recursive loops with a
 *        coefficient close to 1 (see Reverb::Comb::Process); fine for the first order filters used elsewhere.
 */
inline int32_t MulQ15Trunc(int32_t a, int32_t q15)
{
    return (a * q15) / 32768;
}

/**
 * @brief Cheap sine-like LFO from a 32-bit phase (parabolic approximation, ~6% error, no tables).
 * @param phase Full cycle = 0 .. 2^32.
 * @return -32767 .. 32767
 */
inline int32_t LfoSine(uint32_t phase)
{
    // Shift so that phase 0 is the zero crossing going up
    int32_t x = static_cast<int32_t>(phase) >> 16; // -32768 .. 32767, one full cycle
    int32_t ax = x < 0 ? -x : x;
    int32_t y = (x * (32768 - ax)) >> 13; // 4 * x * (1 - |x|), peak 32768 at |x| = 0.5
    return y > 32767 ? 32767 : (y < -32767 ? -32767 : y);
}

/* ------------------------------------------------------------------------------------------------
 *  REVERB
 * ---------------------------------------------------------------------------------------------- */

namespace reverb_detail
{
constexpr int kCombs = 8;
constexpr int kAllpasses = 4;
// Classic Freeverb lengths (tuned for 44.1 kHz, close enough to our 44 kHz)
constexpr uint16_t kCombLen[kCombs] = {1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617};
constexpr uint16_t kAllpassLen[kAllpasses] = {556, 441, 341, 225};
constexpr uint16_t kStereoSpread = 23; // right side is this much longer

constexpr size_t SideWords()
{
    size_t w = 0;
    for (int i = 0; i < kCombs; i++)
    {
        w += kCombLen[i] + kStereoSpread;
    }
    for (int i = 0; i < kAllpasses; i++)
    {
        w += kAllpassLen[i] + kStereoSpread;
    }
    return w;
}
} // namespace reverb_detail

/**
 * @class Reverb
 * @brief Stereo Schroeder/Moorer style reverb (8 damped combs + 4 allpasses per side, Freeverb topology).
 *
 * Each comb's feedback gain is derived from the requested RT60, so the decay time is the same for
 * all combs (original Freeverb uses one gain for all, which makes the decay depend on the delay length).
 * A damping low-pass sits in every comb's feedback loop.
 *
 * Needs two buffers: kLeftWords / kRightWords int16 samples respectively (about 17k and 13k samples, 34 kB and 26 kB).
 */
class Reverb
{
public:
    static constexpr int kCombs = reverb_detail::kCombs;
    static constexpr int kAllpasses = reverb_detail::kAllpasses;
    static constexpr const uint16_t *kCombLen = reverb_detail::kCombLen;
    static constexpr const uint16_t *kAllpassLen = reverb_detail::kAllpassLen;
    static constexpr uint16_t kStereoSpread = reverb_detail::kStereoSpread;
    static constexpr size_t kPreDelayWords = 4096; // power of two
    static constexpr size_t kSampleRate = 44000;

    static constexpr size_t kLeftWords = reverb_detail::SideWords() + kPreDelayWords; // pre-delay lives in the left buffer
    static constexpr size_t kRightWords = reverb_detail::SideWords();

    /**
     * @brief Assigns the buffers. Call Clear() afterwards.
     * @return false if the buffers are too small
     */
    bool Init(int16_t *left_buffer, size_t left_words, int16_t *right_buffer, size_t right_words)
    {
        if (left_buffer == nullptr || right_buffer == nullptr || left_words < kLeftWords || right_words < kRightWords)
        {
            ready_ = false;
            return false;
        }
        int16_t *l = left_buffer;
        int16_t *r = right_buffer;
        for (int i = 0; i < kCombs; i++)
        {
            comb_l_[i].Setup(l, kCombLen[i]);
            l += kCombLen[i];
            comb_r_[i].Setup(r, kCombLen[i] + kStereoSpread);
            r += kCombLen[i] + kStereoSpread;
        }
        for (int i = 0; i < kAllpasses; i++)
        {
            ap_l_[i].Setup(l, kAllpassLen[i]);
            l += kAllpassLen[i];
            ap_r_[i].Setup(r, kAllpassLen[i] + kStereoSpread);
            r += kAllpassLen[i] + kStereoSpread;
        }
        predelay_ = l;
        predelay_write_ = 0;
        ready_ = true;
        SetDecay(2.f);
        SetDamping(32767);
        SetPreDelay(0);
        return true;
    }

    /** @brief Zeroes all buffers and filter states (wipes the tail). */
    void Clear()
    {
        if (!ready_)
        {
            return;
        }
        for (int i = 0; i < kCombs; i++)
        {
            comb_l_[i].Clear();
            comb_r_[i].Clear();
        }
        for (int i = 0; i < kAllpasses; i++)
        {
            ap_l_[i].Clear();
            ap_r_[i].Clear();
        }
        std::memset(predelay_, 0, kPreDelayWords * sizeof(int16_t));
        predelay_write_ = 0;
    }

    /**
     * @brief Sets the decay time. Not realtime safe (uses floating point), call from the UI loop only
     *        and only when the value actually changed.
     * @param rt60_seconds Time for the tail to fall by 60 dB (0.1 - 30)
     */
    void SetDecay(float rt60_seconds)
    {
        if (rt60_seconds < 0.1f)
        {
            rt60_seconds = 0.1f;
        }
        if (rt60_seconds > 30.f)
        {
            rt60_seconds = 30.f;
        }
        float g_sum = 0.f;
        for (int i = 0; i < kCombs; i++)
        {
            // per pass gain such that the loop falls by 60 dB in rt60 seconds
            float g = std::pow(10.f, -3.f * static_cast<float>(kCombLen[i]) / (static_cast<float>(kSampleRate) * rt60_seconds));
            int32_t gq = static_cast<int32_t>(g * 32768.f);
            if (gq > 32760)
            {
                gq = 32760;
            }
            comb_l_[i].gain = gq;
            // right side is slightly longer, compensate so both sides decay in the same time
            float gr = std::pow(10.f, -3.f * static_cast<float>(kCombLen[i] + kStereoSpread) / (static_cast<float>(kSampleRate) * rt60_seconds));
            int32_t grq = static_cast<int32_t>(gr * 32768.f);
            comb_r_[i].gain = grq > 32760 ? 32760 : grq;
            g_sum += g;
        }
        g_avg_ = g_sum / static_cast<float>(kCombs);
        UpdateInputGain();
    }

    /**
     * @brief Sets the damping filter in every comb loop.
     * @param alpha q15 low-pass coefficient: 32767 = no damping (bright), small = dark (about 4000 = very dark)
     */
    void SetDamping(int32_t alpha)
    {
        damping_ = alpha < 200 ? 200 : (alpha > 32767 ? 32767 : alpha);
        UpdateInputGain();
    }

    /** @brief Pre-delay in samples (0 - kPreDelayWords - 1). */
    void SetPreDelay(size_t samples)
    {
        predelay_len_ = samples >= kPreDelayWords ? kPreDelayWords - 1 : samples;
    }

    /**
     * @brief Processes one stereo sample. Output is 100% wet.
     */
    inline void Process(int32_t in_l, int32_t in_r, int32_t &out_l, int32_t &out_r)
    {
        if (!ready_)
        {
            out_l = 0;
            out_r = 0;
            return;
        }

        // Mono sum -> pre-delay -> input gain
        predelay_[predelay_write_] = static_cast<int16_t>(Sat16((in_l + in_r) >> 1));
        int32_t x = predelay_[(predelay_write_ - predelay_len_) & (kPreDelayWords - 1)];
        predelay_write_ = (predelay_write_ + 1) & (kPreDelayWords - 1);
        x = (x * input_gain_) >> 15;

        int32_t sum_l = 0;
        int32_t sum_r = 0;
        for (int i = 0; i < kCombs; i++)
        {
            // xorshift32, 30 fresh random bits per comb
            rng_ ^= rng_ << 13;
            rng_ ^= rng_ >> 17;
            rng_ ^= rng_ << 5;
            sum_l += comb_l_[i].Process(x, damping_, rng_);
            rng_ ^= rng_ << 13;
            rng_ ^= rng_ >> 17;
            rng_ ^= rng_ << 5;
            sum_r += comb_r_[i].Process(x, damping_, rng_);
        }
        for (int i = 0; i < kAllpasses; i++)
        {
            sum_l = ap_l_[i].Process(sum_l);
            sum_r = ap_r_[i].Process(sum_r);
        }
        out_l = Sat16(sum_l);
        out_r = Sat16(sum_r);
    }

    bool IsReady() const
    {
        return ready_;
    }

private:
    /**
     * Input gain: keeps the loudness of the wet signal roughly the same regardless of decay time and damping.
     * For uncorrelated input the comb bank amplifies by about sqrt(8 / (1 - g^2)), so we divide that out.
     * Damping removes high frequencies from the loop, which makes the (white noise) test signal quieter,
     * so it gets some of that back.
     */
    void UpdateInputGain()
    {
        float in_gain = std::sqrt(1.f - g_avg_ * g_avg_) * kInputTrim;
        in_gain *= 1.f + 0.6f * (1.f - static_cast<float>(damping_) / 32768.f);
        if (in_gain > 1.f)
        {
            in_gain = 1.f;
        }
        input_gain_ = static_cast<int32_t>(in_gain * 32767.f);
    }

    // Overall make-up so that wet RMS is about the same as the input RMS with noise-like material.
    // Determined with tools/fxplus_test.cpp.
    static constexpr float kInputTrim = 0.065f;

    struct Comb
    {
        int16_t *buf = nullptr;
        uint16_t len = 1;
        uint16_t idx = 0;
        int32_t lp = 0;
        int32_t gain = 0;

        void Setup(int16_t *b, uint16_t l)
        {
            buf = b;
            len = l;
            idx = 0;
            lp = 0;
            gain = 0;
        }
        void Clear()
        {
            std::memset(buf, 0, len * sizeof(int16_t));
            idx = 0;
            lp = 0;
        }
        /**
         * @param x input sample
         * @param damp_alpha low-pass coefficient, q15
         * @param rnd 30 random bits, used as dither for the two roundings
         *
         * Both multiplications round stochastically (floor(v + random in [0,1)) instead of truncating or rounding to
         * nearest. That is unbiased, so a long, quiet tail decays at exactly the requested rate. Plain truncation
         * removes at least 1 LSB per pass, which at the low levels of a long tail is a much faster decay than
         * requested. Rounding to nearest does the opposite, the loop stalls once |x| * (1 - gain) < 0.5, which is a
         * constant hiss. The price is +-1 LSB of noise (-96 dBFS).
         */
        inline int32_t Process(int32_t x, int32_t damp_alpha, uint32_t rnd)
        {
            int32_t out = buf[idx];
            lp += ((out - lp) * damp_alpha + static_cast<int32_t>(rnd & 0x7FFF)) >> 15;
            buf[idx] = static_cast<int16_t>(Sat16(x + ((lp * gain + static_cast<int32_t>((rnd >> 15) & 0x7FFF)) >> 15)));
            if (++idx >= len)
            {
                idx = 0;
            }
            return out;
        }
    };

    struct Allpass
    {
        int16_t *buf = nullptr;
        uint16_t len = 1;
        uint16_t idx = 0;

        void Setup(int16_t *b, uint16_t l)
        {
            buf = b;
            len = l;
            idx = 0;
        }
        void Clear()
        {
            std::memset(buf, 0, len * sizeof(int16_t));
            idx = 0;
        }
        inline int32_t Process(int32_t in)
        {
            int32_t b = buf[idx];
            buf[idx] = static_cast<int16_t>(Sat16(in + (b / 2)));
            if (++idx >= len)
            {
                idx = 0;
            }
            return b - in;
        }
    };

    Comb comb_l_[kCombs];
    Comb comb_r_[kCombs];
    Allpass ap_l_[kAllpasses];
    Allpass ap_r_[kAllpasses];
    int16_t *predelay_ = nullptr;
    size_t predelay_write_ = 0;
    size_t predelay_len_ = 0;
    int32_t input_gain_ = 0;
    float g_avg_ = 0.9f;
    int32_t damping_ = 32767;
    uint32_t rng_ = 0x12345678u;
    bool ready_ = false;
};

/* ------------------------------------------------------------------------------------------------
 *  CHORUS
 * ---------------------------------------------------------------------------------------------- */

/**
 * @class Chorus
 * @brief Stereo chorus: two modulated, linearly interpolated delay taps per side.
 *
 * The right side runs the same LFO with an adjustable phase offset, which is what creates the stereo width.
 * Needs kWords int16 samples per side (4 kB).
 */
class Chorus
{
public:
    static constexpr size_t kWords = 2048; // power of two, 46 ms
    static constexpr int32_t kCenterA = 640;  // samples, 14.5 ms
    static constexpr int32_t kCenterB = 880;  // samples, 20 ms
    static constexpr int32_t kMaxDepth = 360; // samples, +-8 ms

    bool Init(int16_t *left_buffer, size_t left_words, int16_t *right_buffer, size_t right_words)
    {
        if (left_buffer == nullptr || right_buffer == nullptr || left_words < kWords || right_words < kWords)
        {
            ready_ = false;
            return false;
        }
        buf_l_ = left_buffer;
        buf_r_ = right_buffer;
        ready_ = true;
        Clear();
        return true;
    }

    void Clear()
    {
        if (!ready_)
        {
            return;
        }
        std::memset(buf_l_, 0, kWords * sizeof(int16_t));
        std::memset(buf_r_, 0, kWords * sizeof(int16_t));
        write_ = 0;
        phase_ = 0;
    }

    /** @brief LFO rate as a 32-bit phase increment per sample (rate_hz / sample_rate * 2^32). */
    void SetRate(float hz)
    {
        inc_ = static_cast<uint32_t>(hz / static_cast<float>(kSampleRate) * 4294967296.f);
    }

    /** @brief Modulation depth in samples (0 - kMaxDepth). */
    void SetDepth(int32_t samples)
    {
        depth_ = samples < 0 ? 0 : (samples > kMaxDepth ? kMaxDepth : samples);
    }

    /** @brief Phase offset of the right side, 0 (mono) - 0.5 of a cycle (opposite). Given as 32-bit phase. */
    void SetStereoPhase(uint32_t phase_offset)
    {
        stereo_phase_ = phase_offset;
    }

    void ResetPhase()
    {
        phase_ = 0;
    }

    /** @brief Processes one stereo sample, output is 100% wet. */
    inline void Process(int32_t in_l, int32_t in_r, int32_t &out_l, int32_t &out_r)
    {
        if (!ready_)
        {
            out_l = 0;
            out_r = 0;
            return;
        }
        buf_l_[write_] = static_cast<int16_t>(Sat16(in_l));
        buf_r_[write_] = static_cast<int16_t>(Sat16(in_r));

        uint32_t pl = phase_;
        uint32_t pr = phase_ + stereo_phase_;
        // second voice sits a third of a cycle further along, so the voices never move in lockstep
        int32_t a_l = ReadTap(buf_l_, (kCenterA << 8) + Mod(LfoSine(pl)));
        int32_t b_l = ReadTap(buf_l_, (kCenterB << 8) + Mod(LfoSine(pl + kVoiceOffset)));
        int32_t a_r = ReadTap(buf_r_, (kCenterA << 8) + Mod(LfoSine(pr)));
        int32_t b_r = ReadTap(buf_r_, (kCenterB << 8) + Mod(LfoSine(pr + kVoiceOffset)));

        out_l = Sat16((a_l + b_l) * 7 / 10);
        out_r = Sat16((a_r + b_r) * 7 / 10);

        write_ = (write_ + 1) & (kWords - 1);
        phase_ += inc_;
    }

    bool IsReady() const
    {
        return ready_;
    }

private:
    static constexpr size_t kSampleRate = 44000;
    static constexpr uint32_t kVoiceOffset = 0x55555555u; // 1/3 cycle

    /** @brief LFO value (q15) to a delay offset in 1/256 samples. */
    inline int32_t Mod(int32_t lfo) const
    {
        return (depth_ * lfo) >> 7; // depth (samples) * lfo (q15) / 128 = samples * 256
    }

    /** @brief Reads the buffer at a delay given in 1/256 samples, with linear interpolation. */
    inline int32_t ReadTap(const int16_t *buf, int32_t delay_fixed) const
    {
        int32_t di = delay_fixed >> 8;
        int32_t frac = delay_fixed & 255;
        int32_t s0 = buf[(write_ - di) & (kWords - 1)];
        int32_t s1 = buf[(write_ - di - 1) & (kWords - 1)];
        return s0 + (((s1 - s0) * frac) >> 8);
    }

    int16_t *buf_l_ = nullptr;
    int16_t *buf_r_ = nullptr;
    size_t write_ = 0;
    uint32_t phase_ = 0;
    uint32_t inc_ = 0;
    uint32_t stereo_phase_ = 0;
    int32_t depth_ = 0;
    bool ready_ = false;
};

/* ------------------------------------------------------------------------------------------------
 *  PITCH SHIFTER
 * ---------------------------------------------------------------------------------------------- */

/**
 * @class PitchShifter
 * @brief Two-tap crossfaded delay-line pitch shifter ("rotating tape head") with a stereo detune option.
 *
 * Two read taps sweep through a 4096 sample window half a window apart. A triangular crossfade hides the
 * moment a tap wraps around. Left and right can be shifted by slightly different amounts (detune).
 * Needs kWords int16 samples per side (16 kB). Adds up to one window (93 ms) of latency to the wet signal.
 *
 * @note At exactly unity pitch the taps stand still, which turns the effect into a fixed ~46 ms delay.
 *       Combined with the dry signal this gives a comb filter; use a non-zero shift or the stereo detune.
 */
class PitchShifter
{
public:
    static constexpr size_t kWords = 8192; // power of two, must hold a window plus interpolation
    static constexpr int kWindowBits = 12; // 4096 samples = 93 ms. Longer is smoother on sustained notes, shorter smears less.

    bool Init(int16_t *left_buffer, size_t left_words, int16_t *right_buffer, size_t right_words)
    {
        if (left_buffer == nullptr || right_buffer == nullptr || left_words < kWords || right_words < kWords)
        {
            ready_ = false;
            return false;
        }
        buf_l_ = left_buffer;
        buf_r_ = right_buffer;
        ready_ = true;
        Clear();
        return true;
    }

    void Clear()
    {
        if (!ready_)
        {
            return;
        }
        std::memset(buf_l_, 0, kWords * sizeof(int16_t));
        std::memset(buf_r_, 0, kWords * sizeof(int16_t));
        write_ = 0;
        phase_l_ = 0;
        phase_r_ = 0;
    }

    /**
     * @brief Sets the pitch ratios. Not realtime safe (floating point), call from the UI loop.
     * @param semitones Shift in semitones (positive = up), applies to both sides
     * @param detune_cents Right is shifted +detune/2, left -detune/2
     */
    void SetShift(float semitones, float detune_cents)
    {
        inc_l_ = IncrementFor(std::exp2((semitones - detune_cents / 200.f) / 12.f));
        inc_r_ = IncrementFor(std::exp2((semitones + detune_cents / 200.f) / 12.f));
    }

    /** @brief Processes one stereo sample, output is 100% wet. */
    inline void Process(int32_t in_l, int32_t in_r, int32_t &out_l, int32_t &out_r)
    {
        if (!ready_)
        {
            out_l = 0;
            out_r = 0;
            return;
        }
        buf_l_[write_] = static_cast<int16_t>(Sat16(in_l));
        buf_r_[write_] = static_cast<int16_t>(Sat16(in_r));

        out_l = Shift(buf_l_, phase_l_);
        out_r = Shift(buf_r_, phase_r_);

        write_ = (write_ + 1) & (kWords - 1);
        phase_l_ += static_cast<uint32_t>(inc_l_);
        phase_r_ += static_cast<uint32_t>(inc_r_);
    }

    bool IsReady() const
    {
        return ready_;
    }

private:
    /** @brief Phase increment per sample: the delay changes by (1 - ratio) samples every sample. */
    static int32_t IncrementFor(float ratio)
    {
        // a full phase cycle (2^32) corresponds to a delay of one window (2^kWindowBits samples)
        double inc = (1.0 - static_cast<double>(ratio)) * 4294967296.0 / static_cast<double>(1 << kWindowBits);
        // cannot exceed half a window per sample, which is far above the range we use (ratio 0.25 .. 4)
        if (inc > 2147483647.0)
        {
            inc = 2147483647.0;
        }
        if (inc < -2147483647.0)
        {
            inc = -2147483647.0;
        }
        return static_cast<int32_t>(inc);
    }

    inline int32_t Shift(const int16_t *buf, uint32_t phase) const
    {
        uint32_t p2 = phase + 0x80000000u;

        // delay in 1/65536 samples, 0 .. 2048 samples
        int32_t d1 = static_cast<int32_t>(phase >> (32 - kWindowBits - 16));
        int32_t d2 = static_cast<int32_t>(p2 >> (32 - kWindowBits - 16));

        // triangular crossfade, 0 .. 65535 (g2 = 65536 - g1)
        int32_t g1 = static_cast<int32_t>(phase >> 16);
        g1 = g1 < 32768 ? g1 * 2 : (65535 - g1) * 2;

        int32_t s1 = ReadTap(buf, d1);
        int32_t s2 = ReadTap(buf, d2);
        int32_t mixed = (s1 * g1 + s2 * (65536 - g1)) >> 16;
        // +3.5 dB make-up: a linear crossfade of two taps loses level when they are uncorrelated (-1.8 dB for noise)
        // or partly out of phase (sines), measured at -3.5 .. -4.8 dB
        return Sat16(mixed + (mixed >> 1));
    }

    inline int32_t ReadTap(const int16_t *buf, int32_t delay_q16) const
    {
        int32_t di = delay_q16 >> 16;
        int32_t frac = (delay_q16 >> 8) & 255;
        int32_t a = buf[(write_ - di) & (kWords - 1)];
        int32_t b = buf[(write_ - di - 1) & (kWords - 1)];
        return a + (((b - a) * frac) >> 8);
    }

    int16_t *buf_l_ = nullptr;
    int16_t *buf_r_ = nullptr;
    size_t write_ = 0;
    uint32_t phase_l_ = 0;
    uint32_t phase_r_ = 0;
    int32_t inc_l_ = 0;
    int32_t inc_r_ = 0;
    bool ready_ = false;
};

/* ------------------------------------------------------------------------------------------------
 *  PHASER
 * ---------------------------------------------------------------------------------------------- */

/**
 * @brief First order all-pass coefficients for a sweep of the break frequency from 100 Hz to 6.4 kHz
 *        (exponential, 65 points). a = (tan(pi f / fs) - 1) / (tan(pi f / fs) + 1), q15 (negative for low frequencies).
 */
static constexpr int16_t kPhaserCoefficients[65] = {
    -32303, -32272, -32239, -32204, -32167, -32127, -32084, -32039, -31990, -31939, -31884, -31825, -31763,
    -31697, -31626, -31551, -31471, -31385, -31294, -31198, -31095, -30986, -30869, -30746, -30614, -30475,
    -30326, -30169, -30002, -29824, -29635, -29435, -29223, -28998, -28760, -28508, -28240, -27958, -27659,
    -27342, -27008, -26655, -26282, -25889, -25474, -25038, -24577, -24093, -23584, -23048, -22486, -21896,
    -21276, -20627, -19946, -19234, -18488, -17708, -16892, -16040, -15149, -14218, -13245, -12229, -11167
};

/**
 * @class Phaser
 * @brief Six stage all-pass phaser with feedback and a stereo LFO offset. Needs no buffers.
 *
 * The output is the all-pass chain only (100% wet, same level as the input); mix it with the dry signal 1:1
 * to get the notches. The sweep covers 10% - 90% of the coefficient table, about 150 Hz to 4.2 kHz.
 */
class Phaser
{
public:
    static constexpr int kStages = 6;

    void Reset()
    {
        for (auto &c : channel_)
        {
            for (int i = 0; i < kStages; i++)
            {
                c.x1[i] = 0;
                c.y1[i] = 0;
            }
            c.last = 0;
        }
        phase_ = 0;
    }

    /** @brief LFO rate in Hz, call from the UI loop only (floating point). */
    void SetRate(float hz)
    {
        inc_ = static_cast<uint32_t>(hz / 44000.f * 4294967296.f);
    }

    /** @brief Feedback (resonance), q15. Limited to 0.85, which keeps the loop stable. */
    void SetFeedback(int32_t q15)
    {
        feedback_ = q15 < 0 ? 0 : (q15 > 27853 ? 27853 : q15);
    }

    /** @brief Phase offset of the right LFO as a 32-bit phase (0x80000000 = opposite). */
    void SetStereoPhase(uint32_t phase_offset)
    {
        stereo_phase_ = phase_offset;
    }

    inline void Process(int32_t in_l, int32_t in_r, int32_t &out_l, int32_t &out_r)
    {
        out_l = Run(channel_[0], phase_, in_l);
        out_r = Run(channel_[1], phase_ + stereo_phase_, in_r);
        phase_ += inc_;
    }

private:
    struct Channel
    {
        int32_t x1[kStages] = {0};
        int32_t y1[kStages] = {0};
        int32_t last = 0;
    };

    inline int32_t Run(Channel &c, uint32_t phase, int32_t in)
    {
        // all-pass coefficient for this moment of the sweep (same for all stages)
        int32_t pos = 16384 + ((LfoSine(phase) * 13107) >> 15); // 0.5 +- 0.4, q15
        int32_t idx = pos >> 9;
        int32_t frac = pos & 511;
        int32_t a = kPhaserCoefficients[idx] + (((kPhaserCoefficients[idx + 1] - kPhaserCoefficients[idx]) * frac) >> 9);

        int32_t x = Sat16(in + ((c.last * feedback_ + 0x4000) >> 15));
        for (int s = 0; s < kStages; s++)
        {
            // y[n] = a * (x[n] - y[n-1]) + x[n-1]
            int32_t y = Sat16(((a * (x - c.y1[s]) + 0x4000) >> 15) + c.x1[s]);
            c.x1[s] = x;
            c.y1[s] = y;
            x = y;
        }
        c.last = x;
        return x;
    }

    Channel channel_[2];
    uint32_t phase_ = 0;
    uint32_t inc_ = 0;
    uint32_t stereo_phase_ = 0;
    int32_t feedback_ = 0;
};

/* ------------------------------------------------------------------------------------------------
 *  PLATE REVERB
 * ---------------------------------------------------------------------------------------------- */

namespace plate_detail
{
// Jon Dattorro's plate reverb ("Effect Design Part 1", 1997), delay lengths scaled from his 29761 Hz to 44 kHz.
constexpr int kInputAllpasses = 4;
constexpr uint16_t kInputAllpassLen[kInputAllpasses] = {210, 158, 560, 410};
constexpr int32_t kInputAllpassGain[kInputAllpasses] = {24576, 24576, 20480, 20480}; // 0.75 0.75 0.625 0.625

constexpr uint16_t kExcursion = 24; // samples of modulation of the first tank all-pass

constexpr uint16_t kLeftModLen = 994;
constexpr uint16_t kLeftDelay1Len = 6584;
constexpr uint16_t kLeftAllpass2Len = 2661;
constexpr uint16_t kLeftDelay2Len = 5500;

constexpr uint16_t kRightModLen = 1342;
constexpr uint16_t kRightDelay1Len = 6235;
constexpr uint16_t kRightAllpass2Len = 3927;
constexpr uint16_t kRightDelay2Len = 4676;

constexpr size_t kPreDelayWords = 4096; // power of two

constexpr size_t kInputWords = 210 + 158 + 560 + 410;
constexpr size_t kLeftWords = kPreDelayWords + kInputWords + (kLeftModLen + kExcursion + 2) + kLeftDelay1Len + kLeftAllpass2Len + kLeftDelay2Len;
constexpr size_t kRightWords = (kRightModLen + kExcursion + 2) + kRightDelay1Len + kRightAllpass2Len + kRightDelay2Len;
} // namespace plate_detail

/**
 * @class PlateReverb
 * @brief Dattorro plate reverb: input diffusion, then two cross-coupled tanks with a modulated all-pass,
 *        damping low-pass and decay in each. Dense and smooth, in contrast to the comb based Reverb.
 *
 * Needs kLeftWords / kRightWords int16 samples (about 21k and 16k).
 * Decay and damping are set like on Reverb. Output is 100% wet.
 */
class PlateReverb
{
public:
    static constexpr size_t kLeftWords = plate_detail::kLeftWords;
    static constexpr size_t kRightWords = plate_detail::kRightWords;
    static constexpr size_t kPreDelayWords = plate_detail::kPreDelayWords;

    bool Init(int16_t *left_buffer, size_t left_words, int16_t *right_buffer, size_t right_words)
    {
        using namespace plate_detail;
        if (left_buffer == nullptr || right_buffer == nullptr || left_words < kLeftWords || right_words < kRightWords)
        {
            ready_ = false;
            return false;
        }
        int16_t *l = left_buffer;
        predelay_ = l;
        l += kPreDelayWords;
        for (int i = 0; i < kInputAllpasses; i++)
        {
            input_ap_[i].Setup(l, kInputAllpassLen[i]);
            l += kInputAllpassLen[i];
        }
        left_mod_.Setup(l, kLeftModLen, kLeftModLen + kExcursion + 2);
        l += kLeftModLen + kExcursion + 2;
        left_d1_.Setup(l, kLeftDelay1Len);
        l += kLeftDelay1Len;
        left_ap2_.Setup(l, kLeftAllpass2Len);
        l += kLeftAllpass2Len;
        left_d2_.Setup(l, kLeftDelay2Len);

        int16_t *r = right_buffer;
        right_mod_.Setup(r, kRightModLen, kRightModLen + kExcursion + 2);
        r += kRightModLen + kExcursion + 2;
        right_d1_.Setup(r, kRightDelay1Len);
        r += kRightDelay1Len;
        right_ap2_.Setup(r, kRightAllpass2Len);
        r += kRightAllpass2Len;
        right_d2_.Setup(r, kRightDelay2Len);

        ready_ = true;
        SetDecay(2.f);
        SetDamping(32767);
        SetPreDelay(0);
        return true;
    }

    void Clear()
    {
        if (!ready_)
        {
            return;
        }
        std::memset(predelay_, 0, plate_detail::kPreDelayWords * sizeof(int16_t));
        predelay_write_ = 0;
        for (auto &a : input_ap_)
        {
            a.Clear();
        }
        left_mod_.Clear();
        left_d1_.Clear();
        left_ap2_.Clear();
        left_d2_.Clear();
        right_mod_.Clear();
        right_d1_.Clear();
        right_ap2_.Clear();
        right_d2_.Clear();
        lp_in_ = 0;
        lp_left_ = 0;
        lp_right_ = 0;
        lfo_phase_ = 0;
        lfo_phase_right_ = 0x40000000u;
    }

    /**
     * @brief Decay time, RT60 in seconds (0.1 - 30). Floating point, UI loop only.
     * @note The two tanks are in series, so one trip around takes kLoopSeconds and passes two decay stages.
     */
    void SetDecay(float rt60_seconds)
    {
        if (rt60_seconds < 0.1f)
        {
            rt60_seconds = 0.1f;
        }
        if (rt60_seconds > 30.f)
        {
            rt60_seconds = 30.f;
        }
        float decay = std::pow(10.f, -1.5f * kLoopSeconds / rt60_seconds);
        if (decay > 0.985f)
        {
            decay = 0.985f;
        }
        decay_ = static_cast<int32_t>(decay * 32768.f);
        input_gain_ = static_cast<int32_t>(kInputTrim * std::sqrt(1.f - decay * decay) * 32767.f);
        // a long tail has more energy in total, but the damped, dark settings lose some of it
    }

    /** @brief Damping low-pass in the tanks, q15 coefficient: 32767 = bright, small = dark. */
    void SetDamping(int32_t alpha)
    {
        damping_ = alpha < 200 ? 200 : (alpha > 32767 ? 32767 : alpha);
    }

    void SetPreDelay(size_t samples)
    {
        predelay_len_ = samples >= plate_detail::kPreDelayWords ? plate_detail::kPreDelayWords - 1 : samples;
    }

    inline void Process(int32_t in_l, int32_t in_r, int32_t &out_l, int32_t &out_r)
    {
        using namespace plate_detail;
        if (!ready_)
        {
            out_l = 0;
            out_r = 0;
            return;
        }

        // mono sum -> pre-delay -> gain -> bandwidth low-pass -> four all-passes
        predelay_[predelay_write_] = static_cast<int16_t>(Sat16((in_l + in_r) >> 1));
        int32_t x = predelay_[(predelay_write_ - predelay_len_) & (kPreDelayWords - 1)];
        predelay_write_ = (predelay_write_ + 1) & (kPreDelayWords - 1);
        x = (x * input_gain_) >> 15;
        lp_in_ += ((x - lp_in_) * 28000) >> 15; // about 0.85, takes the edge off
        x = lp_in_;
        for (int i = 0; i < kInputAllpasses; i++)
        {
            x = input_ap_[i].Process(x, kInputAllpassGain[i]);
        }

        // two slow LFOs for the modulated all-passes
        int32_t mod_l = (static_cast<int32_t>(kExcursion) * LfoSine(lfo_phase_)) >> 7;
        int32_t mod_r = (static_cast<int32_t>(kExcursion) * LfoSine(lfo_phase_right_)) >> 7;
        lfo_phase_ += kLfoIncrementLeft;
        lfo_phase_right_ += kLfoIncrementRight;

        // outputs of the second delay of each tank, they feed the other tank
        int32_t from_left = left_d2_.Read();
        int32_t from_right = right_d2_.Read();

        // left tank
        int32_t t = Sat16(x + from_right);
        t = left_mod_.Process(t, -22938, mod_l); // -0.7
        int32_t d1 = left_d1_.Read();
        left_d1_.Write(t);
        rng_ ^= rng_ << 13;
        rng_ ^= rng_ >> 17;
        rng_ ^= rng_ << 5;
        lp_left_ += ((d1 - lp_left_) * damping_ + static_cast<int32_t>(rng_ & 0x7FFF)) >> 15;
        t = (lp_left_ * decay_ + static_cast<int32_t>((rng_ >> 15) & 0x7FFF)) >> 15;
        t = left_ap2_.Process(t, 16384); // 0.5
        left_d2_.Write(t);

        // right tank
        t = Sat16(x + from_left);
        t = right_mod_.Process(t, -22938, mod_r);
        d1 = right_d1_.Read();
        right_d1_.Write(t);
        rng_ ^= rng_ << 13;
        rng_ ^= rng_ >> 17;
        rng_ ^= rng_ << 5;
        lp_right_ += ((d1 - lp_right_) * damping_ + static_cast<int32_t>(rng_ & 0x7FFF)) >> 15;
        t = (lp_right_ * decay_ + static_cast<int32_t>((rng_ >> 15) & 0x7FFF)) >> 15;
        t = right_ap2_.Process(t, 16384);
        right_d2_.Write(t);

        // output taps (Dattorro's table, scaled to 44 kHz)
        int32_t yl = right_d1_.Tap(393) + right_d1_.Tap(4397) - right_ap2_.Tap(2828) + right_d2_.Tap(2951) -
                     left_d1_.Tap(2942) - left_ap2_.Tap(276) - left_d2_.Tap(1576);
        int32_t yr = left_d1_.Tap(522) + left_d1_.Tap(5362) - left_ap2_.Tap(1816) + left_d2_.Tap(3952) -
                     right_d1_.Tap(3121) - right_ap2_.Tap(495) - right_d2_.Tap(179);
        // seven taps add up to 18 bits, scale before multiplying to stay within 32 bits
        out_l = Sat16(((yl >> 2) * kOutputGain) >> 13);
        out_r = Sat16(((yr >> 2) * kOutputGain) >> 13);
    }

    bool IsReady() const
    {
        return ready_;
    }

private:
    static constexpr float kLoopSeconds = 31915.f / 44000.f; // total length of the two tanks
    static constexpr float kInputTrim = 0.9f;
    static constexpr int32_t kOutputGain = 19661; // 0.6
    static constexpr uint32_t kLfoIncrementLeft = 91000;   // about 0.93 Hz
    static constexpr uint32_t kLfoIncrementRight = 114000; // about 1.17 Hz

    /** Plain delay line: Read() gives the oldest sample, Write() replaces it and advances. */
    struct Delay
    {
        int16_t *buf = nullptr;
        uint16_t len = 1;
        uint16_t idx = 0;
        void Setup(int16_t *b, uint16_t l)
        {
            buf = b;
            len = l;
            idx = 0;
        }
        void Clear()
        {
            std::memset(buf, 0, len * sizeof(int16_t));
            idx = 0;
        }
        inline int32_t Read() const
        {
            return buf[idx];
        }
        inline void Write(int32_t v)
        {
            buf[idx] = static_cast<int16_t>(v);
            if (++idx >= len)
            {
                idx = 0;
            }
        }
        /** The sample written k samples ago (1 = the newest), valid after Write() */
        inline int32_t Tap(uint16_t k) const
        {
            return buf[idx >= k ? idx - k : idx + len - k];
        }
    };

    /** Schroeder all-pass, w[n] = x[n] + g w[n-D], y[n] = w[n-D] - g w[n] */
    struct Allpass : Delay
    {
        inline int32_t Process(int32_t x, int32_t g)
        {
            int32_t d = buf[idx];
            int32_t w = Sat16(x + ((g * d + 0x4000) >> 15));
            int32_t y = Sat16(d - ((g * w + 0x4000) >> 15));
            Write(w);
            return y;
        }
    };

    /** The same with a delay modulated by +-kExcursion samples (linear interpolation) */
    struct ModAllpass
    {
        int16_t *buf = nullptr;
        uint16_t nominal = 1;
        uint16_t len = 1;
        uint16_t idx = 0;
        void Setup(int16_t *b, uint16_t nominal_delay, uint16_t buffer_len)
        {
            buf = b;
            nominal = nominal_delay;
            len = buffer_len;
            idx = 0;
        }
        void Clear()
        {
            std::memset(buf, 0, len * sizeof(int16_t));
            idx = 0;
        }
        /** @param mod delay offset in 1/256 samples */
        inline int32_t Process(int32_t x, int32_t g, int32_t mod)
        {
            int32_t fixed = (static_cast<int32_t>(nominal) << 8) + mod;
            int32_t di = fixed >> 8;
            int32_t frac = fixed & 255;
            int32_t p0 = idx - di;
            int32_t p1 = p0 - 1;
            if (p0 < 0)
            {
                p0 += len;
            }
            if (p1 < 0)
            {
                p1 += len;
            }
            int32_t a = buf[p0];
            int32_t d = a + (((buf[p1] - a) * frac) >> 8);
            int32_t w = Sat16(x + ((g * d + 0x4000) >> 15));
            int32_t y = Sat16(d - ((g * w + 0x4000) >> 15));
            buf[idx] = static_cast<int16_t>(w);
            if (++idx >= len)
            {
                idx = 0;
            }
            return y;
        }
    };

    int16_t *predelay_ = nullptr;
    size_t predelay_write_ = 0;
    size_t predelay_len_ = 0;
    Allpass input_ap_[plate_detail::kInputAllpasses];
    ModAllpass left_mod_;
    Delay left_d1_;
    Allpass left_ap2_;
    Delay left_d2_;
    ModAllpass right_mod_;
    Delay right_d1_;
    Allpass right_ap2_;
    Delay right_d2_;
    int32_t lp_in_ = 0;
    int32_t lp_left_ = 0;
    int32_t lp_right_ = 0;
    int32_t decay_ = 0;
    int32_t input_gain_ = 0;
    int32_t damping_ = 32767;
    uint32_t lfo_phase_ = 0;
    uint32_t lfo_phase_right_ = 0x40000000u;
    uint32_t rng_ = 0x2468ace1u;
    bool ready_ = false;
};

/* ------------------------------------------------------------------------------------------------
 *  DISTORTION
 * ---------------------------------------------------------------------------------------------- */

/**
 * @brief tanh() lookup, 257 points covering -8.0 .. +8.0 in steps of 1/16, scaled to +-32767.
 *        Generated: round(tanh((i - 128) / 16) * 32767)
 */
static constexpr int16_t kTanhTable[257] = {
    -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767,
    -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767,
    -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767, -32766, -32766,
    -32766, -32766, -32766, -32766, -32766, -32766, -32766, -32765, -32765, -32765, -32765, -32764,
    -32764, -32764, -32763, -32763, -32762, -32761, -32761, -32760, -32759, -32758, -32757, -32755,
    -32754, -32752, -32750, -32748, -32745, -32742, -32739, -32735, -32731, -32726, -32720, -32714,
    -32707, -32699, -32690, -32680, -32669, -32656, -32641, -32624, -32605, -32583, -32559, -32531,
    -32500, -32465, -32425, -32380, -32328, -32270, -32205, -32131, -32047, -31952, -31845, -31725,
    -31588, -31435, -31261, -31066, -30846, -30599, -30321, -30009, -29659, -29267, -28829, -28340,
    -27796, -27190, -26518, -25775, -24955, -24053, -23065, -21986, -20812, -19541, -18173, -16706,
    -15142, -13486, -11742, -9919, -8025, -6073, -4075, -2045, 0, 2045, 4075, 6073,
    8025, 9919, 11742, 13486, 15142, 16706, 18173, 19541, 20812, 21986, 23065, 24053,
    24955, 25775, 26518, 27190, 27796, 28340, 28829, 29267, 29659, 30009, 30321, 30599,
    30846, 31066, 31261, 31435, 31588, 31725, 31845, 31952, 32047, 32131, 32205, 32270,
    32328, 32380, 32425, 32465, 32500, 32531, 32559, 32583, 32605, 32624, 32641, 32656,
    32669, 32680, 32690, 32699, 32707, 32714, 32720, 32726, 32731, 32735, 32739, 32742,
    32745, 32748, 32750, 32752, 32754, 32755, 32757, 32758, 32759, 32760, 32761, 32761,
    32762, 32763, 32763, 32764, 32764, 32764, 32765, 32765, 32765, 32765, 32766, 32766,
    32766, 32766, 32766, 32766, 32766, 32766, 32766, 32767, 32767, 32767, 32767, 32767,
    32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767,
    32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767,
    32767, 32767, 32767, 32767, 32767,
};

/**
 * @class Distortion
 * @brief Drive -> soft clipper (with adjustable asymmetry) -> DC blocker -> tone low-pass -> level compensation.
 *
 * No buffers needed, one instance per channel.
 */
class Distortion
{
public:
    void Reset()
    {
        dc_x_ = 0;
        dc_y_ = 0;
        tone_ = 0;
    }

    /**
     * @brief Pre-gain in 1/256 (256 = 1x, max 128x).
     *
     * The output is scaled so that a reference tone at about -14 dBFS comes out at the same RMS level at every
     * drive setting (the drive knob changes the character, not the volume). Quieter material gets louder with
     * more drive, hotter material gets quieter, as with a real drive pedal.
     */
    void SetDrive(int32_t gain_q8)
    {
        gain_q8_ = gain_q8 < 256 ? 256 : (gain_q8 > 32768 ? 32768 : gain_q8);

        // Piecewise linear compensation curve. Generated from the actual clipper: comp = rms(ref) / rms(tanh(g * ref)),
        // ref = sine with 0.2 peak (tools/fxplus_test.cpp checks the result).
        static constexpr int32_t kGain[14] = {256, 384, 512, 768, 1024, 1536, 2048, 3072, 4096, 6144, 8192, 12288, 16384, 25600};
        static constexpr int32_t kComp[14] = {32767, 22332, 17029, 11873, 9429, 7206, 6258, 5500, 5212, 4984, 4886, 4796, 4754, 4710};
        int32_t i = 0;
        while (i < 12 && gain_q8_ > kGain[i + 1])
        {
            i++;
        }
        int32_t g = gain_q8_ > kGain[13] ? kGain[13] : gain_q8_;
        comp_q15_ = kComp[i] + (kComp[i + 1] - kComp[i]) * (g - kGain[i]) / (kGain[i + 1] - kGain[i]);
        UpdateBias();
    }

    /** @brief Tone low-pass coefficient (q15, 32767 = fully open). */
    void SetTone(int32_t alpha)
    {
        tone_alpha_ = alpha < 100 ? 100 : (alpha > 32767 ? 32767 : alpha);
    }

    /**
     * @brief Asymmetry (0 - 32767). Offsets the input ahead of the drive stage by up to 0.3 of full scale,
     *        which creates even harmonics. The offset is amplified by the drive like the signal is, so the amount
     *        of asymmetry stays the same relative to the signal at any drive setting.
     */
    void SetBias(int32_t bias)
    {
        bias_ = bias < 0 ? 0 : (bias > 32767 ? 32767 : bias);
        UpdateBias();
    }

    inline int32_t Process(int32_t in)
    {
        // pre-gain, headroom for the offset
        int32_t x = (in * gain_q8_) >> 8;
        x = x + bias_x_;
        x = x > 262143 ? 262143 : (x < -262143 ? -262143 : x); // limit to 8x full scale (table range), keeps the math in 32 bits
        // level compensation first: it brings the signal well below full scale, which leaves headroom for the DC blocker
        int32_t y = MulQ15Trunc(Clip(x) - bias_out_, comp_q15_);

        // DC blocker, one pole high-pass at about 27 Hz
        // (clamped: the output of this filter can briefly exceed 16 bits)
        int32_t hp = Sat16(y - dc_x_ + MulQ15Trunc(dc_y_, 32640));
        dc_x_ = y;
        dc_y_ = hp;

        // tone
        tone_ += MulQ15Trunc(hp - tone_, tone_alpha_);

        return Sat16(tone_);
    }

    /**
     * @brief tanh soft clipper with linear interpolation.
     * @param x q15 with headroom: +-262143 is +-8.0 (inputs beyond that must be limited by the caller)
     */
    static inline int32_t Clip(int32_t x)
    {
        int32_t u = x + 262144; // 0 .. 524287
        int32_t idx = u >> 11;
        int32_t frac = u & 2047;
        int32_t a = kTanhTable[idx];
        int32_t b = kTanhTable[idx + 1];
        return a + (((b - a) * frac) >> 11);
    }

private:
    void UpdateBias()
    {
        int32_t bias_in = bias_ * 3 / 10; // 0 - 9830
        int32_t bx = (bias_in * gain_q8_) >> 8;
        bias_x_ = bx > 262143 ? 262143 : bx;
        // the clipper is not zero at the offset point, remember what to subtract
        bias_out_ = Clip(bias_x_);
    }

    int32_t gain_q8_ = 256;
    int32_t comp_q15_ = 32767;
    int32_t tone_alpha_ = 32767;
    int32_t bias_ = 0;
    int32_t bias_x_ = 0;
    int32_t bias_out_ = 0;
    int32_t dc_x_ = 0;
    int32_t dc_y_ = 0;
    int32_t tone_ = 0;
};

} // namespace kastle2::fxplus
