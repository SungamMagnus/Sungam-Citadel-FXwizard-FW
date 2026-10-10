// Host-side tests and demo renderer for ExtraEffects.hpp. Not part of the firmware build.
//
//   clang++ -std=c++20 -O1 -g -fsanitize=address,undefined -I.. fxplus_test.cpp -o fxplus_test && ./fxplus_test [demo_output_dir]
//
// The first part measures the effects (decay time, pitch, level, DC...) and prints PASS/FAIL.
// The second part is a randomized stress test, meant to be run with the sanitizers above.
// If a demo directory is given, it also renders a few WAV files so the effects can be auditioned.

#include "ExtraEffects.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <random>
#include <algorithm>

using namespace kastle2::fxplus;

static constexpr double kFs = 44000.0;
static constexpr size_t kPool = 50800; // same as the long delay lines in the app

static int g_fail = 0;
static void Check(bool ok, const char *what, const char *detail = "")
{
    std::printf("  [%s] %s %s\n", ok ? "PASS" : "FAIL", what, detail);
    if (!ok)
    {
        g_fail++;
    }
}
static std::string Fmt(const char *f, double a, double b = 0, double c = 0)
{
    char buf[200];
    std::snprintf(buf, sizeof buf, f, a, b, c);
    return buf;
}

struct Pools
{
    std::vector<int16_t> l = std::vector<int16_t>(kPool, 1234); // non-zero on purpose: effects must not rely on zeroed memory
    std::vector<int16_t> r = std::vector<int16_t>(kPool, -4321);
};

static double Rms(const std::vector<double> &x, size_t from = 0, size_t to = 0)
{
    if (to == 0 || to > x.size())
    {
        to = x.size();
    }
    double s = 0;
    for (size_t i = from; i < to; i++)
    {
        s += x[i] * x[i];
    }
    return std::sqrt(s / std::max<size_t>(1, to - from)) / 32768.0;
}

static double Db(double x)
{
    return 20.0 * std::log10(std::max(x, 1e-12));
}

// Magnitude of a single frequency (Goertzel)
static double Goertzel(const std::vector<double> &x, double f, size_t from, size_t to)
{
    double w = 2 * M_PI * f / kFs;
    double c = 2 * std::cos(w), s1 = 0, s2 = 0;
    for (size_t i = from; i < to; i++)
    {
        double win = 0.5 - 0.5 * std::cos(2 * M_PI * (i - from) / (to - from - 1)); // Hann
        double s = x[i] * win + c * s1 - s2;
        s2 = s1;
        s1 = s;
    }
    double re = s1 - s2 * std::cos(w), im = s2 * std::sin(w);
    return std::sqrt(re * re + im * im) / (to - from) * 4.0 / 32768.0;
}

// Strongest frequency in [lo, hi] using a fine Goertzel sweep
static double PeakFreq(const std::vector<double> &x, double lo, double hi, size_t from, size_t to, double *mag = nullptr)
{
    double best = 0, bf = lo;
    for (double f = lo; f <= hi; f += 0.5)
    {
        double m = Goertzel(x, f, from, to);
        if (m > best)
        {
            best = m;
            bf = f;
        }
    }
    if (mag)
    {
        *mag = best;
    }
    return bf;
}

// Power-weighted mean frequency in [lo, hi]: closer to the perceived pitch than the raw peak when there are sidebands
static double Centroid(const std::vector<double> &x, double lo, double hi, size_t from, size_t to)
{
    double num = 0, den = 0;
    for (double f = lo; f <= hi; f += 1.0)
    {
        double m = Goertzel(x, f, from, to);
        m *= m;
        num += m * f;
        den += m;
    }
    return den > 0 ? num / den : 0;
}

static void WriteWav(const std::string &path, const std::vector<double> &l, const std::vector<double> &r)
{
    FILE *f = std::fopen(path.c_str(), "wb");
    if (!f)
    {
        std::printf("  cannot write %s\n", path.c_str());
        return;
    }
    uint32_t n = static_cast<uint32_t>(l.size());
    uint32_t data = n * 4, rate = 44000, br = rate * 4, riff = 36 + data;
    uint16_t fmt = 1, ch = 2, ba = 4, bits = 16;
    uint32_t sixteen = 16;
    std::fwrite("RIFF", 1, 4, f);
    std::fwrite(&riff, 4, 1, f);
    std::fwrite("WAVEfmt ", 1, 8, f);
    std::fwrite(&sixteen, 4, 1, f);
    std::fwrite(&fmt, 2, 1, f);
    std::fwrite(&ch, 2, 1, f);
    std::fwrite(&rate, 4, 1, f);
    std::fwrite(&br, 4, 1, f);
    std::fwrite(&ba, 2, 1, f);
    std::fwrite(&bits, 2, 1, f);
    std::fwrite("data", 1, 4, f);
    std::fwrite(&data, 4, 1, f);
    for (uint32_t i = 0; i < n; i++)
    {
        int16_t a = static_cast<int16_t>(Sat16(static_cast<int32_t>(l[i])));
        int16_t b = static_cast<int16_t>(Sat16(static_cast<int32_t>(r[i])));
        std::fwrite(&a, 2, 1, f);
        std::fwrite(&b, 2, 1, f);
    }
    std::fclose(f);
}

// ----- test signals -----

static std::vector<double> Noise(size_t n, double rms_dbfs, uint32_t seed = 1)
{
    std::mt19937 g(seed);
    std::normal_distribution<double> d(0.0, 1.0);
    double a = std::pow(10.0, rms_dbfs / 20.0) * 32768.0;
    std::vector<double> x(n);
    for (auto &v : x)
    {
        v = std::max(-32768.0, std::min(32767.0, d(g) * a));
    }
    return x;
}

static std::vector<double> Sine(size_t n, double f, double peak_dbfs)
{
    std::vector<double> x(n);
    double a = std::pow(10.0, peak_dbfs / 20.0) * 32768.0;
    for (size_t i = 0; i < n; i++)
    {
        x[i] = std::min(32767.0, a * std::sin(2 * M_PI * f * i / kFs));
    }
    return x;
}

// Plucked string (Karplus-Strong), for demos
static void Pluck(std::vector<double> &out, size_t start, double f, double amp, uint32_t seed)
{
    size_t n = static_cast<size_t>(kFs / f);
    std::vector<double> d(n);
    std::mt19937 g(seed);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (auto &v : d)
    {
        v = u(g);
    }
    size_t idx = 0;
    for (size_t i = start; i < out.size(); i++)
    {
        double y = d[idx];
        d[idx] = 0.5 * (d[idx] + d[(idx + 1) % n]) * 0.9985;
        idx = (idx + 1) % n;
        out[i] += y * amp * 32768.0;
    }
}

static std::vector<double> DemoMelody(size_t seconds)
{
    std::vector<double> x(static_cast<size_t>(kFs) * seconds, 0.0);
    // A minor-ish arpeggio, one note every 0.4 s
    const double notes[] = {220.0, 261.63, 329.63, 440.0, 329.63, 261.63, 196.0, 246.94, 293.66, 392.0, 293.66, 246.94};
    size_t step = static_cast<size_t>(kFs * 0.4);
    for (size_t i = 0; i * step < x.size(); i++)
    {
        Pluck(x, i * step, notes[i % 12], 0.22, 100 + static_cast<uint32_t>(i));
    }
    return x;
}

// ----- tests -----

static void TestReverb(const std::string &demo_dir, const std::vector<double> &melody)
{
    std::printf("Reverb\n");
    Pools p;
    Reverb rv;
    bool ok = rv.Init(p.l.data(), p.l.size(), p.r.data(), p.r.size());
    Check(ok, "Init with 50800-word buffers", Fmt("(needs %.0f + %.0f words)", Reverb::kLeftWords, Reverb::kRightWords).c_str());
    bool too_small = !Reverb().Init(p.l.data(), 1000, p.r.data(), 1000);
    Check(too_small, "Init rejects buffers that are too small");
    rv.Clear();

    // Impulse response -> RT60 from backwards-integrated energy (Schroeder). We fit the -5..-25 dB span and extrapolate to -60.
    for (float rt : {0.5f, 2.0f, 8.0f})
    {
        rv.Clear();
        rv.SetDecay(rt);
        rv.SetDamping(32767);
        size_t n = static_cast<size_t>(kFs * (rt * 2.5 + 1));
        std::vector<double> ir(n);
        for (size_t i = 0; i < n; i++)
        {
            int32_t l, r;
            rv.Process(i == 0 ? 30000 : 0, i == 0 ? 30000 : 0, l, r);
            ir[i] = l;
        }
        std::vector<double> edc(n);
        double acc = 0;
        for (size_t i = n; i-- > 0;)
        {
            acc += ir[i] * ir[i];
            edc[i] = acc;
        }
        double e0 = edc[0];
        size_t i5 = 0, i25 = 0;
        for (size_t i = 0; i < n; i++)
        {
            double d = 10 * std::log10(edc[i] / e0 + 1e-30);
            if (!i5 && d <= -5)
            {
                i5 = i;
            }
            if (!i25 && d <= -25)
            {
                i25 = i;
                break;
            }
        }
        double measured = (i25 - i5) / kFs * (60.0 / 20.0);
        Check(measured > rt * 0.7 && measured < rt * 1.3, "RT60 matches request", Fmt("(requested %.1f s, measured %.2f s)", rt, measured).c_str());
    }

    // Tail dies out completely (no stuck hiss from fixed point rounding)
    {
        rv.Clear();
        rv.SetDecay(3.f);
        std::vector<double> out;
        for (size_t i = 0; i < static_cast<size_t>(kFs * 90); i++)
        {
            int32_t l, r;
            rv.Process(i < 2000 ? 20000 : 0, i < 2000 ? 20000 : 0, l, r);
            out.push_back(l);
        }
        double tail = Rms(out, out.size() - 44000, out.size());
        Check(tail < 0.0001, "Tail decays to (almost) silence, no stuck hiss, <= 1 LSB dither", Fmt("(final RMS %.6f of full scale)", tail).c_str());
    }

    // Level: wet noise RMS relative to input noise RMS, at several decays and dampings
    {
        auto in = Noise(static_cast<size_t>(kFs * 6), -20.0);
        double worst_hi = -99, worst_lo = 99;
        for (float rt : {0.3f, 1.f, 4.f, 15.f, 30.f})
        {
            for (int damp : {32767, 12000, 3000})
            {
                rv.Clear();
                rv.SetDecay(rt);
                rv.SetDamping(damp);
                std::vector<double> out;
                for (double s : in)
                {
                    int32_t l, r;
                    rv.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
                    out.push_back(l);
                }
                double g = Db(Rms(out, out.size() / 2)) - Db(Rms(in, in.size() / 2));
                if (std::getenv("FXPLUS_VERBOSE"))
                {
                    std::printf("    rt60 %5.1f s  damping %5d  wet level %+6.1f dB\n", rt, damp, g);
                }
                worst_hi = std::max(worst_hi, g);
                worst_lo = std::min(worst_lo, g);
            }
        }
        Check(worst_hi < 3.0 && worst_lo > -14.0, "Wet level stays within -14..+3 dB of the input for noise", Fmt("(range %.1f .. %.1f dB)", worst_lo, worst_hi).c_str());
    }

    // Full-scale square wave burst must not wrap around (saturates instead)
    {
        rv.Clear();
        rv.SetDecay(30.f);
        rv.SetDamping(32767);
        int32_t peak = 0;
        for (size_t i = 0; i < 200000; i++)
        {
            int32_t l, r;
            int32_t s = ((i / 50) & 1) ? 32767 : -32768;
            rv.Process(s, s, l, r);
            peak = std::max(peak, std::abs(l));
        }
        Check(peak <= 32768, "Full scale square wave input does not overflow");
    }

    // Stereo: left/right wet outputs should be decorrelated
    {
        rv.Clear();
        rv.SetDecay(2.f);
        auto in = Noise(static_cast<size_t>(kFs * 3), -20.0);
        double sll = 0, srr = 0, slr = 0;
        for (size_t i = 0; i < in.size(); i++)
        {
            int32_t l, r;
            rv.Process(static_cast<int32_t>(in[i]), static_cast<int32_t>(in[i]), l, r);
            if (i > 44000)
            {
                sll += double(l) * l;
                srr += double(r) * r;
                slr += double(l) * r;
            }
        }
        double corr = slr / std::sqrt(sll * srr);
        Check(std::fabs(corr) < 0.5, "Left/right outputs decorrelated", Fmt("(correlation %.2f)", corr).c_str());
    }

    // Pre-delay: first output sample must appear later when pre-delay is set
    {
        rv.Clear();
        rv.SetDecay(1.f);
        rv.SetPreDelay(2000);
        size_t first = 0;
        for (size_t i = 0; i < 20000 && !first; i++)
        {
            int32_t l, r;
            rv.Process(i == 0 ? 30000 : 0, i == 0 ? 30000 : 0, l, r);
            if (l != 0 || r != 0)
            {
                first = i;
            }
        }
        Check(first >= 2000 + 1000 && first < 2000 + 1300, "Pre-delay shifts the onset (plus the 1116 sample shortest comb)", Fmt("(first output at sample %.0f, pre-delay 2000)", double(first)).c_str());
    }

    if (!demo_dir.empty())
    {
        for (auto [name, rt, damp] : {std::tuple{"reverb_room", 1.2f, 20000}, std::tuple{"reverb_hall", 4.f, 9000}, std::tuple{"reverb_huge_dark", 14.f, 3500}})
        {
            rv.Clear();
            rv.SetDecay(rt);
            rv.SetDamping(damp);
            std::vector<double> ol, orr;
            for (size_t i = 0; i < melody.size() + static_cast<size_t>(kFs * 3); i++)
            {
                double s = i < melody.size() ? melody[i] : 0;
                int32_t l, r;
                rv.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
                // 70% dry / 100% wet is what the app does at full Amount
                ol.push_back(s * 0.4 + l);
                orr.push_back(s * 0.4 + r);
            }
            WriteWav(demo_dir + "/" + name + ".wav", ol, orr);
        }
    }
}

static void TestChorus(const std::string &demo_dir, const std::vector<double> &melody)
{
    std::printf("Chorus\n");
    Pools p;
    Chorus ch;
    Check(ch.Init(p.l.data(), p.l.size(), p.r.data(), p.r.size()), "Init with 50800-word buffers");
    Check(!Chorus().Init(p.l.data(), 100, p.r.data(), 100), "Init rejects buffers that are too small");

    // Measure the delay modulation using an impulse train: find where each echo lands
    ch.Clear();
    ch.SetRate(1.0f);
    ch.SetDepth(300);
    ch.SetStereoPhase(0);
    int min_d = 99999, max_d = 0;
    {
        // Feed an impulse every 4000 samples, measure the position of the first output peak after each
        std::vector<int32_t> out(44000 * 3);
        for (size_t i = 0; i < out.size(); i++)
        {
            int32_t l, r;
            ch.Process((i % 4000) == 0 ? 30000 : 0, 0, l, r);
            out[i] = l;
        }
        for (size_t k = 1; k + 1 < out.size() / 4000; k++)
        {
            size_t base = k * 4000;
            int best = 0, bi = 0;
            for (int j = 0; j < 1500; j++)
            {
                if (out[base + j] > best)
                {
                    best = out[base + j];
                    bi = j;
                }
            }
            if (best > 0)
            {
                min_d = std::min(min_d, bi);
                max_d = std::max(max_d, bi);
            }
        }
    }
    Check(min_d >= 640 - 300 - 3 && max_d <= 880 + 300 + 3 && max_d - min_d > 150, "Delay sweeps within the expected range",
          Fmt("(first-echo positions %.0f .. %.0f samples)", min_d, max_d).c_str());

    // Level
    {
        ch.Clear();
        ch.SetRate(0.7f);
        ch.SetDepth(200);
        auto in = Noise(static_cast<size_t>(kFs * 3), -20.0);
        std::vector<double> out;
        for (double s : in)
        {
            int32_t l, r;
            ch.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
            out.push_back(l);
        }
        double g = Db(Rms(out, 44000)) - Db(Rms(in, 44000));
        Check(std::fabs(g) < 3.0, "Wet level close to input level", Fmt("(%.1f dB)", g).c_str());
    }

    // Vibrato check: a pure chorus (wet only) of a sine should have its frequency modulated, not shifted on average
    {
        ch.Clear();
        ch.SetRate(0.5f);
        ch.SetDepth(300);
        auto in = Sine(static_cast<size_t>(kFs * 4), 1000.0, -12.0);
        std::vector<double> out;
        for (double s : in)
        {
            int32_t l, r;
            ch.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
            out.push_back(l);
        }
        double mag;
        double pf = PeakFreq(out, 950, 1050, 44000, 44000 * 4, &mag);
        Check(std::fabs(pf - 1000.0) < 25.0, "Average pitch unchanged (vibrato of +-2% allowed)", Fmt("(peak at %.1f Hz)", pf).c_str());
    }

    if (!demo_dir.empty())
    {
        for (auto [name, rate, depth, stereo] : {std::tuple{"chorus_slow_wide", 0.45f, 320, 0x80000000u}, std::tuple{"chorus_fast_narrow", 3.2f, 110, 0x20000000u}})
        {
            ch.Clear();
            ch.SetRate(rate);
            ch.SetDepth(depth);
            ch.SetStereoPhase(stereo);
            std::vector<double> ol, orr;
            for (double s : melody)
            {
                int32_t l, r;
                ch.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
                ol.push_back(s * 0.6 + l * 0.6);
                orr.push_back(s * 0.6 + r * 0.6);
            }
            WriteWav(demo_dir + "/" + name + ".wav", ol, orr);
        }
    }
}

static void TestPitch(const std::string &demo_dir, const std::vector<double> &melody)
{
    std::printf("Pitch shifter\n");
    Pools p;
    PitchShifter ps;
    Check(ps.Init(p.l.data(), p.l.size(), p.r.data(), p.r.size()), "Init with 50800-word buffers");
    Check(!PitchShifter().Init(p.l.data(), 100, p.r.data(), 100), "Init rejects buffers that are too small");

    for (double semis : {-12.0, -7.0, -5.0, -1.0, 1.0, 3.0, 7.0, 12.0})
    {
        ps.Clear();
        ps.SetShift(static_cast<float>(semis), 0.f);
        double f_in = 440.0;
        auto in = Sine(static_cast<size_t>(kFs * 2), f_in, -12.0);
        std::vector<double> out;
        for (double s : in)
        {
            int32_t l, r;
            ps.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
            out.push_back(l);
        }
        double expect = f_in * std::pow(2.0, semis / 12.0);
        double mag;
        double pf = PeakFreq(out, expect * 0.7, expect * 1.4, 22000, 88000, &mag);
        double cf = Centroid(out, expect * 0.8, expect * 1.25, 22000, 88000);
        double cents = 1200.0 * std::log2(cf / expect);
        double level = Db(Rms(out, 22000, 88000)) - Db(Rms(in, 22000, 88000));
        (void)mag;
        char d[220];
        std::snprintf(d, sizeof d, "(%+.0f st: expected %.1f Hz, centroid %.1f Hz = %+.1f cents, peak %.1f Hz, level %+.1f dB)", semis, expect, cf, cents, pf, level);
        Check(std::fabs(cents) < 30.0 && level > -3.0 && level < 2.0, "Shifted sine lands on the right pitch and keeps its level", d);
    }

    // Detune: left and right differ
    {
        ps.Clear();
        ps.SetShift(0.f, 40.f); // +-20 cents
        auto in = Sine(static_cast<size_t>(kFs * 2), 440.0, -12.0);
        std::vector<double> ol, orr;
        for (double s : in)
        {
            int32_t l, r;
            ps.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
            ol.push_back(l);
            orr.push_back(r);
        }
        double fl = PeakFreq(ol, 420, 460, 22000, 88000), fr = PeakFreq(orr, 420, 460, 22000, 88000);
        double span = 1200.0 * std::log2(fr / fl);
        Check(span > 25.0 && span < 55.0, "Stereo detune splits left and right", Fmt("(L %.1f Hz, R %.1f Hz, %.0f cents apart)", fl, fr, span).c_str());
    }

    // Broadband signal stays bounded
    {
        ps.Clear();
        ps.SetShift(5.f, 10.f);
        auto in = Noise(static_cast<size_t>(kFs * 2), -12.0);
        std::vector<double> out;
        for (double s : in)
        {
            int32_t l, r;
            ps.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
            out.push_back(l);
        }
        double g = Db(Rms(out, 44000)) - Db(Rms(in, 44000));
        Check(std::fabs(g) < 3.0, "Noise level preserved", Fmt("(%.1f dB)", g).c_str());
    }

    if (!demo_dir.empty())
    {
        for (auto [name, st, cents] : {std::tuple{"pitch_up_fifth", 7.f, 0.f}, std::tuple{"pitch_down_octave", -12.f, 0.f}, std::tuple{"pitch_detune_thick", 0.f, 36.f}, std::tuple{"pitch_up_octave", 12.f, 0.f}})
        {
            ps.Clear();
            ps.SetShift(st, cents);
            std::vector<double> ol, orr;
            for (double s : melody)
            {
                int32_t l, r;
                ps.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
                ol.push_back(st == 0.f ? s * 0.6 + l * 0.7 : l);
                orr.push_back(st == 0.f ? s * 0.6 + r * 0.7 : r);
            }
            WriteWav(demo_dir + "/" + name + ".wav", ol, orr);
        }
    }
}

static void TestDistortion(const std::string &demo_dir, const std::vector<double> &melody)
{
    std::printf("Distortion\n");
    Distortion d;

    // Level compensation across the drive range. Reference tone (peak 0.2 = -14 dBFS) must keep its RMS,
    // quieter material gets louder with drive (that is what drive does), hotter material gets quieter.
    for (double peak : {-14.0, -24.0, -6.0})
    {
        double lo = 99, hi = -99;
        for (int gain_q8 : {256, 384, 768, 1536, 2560, 6144, 12288, 25600, 32768})
        {
            d.Reset();
            d.SetDrive(gain_q8);
            d.SetTone(32767);
            d.SetBias(0);
            auto in = Sine(static_cast<size_t>(kFs), 220.0, peak);
            std::vector<double> out;
            for (double s : in)
            {
                out.push_back(d.Process(static_cast<int32_t>(s)));
            }
            double g = Db(Rms(out, 8000)) - Db(Rms(in, 8000));
            lo = std::min(lo, g);
            hi = std::max(hi, g);
        }
        char det[200];
        std::snprintf(det, sizeof det, "(sine peak %.0f dBFS: output vs input RMS %+.1f .. %+.1f dB over drive 1x..128x)", peak, lo, hi);
        if (peak == -14.0)
        {
            Check(lo > -2.0 && hi < 2.0, "Reference level is constant over the whole drive range", det);
        }
        else if (peak < -14.0)
        {
            Check(hi < 16.0, "Quiet material: boost stays bounded", det);
        }
        else
        {
            Check(lo > -16.0 && hi < 3.0, "Hot material: never louder than the input", det);
        }
    }

    // Unity drive stays clean (THD)
    {
        d.Reset();
        d.SetDrive(256);
        d.SetTone(32767);
        d.SetBias(0);
        auto in = Sine(static_cast<size_t>(kFs), 220.0, -30.0);
        std::vector<double> out;
        for (double s : in)
        {
            out.push_back(d.Process(static_cast<int32_t>(s)));
        }
        double f1 = Goertzel(out, 220.0, 8000, 40000), f3 = Goertzel(out, 660.0, 8000, 40000);
        Check(Db(f3 / f1) < -35.0, "Low drive on a quiet signal stays clean", Fmt("(3rd harmonic %.1f dB)", Db(f3 / f1)).c_str());
    }

    // Harmonics grow with drive; symmetric clipping makes odd harmonics only
    {
        d.Reset();
        d.SetDrive(8192);
        d.SetTone(32767);
        d.SetBias(0);
        auto in = Sine(static_cast<size_t>(kFs), 220.0, -12.0);
        std::vector<double> out;
        for (double s : in)
        {
            out.push_back(d.Process(static_cast<int32_t>(s)));
        }
        double h1 = Goertzel(out, 220, 8000, 40000), h2 = Goertzel(out, 440, 8000, 40000), h3 = Goertzel(out, 660, 8000, 40000);
        Check(h3 / h1 > 0.1 && h2 / h1 < 0.02, "Symmetric mode: strong odd harmonics, weak even ones", Fmt("(3rd %.1f dB, 2nd %.1f dB)", Db(h3 / h1), Db(h2 / h1)).c_str());

        d.Reset();
        d.SetDrive(2560);
        d.SetBias(9000);
        out.clear();
        for (double s : in)
        {
            out.push_back(d.Process(static_cast<int32_t>(s)));
        }
        h1 = Goertzel(out, 220, 8000, 40000);
        h2 = Goertzel(out, 440, 8000, 40000);
        Check(h2 / h1 > 0.05, "Bias adds even harmonics", Fmt("(2nd %.1f dB)", Db(h2 / h1)).c_str());
        double mean = 0;
        for (size_t i = 20000; i < out.size(); i++)
        {
            mean += out[i];
        }
        mean /= (out.size() - 20000);
        Check(std::fabs(mean) < 150.0, "DC offset removed after biasing", Fmt("(mean %.1f LSB)", mean).c_str());
    }

    // Tone filter darkens
    {
        auto in = Noise(static_cast<size_t>(kFs), -20.0);
        double hf[2];
        int k = 0;
        for (int alpha : {32767, 2000})
        {
            d.Reset();
            d.SetDrive(2560);
            d.SetBias(0);
            d.SetTone(alpha);
            std::vector<double> out;
            for (double s : in)
            {
                out.push_back(d.Process(static_cast<int32_t>(s)));
            }
            hf[k++] = Goertzel(out, 10000, 8000, 40000);
        }
        Check(Db(hf[1] / hf[0]) < -12.0, "Tone knob removes high frequencies", Fmt("(10 kHz: %.1f dB)", Db(hf[1] / hf[0])).c_str());
    }

    // Extreme inputs never go out of range
    {
        d.Reset();
        d.SetDrive(32768);
        d.SetBias(32767);
        d.SetTone(32767);
        int32_t peak = 0;
        for (int i = 0; i < 100000; i++)
        {
            int32_t s = (i / 7 & 1) ? 32767 : -32768;
            peak = std::max(peak, std::abs(d.Process(s)));
        }
        Check(peak <= 32768, "Extreme drive + bias + full scale input stays in range");
    }

    if (!demo_dir.empty())
    {
        for (auto [name, gain, bias, tone] : {std::tuple{"dist_warm", 1536, 0, 9000}, std::tuple{"dist_fuzz", 14000, 6000, 14000}, std::tuple{"dist_asym_crunch", 5000, 14000, 20000}})
        {
            Distortion dl, dr;
            for (auto *x : {&dl, &dr})
            {
                x->Reset();
                x->SetDrive(gain);
                x->SetBias(bias);
                x->SetTone(tone);
            }
            std::vector<double> ol, orr;
            for (double s : melody)
            {
                ol.push_back(dl.Process(static_cast<int32_t>(s)));
                orr.push_back(dr.Process(static_cast<int32_t>(s)));
            }
            WriteWav(demo_dir + "/" + name + ".wav", ol, orr);
        }
    }
}


static void TestPhaser(const std::string &demo_dir, const std::vector<double> &melody)
{
    std::printf("Phaser\n");
    Phaser ph;

    // The all-pass chain alone must not change the level
    {
        ph.Reset();
        ph.SetRate(0.5f);
        ph.SetFeedback(0);
        auto in = Noise(static_cast<size_t>(kFs * 3), -20.0);
        std::vector<double> out;
        for (double s : in)
        {
            int32_t l, r;
            ph.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
            out.push_back(l);
        }
        double g = Db(Rms(out, 20000)) - Db(Rms(in, 20000));
        Check(std::fabs(g) < 0.5, "All-pass chain keeps the level", Fmt("(%.2f dB)", g).c_str());
    }

    // Frequency response at two fixed sweep positions (rate 0; the right side is 90 degrees ahead = the sweep's top)
    {
        ph.Reset();
        ph.SetRate(0.f);
        ph.SetFeedback(0);
        ph.SetStereoPhase(0x40000000u);
        auto in = Noise(static_cast<size_t>(kFs * 6), -20.0);
        std::vector<double> ol, orr, mixl, mixr;
        for (double s : in)
        {
            int32_t l, r;
            ph.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
            mixl.push_back((s + l) * 0.5);
            mixr.push_back((s + r) * 0.5);
        }
        auto notch = [&](const std::vector<double> &mix, double &deepest_db, double &deepest_hz, double &peak_db)
        {
            deepest_db = 99;
            peak_db = -99;
            deepest_hz = 0;
            for (double f = 120; f < 9000; f *= 1.04)
            {
                double h = Db(Goertzel(mix, f, 22000, 200000) / std::max(1e-9, Goertzel(in, f, 22000, 200000)));
                if (h < deepest_db)
                {
                    deepest_db = h;
                    deepest_hz = f;
                }
                peak_db = std::max(peak_db, h);
            }
        };
        double dl, hl, pl, dr, hr, pr;
        notch(mixl, dl, hl, pl);
        notch(mixr, dr, hr, pr);
        // Noise gives a noisy estimate of the response, so the notch depth is judged generously
        Check(dl < -12.0, "50/50 mix has a deep notch (sweep centre)", Fmt("(%.1f dB at %.0f Hz)", dl, hl).c_str());
        Check(dr < -12.0, "50/50 mix has a deep notch (sweep top)", Fmt("(%.1f dB at %.0f Hz)", dr, hr).c_str());
        Check(hr > hl * 1.5, "Notch moves up when the sweep moves up", Fmt("(%.0f Hz -> %.0f Hz)", hl, hr).c_str());
        Check(pl < 1.0 && pr < 1.0, "No gain above 0 dB without feedback", Fmt("(peaks %+.1f / %+.1f dB)", pl, pr).c_str());

        // Feedback makes the response peakier
        ph.Reset();
        ph.SetFeedback(27000);
        std::vector<double> fb;
        for (double s : in)
        {
            int32_t l, r;
            ph.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
            fb.push_back((s + l) * 0.5);
        }
        double d2, h2, p2;
        notch(fb, d2, h2, p2);
        Check(p2 > 6.0 && p2 < 20.0, "Feedback creates resonant peaks", Fmt("(peak %+.1f dB)", p2).c_str());
    }

    // Full-scale noise with maximum feedback stays bounded
    {
        ph.Reset();
        ph.SetRate(7.f);
        ph.SetFeedback(32767);
        int32_t peak = 0;
        std::mt19937 g(5);
        for (int i = 0; i < 400000; i++)
        {
            int32_t a = (g() & 1) ? 32767 : -32768, l, r;
            ph.Process(a, -a, l, r);
            peak = std::max({peak, std::abs(l), std::abs(r)});
        }
        Check(peak <= 32768, "Maximum feedback with full scale input stays in range");
    }

    if (!demo_dir.empty())
    {
        for (auto [name, rate, fb] : {std::tuple{"phaser_slow", 0.2f, 12000}, std::tuple{"phaser_resonant", 0.6f, 26000}})
        {
            ph.Reset();
            ph.SetRate(rate);
            ph.SetFeedback(fb);
            ph.SetStereoPhase(0x40000000u);
            std::vector<double> ol, orr;
            for (double s : melody)
            {
                int32_t l, r;
                ph.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
                ol.push_back((s + l) * 0.5);
                orr.push_back((s + r) * 0.5);
            }
            WriteWav(demo_dir + "/" + name + ".wav", ol, orr);
        }
    }
}

static void TestPlate(const std::string &demo_dir, const std::vector<double> &melody)
{
    std::printf("Plate reverb\n");
    Pools p;
    PlateReverb pr;
    Check(pr.Init(p.l.data(), p.l.size(), p.r.data(), p.r.size()), "Init with 50800-word buffers", Fmt("(needs %.0f + %.0f words)", PlateReverb::kLeftWords, PlateReverb::kRightWords).c_str());
    Check(!PlateReverb().Init(p.l.data(), 1000, p.r.data(), 1000), "Init rejects buffers that are too small");

    for (float rt : {0.5f, 1.5f, 4.0f, 10.0f})
    {
        pr.Clear();
        pr.SetDecay(rt);
        pr.SetDamping(32767);
        size_t n = static_cast<size_t>(kFs * (rt * 2.5 + 1));
        std::vector<double> ir(n);
        for (size_t i = 0; i < n; i++)
        {
            int32_t l, r;
            pr.Process(i == 0 ? 30000 : 0, i == 0 ? 30000 : 0, l, r);
            ir[i] = l;
        }
        std::vector<double> edc(n);
        double acc = 0;
        for (size_t i = n; i-- > 0;)
        {
            acc += ir[i] * ir[i];
            edc[i] = acc;
        }
        size_t i5 = 0, i25 = 0;
        for (size_t i = 0; i < n; i++)
        {
            double d = 10 * std::log10(edc[i] / edc[0] + 1e-30);
            if (!i5 && d <= -5)
            {
                i5 = i;
            }
            if (!i25 && d <= -25)
            {
                i25 = i;
                break;
            }
        }
        double measured = (i25 - i5) / kFs * 3.0;
        Check(measured > rt * 0.6 && measured < rt * 1.5, "RT60 close to the requested time", Fmt("(requested %.1f s, measured %.2f s)", rt, measured).c_str());
    }

    {
        pr.Clear();
        pr.SetDecay(3.f);
        std::vector<double> out;
        for (size_t i = 0; i < static_cast<size_t>(kFs * 90); i++)
        {
            int32_t l, r;
            pr.Process(i < 2000 ? 20000 : 0, i < 2000 ? 20000 : 0, l, r);
            out.push_back(l);
        }
        double tail = Rms(out, out.size() - 44000, out.size());
        Check(tail < 0.0001, "Tail decays to (almost) silence", Fmt("(final RMS %.6f of full scale)", tail).c_str());
    }

    {
        auto in = Noise(static_cast<size_t>(kFs * 8), -20.0);
        double hi = -99, lo = 99;
        for (float rt : {0.3f, 1.f, 4.f, 15.f, 30.f})
        {
            for (int damp : {32767, 12000, 3000})
            {
                pr.Clear();
                pr.SetDecay(rt);
                pr.SetDamping(damp);
                std::vector<double> out;
                for (double s : in)
                {
                    int32_t l, r;
                    pr.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
                    out.push_back(l);
                }
                double g = Db(Rms(out, out.size() / 2)) - Db(Rms(in, in.size() / 2));
                if (std::getenv("FXPLUS_VERBOSE"))
                {
                    std::printf("    plate rt60 %5.1f s  damping %5d  wet level %+6.1f dB\n", rt, damp, g);
                }
                hi = std::max(hi, g);
                lo = std::min(lo, g);
            }
        }
        Check(hi < 3.0 && lo > -14.0, "Wet level stays within -14..+3 dB of the input for noise", Fmt("(range %.1f .. %.1f dB)", lo, hi).c_str());
    }

    {
        pr.Clear();
        pr.SetDecay(30.f);
        pr.SetDamping(32767);
        int32_t peak = 0;
        for (size_t i = 0; i < 300000; i++)
        {
            int32_t l, r, s = ((i / 50) & 1) ? 32767 : -32768;
            pr.Process(s, s, l, r);
            peak = std::max({peak, std::abs(l), std::abs(r)});
        }
        Check(peak <= 32768, "Full scale square wave input does not overflow");
    }

    {
        pr.Clear();
        pr.SetDecay(2.f);
        auto in = Noise(static_cast<size_t>(kFs * 3), -20.0);
        double sll = 0, srr = 0, slr = 0;
        for (size_t i = 0; i < in.size(); i++)
        {
            int32_t l, r;
            pr.Process(static_cast<int32_t>(in[i]), static_cast<int32_t>(in[i]), l, r);
            if (i > 44000)
            {
                sll += double(l) * l;
                srr += double(r) * r;
                slr += double(l) * r;
            }
        }
        double corr = slr / std::sqrt(sll * srr);
        Check(std::fabs(corr) < 0.4, "Left/right outputs decorrelated", Fmt("(correlation %.2f)", corr).c_str());
    }

    // Density: the impulse response must not have a few strong isolated echoes. Compare the loudest 5 ms block
    // between 150 and 600 ms with the average of that span.
    {
        pr.Clear();
        pr.SetDecay(3.f);
        std::vector<double> ir(static_cast<size_t>(kFs * 0.7));
        for (size_t i = 0; i < ir.size(); i++)
        {
            int32_t l, r;
            pr.Process(i == 0 ? 30000 : 0, i == 0 ? 30000 : 0, l, r);
            ir[i] = l;
        }
        size_t a = static_cast<size_t>(kFs * 0.15), b = static_cast<size_t>(kFs * 0.6), blk = static_cast<size_t>(kFs * 0.005);
        double avg = Rms(ir, a, b), worst = 0;
        for (size_t i = a; i + blk < b; i += blk)
        {
            worst = std::max(worst, Rms(ir, i, i + blk));
        }
        Check(Db(worst) - Db(avg) < 8.0, "Dense, no isolated echoes in the early tail", Fmt("(loudest 5 ms block is %.1f dB above the average)", Db(worst) - Db(avg)).c_str());
    }

    if (!demo_dir.empty())
    {
        for (auto [name, rt, damp] : {std::tuple{"plate_bright", 2.5f, 30000}, std::tuple{"plate_long_dark", 9.f, 5000}})
        {
            pr.Clear();
            pr.SetDecay(rt);
            pr.SetDamping(damp);
            std::vector<double> ol, orr;
            for (size_t i = 0; i < melody.size() + static_cast<size_t>(kFs * 3); i++)
            {
                double s = i < melody.size() ? melody[i] : 0;
                int32_t l, r;
                pr.Process(static_cast<int32_t>(s), static_cast<int32_t>(s), l, r);
                ol.push_back(s * 0.4 + l);
                orr.push_back(s * 0.4 + r);
            }
            WriteWav(demo_dir + "/" + name + ".wav", ol, orr);
        }
    }
}

// Randomized stress test: parameters thrown around while loud random audio runs through. Meant for ASan/UBSan.
static void Stress()
{
    std::printf("Stress test (randomized parameters, full scale noise)\n");
    Pools p;
    Reverb rv;
    Chorus ch;
    PitchShifter ps;
    Distortion dl, dr;
    PlateReverb plate;
    Phaser phaser;
    std::mt19937 g(42);
    auto rnd = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(g); };

    // Each effect lives on its own part of the pool in the app? No: they share it, one at a time. Test each in turn on the same memory.
    int32_t peak = 0;
    for (int round = 0; round < 60; round++)
    {
        int which = round % 6;
        rv.Init(p.l.data(), p.l.size(), p.r.data(), p.r.size());
        ch.Init(p.l.data(), p.l.size(), p.r.data(), p.r.size());
        ps.Init(p.l.data(), p.l.size(), p.r.data(), p.r.size());
        plate.Init(p.l.data(), p.l.size(), p.r.data(), p.r.size());
        phaser.Reset();
        for (size_t i = 0; i < 30000; i++)
        {
            if (i % 997 == 0)
            {
                rv.SetDecay(static_cast<float>(rnd(0.0, 40.0)));
                rv.SetDamping(static_cast<int32_t>(rnd(-1000, 40000)));
                rv.SetPreDelay(static_cast<size_t>(rnd(0, 6000)));
                ch.SetRate(static_cast<float>(rnd(0.0, 30.0)));
                ch.SetDepth(static_cast<int32_t>(rnd(-100, 1000)));
                ch.SetStereoPhase(static_cast<uint32_t>(g()));
                ps.SetShift(static_cast<float>(rnd(-24, 24)), static_cast<float>(rnd(-100, 100)));
                plate.SetDecay(static_cast<float>(rnd(0.0, 40.0)));
                plate.SetDamping(static_cast<int32_t>(rnd(-1000, 40000)));
                plate.SetPreDelay(static_cast<size_t>(rnd(0, 6000)));
                phaser.SetRate(static_cast<float>(rnd(0.0, 30.0)));
                phaser.SetFeedback(static_cast<int32_t>(rnd(-1000, 40000)));
                phaser.SetStereoPhase(static_cast<uint32_t>(g()));
                for (auto *x : {&dl, &dr})
                {
                    x->SetDrive(static_cast<int32_t>(rnd(-100, 40000)));
                    x->SetTone(static_cast<int32_t>(rnd(-100, 40000)));
                    x->SetBias(static_cast<int32_t>(rnd(-100, 40000)));
                }
            }
            int32_t a = (g() & 1) ? 32767 : -32768;
            int32_t b = static_cast<int32_t>(g() % 65536) - 32768;
            int32_t l = 0, r = 0;
            switch (which)
            {
            case 0:
                rv.Process(a, b, l, r);
                break;
            case 1:
                ch.Process(a, b, l, r);
                break;
            case 2:
                ps.Process(a, b, l, r);
                break;
            case 3:
                l = dl.Process(a);
                r = dr.Process(b);
                break;
            case 4:
                plate.Process(a, b, l, r);
                break;
            case 5:
                phaser.Process(a, b, l, r);
                break;
            }
            peak = std::max({peak, std::abs(l), std::abs(r)});
        }
    }
    Check(peak <= 32768, "No output ever leaves the 16 bit range", Fmt("(peak %.0f)", peak).c_str());
    Check(true, "Completed without sanitizer errors (if any, the process would have aborted)");
}

int main(int argc, char **argv)
{
    std::string demo = argc > 1 ? argv[1] : "";
    std::vector<double> melody;
    if (!demo.empty())
    {
        melody = DemoMelody(10);
    }
    TestReverb(demo, melody);
    TestChorus(demo, melody);
    TestPitch(demo, melody);
    TestDistortion(demo, melody);
    TestPhaser(demo, melody);
    TestPlate(demo, melody);
    Stress();
    std::printf("\n%s (%d failed)\n", g_fail ? "FAILURES" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
