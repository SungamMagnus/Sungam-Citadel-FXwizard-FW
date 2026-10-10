/*
MIT License

Copyright (c) 2024 Marek Mach (Bastl Instruments)
Copyright (c) 2024 Vaclav Mach (Bastl Instruments)

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

#include <cstddef>
#include <cstdint>
#include "common/controls/FancyMode.hpp"
#include "common/controls/FancyPot.hpp"
#include "common/core/App.hpp"
#include "common/core/Hardware.hpp"
#include "common/dsp/control/AdsrEnv.hpp"
#include "common/dsp/control/BeatDetector.hpp"
#include "common/dsp/control/EnvelopeFollower.hpp"
#include "common/dsp/effects/SoftClipper.hpp"
#include "common/dsp/filters/DjFilter.hpp"
#include "common/dsp/math/Fraction.hpp"
#include "common/dsp/math/qmath.hpp"
#include "common/dsp/synthesis/Oscillator.hpp"
#include "common/dsp/utility/AdvancedDynamicDelayLine.hpp"
#include "common/dsp/utility/AutoFreeze.hpp"
#include "common/fastcode.hpp"
#include "common/peripherals/WS2812.hpp"
#include "ExtraEffects.hpp"
#include "FxWizardFile.hpp"
#include "FxWizardParameterMaps.hpp"
#include "cc.hpp"

// If the dry-wet starts being unresponsive, either change
// value of kDryWetChangeCounterMax and kLowSignal or
// disable this feature by writing a 0 below.
#define NEAR_ZERO_DRYWET 1

namespace kastle2
{

/**
 * @class AppFxWizardPlus
 * @ingroup apps
 * @brief FX Wizard variant with seven effects: delay, chorus, phaser, reverb (hall), reverb (plate),
 *        distortion and pitcher. Based on the Bastl Instruments FX Wizard (Marek Mach, Vaclav Mach, 2024).
 *
 * Delay and Pitcher are the stock FX Wizard modes, the others are new, see ExtraEffects.hpp for their DSP.
 * The stock Flanger, Freezer, Panner, Crusher, Slicer, Replayer and Shifter were removed.
 */

class AppFxWizardPlus : public virtual App
{
public:
    /**
     * @brief All the modes of the FX Wizard in order.
     * @note Make sure the order matches the LED colors in kLedColors.
     */
    enum class Mode
    {
        DELAY,      // GREEN
        CHORUS,     // CYAN
        PHASER,     // ORANGE
        REVERB,     // PURPLE, comb based hall / room
        PLATE,      // BLUE, Dattorro plate
        DISTORTION, // YELLOW
        PITCHER,    // RASPBERRY
        COUNT,
    };

    /**
     * @brief Mode LED colors, order must match order of modes in the enum above
     */
    static inline const EnumArray<Mode, uint32_t> kLedColors = {
        WS2812::GREEN,
        WS2812::CYAN,
        WS2812::ORANGE,
        WS2812::PURPLE,
        WS2812::BLUE,
        WS2812::YELLOW,
        WS2812::RASPBERRY,
    };

    /**
     * @brief App ID stored in the EEPROM. Different from the stock FX Wizard (0xB1), so the two firmwares
     *        never read each other's settings (the mode list is different).
     *        Picked from the community range, see APP_LIST.md.
     */
    static constexpr uint8_t kAppId = 0x7F;

    /**
     * @brief Initializes all the parameters, memory, etc.
     */
    void Init();

    /**
     * @brief Deinitializes the app, stops all effects, etc.
     */
    void DeInit();

    /**
     * @brief Called each interrupt loop. Implements all the audio processing.
     * @param input Input buffer.
     * @param output Output buffer.
     * @param size Number of sample pairs in the buffer (real size of the buffer is 2*size).
     */
    FASTCODE void AudioLoop(q15_t *input, q15_t *output, size_t size);

    /**
     * @brief Called each time AudioLoop isn't busy.
     */
    void UiLoop();

    /**
     * @brief Called only ONCE when the app is started.
     */
    FASTCODE void SecondCoreWorker();

    /**
     * @brief Called when the app is first loaded - initializes the memory values.
     */
    void MemoryInitialization();

    /**
     * @brief Handles incoming MIDI messages.
     * @param msg The MIDI message to handle.
     */
    void MidiCallback(midi::Message *msg);

    /**
     * @brief Returns the app ID.
     * @return The app ID.
     */
    uint8_t GetId()
    {
        return kAppId;
    }

private:
    bool inited_ = false;

    static constexpr Hardware::AnalogInput CV_FREE = Hardware::AnalogInput::PITCH_1;
    static constexpr Hardware::AnalogInput CV_STEP = Hardware::AnalogInput::PITCH_2;
    static constexpr Hardware::AnalogInput CV_DRYWET = Hardware::AnalogInput::PARAM_3;
    static constexpr Hardware::AnalogInput CV_FEEDBACK = Hardware::AnalogInput::PARAM_1;
    static constexpr Hardware::AnalogInput CV_MODE = Hardware::AnalogInput::MODE;

    static constexpr size_t kMemMode = Memory::ADDR_APP_SPACE + 0x0;

    // Notes above this trigger the current mode
    // Notes below this change the mode
    static constexpr size_t kMidiMinNote = 48; // C2

    FancyMode mode_selector_ = FancyMode(FancyMode::Config{
        .memory_addr = kMemMode,
        .modes_count = static_cast<size_t>(Mode::COUNT),
        .midi_cc = cc::MODE,
        .midi_attenuator_cc = cc::MODE_MOD,
        .midi_output_cc = cc::OUT_MODE,
        .midi_note_control = FancyPot::MidiNoteControl{
            .start = 0,
            .repeat = 12,
            .end = kMidiMinNote},
    });
    Mode mode_ = Mode::DELAY;
    Mode mode_prev_ = Mode::DELAY;

    size_t trigger_blink_counter = 0;

    q15_t input_left_ = Q15_ZERO;
    q15_t input_right_ = Q15_ZERO;
    q15_t output_left_ = Q15_ZERO;
    q15_t output_right_ = Q15_ZERO;

    EdgeDetector trigger_detect_ = EdgeDetector(EdgeDetector::Type::RISING);
    size_t sample_being_processed_ = 0;

    bool trigger_trigger_ = false;
    bool clock_trigger_ = false;
    bool mode_sh_trigger_ = false;
    bool time_sh_trigger_ = false;

    int32_t cv_step_ = 0;

    /**
     * @brief Potentiometer abstractions
     */
    enum class Pot
    {
        FEEDBACK,
        FEEDBACK_MOD,
        AMOUNT,
        AMOUNT_MOD,
        STEREO,
        TIME,
        TIME_MOD,
        FILTER,
        COUNT
    };
    EnumArray<Pot, std::unique_ptr<FancyPot>> pots_;

    int32_t cv_mode_ = 0;
    int32_t cv_mode_prev_ = 0;
    int32_t cv_mode_in_use_ = 0;

    static constexpr int32_t kModeCvStep = ADC_5V / static_cast<size_t>(Mode::COUNT);
    static constexpr int32_t kModeCvChanged = ADC_5V / 100;
    static constexpr int32_t kModeCvJumped = ADC_3V;

    q15_t feedback_volume_ = Q15_ZERO;
    size_t feedback_delay_ = 0;
    q15_t global_dry_wet_ = Q15_ZERO;
    q15_t global_dry_wet_queue_ = Q15_ZERO;

    // pitcher
    q31_t pitcher_frequency_ = 0.f;
    q15_t pitcher_dry_wet_ = Q15_ZERO;
    q31_t pitcher_depth_ = Q31_ZERO;
    q31_t pitcher_stereo_mix_ = Q15_ZERO;
    AdsrEnv pitcher_env_;
    q15_t pitcher_env_depth_ = Q15_ZERO;
    q15_t pitcher_depth_modulated_ = Q15_ZERO;

    // delay
    q15_t delay_wet_ = Q15_ZERO;
    q15_t delay_dry_ = Q15_ZERO;
    q15_t delay_feedback_ = Q15_ZERO;
    SoftClipper delay_clip_;
    EnvelopeFollower delay_compressor_;
    size_t delay_peak_counter_ = 0;
    static constexpr size_t delay_peak_counter_max_ = 8;
    q15_t delay_peak_ = Q15_ZERO;
    size_t delay_stereo_ = 0;
    size_t delay_length_ = 0;
    size_t delay_ticks_time_ = 0;
    size_t delay_ticks_time_last_ = 0;
    size_t delay_ticks_time_last_prev_ = 0;
    bool delay_synced_ = false;
    bool delay_synced_first_measurement_ = false;
    std::array<size_t, kDelaySyncedDividers.size()> delay_synced_times_ = {0};
    size_t delay_synced_highest_index_ = 0;
    size_t delay_time_left_prev_ = 0;
    size_t delay_time_right_prev_ = 0;

    // --- extra effects (see ExtraEffects.hpp) ---
    // They work in the memory of the long delay lines (delay_left_ / delay_right_), which is otherwise unused
    // while one of these modes is active. There is no RAM to spare for separate buffers.

    // reverb
    fxplus::Reverb reverb_;
    q15_t reverb_dry_ = Q15_MAX;
    q15_t reverb_wet_ = Q15_ZERO;
    float reverb_decay_prev_ = -1.f;
    int32_t reverb_damping_prev_ = -1;

    // plate reverb
    fxplus::PlateReverb plate_;
    q15_t plate_dry_ = Q15_MAX;
    q15_t plate_wet_ = Q15_ZERO;
    float plate_decay_prev_ = -1.f;
    int32_t plate_damping_prev_ = -1;

    // phaser
    fxplus::Phaser phaser_;
    q15_t phaser_dry_ = Q15_MAX;
    q15_t phaser_wet_ = Q15_ZERO;

    // chorus
    fxplus::Chorus chorus_;
    q15_t chorus_dry_ = Q15_MAX;
    q15_t chorus_wet_ = Q15_ZERO;

    // distortion
    fxplus::Distortion distortion_left_;
    fxplus::Distortion distortion_right_;
    q15_t distortion_dry_ = Q15_ZERO;
    q15_t distortion_wet_ = Q15_MAX;

    /**
     * @brief The effects that use the delay memory may only run once their Mode*Init() has wiped it.
     *        UiLoop changes mode_ first and runs ModeInit() afterwards, and the audio loop can interrupt in between.
     *        So AudioLoop clears these flags whenever the mode isn't the matching one, and Mode*Init() sets them
     *        when done. Until then the mode passes the dry signal.
     */
    volatile bool reverb_ready_ = false;
    volatile bool chorus_ready_ = false;
    volatile bool plate_ready_ = false;

    /**
     * @brief True while the long delay lines contain data of the extra effects (not audio for the stock modes).
     *        Cleared again when switching back to a stock mode, otherwise the delay would play back garbage.
     */
    bool delay_pool_dirty_ = false;

    /**
     * @brief True for modes which use the memory of the long delay lines for their own purposes.
     */
    static constexpr bool UsesDelayPool(Mode mode)
    {
        return mode == Mode::REVERB || mode == Mode::PLATE || mode == Mode::CHORUS;
    }

    std::unique_ptr<AdvancedDynamicDelayLine<q15least_t>> feedback_delay_left_;
    std::unique_ptr<AdvancedDynamicDelayLine<q15least_t>> feedback_delay_right_;
    std::unique_ptr<AdvancedDynamicDelayLine<q15least_t>> delay_left_;
    std::unique_ptr<AdvancedDynamicDelayLine<q15least_t>> delay_right_;
    SoftClipper feedback_clip_;
    Svf feedback_filter_left_;
    Svf feedback_filter_right_;
    Svf feedback_filter_lp_left_;
    Svf feedback_filter_lp_right_;
    DjFilter dj_filter_left_;
    DjFilter dj_filter_right_;
    Oscillator lfo_left_;
    Oscillator lfo_right_;

    void ModeInit();
    void ModeTrigger();
    void WorkLongDelay();

    void ModePitcherInit();
    void ModePitcher();
    void ModeDelayInit();
    void ModeDelay();
    void ModeReverbInit();
    void ModeReverb();
    void ModePlateInit();
    void ModePlate();
    void ModePhaserInit();
    void ModePhaser();
    void ModeChorusInit();
    void ModeChorus();
    void ModeDistortionInit();
    void ModeDistortion();

    /**
     * @brief Here the second core processes the audio. It is called by the SecondCoreWorker.
     */
    void SecondCoreProcess(size_t index);

    /**
     * @brief How many samples are processed by the second core
     */
    size_t second_core_processed_samples_ = 0;

    /**
     * @brief Total samples second core should process
     */
    size_t buffer_size_ = 0;

    /**
     * @brief Buffer for the second core to use for clean passthrough (usually the input buffer)
     */
    q15_t *input_buffer_;

    /**
     * @brief Buffer for the second core to process (usually the output buffer)
     */
    q15_t *output_buffer_;

    /**
     * @brief Sets dry wet, will wait signals to be close to each other (avoid crackle)
     * @param dry_wet
     */
    void SetDryWet(q15_t dry_wet);

    /**
     * @brief Waits for the signals to get close to each other and than sets the dry-wet
     */
    void CheckDryWet();

    size_t dry_wet_change_counter_ = 0;
    static constexpr size_t kDryWetChangeCounterMax = 200;
    static constexpr q15_t kLowSignal = 3277; // (0.1)

    // For the mode change you need to press it shorter than 1.5s
    static constexpr size_t kModeShortPressUnder = s2alr(1.5f);

    /**
     * @brief Structure which contains rhythms and other settings loaded from file
     */
    FxWizardFile settings_file_;

    /**
     * @brief Loads the settings file from memory
     * @return true if loading was successful, false otherwise
     */
    bool LoadFile();
};
}
