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

#include <cstring>

#include "AppFxWizardPlus.hpp"
#include "common/core/Kastle2.hpp"
#include "common/core/MultiCore.hpp"
#include "common/core/UserDataFile.hpp"
#include "common/utils.hpp"
#include "FxWizardParameterMaps.hpp"

#include "cc.hpp"

// Needs AdvancedDynamicDelayLine::SetFastSlew to be implemented.
// When that was implemented, it caused crashes in some modes.
#define CHECK_DELAY_JUMP 0

using namespace kastle2;

void AppFxWizardPlus::Init()
{
    inited_ = false;

    // Load custom scales and rhythms (if available)
    if (LoadFile())
    {
        // Update rhythm generator with the loaded rhythms
        Kastle2::base.GetSequencer().SetTriggerGeneratorTable(std::span{
            settings_file_.rhythms, settings_file_.num_rhythms});

        // Set sequencer length
        Kastle2::base.GetSequencer().SetLength(settings_file_.sequencer_length);
    }
    else
    {
        // Default length
        Kastle2::base.GetSequencer().SetLength(kFxSequencerLength);

        // Default rhythms are just the classic ones from Kastle2_params.hpp
        // ...
    }

    // Disable audio chain, since we process the audio ourselves
    Kastle2::base.SetFeatureEnabled(Base::Feature::AUDIO_CHAIN, false);

    // Configure lfo mod destinations
    Kastle2::base.SetLfoModDefaultSelectionEnabled();
    Kastle2::base.SetLfoModSelectionEnabled(false, LfoMod::Destination::MODE_1, LfoMod::Destination::MODE_2, LfoMod::Destination::MODE_3, LfoMod::Destination::MODE_5, LfoMod::Destination::MODE_6);

    // Fake Tempo and LFO LEDs
    Kastle2::base.GetFakeBlinker().SetEnabled(true);
    // If under 30 cycles
    Kastle2::base.GetFakeBlinker().SetTempoThreshold(30);
    Kastle2::base.GetFakeBlinker().SetLfoThreshold(q31(0.05f));

    // Initialize the parameters
    input_left_ = Q15_ZERO;
    input_right_ = Q15_ZERO;
    output_left_ = Q15_ZERO;
    output_right_ = Q15_ZERO;
    cv_step_ = 0;
    cv_mode_ = 0;
    cv_mode_prev_ = 0;
    cv_mode_in_use_ = 0;

    trigger_trigger_ = false;
    clock_trigger_ = false;
    mode_sh_trigger_ = false;
    time_sh_trigger_ = false;

    feedback_delay_left_ = std::make_unique<AdvancedDynamicDelayLine<q15least_t>>(2000);  // 44ms
    feedback_delay_right_ = std::make_unique<AdvancedDynamicDelayLine<q15least_t>>(2000); // 44ms

    feedback_clip_.SetDrive(Q15_MAX);

    feedback_filter_left_.Init(SAMPLE_RATE);
    feedback_filter_left_.SetFrequency(kFeedbackFilterLeftFreq);
    feedback_filter_left_.SetResonance(0.f, Svf::ForceValue::TRUE);
    feedback_filter_left_.SetType(Svf::Type::HIGHPASS);
    feedback_filter_left_.SetDrive(1.f);
    feedback_filter_right_.Init(SAMPLE_RATE);
    feedback_filter_right_.SetFrequency(kFeedbackFilterRightFreq);
    feedback_filter_right_.SetResonance(0.f, Svf::ForceValue::TRUE);
    feedback_filter_right_.SetType(Svf::Type::HIGHPASS);
    feedback_filter_right_.SetDrive(1.f);

    feedback_filter_lp_left_.Init(SAMPLE_RATE);
    feedback_filter_lp_left_.SetFrequency(kFeedbackFilterLpLeftFreq);
    feedback_filter_lp_left_.SetResonance(0.f, Svf::ForceValue::TRUE);
    feedback_filter_lp_left_.SetType(Svf::Type::LOWPASS);
    feedback_filter_lp_left_.SetDrive(1.f);
    feedback_filter_lp_right_.Init(SAMPLE_RATE);
    feedback_filter_lp_right_.SetFrequency(kFeedbackFilterLpRightFreq);
    feedback_filter_lp_right_.SetResonance(0.f, Svf::ForceValue::TRUE);
    feedback_filter_lp_right_.SetType(Svf::Type::LOWPASS);
    feedback_filter_lp_right_.SetDrive(1.f);

    delay_clip_.Init(SAMPLE_RATE);
    feedback_volume_ = Q15_ZERO;
    feedback_delay_ = 0;
    global_dry_wet_ = Q15_ZERO;
    global_dry_wet_queue_ = Q15_ZERO;
    trigger_blink_counter = 0;

    dj_filter_left_.Init(SAMPLE_RATE);
    dj_filter_right_.Init(SAMPLE_RATE);
    dj_filter_left_.SetResonance(0.0f, Svf::ForceValue::TRUE);
    dj_filter_right_.SetResonance(0.0f, Svf::ForceValue::TRUE);

    delay_left_ = std::make_unique<AdvancedDynamicDelayLine<q15least_t>>(50800);
    delay_right_ = std::make_unique<AdvancedDynamicDelayLine<q15least_t>>(50800);

    // The extra effects borrow the memory of the long delay lines (only one mode is active at a time).
    // Each of them checks that the buffers are big enough and stays silent if not.
    reverb_.Init(delay_left_->GetBuffer(), delay_left_->GetMaxLength(), delay_right_->GetBuffer(), delay_right_->GetMaxLength());
    chorus_.Init(delay_left_->GetBuffer(), delay_left_->GetMaxLength(), delay_right_->GetBuffer(), delay_right_->GetMaxLength());
    plate_.Init(delay_left_->GetBuffer(), delay_left_->GetMaxLength(), delay_right_->GetBuffer(), delay_right_->GetMaxLength());
    pitch_shifter_.Init(delay_left_->GetBuffer(), delay_left_->GetMaxLength(), delay_right_->GetBuffer(), delay_right_->GetMaxLength());
    phaser_.Reset();
    delay_pool_dirty_ = false; // the delay lines start out zeroed
    delay_compressor_.Init(SAMPLE_RATE / delay_peak_counter_max_);
    delay_compressor_.SetAttackTime(50.f / 1000.f);
    delay_compressor_.SetReleaseTime(100.f / 1000.f);
    delay_wet_ = Q15_ZERO;
    delay_dry_ = Q15_ZERO;
    delay_feedback_ = Q15_ZERO;
    delay_peak_counter_ = 0;
    delay_peak_ = Q15_ZERO;
    delay_stereo_ = 0;
    delay_length_ = 0;
    delay_ticks_time_ = 0;
    delay_ticks_time_last_ = 0;
    delay_ticks_time_last_prev_ = 0;
    delay_synced_ = false;
    delay_synced_first_measurement_ = false;
    delay_synced_highest_index_ = 0;
    for (auto &time : delay_synced_times_)
    {
        time = 0;
    }
    delay_time_left_prev_ = 0;
    delay_time_right_prev_ = 0;

    lfo_left_.Init(SAMPLE_RATE);
    lfo_right_.Init(SAMPLE_RATE);

    // Normal layer
    pots_[Pot::TIME_MOD] = FancyPot::Create(
        {.pot = Hardware::Pot::POT_1,
         .layer = Hardware::Layer::NORMAL,
         .initial_value = POT_HALF,
         .midi_cc = cc::TIME_MOD,
         .deadzone = true});
    pots_[Pot::FEEDBACK_MOD] = FancyPot::Create(
        {.pot = Hardware::Pot::POT_2,
         .layer = Hardware::Layer::NORMAL,
         .initial_value = POT_HALF,
         .midi_cc = cc::FEEDBACK_MOD,
         .deadzone = true});
    pots_[Pot::AMOUNT] = FancyPot::Create(
        {.pot = Hardware::Pot::POT_4,
         .layer = Hardware::Layer::NORMAL,
         .initial_value = POT_HALF,
         .midi_cc = cc::AMOUNT});
    pots_[Pot::TIME] = FancyPot::Create(
        {.pot = Hardware::Pot::POT_5,
         .layer = Hardware::Layer::NORMAL,
         .initial_value = POT_HALF,
         .midi_cc = cc::TIME,
         .freeze = true});
    pots_[Pot::FEEDBACK] = FancyPot::Create(
        {.pot = Hardware::Pot::POT_6,
         .layer = Hardware::Layer::NORMAL,
         .initial_value = POT_MIN,
         .midi_cc = cc::FEEDBACK});

    // Shift layer
    pots_[Pot::STEREO] = FancyPot::Create(
        {.pot = Hardware::Pot::POT_2,
         .layer = Hardware::Layer::SHIFT,
         .initial_value = POT_MIN,
         .midi_cc = cc::STEREO,
         .freeze = true});
    pots_[Pot::AMOUNT_MOD] = FancyPot::Create(
        {.pot = Hardware::Pot::POT_4,
         .layer = Hardware::Layer::SHIFT,
         .initial_value = POT_MAX,
         .midi_cc = cc::AMOUNT_MOD});
    pots_[Pot::FILTER] = FancyPot::Create(
        {.pot = Hardware::Pot::POT_6,
         .layer = Hardware::Layer::SHIFT,
         .initial_value = POT_HALF,
         .midi_cc = cc::FILTER,
         .deadzone = true});

    // Init pots
    for (auto &pot : pots_)
    {
        pot->Init(AUDIO_LOOP_RATE);
    }

    // This disables the next change when the pots are moved
    // or when the current layer time is over the specified number of ticks
    mode_selector_.DisableNextChangeWhen(pots_, kModeShortPressUnder);

    // Set up the second core
    second_core_processed_samples_ = 0;

    // Set up MODE
    mode_selector_.Init();
    mode_ = static_cast<Mode>(mode_selector_.GetMode());
    ModeInit();

    inited_ = true;
}

void AppFxWizardPlus::DeInit()
{
    inited_ = false;
}

FASTCODE void AppFxWizardPlus::AudioLoop(q15_t *input, q15_t *output, size_t size)
{
    if (!inited_)
    {
        return;
    }

    // Autofreeze pots
    for (auto &pot : pots_)
    {
        pot->Process();
    }
    mode_selector_.Process();

    // Process clock triggers etc.
    if (Kastle2::base.GetClock().IsNowTrigger())
    {
        clock_trigger_ = true;
    }

    // Process triggers
    if (trigger_detect_.Process(Kastle2::hw.GetTriggerIn()))
    {
        trigger_trigger_ = true;
    }

    size_t elapsed_ticks = Kastle2::base.GetClock().GetCurrentTicks();
    // Approx 5ms after clock - time SH
    if (elapsed_ticks == 6)
    {
        time_sh_trigger_ = true;
    }
    // Approx 20ms after clock - mode SH
    else if (elapsed_ticks == 23)
    {
        mode_sh_trigger_ = true;
    }

    // See the comment at reverb_ready_
    reverb_ready_ = reverb_ready_ && mode_ == Mode::REVERB;
    chorus_ready_ = chorus_ready_ && mode_ == Mode::CHORUS;
    plate_ready_ = plate_ready_ && mode_ == Mode::PLATE;
    pitch_shifter_ready_ = pitch_shifter_ready_ && mode_ == Mode::PITCH_SHIFTER;

    // Set data for the second core
    input_buffer_ = input;
    output_buffer_ = output;
    buffer_size_ = size;

    // Tell the second core we're starting a new block
    MultiCore::SendMessage(MultiCore::MessageType::BEGIN);

    // Do the processing for each sample
    for (size_t i = 0; i < size; i++)
    {
        input_left_ = input[2 * i];
        input_right_ = input[2 * i + 1];
        // Because of performance requirements, some things need to be calculated only a few times per buffer
        sample_being_processed_ = i;
        if (mode_ != Mode::DELAY)
        {
            input_left_ = q15_add(
                q15_mult(q15_div(feedback_delay_left_->Read(), 27853), feedback_volume_),
                q15_mult(input_left_, (Q15_MAX - feedback_volume_) / 2 + Q15_HALF));
            input_right_ = q15_add(
                q15_mult(q15_div(feedback_delay_right_->Read(), 27853), feedback_volume_),
                q15_mult(input_right_, (Q15_MAX - feedback_volume_) / 2 + Q15_HALF));
        }

        // Output resetting just to be sure there is no junk there
        output_left_ = 0;
        output_right_ = 0;

        WorkLongDelay();

        switch (mode_)
        {
        case Mode::DELAY:
            ModeDelay();
            break;

        case Mode::CHORUS:
            ModeChorus();
            break;

        case Mode::PHASER:
            ModePhaser();
            break;

        case Mode::REVERB:
            ModeReverb();
            break;

        case Mode::PLATE:
            ModePlate();
            break;

        case Mode::DISTORTION:
            ModeDistortion();
            break;

        case Mode::PITCH_SHIFTER:
            ModePitchShifter();
            break;
        }

        // Write the output.
        // The second core will continue with the output.
        // See above that output_buffer_ = output
        output[2 * i] = output_left_;
        output[2 * i + 1] = output_right_;

        // Waiting for dry/wet change?
        CheckDryWet();

        // Tell the second core it can process this sample
        MultiCore::SendMessage(MultiCore::MessageType::SAMPLE_REQUEST, i);
    }

    // Process counters and timers
    if (trigger_blink_counter > 0)
    {
        trigger_blink_counter--;
    }

    // Wait for samples to finish processing
    MultiCore::WaitForMessage(MultiCore::MessageType::DONE);
}

FASTCODE void AppFxWizardPlus::SecondCoreProcess(size_t index)
{
    // Read samples from the buffer
    q15_t left = output_buffer_[2 * index];
    q15_t right = output_buffer_[2 * index + 1];

    // Apply DJ filter
    left = dj_filter_left_.Process(left);
    right = dj_filter_right_.Process(right);

    // Compensate filter volume drop
    left = q15_div(left, Q15_HALF);
    right = q15_div(right, Q15_HALF);

    // Write back feedback (after DJ filter)
    if (mode_ != Mode::DELAY)
    {
        // feedback_delay_left_->Write(feedback_filter_lp_left_.Process(feedback_filter_left_.Process(feedback_clip_.Process(left))));
        // feedback_delay_right_->Write(feedback_filter_lp_right_.Process(feedback_filter_right_.Process(feedback_clip_.Process(right))));

        feedback_delay_left_->Write(feedback_filter_left_.Process(feedback_filter_lp_left_.Process(feedback_clip_.Process(left))));
        feedback_delay_right_->Write(feedback_filter_right_.Process(feedback_filter_lp_right_.Process(feedback_clip_.Process(right))));

        // feedback_delay_left_->Write(feedback_clip_.Process(feedback_filter_left_.Process(feedback_filter_lp_left_.Process(left))));
        // feedback_delay_right_->Write(feedback_clip_.Process(feedback_filter_right_.Process(feedback_filter_lp_right_.Process(right))));
    }

    // Do DryWet and write the processed samples back to the output buffer
    output_buffer_[2 * index] = q15_add(q15_mult(input_buffer_[2 * index], Q15_MAX - global_dry_wet_), q15_mult(left, global_dry_wet_));
    output_buffer_[2 * index + 1] = q15_add(q15_mult(input_buffer_[2 * index + 1], Q15_MAX - global_dry_wet_), q15_mult(right, global_dry_wet_));
}

FASTCODE void AppFxWizardPlus::SecondCoreWorker()
{
    while (inited_)
    {
        if (MultiCore::HasMessage())
        {
            // Monitoring second core performance
            Kastle2::hw.SetDebugPin(1, 1);

            MultiCore::Message m = MultiCore::GetMessage();
            switch (m.type)
            {
            case MultiCore::MessageType::BEGIN:
                second_core_processed_samples_ = 0;
                break;
            case MultiCore::MessageType::SAMPLE_REQUEST:
                SecondCoreProcess(m.data);
                second_core_processed_samples_++;
                if (second_core_processed_samples_ == buffer_size_)
                {
                    MultiCore::SendMessage(MultiCore::MessageType::DONE);
                }
                break;
            }

            Kastle2::hw.SetDebugPin(1, 0);
        }
    }
}

void AppFxWizardPlus::SetDryWet(q15_t dry_wet)
{
#if NEAR_ZERO_DRYWET
    // Small change, ignore
    if (q15_abs(global_dry_wet_queue_ - dry_wet) < 8)
    {
        return;
    }
    global_dry_wet_queue_ = dry_wet;
    // if after X samples the signal not in range, change the dry/wet
    dry_wet_change_counter_ = kDryWetChangeCounterMax;
#else
    global_dry_wet_ = dry_wet;
#endif
}

void AppFxWizardPlus::CheckDryWet()
{
#if NEAR_ZERO_DRYWET
    if (global_dry_wet_ != global_dry_wet_queue_ && (((q15_abs(output_left_ - input_left_) < kLowSignal) &&
                                                      (q15_abs(output_right_ - input_right_) < kLowSignal)) ||
                                                     dry_wet_change_counter_ == 0))
    {
        global_dry_wet_ = global_dry_wet_queue_;
    }

    if (dry_wet_change_counter_ > 0)
    {
        dry_wet_change_counter_--;
    }
#endif
}

void AppFxWizardPlus::UiLoop()
{
    for (auto &pot : pots_)
    {
        pot->ReadValue();
    }
    mode_selector_.ReadValue();

    // Enable zero cross update if volume not low
    Kastle2::codec.SetZeroCrossUpdate(Kastle2::base.GetInputEnvelopeFollower().GetEnvelope() > q15(0.05f));

    // Get clock and sh triggers
    // (none of the remaining modes needs the clock trigger on its own)
    clock_trigger_ = false;

    bool time_sh_triggered = time_sh_trigger_;
    time_sh_trigger_ = false;

    bool mode_sh_triggered = mode_sh_trigger_;
    mode_sh_trigger_ = false;

    // Get trigger state
    bool trigger_triggered = trigger_trigger_;
    trigger_trigger_ = false;

    // Get the mode CV value
    cv_mode_ = Kastle2::hw.GetAnalogValue(CV_MODE);

    // Check for jump
    bool cv_mode_jumped = (diff(cv_mode_, cv_mode_prev_) > kModeCvJumped);

    // Store prev value
    cv_mode_prev_ = cv_mode_;

    // Force the mode selector to re-read the ADC
    // This is done so the mode changes in sync with the tempo
    if ((mode_sh_triggered && (diff(cv_mode_, cv_mode_in_use_) > kModeCvChanged)) || cv_mode_jumped)
    {
        mode_selector_.TriggerAdcRead();
        cv_mode_in_use_ = cv_mode_;
    }

    // S&H stepped CV on clock trigger
    if (time_sh_triggered)
    {
        cv_step_ = Kastle2::hw.GetAnalogValue(CV_STEP);
    }

    int32_t feedback = pots_[Pot::FEEDBACK]->GetValue();
    feedback += apply_pot_mod_attenuvert(Kastle2::hw.GetAnalogValue(CV_FEEDBACK), pots_[Pot::FEEDBACK_MOD]->GetValue());
    feedback = constrain(feedback, POT_MIN, POT_MAX);

    feedback_delay_left_->SetDelay(feedback_delay_);
    feedback_delay_right_->SetDelay(feedback_delay_);

    int32_t filter = Kastle2::base.GetLfoMod().AdjustedPotValue(pots_[Pot::FILTER].get());
    int32_t filter_crossfade = curve_map(filter, kMapDjFilter);
    dj_filter_left_.SetCrossfade(filter_crossfade);
    dj_filter_right_.SetCrossfade(filter_crossfade);

    int32_t dry_wet = pots_[Pot::AMOUNT]->GetValue();
    dry_wet += apply_pot_mod_attenuvert(Kastle2::hw.GetAnalogValue(CV_DRYWET),
                                        Kastle2::base.GetLfoMod().AdjustedPotValue(pots_[Pot::AMOUNT_MOD].get()));
    dry_wet = constrain(dry_wet, POT_MIN, POT_MAX);

    int32_t time = pots_[Pot::TIME]->GetValue();
    int32_t stereo = Kastle2::base.GetLfoMod().AdjustedPotValue(pots_[Pot::STEREO].get());
    int32_t time_mod = pots_[Pot::TIME_MOD]->GetValue();
    time += apply_pot_mod_attenuvert(Kastle2::hw.GetAnalogValue(CV_FREE), time_mod);
    time += apply_pot_mod_attenuvert(cv_step_, time_mod);
    time = constrain(time, POT_MIN, POT_MAX);

    size_t mapped_time;
    size_t delay_time_left;
    size_t delay_time_right;

    switch (mode_)
    {
    case Mode::DELAY:
        SetDryWet(curve_map(dry_wet, kMapDelayGlobalDryWet));
        feedback_delay_ = curve_map(feedback, kMapDelayGlobalFeedbackDelay);
        feedback_volume_ = curve_map(feedback, kMapDelayGlobalFeedbackVolume);
        delay_dry_ = curve_map(dry_wet, kMapDelayDry);
        delay_wet_ = curve_map(dry_wet, kMapDelayWet);
        delay_feedback_ = curve_map(feedback, kMapDelayFeedback);

        mapped_time = curve_map(time, kMapDelayLength);

        if (delay_ticks_time_ > kDelayTicksSyncUpperLimit || ((delay_ticks_time_ > (delay_ticks_time_last_ * 2 + delay_ticks_time_last_ / 2)) && !delay_synced_first_measurement_))
        {
            delay_synced_ = false;
        }
        else
        {
            if (delay_ticks_time_last_ != delay_ticks_time_last_prev_)
            {
                size_t i = 0;
                delay_synced_ = true;
                delay_synced_times_[kDelaySyncedDividersCenter] = delay_ticks_time_last_;
                delay_synced_highest_index_ = kDelaySyncedDividersCenter + 1;
                // fill the array in the multipliers zone
                for (i = kDelaySyncedDividersCenter; delay_synced_times_[i] < delay_left_->GetMaxLength() && i > 0; i--)
                {
                    delay_synced_times_[i - 1] = delay_synced_times_[i];
                    delay_synced_times_[i - 1] *= kDelaySyncedDividers[i - 1].n;
                    delay_synced_times_[i - 1] /= kDelaySyncedDividers[i - 1].d;
                    // set the highest index to check for match to avoid reading unset values
                    delay_synced_highest_index_--;
                }
                // fill the array in the dividers zone
                for (i = kDelaySyncedDividersCenter; i < kDelaySyncedDividers.size() - 1; i++)
                {
                    delay_synced_times_[i + 1] = delay_synced_times_[i];
                    delay_synced_times_[i + 1] *= kDelaySyncedDividers[i + 1].n;
                    delay_synced_times_[i + 1] /= kDelaySyncedDividers[i + 1].d;
                }
            }
            delay_ticks_time_last_prev_ = delay_ticks_time_last_;
        }
        if (delay_synced_)
        {
            // find a value that is lower than time value
            size_t i = delay_synced_highest_index_;
            while (delay_synced_times_[i] > mapped_time && i < kDelaySyncedDividers.size())
            {
                i++;
            }
            // select it
            delay_length_ = delay_synced_times_[i];
        }
        else
        {
            delay_length_ = mapped_time;
        }

        delay_stereo_ = curve_map(stereo, kMapDelayStereo);

        // Calculate delay times
        delay_time_left = constrain(static_cast<int32_t>(delay_length_) - static_cast<int32_t>(delay_stereo_), 0, delay_left_->GetMaxLength());
        delay_time_right = constrain(delay_length_ + delay_stereo_, 0, delay_right_->GetMaxLength());

#if CHECK_DELAY_JUMP
        // Check for how much the value changed
        if (diff(delay_time_left, delay_time_left_prev_) > kDelayJumpIfLargerThan)
        {
            // Big jump? Set slewing to be fast
            delay_left_->SetFastSlew(true);
        }

        // Check for how much the value changed
        if (diff(delay_time_right, delay_time_right_prev_) > kDelayJumpIfLargerThan)
        {
            // Big jump? Set slewing to be fast
            delay_right_->SetFastSlew(true);
        }

        delay_time_left_prev_ = delay_time_left;
        delay_time_right_prev_ = delay_time_right;
#endif
        delay_left_->SetDelay(delay_time_left);
        delay_right_->SetDelay(delay_time_right);

        break;

    case Mode::REVERB:
    {
        SetDryWet(curve_map(dry_wet, kMapReverbGlobalDryWet));
        // the reverb has its own decay, so the global feedback is off
        feedback_delay_ = kNoFeedbackDelay;
        feedback_volume_ = Q15_ZERO;
        reverb_dry_ = curve_map(dry_wet, kMapReverbDry);
        reverb_wet_ = curve_map(dry_wet, kMapReverbWet);
        reverb_.SetPreDelay(curve_map(stereo, kMapReverbPreDelay));

        // Changing the decay is expensive (floating point), ignore small changes (knob / ADC jitter)
        float decay = curve_map(time, kMapReverbDecay);
        if (decay > reverb_decay_prev_ * 1.02f || decay < reverb_decay_prev_ * 0.98f)
        {
            reverb_decay_prev_ = decay;
            reverb_.SetDecay(decay);
        }
        int32_t damping = curve_map(feedback, kMapReverbDamping);
        if (damping > reverb_damping_prev_ + 150 || damping < reverb_damping_prev_ - 150)
        {
            reverb_damping_prev_ = damping;
            reverb_.SetDamping(damping);
        }
        break;
    }

    case Mode::CHORUS:
        SetDryWet(curve_map(dry_wet, kMapChorusGlobalDryWet));
        feedback_delay_ = curve_map(feedback, kMapChorusGlobalFeedbackDelay);
        feedback_volume_ = curve_map(feedback, kMapChorusGlobalFeedbackVolume);
        chorus_dry_ = curve_map(dry_wet, kMapChorusDry);
        chorus_wet_ = curve_map(dry_wet, kMapChorusWet);
        chorus_.SetRate(curve_map(time, kMapChorusRate, MapClamp::TRUE));
        chorus_.SetDepth(curve_map(dry_wet, kMapChorusDepth, MapClamp::TRUE));
        chorus_.SetStereoPhase(static_cast<uint32_t>(curve_map(stereo, kMapChorusStereoPhase, MapClamp::TRUE)) << 17);
        break;

    case Mode::PHASER:
        SetDryWet(curve_map(dry_wet, kMapPhaserGlobalDryWet));
        // the phaser has its own resonance, so the global feedback is off
        feedback_delay_ = kNoFeedbackDelay;
        feedback_volume_ = Q15_ZERO;
        phaser_dry_ = curve_map(dry_wet, kMapPhaserDry);
        phaser_wet_ = curve_map(dry_wet, kMapPhaserWet);
        phaser_.SetRate(curve_map(time, kMapPhaserRate, MapClamp::TRUE));
        phaser_.SetFeedback(curve_map(feedback, kMapPhaserFeedback, MapClamp::TRUE));
        phaser_.SetStereoPhase(static_cast<uint32_t>(curve_map(stereo, kMapChorusStereoPhase, MapClamp::TRUE)) << 17);
        break;

    case Mode::PLATE:
    {
        SetDryWet(curve_map(dry_wet, kMapReverbGlobalDryWet));
        feedback_delay_ = kNoFeedbackDelay;
        feedback_volume_ = Q15_ZERO;
        plate_dry_ = curve_map(dry_wet, kMapReverbDry);
        plate_wet_ = curve_map(dry_wet, kMapReverbWet);
        plate_.SetPreDelay(curve_map(stereo, kMapReverbPreDelay));

        // same as the reverb: changing decay and damping is expensive, ignore small changes
        float decay = curve_map(time, kMapReverbDecay);
        if (decay > plate_decay_prev_ * 1.02f || decay < plate_decay_prev_ * 0.98f)
        {
            plate_decay_prev_ = decay;
            plate_.SetDecay(decay);
        }
        int32_t damping = curve_map(feedback, kMapReverbDamping);
        if (damping > plate_damping_prev_ + 150 || damping < plate_damping_prev_ - 150)
        {
            plate_damping_prev_ = damping;
            plate_.SetDamping(damping);
        }
        break;
    }

    case Mode::PITCH_SHIFTER:
    {
        SetDryWet(curve_map(dry_wet, kMapPitchShifterGlobalDryWet));
        feedback_delay_ = curve_map(feedback, kMapPitchShifterGlobalFeedbackDelay);
        feedback_volume_ = curve_map(feedback, kMapPitchShifterGlobalFeedbackVolume);
        pitch_shifter_dry_ = curve_map(dry_wet, kMapPitchShifterDry);
        pitch_shifter_wet_ = curve_map(dry_wet, kMapPitchShifterWet);

        // Time knob: +- kPitchShifterRangeSemitones, snapping to whole semitones near them
        float semitones = (static_cast<float>(time) / static_cast<float>(POT_MAX) * 2.f - 1.f) * kPitchShifterRangeSemitones;
        float nearest = static_cast<float>(static_cast<int32_t>(semitones + (semitones >= 0.f ? 0.5f : -0.5f)));
        if (semitones - nearest < kPitchShifterDetentWidth && nearest - semitones < kPitchShifterDetentWidth)
        {
            semitones = nearest;
        }
        float detune = curve_map(stereo, kMapPitchShifterDetune, MapClamp::TRUE);

        // Changing the pitch is expensive (floating point exp2), only do it when the knobs really moved:
        // 1/16 semitone and 2 cent resolution
        int32_t semitones_key = static_cast<int32_t>(semitones * 16.f + (semitones >= 0.f ? 0.5f : -0.5f));
        int32_t detune_key = static_cast<int32_t>(detune * 0.5f);
        if (semitones_key != pitch_shifter_semitones_key_ || detune_key != pitch_shifter_detune_key_)
        {
            pitch_shifter_semitones_key_ = semitones_key;
            pitch_shifter_detune_key_ = detune_key;
            pitch_shifter_.SetShift(semitones, detune);
        }
        break;
    }

    case Mode::DISTORTION:
    {
        SetDryWet(curve_map(dry_wet, kMapDistortionGlobalDryWet));
        // feedback into a distortion is mostly noise, and the feedback knob is the tone control here
        feedback_delay_ = kNoFeedbackDelay;
        feedback_volume_ = Q15_ZERO;
        distortion_dry_ = curve_map(dry_wet, kMapDistortionDry);
        distortion_wet_ = curve_map(dry_wet, kMapDistortionWet);

        int32_t drive = curve_map(time, kMapDistortionDrive, MapClamp::TRUE);
        int32_t tone = curve_map(feedback, kMapDistortionTone, MapClamp::TRUE);
        int32_t bias = curve_map(stereo, kMapDistortionBias, MapClamp::TRUE);
        distortion_left_.SetDrive(drive);
        distortion_right_.SetDrive(drive);
        distortion_left_.SetTone(tone);
        distortion_right_.SetTone(tone);
        distortion_left_.SetBias(bias);
        distortion_right_.SetBias(bias);
        break;
    }
    }

    // Mode is the selected + offset (by CV)
    mode_ = static_cast<Mode>(mode_selector_.GetMode());

    if (mode_ != mode_prev_)
    {
        mode_prev_ = mode_;
        ModeInit();
    }

    // do Mode triggers
    if (trigger_triggered)
    {
        ModeTrigger();
    }

    // NORMAL LAYER UI
    if (Kastle2::hw.GetLayer() == Hardware::Layer::NORMAL || Kastle2::hw.GetLayer() == Hardware::Layer::MODE)
    {
        uint32_t color = kLedColors[mode_];

        // dip the LEDs with a detected trigger
        if (trigger_blink_counter > 0)
        {
            color = WS2812::ApplyBrightness(color, 128);
        }

        Kastle2::hw.SetLed(Hardware::Led::LED_1, color);
        Kastle2::hw.SetLed(Hardware::Led::LED_2, color);
    }
}

void AppFxWizardPlus::ModeInit()
{
    mode_selector_.SendMidi();

    // The reverbs, chorus and pitch shifter use the memory of the long delay lines.
    // When switching to anything else, wipe it so the delay doesn't play back what's left of them.
    if (delay_pool_dirty_ && !UsesDelayPool(mode_))
    {
        std::memset(delay_left_->GetBuffer(), 0, delay_left_->GetMaxLength() * sizeof(q15least_t));
        std::memset(delay_right_->GetBuffer(), 0, delay_right_->GetMaxLength() * sizeof(q15least_t));
        delay_pool_dirty_ = false;
    }

    switch (mode_)
    {
    case Mode::DELAY:
        ModeDelayInit();
        break;
    case Mode::REVERB:
        ModeReverbInit();
        break;
    case Mode::CHORUS:
        ModeChorusInit();
        break;
    case Mode::PHASER:
        ModePhaserInit();
        break;
    case Mode::PLATE:
        ModePlateInit();
        break;
    case Mode::PITCH_SHIFTER:
        ModePitchShifterInit();
        break;
    case Mode::DISTORTION:
        ModeDistortionInit();
        break;
    }

    // reset oversampling, direction and length of the long delay
    delay_left_->SetOversampling(1 << 16);
    delay_right_->SetOversampling(1 << 16);
    delay_left_->SetReverse(false);
    delay_right_->SetReverse(false);
    delay_left_->SetLengthBothToMax();
    delay_right_->SetLengthBothToMax();
}

void AppFxWizardPlus::ModeTrigger()
{
    trigger_blink_counter = 40;

    switch (mode_)
    {
    case Mode::DELAY:
        if (delay_ticks_time_ >= kDelayTicksSyncLowerLimit)
        {
            if (delay_synced_ || delay_synced_first_measurement_)
            {
                delay_ticks_time_last_ = delay_ticks_time_;
                delay_synced_first_measurement_ = false;
            }
            else
            {
                delay_synced_first_measurement_ = true;
            }
            delay_ticks_time_ = 0;
        }
        break;

    case Mode::CHORUS:
        chorus_.ResetPhase();
        break;

    default:
        // nothing to trigger in the other modes, the LEDs still blink
        break;
    }
}

void AppFxWizardPlus::WorkLongDelay()
{
    // the delay has to be fed with newest data when not directly being used

    switch (mode_)
    {
    case Mode::DELAY:
    // These use the memory of the long delay for their own purposes, it must not be written to
    case Mode::CHORUS:
    case Mode::REVERB:
    case Mode::PLATE:
    case Mode::PITCH_SHIFTER:
        break;

    default:
        delay_left_->Write(input_left_);
        delay_right_->Write(input_right_);
    }
}

void AppFxWizardPlus::ModeDelayInit()
{
    delay_clip_.SetDrive(120);
    delay_peak_counter_ = 0;
}

void AppFxWizardPlus::ModeDelay()
{
    q15_t tmp_left, tmp_right;
    if (delay_ticks_time_ < SIZE_MAX)
    {
        delay_ticks_time_++;
    }

    q15_t delay_reading_l = delay_left_->Read();
    q15_t delay_reading_r = delay_right_->Read();

    tmp_left = q15_add((input_left_ / 2), (delay_clip_.Process(q15_mult(delay_reading_l, delay_feedback_)) / 2));
    tmp_right = q15_add((input_right_ / 2), (delay_clip_.Process(q15_mult(delay_reading_r, delay_feedback_)) / 2));

    if (q15_abs(tmp_left) > delay_peak_)
    {
        delay_peak_ = q15_abs(tmp_left);
    }
    if (q15_abs(tmp_right) > delay_peak_)
    {
        delay_peak_ = q15_abs(tmp_right);
    }
    delay_peak_counter_++;

    if (delay_peak_counter_ >= delay_peak_counter_max_)
    {
        delay_compressor_.Track(delay_peak_);
        delay_peak_ = Q15_ZERO;
        delay_peak_counter_ = 0;
    }

    q15_t compress_amount_ = Q15_MAX - delay_compressor_.GetEnvelope();

    tmp_left = q15_mult(tmp_left, compress_amount_);
    tmp_right = q15_mult(tmp_right, compress_amount_);

    delay_left_->Write(q15_div(tmp_left, Q15_HALF));
    delay_right_->Write(q15_div(tmp_right, Q15_HALF));

    output_left_ = q15_add(q15_mult(input_left_, delay_dry_), q15_mult(delay_reading_l, delay_wet_));
    output_right_ = q15_add(q15_mult(input_right_, delay_dry_), q15_mult(delay_reading_r, delay_wet_));

    // might need a compressor at the end here too, I'm not sure
}

void AppFxWizardPlus::ModeReverbInit()
{
    delay_pool_dirty_ = true;
    reverb_.Clear();
    // make UiLoop send the parameters again
    reverb_decay_prev_ = -1.f;
    reverb_damping_prev_ = -100000;
    reverb_ready_ = true;
}

void AppFxWizardPlus::ModeReverb()
{
    if (!reverb_ready_)
    {
        output_left_ = input_left_;
        output_right_ = input_right_;
        return;
    }

    int32_t wet_left, wet_right;
    reverb_.Process(input_left_, input_right_, wet_left, wet_right);

    output_left_ = q15_add(q15_mult(input_left_, reverb_dry_), q15_mult(wet_left, reverb_wet_));
    output_right_ = q15_add(q15_mult(input_right_, reverb_dry_), q15_mult(wet_right, reverb_wet_));
}

void AppFxWizardPlus::ModePhaserInit()
{
    phaser_.Reset();
}

void AppFxWizardPlus::ModePhaser()
{
    int32_t wet_left, wet_right;
    phaser_.Process(input_left_, input_right_, wet_left, wet_right);

    output_left_ = q15_add(q15_mult(input_left_, phaser_dry_), q15_mult(wet_left, phaser_wet_));
    output_right_ = q15_add(q15_mult(input_right_, phaser_dry_), q15_mult(wet_right, phaser_wet_));
}

void AppFxWizardPlus::ModePlateInit()
{
    delay_pool_dirty_ = true;
    plate_.Clear();
    plate_decay_prev_ = -1.f;
    plate_damping_prev_ = -100000;
    plate_ready_ = true;
}

void AppFxWizardPlus::ModePlate()
{
    if (!plate_ready_)
    {
        output_left_ = input_left_;
        output_right_ = input_right_;
        return;
    }

    int32_t wet_left, wet_right;
    plate_.Process(input_left_, input_right_, wet_left, wet_right);

    output_left_ = q15_add(q15_mult(input_left_, plate_dry_), q15_mult(wet_left, plate_wet_));
    output_right_ = q15_add(q15_mult(input_right_, plate_dry_), q15_mult(wet_right, plate_wet_));
}

void AppFxWizardPlus::ModePitchShifterInit()
{
    delay_pool_dirty_ = true;
    pitch_shifter_.Clear();
    // make UiLoop send the pitch again
    pitch_shifter_semitones_key_ = INT32_MIN;
    pitch_shifter_detune_key_ = INT32_MIN;
    pitch_shifter_ready_ = true;
}

void AppFxWizardPlus::ModePitchShifter()
{
    if (!pitch_shifter_ready_)
    {
        output_left_ = input_left_;
        output_right_ = input_right_;
        return;
    }

    int32_t wet_left, wet_right;
    pitch_shifter_.Process(input_left_, input_right_, wet_left, wet_right);

    output_left_ = q15_add(q15_mult(input_left_, pitch_shifter_dry_), q15_mult(wet_left, pitch_shifter_wet_));
    output_right_ = q15_add(q15_mult(input_right_, pitch_shifter_dry_), q15_mult(wet_right, pitch_shifter_wet_));
}

void AppFxWizardPlus::ModeChorusInit()
{
    delay_pool_dirty_ = true;
    chorus_.Clear();
    chorus_ready_ = true;
}

void AppFxWizardPlus::ModeChorus()
{
    if (!chorus_ready_)
    {
        output_left_ = input_left_;
        output_right_ = input_right_;
        return;
    }

    int32_t wet_left, wet_right;
    chorus_.Process(input_left_, input_right_, wet_left, wet_right);

    output_left_ = q15_add(q15_mult(input_left_, chorus_dry_), q15_mult(wet_left, chorus_wet_));
    output_right_ = q15_add(q15_mult(input_right_, chorus_dry_), q15_mult(wet_right, chorus_wet_));
}

void AppFxWizardPlus::ModeDistortionInit()
{
    distortion_left_.Reset();
    distortion_right_.Reset();
}

void AppFxWizardPlus::ModeDistortion()
{
    q15_t wet_left = distortion_left_.Process(input_left_);
    q15_t wet_right = distortion_right_.Process(input_right_);

    output_left_ = q15_add(q15_mult(input_left_, distortion_dry_), q15_mult(wet_left, distortion_wet_));
    output_right_ = q15_add(q15_mult(input_right_, distortion_dry_), q15_mult(wet_right, distortion_wet_));
}

void AppFxWizardPlus::MemoryInitialization()
{
    Kastle2::memory.Write8(kMemMode, 0);
}

void AppFxWizardPlus::MidiCallback(midi::Message *msg)
{
    for (auto &pot : pots_)
    {
        pot->MidiCallback(msg);
    }
    mode_selector_.MidiCallback(msg);
    if (msg->IsNoteOn())
    {
        uint8_t note = msg->GetData1();
        if (note >= kMidiMinNote)
        {
            ModeTrigger();
        }
    }
}

bool AppFxWizardPlus::LoadFile()
{
    UserDataFile file_reader;

    // Validate file header
    if (!file_reader.Validate("k2fx"))
    {
        return false;
    }

    // Read the header data
    file_reader.Read(&settings_file_, 10);

    // Validate counts (it's also checked by the file reader, but just in case)
    if (!between(settings_file_.num_rhythms, 1, UserDataFile::kMaxRhythms))
    {
        return false;
    }

    // Validate sequencer length
    if (!between(settings_file_.sequencer_length, Sequencer::kMinLength, Sequencer::kMaxLength))
    {
        return false;
    }

    // Jump to the rhythms data
    file_reader.JumpTo(20);

    // Load rhythms
    settings_file_.rhythms = file_reader.LoadRhythms(settings_file_.num_rhythms);
    if (!settings_file_.rhythms)
    {
        return false;
    }

    return true;
}