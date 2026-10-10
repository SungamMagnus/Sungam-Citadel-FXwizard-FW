/*
MIT License

Copyright (c) 2024 Marek Mach (Bastl Instruments)

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

#include <array>
#include "common/config.hpp"
#include "common/utils.hpp"
#include "common/core/Hardware.hpp"
#include "common/dsp/math/Fraction.hpp"
#include "common/dsp/math/math_utils.hpp"
#include "common/dsp/math/qmath.hpp"

namespace kastle2
{

// --- GLOBAL PARAMETERS ---
static constexpr float kFeedbackFilterLeftFreq = 50.f;
static constexpr float kFeedbackFilterRightFreq = 50.f;
static constexpr float kFeedbackFilterLpLeftFreq = 15000.f;
static constexpr float kFeedbackFilterLpRightFreq = 15000.f;


/*
    All maps are defined using MapDef from math_utils.
    MapDef<type, size> contains two arrays: input and output.
*/

// --- GLOBAL ---
static constexpr auto kMapDjFilter = MapDef<int32_t, 4>{
    {pot(0.0f), pot(0.49f), pot(0.51f), pot(1.0f)},
    {q15(-0.8f), q15(0.0f), q15(0.0f), q15(0.9f)}};

// --- DELAY ---
static constexpr auto kMapDelayDry = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.5f), pot(1.0f)},
    {q15(1.0f), q15(1.0f), q15(0.0f)}};

static constexpr auto kMapDelayWet = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.5f), pot(1.0f)},
    {q15(0.0f), q15(0.5f), q15(1.0f)}};

static constexpr auto kMapDelayFeedback = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.3f), pot(1.0f)},
    {q15(0.0f), q15(0.2f), q15(1.0f)}};

static constexpr auto kMapDelayLength = MapDef<int32_t, 5>{
    {pot(0.0f), pot(0.45f), pot(0.65f), pot(0.85f), pot(1.0f)},
    {50799, 12000, 7000, 4000, 100}};

static constexpr auto kMapDelayStereo = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.5f), pot(1.0f)},
    {0, 20, 2000}};

static constexpr auto kMapDelayGlobalDryWet = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.4f), pot(1.0f)},
    {q15(0.0f), q15(1.0f), q15(1.0f)}};

static constexpr size_t kDelayTicksSyncUpperLimit = 50800 * 5;
static constexpr size_t kDelayTicksSyncLowerLimit = 6000;
static constexpr auto kDelaySyncedDividers = std::array<Fraction, 22>{
    Fraction{32, 1},
    Fraction{16, 1},
    Fraction{12, 1},
    Fraction{8, 1},
    Fraction{6, 1},
    Fraction{4, 1},
    Fraction{3, 1},
    Fraction{3, 2},
    Fraction{2, 1},
    Fraction{1, 1},
    Fraction{1, 2},
    Fraction{3, 2},
    Fraction{1, 3},
    Fraction{1, 4},
    Fraction{1, 6},
    Fraction{1, 8},
    Fraction{1, 12},
    Fraction{1, 16},
    Fraction{1, 32},
    Fraction{1, 64},
    Fraction{1, 128},
    Fraction{1, 256}};

// Compute the index of {1, 1} fraction in kDelaySyncedDividers
static constexpr size_t kDelaySyncedDividersCenter = []() constexpr
{
    for (size_t i = 0; i < kDelaySyncedDividers.size(); ++i)
    {
        if (kDelaySyncedDividers[i] == Fraction{1, 1})
        {
            return i;
        }
    }
    // Fallback (should never happen if array is properly defined)
    return size_t{0};
}();

static constexpr auto kMapDelayGlobalFeedbackDelay = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.55f), pot(1.0f)},
    {0, 0, 0}};

static constexpr auto kMapDelayGlobalFeedbackVolume = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.23f), pot(1.0f)},
    {q15(0.0f), q15(0.5f), q15(0.75f)}};

// If larger delay difference than that (in seconds), don't slew delay time and just jump to that time
static constexpr size_t kDelayJumpIfLargerThan = s2sr(0.02f);

// =====================================================================================================
// FX Wizard Plus: maps for the extra effects. Knob roles are the same as in the other modes:
//   Time = TIME, Feedback = FEEDBACK, Amount = AMOUNT, Stereo = STEREO (shift layer).
// For all of them the global dry/wet fades in over the first 15% of Amount, so Amount at zero is a clean bypass.
// =====================================================================================================

// --- REVERB ---
// TIME: decay time (RT60, seconds). FEEDBACK: damping, fully left = bright, fully right = dark.
// AMOUNT: wet level. STEREO: pre-delay.
static constexpr auto kMapReverbGlobalDryWet = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.15f), pot(1.0f)},
    {q15(0.0f), q15(1.0f), q15(1.0f)}};

static constexpr auto kMapReverbDecay = MapDef<float, 5>{
    {pot(0.0f), pot(0.3f), pot(0.6f), pot(0.85f), pot(1.0f)},
    {0.3f, 1.0f, 3.0f, 10.0f, 30.0f}};

static constexpr auto kMapReverbDamping = MapDef<int32_t, 4>{
    {pot(0.0f), pot(0.3f), pot(0.7f), pot(1.0f)},
    {32767, 20000, 8000, 2500}};

static constexpr auto kMapReverbPreDelay = MapDef<int32_t, 2>{
    {pot(0.0f), pot(1.0f)},
    {0, 3300}}; // samples, 75 ms

static constexpr auto kMapReverbWet = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.5f), pot(1.0f)},
    {q15(0.0f), q15(0.5f), q15(1.0f)}};

static constexpr auto kMapReverbDry = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.5f), pot(1.0f)},
    {q15(1.0f), q15(0.85f), q15(0.5f)}};

static constexpr int32_t kNoFeedbackDelay = 300; ///< The modes with their own feedback or decay turn the global feedback off

// --- CHORUS ---
// TIME: LFO rate. FEEDBACK: global feedback (as in the flanger). AMOUNT: depth and wet level. STEREO: left/right LFO phase.
static constexpr auto kMapChorusGlobalDryWet = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.15f), pot(1.0f)},
    {q15(0.0f), q15(1.0f), q15(1.0f)}};

static constexpr auto kMapChorusRate = MapDef<float, 5>{
    {pot(0.0f), pot(0.35f), pot(0.65f), pot(0.85f), pot(1.0f)},
    {0.06f, 0.35f, 1.2f, 3.5f, 9.0f}}; // Hz

static constexpr auto kMapChorusDepth = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.3f), pot(1.0f)},
    {60, 180, 340}}; // samples, max is fxplus::Chorus::kMaxDepth

static constexpr auto kMapChorusWet = MapDef<int32_t, 4>{
    {pot(0.0f), pot(0.15f), pot(0.6f), pot(1.0f)},
    {q15(0.0f), q15(0.4f), q15(0.7f), q15(0.8f)}};

static constexpr auto kMapChorusDry = MapDef<int32_t, 2>{
    {pot(0.0f), pot(1.0f)},
    {q15(1.0f), q15(0.6f)}};

static constexpr auto kMapChorusStereoPhase = MapDef<int32_t, 2>{
    {pot(0.0f), pot(1.0f)},
    {0, 16384}}; // fraction of a cycle in q15, 16384 = half a cycle. Shifted left by 17 to get a 32 bit phase.

static constexpr auto kMapChorusGlobalFeedbackDelay = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.55f), pot(1.0f)},
    {200, 400, 700}};

static constexpr auto kMapChorusGlobalFeedbackVolume = MapDef<int32_t, 4>{
    {pot(0.0f), pot(0.23f), pot(0.5f), pot(1.0f)},
    {q15(0.0f), q15(0.04f), q15(0.05f), q15(0.08f)}};

// --- DISTORTION ---
// TIME: drive. FEEDBACK: tone, fully left = bright, fully right = dark. AMOUNT: dry/wet. STEREO: asymmetry (even harmonics).
static constexpr auto kMapDistortionGlobalDryWet = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.1f), pot(1.0f)},
    {q15(0.0f), q15(1.0f), q15(1.0f)}};

static constexpr auto kMapDistortionDrive = MapDef<int32_t, 4>{
    {pot(0.0f), pot(0.4f), pot(0.75f), pot(1.0f)},
    {256, 1024, 4096, 25600}}; // in 1/256, so 1x, 4x, 16x, 100x

static constexpr auto kMapDistortionTone = MapDef<int32_t, 4>{
    {pot(0.0f), pot(0.3f), pot(0.65f), pot(1.0f)},
    {32767, 22000, 8000, 1800}}; // low-pass coefficient, about: open, 8 kHz, 2 kHz, 400 Hz

static constexpr auto kMapDistortionBias = MapDef<int32_t, 2>{
    {pot(0.0f), pot(1.0f)},
    {0, 32767}};

static constexpr auto kMapDistortionWet = MapDef<int32_t, 2>{
    {pot(0.0f), pot(1.0f)},
    {q15(0.0f), q15(1.0f)}};

static constexpr auto kMapDistortionDry = MapDef<int32_t, 2>{
    {pot(0.0f), pot(1.0f)},
    {q15(1.0f), q15(0.0f)}};

// --- PITCH SHIFTER ---
// TIME: pitch, -12 .. +12 semitones with detents at whole semitones. FEEDBACK: global feedback (cascading shifts).
// AMOUNT: dry/wet. STEREO: detune between left and right.
static constexpr auto kMapPitchShifterGlobalDryWet = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.1f), pot(1.0f)},
    {q15(0.0f), q15(1.0f), q15(1.0f)}};

static constexpr float kPitchShifterRangeSemitones = 12.f; ///< Time knob covers +- this
static constexpr float kPitchShifterDetentWidth = 0.25f;   ///< Within this of a whole semitone the pitch snaps to it

static constexpr auto kMapPitchShifterDetune = MapDef<float, 2>{
    {pot(0.0f), pot(1.0f)},
    {0.f, 40.f}}; // cents between left and right

static constexpr auto kMapPitchShifterWet = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.5f), pot(1.0f)},
    {q15(0.0f), q15(0.7f), q15(1.0f)}};

static constexpr auto kMapPitchShifterDry = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.5f), pot(1.0f)},
    {q15(1.0f), q15(0.7f), q15(0.0f)}};

static constexpr auto kMapPitchShifterGlobalFeedbackDelay = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.55f), pot(1.0f)},
    {300, 636, 1000}};

static constexpr auto kMapPitchShifterGlobalFeedbackVolume = MapDef<int32_t, 4>{
    {pot(0.0f), pot(0.15f), pot(0.5f), pot(1.0f)},
    {q15(0.0f), q15(0.03f), q15(0.06f), q15(0.1f)}};

// --- PHASER ---
// TIME: LFO rate. FEEDBACK: resonance. AMOUNT: dry/wet (full amount is 1:1, which gives the deepest notches).
// STEREO: left/right LFO phase.
static constexpr auto kMapPhaserGlobalDryWet = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.1f), pot(1.0f)},
    {q15(0.0f), q15(1.0f), q15(1.0f)}};

static constexpr auto kMapPhaserRate = MapDef<float, 5>{
    {pot(0.0f), pot(0.3f), pot(0.6f), pot(0.85f), pot(1.0f)},
    {0.03f, 0.15f, 0.6f, 2.5f, 8.0f}}; // Hz

static constexpr auto kMapPhaserFeedback = MapDef<int32_t, 3>{
    {pot(0.0f), pot(0.5f), pot(1.0f)},
    {q15(0.0f), q15(0.37f), q15(0.82f)}};

static constexpr auto kMapPhaserWet = MapDef<int32_t, 2>{
    {pot(0.0f), pot(1.0f)},
    {q15(0.0f), q15(0.5f)}};

static constexpr auto kMapPhaserDry = MapDef<int32_t, 2>{
    {pot(0.0f), pot(1.0f)},
    {q15(1.0f), q15(0.5f)}};

// --- SEQUENCER
static constexpr size_t kFxSequencerLength = 8; ///< See Sequencer.hpp for the max length

}