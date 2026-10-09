# Kastle 2 FX Wizard Plus

[FX Wizard](../FxWizard/README.md) with four additional effects. The nine original effects are unchanged and keep
their positions; the new ones are appended after them.

| # | Mode | LED | Time | Feedback | Amount | Stereo (shift) |
|---|------|-----|------|----------|--------|----------------|
| 1-9 | Delay, Flanger, Freezer, Panner, Crusher, Slicer, Pitcher, Replayer, Shifter | | as in FX Wizard | | | |
| 10 | **Reverb** | purple | decay time, 0.3 s - 30 s | damping: left = bright, right = dark | wet level | pre-delay, 0 - 75 ms |
| 11 | **Chorus** | aquamarine | LFO rate, 0.06 - 9 Hz | feedback (as in the flanger) | depth and wet level | left/right LFO phase (width) |
| 12 | **Distortion** | red | drive, 1x - 100x | tone: left = bright, right = dark | dry / wet | asymmetry (adds even harmonics) |
| 13 | **Pitch shifter** | indigo | pitch, -12 .. +12 semitones, detents at whole semitones | feedback (cascading shifts) | dry / wet | left/right detune, up to 40 cents |

For all four, Amount at zero is a clean bypass. The filter knob (DJ filter) and the clock / trigger behaviour work as in
the other modes. A trigger restarts the chorus LFO; the other new modes ignore it (the LEDs still blink).

Notes on the sound:
- The reverb is an 8 comb / 4 allpass design (Freeverb topology) with the decay time set per comb, so the tail has the
  same decay time at every setting. The dark end of the damping range is much quieter than the bright end.
- The pitch shifter is a two-tap crossfaded delay line. It has the characteristic slow flutter of that design and adds
  up to 93 ms of latency to the wet signal. At exactly 0 semitones with no detune it is not a clean pass-through (the taps
  stand still, giving a fixed delay), so use it with the pitch away from zero, or with some detune.
- The distortion is a tanh soft clipper with level compensation (no oversampling, so very high drive on bright material
  will alias; the tone knob helps).

## Building

```
cd code/build
make fx-wizard-plus
```

The UF2 ends up in `code/build/output`. It uses app ID `0x7F` (community range), so it keeps its own settings in the
EEPROM and never touches the settings of the stock FX Wizard (`0xB1`). Calibration is stored separately and is not
affected. If you distribute this, register an ID in [APP_LIST.md](../../../../APP_LIST.md) first.

## How it fits in the RAM

The RP2040 has no spare RAM: the stock FX Wizard already uses about 211 kB of the 240 kB heap for its delay lines.
The reverb, chorus and pitch shifter therefore **borrow the memory of the long delay lines**, which is idle while one of
them is active (`AdvancedDynamicDelayLine::GetBuffer()`). The memory is wiped when entering one of these modes and again
when leaving to a stock mode. The distortion needs no buffer.

Reverb: 17k + 13k samples, chorus: 2k + 2k, pitch shifter: 8k + 8k, out of 50.8k per channel.

## Files

- `ExtraEffects.hpp` - the DSP (plain C++, no Pico SDK dependencies)
- `AppFxWizardPlus.cpp/.hpp` - the app (copy of the stock FX Wizard plus the four modes)
- `FxWizardParameterMaps.hpp` - knob mappings, the new ones are in the "FX Wizard Plus" section at the end
- `tools/fxplus_test.cpp` - PC test and demo renderer for the DSP (see below)

## Testing the DSP on a computer

```
cd tools
clang++ -std=c++20 -O1 -g -fsanitize=address,undefined -I.. fxplus_test.cpp -o fxplus_test
./fxplus_test            # measurements + randomized stress test
./fxplus_test demos/     # also render WAV demos into demos/ (directory must exist)
```

This checks, among others: reverb RT60 against the requested decay, tail decays to silence, pitch shifter accuracy
(cents), distortion level compensation and DC offset, and that nothing ever leaves the 16 bit range or touches memory
outside its buffers. It does not check the app glue, the CPU load or how it sounds on the hardware.
