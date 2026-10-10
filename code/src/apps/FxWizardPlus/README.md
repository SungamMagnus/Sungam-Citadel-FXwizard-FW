# Kastle 2 FX Wizard Plus

A seven-effect variant of the Bastl FX Wizard: **delay, chorus, phaser, reverb (hall), reverb (plate), distortion
and pitch shifter**. Delay is the stock FX Wizard mode, the other six are new. The stock Flanger, Freezer, Panner,
Crusher, Slicer, Replayer, Pitcher and Shifter were removed.

| # | Mode | LED | Time | Feedback | Amount | Stereo (shift layer) |
|---|------|-----|------|----------|--------|----------------------|
| 1 | Delay | green | as in FX Wizard | | | |
| 2 | **Chorus** | cyan | LFO rate, 0.06 - 9 Hz | feedback (as in the stock modes) | depth and wet level | left/right LFO phase (width) |
| 3 | **Phaser** | orange | LFO rate, 0.03 - 8 Hz | resonance | dry/wet (full = 1:1, deepest notches) | left/right LFO phase |
| 4 | **Reverb (hall)** | purple | decay time, 0.3 - 30 s | damping: left = bright, right = dark | wet level | pre-delay, 0 - 75 ms |
| 5 | **Reverb (plate)** | blue | decay time, 0.3 - 30 s | damping: left = bright, right = dark | wet level | pre-delay, 0 - 75 ms |
| 6 | **Distortion** | yellow | drive, 1x - 100x | tone: left = bright, right = dark | dry/wet | asymmetry (even harmonics) |
| 7 | **Pitch shifter** | indigo | pitch, -12 .. +12 semitones, detents at whole semitones | feedback (cascading shifts) | dry/wet | left/right detune, up to 40 cents |

For the new modes, Amount at zero is a clean bypass. The DJ filter (shift + Feedback knob) works in every mode.
A trigger restarts the chorus LFO; in the other modes it only blinks the LEDs.

About the two reverbs:
- **Hall** is a Schroeder / Freeverb style design (8 damped combs and 4 all-passes per side). Sparser, with a
  room-like character. The decay time is set per comb, so it is the same at every setting.
- **Plate** is Jon Dattorro's plate topology (input diffusion, two cross-coupled tanks with modulated all-passes). Denser
  and smoother, with a slow shimmer from the modulation. The stereo image is wider.
- The dark end of the damping range is quieter than the bright end in both.

The pitch shifter is a two-tap crossfaded delay line ("harmonizer" style): clean pitch steps, with the slow flutter
typical of that design, and up to 93 ms of latency on the wet signal. At 0 semitones with no detune it passes the input
through unchanged. Pushing Amount to the end gives the shifted signal alone; somewhere in the middle is the classic
harmonizer blend.

## Building

```
cd code/build
make fx-wizard-plus
```

The UF2 ends up in `code/build/output`. It uses app ID `0x7F` (community range), so it keeps its own settings in the
EEPROM and never touches the settings of the stock FX Wizard (`0xB1`). Calibration is stored separately and is not
affected. If you distribute this, register an ID in [APP_LIST.md](../../../../APP_LIST.md) first.

Size (Release, Arm GNU Toolchain 14.2.Rel1, Pico SDK 2.3.1), compared with the stock FX Wizard: flash 427,716 B
(81.6% of 512 KB, stock 431,828 B), static RAM 23,200 B (stock 23,028 B), FASTCODE 9,200 B of 16 KB.

Outside the Nix shell, with Arm's own toolchain package, the link can fail with `undefined reference to __dso_handle`
(it happens with the stock FX Wizard too: the link uses `-nostartfiles`). Adding
`-DCMAKE_CXX_STANDARD_LIBRARIES=-Wl,--defsym,__dso_handle=0` to the `cmake` command works around it.

## How it fits in the RAM

The RP2040 has no spare RAM: the stock FX Wizard already leaves only about 6 kB of the 240 kB heap free after its delay
lines. The chorus and both reverbs therefore **borrow the memory of the long delay lines**, which is idle while they are
active (`AdvancedDynamicDelayLine::GetBuffer()`). The memory is wiped when entering one of these modes and again when
leaving to Delay or Distortion. Phaser and distortion need no buffer.

Words used per channel, out of 50,800: hall 17k + 13k, plate 21k + 16k, chorus 2k + 2k, pitch shifter 8k + 8k.

## Files

- `ExtraEffects.hpp` - the DSP (plain C++, no Pico SDK dependencies)
- `AppFxWizardPlus.cpp/.hpp` - the app (copy of the stock FX Wizard, reduced to the seven modes)
- `FxWizardParameterMaps.hpp` - knob mappings, the new ones are in the "FX Wizard Plus" section
- `tools/fxplus_test.cpp` - PC test and demo renderer for the DSP

## Testing the DSP on a computer

```
cd tools
clang++ -std=c++20 -O1 -g -fsanitize=address,undefined -I.. fxplus_test.cpp -o fxplus_test
./fxplus_test            # measurements + randomized stress test
./fxplus_test demos/     # also render WAV demos into demos/ (directory must exist)
```

This checks, among others: reverb RT60 against the requested decay, tails decaying to silence, phaser notch depth and
movement, pitch accuracy and clean unity, distortion level compensation and DC offset, and that nothing leaves the 16 bit range or touches memory
outside its buffers. It does not check the app glue, the CPU load or how it sounds on the hardware.
