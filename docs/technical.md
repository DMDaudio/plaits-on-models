# Technical notes

## The problem

The Model:Cycles and Model:Samples use a Freescale ColdFire MCF54415 at 250 MHz. It has an EMAC (multiply-accumulate
unit) but no floating-point unit. Plaits is written in single-precision float for a 72 MHz Cortex-M4 with an FPU.

Compiled unchanged with software floating point and run in an emulator, one Plaits voice needs between 1x and 20x the
whole CPU, depending on the engine. Compiled for a ColdFire variant that has an FPU (a proxy for a careful
fixed-point port, where each float operation becomes about one integer operation), the same engines need 4 % to 21 %
of the CPU. So the port had to be fixed point, and could fit. Numbers in
[verification/feasibility/](../verification/feasibility/).

## The engines

`engines/macro.c` started as MACRO from gdeo607's digi1_mods (Digitakt), which already had WSHAPE, FM, NOISE,
PARTICLE, BDRUM, SNARE, HIHAT and GRAIN in 32-bit integer C, checked against Plaits. Added here:

- **CHORDS** (`chord_engine.cc`, `chord_bank.cc`, `string_synth_oscillator.h`, `wavetable_oscillator.h`):
  five voices, each a divide-down organ (four octave saws on one 32-bit phase with polyBLEP) crossfaded into a
  wavetable voice. Chord ratios are Q30 and multiplied with an exact 32x32 high multiply, because Q16 ratios gave an
  audible-in-the-long-run pitch drift.
- **SWARM** (`swarm_engine.cc`): eight grain envelopes driving polyBLEP saws (OUT) and quadrature sines (AUX).
- **WAVETABLE** (`wavetable_engine.cc`): all 192 integrated waves, eight Hermite reads a sample, trilinear blend,
  differentiation normalised by 1/f once a block.
- **Engine selection** (`MACRO_SEL`, `MACRO_MASK`): engines left out are not linked.

Conventions follow MACRO: 32-bit phases, Q15 signals, knob values 0..127, no 64-bit arithmetic and no library calls
(the firmware links none). Filters use the EMAC through `fmac1` / `fmac2`, which `macro_render` saves and restores.

## How it was checked

Each new engine was compared with Plaits' own source compiled for a PC, at the same knob values and pitch:

- The reference is Plaits compiled with `float` redefined as `double`. Float Plaits drifts in phase over a second
  or two (its float phase accumulator rounds the same way every sample), which would hide real errors.
- Deterministic engines are compared sample by sample and by magnitude spectrum, after the knob smoothing settles.
- SWARM is chaotic (a grain re-randomises when its phase wraps), so it is compared by long-term 1/3-octave band
  levels, with float Plaits against double Plaits as the yardstick.

| Engine | Cases | Waveform difference (median) | Spectrum difference (median / worst) |
|---|---|---|---|
| CHORDS | 8,464 | -56.7 dB | -65.5 dB / phase artifacts only (see below) |
| WAVETABLE | 8,700 | -59.2 dB | -64.1 dB / -36.7 dB |
| SWARM | 2,100 | (chaotic) | band levels: median 0.3 dB, worst 22.7 dB; float Plaits: 0.0 / 23.4 dB |

The remaining CHORDS outliers are the OCT and 5 chords at the top inversion. Float Plaits rounds to inversion 20 there,
the port stays at 19.99: the same notes on different voices, so near-unison voices beat with a different phase.

In the firmware, the ColdFire output of every engine is identical, sample for sample, to the same C compiled for a PC,
inside the Cycles OS's own voice loop running in Unicorn with an exact EMAC model.

## Plaits behaviour found on the way

These are in Plaits itself. The port keeps the first two and avoids the third:

- SWARM's voice ranks run from -1 to +1.33, not ±1: the centre is `(8 - 1) / 2` in integer arithmetic, 3.
- `NoteToFrequency` reads its pitch table in steps of 1/256 semitone. SWARM's grain density follows those steps.
- In WAVETABLE, with the bank knob in its snapping range, a grid position can settle at exactly 7.0 and Plaits reads
  wave 8 of a row, past its 64-wave bank. On a PC both float and double Plaits crash at those settings (18 of the
  8,736 tested). The port clamps to the last wave.

## In the firmware

The PLAITS machine uses the added-machine mechanism of Modded-Cycles (as its Braids MACRO): relocated machine,
name, descriptor and knob tables, a dispatch detour that sends machine 8 to PLAITS and the others to Model-TG, and a
payload copied at boot to 0x46700000 (the top of Model-TG's sample region, given up for it). The engines take 99 KB
of code and tables, 3 KB of variables for six voices. With Model-TG and 6-channel USB the decompressed OS ends at
0x401d91b8, under the bootstrap's 0x40200000 limit.

CPU, in instructions per 32-sample block of the whole voice loop with one PLAITS track (166,667 cycles a block):

| Engine | Average | Peak |
|---|---|---|
| WSHAPE | 4.1 % | 4.2 % |
| FM | 4.1 % | 4.1 % |
| NOISE | 4.0 % | 4.0 % |
| BDRUM | 5.2 % | 8.0 % |
| GRAIN | 5.3 % | 5.4 % |
| CHORDS | 11.5 % | 11.5 % |
| SWARM | 7.4 % | 7.6 % |
| WAVES | 14.4 % | 14.4 % |

Instructions are not cycles; real cost will be higher. A silent PLAITS voice is not computed.
