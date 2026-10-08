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
- **VA** (`virtual_analog_engine.cc` variant 2, `variable_shape_oscillator.h`, `variable_saw_oscillator.h`): a
  variable square with hard sync and a variable saw on OUT, two variable-shape oscillators in "monster sync" on AUX,
  polyBLEP and integrated polyBLEP on every edge. Its pitch offsets use Plaits' two pitch tables (semitones and
  1/256 semitones) in Q31: the 16-bit `exp2` the other engines use put the hard-sync patterns audibly off.
- **MODAL** (`modal_engine.cc`, `modal_voice.cc`, `resonator.cc`): 24 band-pass modes excited by a filtered click. A
  mode's Q reaches tens of thousands, where the damping term is a millionth of the filter's denominator: the
  coefficients are computed with 31-bit mantissas and exponents (`struct uf`, one EMAC multiply each), cached while
  pitch and knobs stay put, and the 24 filters run in a hand-written EMAC loop (23 instructions a mode and sample).
- **STRING** (`string_engine.cc`, `string_voice.cc`, `string.cc`): three delay-line strings, 1024 + 256 samples
  each, with a DC blocker, a damping low-pass, Hermite reads, and either a curved bridge or an allpass dispersion
  driven by noise. It draws its random numbers in the same order as Plaits, so it can be compared sample by sample.
- **6-OP** (`engine2/six_op_engine.cc`, `fm/`): two voices of six operators, 32 algorithms (the render calls dumped from
  Plaits' own `Algorithms<6>::Compile()`), DX7 envelopes, pitch envelope, LFO, keyboard and rate scaling, and the 96
  factory patches unpacked on demand. Operator outputs are phase offsets in turns, Q26. Single operators get
  specialised loops.
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
| VA | 8,736 | -67.4 dB | -78.8 dB / -11.9 dB (see below) |
| MODAL | 8,736 | -70.1 dB | -46.6 dB / -13.4 dB (see below) |
| STRING | 8,736 | -64.2 dB | -66.0 dB / see below |
| 6-OP bank A / B / C | 8,736 each | -68.5 / -71.1 / -49.6 dB | -69.8 / -63.7 / -45.4 dB |
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

VA's worst cases are all AUX at full TIMBRE with HARMONICS in the middle: both oscillators then run at the same
pitch with sync at exactly 16 times it, and their difference is nearly silent (around -45 dBFS) and very sensitive
to rounding.

MODAL's and STRING's worst cases sit where Plaits is itself unstable: modes with Q in the hundreds of thousands near
the top of the band, and the lowest notes of STRING with full dispersion and endless decay. There, float Plaits
and double Plaits differ by -20 to -40 dB, as much as the port does.

6-OP is checked against float Plaits: compiled with `float` as `double`, Plaits' `Pow2Fast` (which builds a float's
exponent bits through a union) returns nonsense. Its worst cases are bright, feedback-heavy patches that are chaotic
in Plaits itself: moving the note by 0.01 semitone changes Plaits' own output by -1 to -7 dB there.

One behaviour of Plaits is not reproduced: `Lfo::Init` leaves the LFO running at 0.1 cycle a sample until a patch is
loaded, so a voice that has not played yet runs its LFO phase up without bound and has a garbage LFO for its first
note. In the port an idle voice's LFO stands still (the reference harness is patched the same way to compare).

Two things in the reference harness matter for anyone redoing this: Plaits sizes its engines' scratch buffers for
24 samples (`kMaxBlockSize`), so a reference rendering 32-sample blocks must give STRING its own buffer, or the
overflow lands in the first string's delay line; and the engines limited by the voice (MODAL, STRING) must go
through the same `stmlib::Limiter` on both sides.

## In the firmware

The PLAITS machine uses the added-machine mechanism of Modded-Cycles (as its Braids MACRO): relocated machine,
name, descriptor and knob tables, a dispatch detour that sends machine 8 to PLAITS and the others to Model-TG, and a
payload copied at boot to 0x46700000 (the top of Model-TG's sample region, given up for it). The engines take 147 KB
of code and tables, and 92 KB of variables for six voices (mostly STRING's delay lines), which the boot hook zeroes
and which take no room in the image. With Model-TG and 6-channel USB the decompressed OS ends at 0x401e4db8, under
the bootstrap's 0x40200000 limit.

CPU, in instructions per 32-sample block of the whole voice loop with one PLAITS track (166,667 cycles a block):

| Engine | Average | Peak |
|---|---|---|
| WSHAPE | 4.1 % | 4.2 % |
| FM | 4.1 % | 4.1 % |
| VA | 5.8 % | 6.0 % |
| MODAL | 14.3 % | 21.6 % while a knob moves |
| STRING | 12.3 % | 12.6 % |
| 6-OP | 6.6 % | 10.7 % (12.1 % while a knob moves) |
| BDRUM | 5.2 % | 8.0 % |
| GRAIN | 5.3 % | 5.4 % |
| CHORDS | 11.5 % | 11.5 % |
| SWARM | 7.4 % | 7.6 % |
| WAVES | 14.4 % | 14.4 % |

Instructions are not cycles; real cost will be higher. A silent PLAITS voice is not computed.
