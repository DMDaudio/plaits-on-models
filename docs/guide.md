# Playing PLAITS

## Getting started

1. Select a track, open the machine list and choose **PLAITS**. With Model-TG installed it comes after the Sampler.
2. Play some trigs. The default engine is WSHAPE.
3. Turn **SHAPE** to change engine. Each step of the knob is one engine:

   | SHAPE | Engine |
   |---|---|
   | 0 | WSHAPE |
   | 1 | FM |
   | 2 | NOISE |
   | 3 | BDRUM |
   | 4 | GRAIN |
   | 5 | CHORDS |
   | 6 | SWARM |
   | 7 | WAVES |

4. COLOR, SWEEP and CONTOUR are Plaits' three macro controls: HARMONICS, TIMBRE and MORPH.

The engine is read when a note starts. A p-lock on SHAPE changes the engine on that trig, and turning SHAPE
while a note rings changes it from the next note. Every other knob acts immediately, can be p-locked and takes
LFOs, as on the stock machines.

PITCH and FINE set the note the same way as on TONE. DECAY, GATE and PUNCH are the stock amp section. With Model-TG,
hold **Preset** and turn DECAY, SWEEP or CONTOUR for Attack, Filter and Resonance, as on every other machine.

## The engines

| Engine | COLOR (HARMONICS) | SWEEP (TIMBRE) | CONTOUR (MORPH) |
|---|---|---|---|
| WSHAPE | waveshaper curve | wavefolder amount | waveform asymmetry |
| FM | frequency ratio, in steps | modulation index | feedback: phase feedback below the middle, self-modulation above |
| NOISE | filter response, low-pass through band-pass to high-pass | clock rate of the noise | resonance |
| BDRUM | attack FM, self-FM and drive | tone | decay |
| GRAIN | formant ratio and carrier bleed | formant frequency | carrier shape |
| CHORDS | chord: OCT, 5, sus4, m, m7, m9, m11, 69, M9, M7, M | inversion, spread over five voices | organ registration, then the wavetable voices from the middle up |
| SWARM | pitch spread of the eight voices | grain density | grain size: small sizes become glissandi |
| WAVES | bank (four, mirrored; in the upper half the position snaps to whole waves) | X position on the 8x8 grid | Y position on the grid |

Notes on some engines:

- **BDRUM** makes its own envelope from each trig. Set DECAY long and let the engine shape the sound.
- **FM** with feedback runs four times oversampled from the start of a note. Moving CONTOUR during a note keeps the
  rate the note started with, so there are no clicks.
- **SWARM**: every trig starts a burst. The grains slow down after it, as with a patched trigger on the module.
- **CHORDS** and **WAVES** are the heaviest engines. Two or three tracks of them at once are fine. More may crackle.

## Things that differ from the module

- The PLAITS machine plays Plaits' main output only. The AUX output (the second, related sound each engine makes)
  is not on a knob yet.
- Engine changes, CHORDS and WAVES position changes start from the current knob values instead of gliding from zero,
  so a p-lock does not sweep through chord inversions or waves first.
- There is no low-pass gate. DECAY, GATE, PUNCH and Model-TG's filter do that job.
