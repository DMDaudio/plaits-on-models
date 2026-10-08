# Playing PLAITS

## Getting started

1. Select a track, open the machine list and choose **PLAITS**. With Model-TG installed it comes after the Sampler.
2. Play some trigs. The default engine is WSHAPE.
3. Turn **SHAPE** to change engine. Each step of the knob is one engine:

   | SHAPE | Engine |
   |---|---|
   | 0 | WSHAPE |
   | 1 | FM |
   | 2 | VA |
   | 3 | BDRUM |
   | 4 | GRAIN |
   | 5 | CHORDS |
   | 6 | SWARM |
   | 7 | WAVES |
   | 8 | MODAL |
   | 9 | STRING |
   | 10 | 6-OP |

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
| VA | detune of the second oscillator: unison in the middle, then fifth, octave, ... up to two octaves either way | square: pulse width, then hard sync above the middle | saw: notch, then triangle, then narrower |
| BDRUM | attack FM, self-FM and drive | tone | decay |
| GRAIN | formant ratio and carrier bleed | formant frequency | carrier shape |
| CHORDS | chord: OCT, 5, sus4, m, m7, m9, m11, 69, M9, M7, M | inversion, spread over five voices | organ registration, then the wavetable voices from the middle up |
| SWARM | pitch spread of the eight voices | grain density | grain size: small sizes become glissandi |
| WAVES | bank (four, mirrored; in the upper half the position snaps to whole waves) | X position on the 8x8 grid | Y position on the grid |
| MODAL | material: bell-like partials at the left, a harmonic series in the middle, stretched partials to the right | brightness of the strike | decay |
| 6-OP | the patch: 96 of them, one or two knob steps each (0-31 basses, synths and leads; 32-63 pianos, keys, mallets, bells and drums; 64-95 organs, pads, strings and brass; COLOR x 3/4 is the number) | brightness: louder modulators | envelope times: shorter to the left, longer to the right |
| STRING | left of the middle a curved bridge (a sitar-like buzz), a plain string in the middle, to the right more and more dispersion (stretched, piano-like, then rough) | brightness of the pluck | decay, endless at the top |

Notes on some engines:

- **BDRUM**, **MODAL** and **STRING** make their own envelope from each trig. Set DECAY long and let the engine shape
  the sound.
- **STRING** plucks three strings in turn, so a note keeps ringing while the next ones play, as on the module.
- **6-OP** plays each trig as a 125 ms gate (the module holds the gate while its trigger input is high): sustaining
  patches release after that, at their own release rate. The slowest pads need CONTOUR below the middle, which
  shortens attacks, to speak quickly. Two voices take the trigs in turn, so a note's release overlaps the next.
- **FM** with feedback runs four times oversampled from the start of a note. Moving CONTOUR during a note keeps the
  rate the note started with, so there are no clicks.
- **SWARM**: every trig starts a burst. The grains slow down after it, as with a patched trigger on the module.
- **MODAL**, **WAVES**, **STRING** and **CHORDS** are the heaviest engines (12 to 15 % of the CPU each, MODAL up to 22 %
  while a knob moves). Two or three tracks of them at once are fine. More may crackle.

## Things that differ from the module

- The PLAITS machine plays Plaits' main output only. The AUX output (the second, related sound each engine makes)
  is not on a knob yet.
- Engine changes, CHORDS and WAVES position changes start from the current knob values instead of gliding from zero,
  so a p-lock does not sweep through chord inversions or waves first.
- There is no low-pass gate. DECAY, GATE, PUNCH and Model-TG's filter do that job.
