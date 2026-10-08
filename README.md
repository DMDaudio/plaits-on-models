# Plaits on the Elektron Model:Cycles and Model:Samples

Engines from Mutable Instruments Plaits running inside the Model:Cycles firmware (OS 1.13), and on a
Model:Samples that runs the Cycles OS. They show up as a new machine, **PLAITS**, next to the stock machines
and Model-TG's Sampler. One knob picks the engine, and it can be p-locked per step.

Unofficial. Not made or endorsed by Elektron or Mutable Instruments. Installing modified firmware is at your own
risk. No firmware is distributed here: you build it from your own copy of the official OS.

**Status: works in emulation, not yet confirmed on hardware.** See [docs/progress.md](docs/progress.md).

## What you get

| SHAPE | Engine | Plaits model |
|---|---|---|
| 0 | WSHAPE | waveshaping oscillator |
| 1 | FM | two-operator FM |
| 2 | VA | two oscillators: variable square and saw, detune, hard sync |
| 3 | BDRUM | analog and synthetic bass drum |
| 4 | GRAIN | granular formant oscillator |
| 5 | CHORDS | chords (string machine, then wavetable) |
| 6 | SWARM | swarm of 8 grains or glissandi |
| 7 | WAVES | 8x8x4 wavetable |
| 8 | MODAL | modal resonator: struck bells, bars and plates |
| 9 | STRING | inharmonic string: plucked, buzzing or piano-like strings |
| 10 | 6-OP | 6-operator FM with the 96 factory patches of Plaits' three banks (DX7 format) |

Controls on the PLAITS machine:

| Knob | Function |
|---|---|
| PITCH, FINE | note, as on the stock machines |
| COLOR | HARMONICS |
| SHAPE | engine, one step per engine, p-lockable |
| SWEEP | TIMBRE |
| CONTOUR | MORPH |
| DECAY, GATE, PUNCH | the stock amp section, set up as on TONE |
| Preset + DECAY / SWEEP / CONTOUR | Model-TG's Attack, Filter and Resonance |

The engine set is chosen at build time. Any of PARTICLE, SNARE and HIHAT can be added back, or engines left out
to save space and CPU.

## Install

There is no ready-made firmware file to download: the firmware is Elektron's, so you build it on your own computer
from the official OS file. It takes a few minutes and no hardware. In short:

1. Install Python 3 and the m68k cross compiler (`brew install m68k-elf-gcc` on macOS).
2. Download `model-cycles_OS1.13.syx` (and, for a Model:Samples, `model-samples_OS1.13.syx`) from elektron.se.
3. Clone the Modded-Cycles fork with the PLAITS files and run two commands: one makes the PLAITS machine, one builds
   the OS with Model-TG, PLAITS and 6-channel USB.
4. On a Model:Samples, convert the result with `tools/pack_for_samples.py`.
5. Back up your projects, hold FUNC while powering on, press TRIG 4 (OS upgrade), and send the file from a MIDI
   interface to the MIDI IN with any SysEx sender (the startup menu ignores USB).

The exact commands, the Model:Cycles route, and how to go back to the stock OS are in
[docs/build.md](docs/build.md). A modified OS is installed at your own risk; the startup menu and the official OS
file always get you back.

## How it is put together

- **The engines** ([engines/](engines/)) are Plaits rewritten in 32-bit integer arithmetic for the ColdFire
  MCF54415, which has no floating-point unit. The base is MACRO from
  [gdeo607/digi1_mods](https://github.com/gdeo607/digi1_mods), a Digitakt mod that ported eight Plaits engines.
  CHORDS, SWARM and WAVES were ported here. Unchanged Plaits code, run with software floating point, would need
  1x to 20x the whole CPU for a single voice.
- **The machine** is built with [18nelli18/Modded-Cycles](https://github.com/18nelli18/Modded-Cycles), using the
  same added-machine mechanism as its Braids MACRO machine. It combines with Model-TG and with 6-channel USB
  audio. The build files for that are on a fork of Modded-Cycles (link in [docs/build.md](docs/build.md)),
  because Modded-Cycles has no licence of its own yet.
- **An earlier version** put Plaits inside Model-TG's Sampler as an eighth playback mode
  ([model-tg-sampler-mode/](model-tg-sampler-mode/)). It still works, but the machine version is the one to use.

## Documentation

- [docs/guide.md](docs/guide.md): playing PLAITS, what each engine does with each knob
- [docs/build.md](docs/build.md): building the firmware, installing it, going back to stock
- [docs/technical.md](docs/technical.md): the port, how it was checked against Plaits, memory and CPU numbers
- [docs/progress.md](docs/progress.md): what is done, what was tested where, what is next

## Credits

- Émilie Gillet, Mutable Instruments: Plaits ([pichenettes/eurorack](https://github.com/pichenettes/eurorack), MIT)
- gdeo607 (Gidede): MACRO, the fixed-point port of Plaits engines for the Digitakt
  ([digi1_mods](https://github.com/gdeo607/digi1_mods), MIT)
- 18nelli18 (Maxime): Modded-Cycles, its added-machine mechanism and the Braids MACRO machine it follows
- TinyGregAudio: [Model-TG](https://github.com/TinyGregAudio/Model-TG) (MIT)
- The Octahackers Discord, where most of this was discussed

## Licence

MIT for the work in this repository, see [LICENSE](LICENSE). The engines keep their original MIT notices
(Plaits, digi1_mods). Elektron firmware is not included and is not covered by any of this.
