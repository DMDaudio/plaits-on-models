# Building and installing

No firmware is distributed. Everything is built from the official OS files, which you download from elektron.se.
Building does not need the hardware. Installing does, and a modified OS is installed at your own risk.

## What you need

- macOS or Linux, Python 3, git.
- The m68k cross compiler: on macOS `brew install m68k-elf-gcc` (this also installs the binutils); on Debian or
  Ubuntu `sudo apt install gcc-m68k-linux-gnu binutils-m68k-linux-gnu`.
- `model-cycles_OS1.13.syx` from the Model:Cycles support page. For a Model:Samples, also
  `model-samples_OS1.13.syx`.
- For the emulation tests: `pip install unicorn numpy`.

## Build the PLAITS machine

The machine is built with Modded-Cycles. The files that add PLAITS to it (`tools/gen_plaits.py`,
`tools/machines/plaits/`, `tools/emu/test_plaits.py`) are on the `plaits-machine` branch of the fork:
<https://github.com/DMDaudio/Modded-Cycles/tree/plaits-machine>.

```sh
git clone -b plaits-machine https://github.com/DMDaudio/Modded-Cycles
cd Modded-Cycles
python3 tools/gen_plaits.py --cycles path/to/model-cycles_OS1.13.syx
python3 tools/build.py -i path/to/model-cycles_OS1.13.syx -t model-tg-st,plaits-tg,6ch-usbup -o plaits.syx
```

- `model-tg-st,plaits-tg` is Model-TG with PLAITS as the eighth machine. Use `plaits` alone, without Model-TG, for
  PLAITS as the seventh machine.
- `6ch-usbup` adds Modded-Cycles' six-channel USB audio. Leave it out if you do not want it.
- PLAITS cannot be combined with Modded-Cycles' Braids MACRO or Syntakt engines: they use the same memory.
- To change the engines or their order, pass `--engines` to `gen_plaits.py`, for example
  `--engines CHORDS,WAVETABLE,SWARM,FM`. The choices are WSHAPE, FM, NOISE, PARTICLE, BDRUM, SNARE, HIHAT, GRAIN,
  CHORDS, SWARM and WAVETABLE, up to 16. Changing the list changes what a saved SHAPE value plays.

Check the build in emulation (a few minutes):

```sh
python3 tools/emu/test_plaits.py --cycles path/to/model-cycles_OS1.13.syx
```

## Install on a Model:Cycles

Like an official update: Elektron Transfer, or the startup menu over MIDI IN (see Modded-Cycles' `FLASH.md`).

## Install on a Model:Samples

A Model:Samples can run the Cycles OS. The rule is to use the file format of the OS that receives the update:

| The Samples is currently running | Send | How |
|---|---|---|
| the Cycles OS (a Model-TG or Modded-Cycles build) | the Cycles-format file (`plaits.syx` above) | Elektron Transfer over USB, with Model-TG's Transfer identity on CYC |
| its startup menu (hold FUNC at power-on, then TRIG 4) | a Samples-format file, made with `tools/pack_for_samples.py` | MIDI IN only: a MIDI interface's OUT to the Samples' MIDI IN. The startup menu ignores USB |

```sh
MODDED_CYCLES=path/to/Modded-Cycles python3 tools/pack_for_samples.py \
    --samples path/to/model-samples_OS1.13.syx --cycles path/to/model-cycles_OS1.13.syx \
    --guest plaits.syx -o plaits_for-model-samples.syx
```

Neither way replaces the Samples' own bootloader, so its startup menu stays the way back.

Before installing: back up projects and samples with Transfer. Use mains power and do not switch off while the
screen says it is writing.

## Going back

Startup menu (FUNC at power-on, TRIG 4), then send the official `model-samples_OS1.13.syx` (or
`model-cycles_OS1.13.syx` on a Cycles) over MIDI IN. If the machine starts but hangs on a project, FUNC at
power-on and TRIG 2 (EMPTY RESET) clears the active project; restore it from your backup.

## The Model-TG Sampler-mode version

[model-tg-sampler-mode/](../model-tg-sampler-mode/) holds the earlier version, where Plaits is an eighth playback
mode of Model-TG's Sampler. To build it, apply the patch to Model-TG v1.1.0 (commit `70b39dd`), copy
`engines/macro*` into `src/macro/` and `macro_glue.c` into `src/`, then run Model-TG's `build.py`.
