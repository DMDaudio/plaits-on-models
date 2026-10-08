# Progress

## Done

- **2026-10-07** Feasibility: Plaits compiled for the ColdFire. Software float is far too slow; an FPU-equivalent
  build shows a fixed-point port fits (4-21 % of the CPU per voice).
- **2026-10-07** Found gdeo607's MACRO (digi1_mods), eight Plaits engines already in fixed point for the Digitakt.
- **2026-10-08** MACRO wired into Model-TG as an eighth Sampler mode, "Plaits". Emulation: boot, menus, saving,
  sound. A naming clash that would have written into Model-TG's code was caught by the UI test and fixed.
- **2026-10-08** Engine selection at build time. PARTICLE, SNARE and HIHAT dropped from the default set.
- **2026-10-08** CHORDS, SWARM and WAVETABLE ported to fixed point and checked against Plaits.
- **2026-10-08** Engine code moved to RAM at boot, freeing filesystem cache.
- **2026-10-08** The Sampler-mode build installed on a Model:Samples over MIDI IN. It boots.
- **2026-10-08** PLAITS as its own machine, built with Modded-Cycles' added-machine mechanism, combined with
  Model-TG and 6-channel USB. Emulation: boot, machine list and interface, all eight engines exact against the
  PC build, SHAPE p-lock, silent voices, Model-TG's Filter and Resonance, stock machines unchanged.

- **2026-10-08** VA (virtual analog, Plaits' two-oscillator engine) ported and checked against Plaits. It replaces
  NOISE in the default set. Emulation: exact against the PC build on the ColdFire, 5.8 % CPU.

## Tested on hardware

| Build | Device | Result |
|---|---|---|
| Model-TG + Plaits Sampler mode (8 engines) | Model:Samples | installs and boots; sound not yet reported |
| Model-TG + PLAITS machine + 6ch USB | | not yet installed |
| Model-TG + PLAITS machine (VA instead of NOISE) + 6ch USB | | not yet installed |

## Next

- Install the PLAITS machine build on the Model:Samples and report what works.
- Engine names on the SHAPE value instead of numbers.
- AUX output on a knob (or a setting).
- Modal resonator and inharmonic string (in progress), then 6-op FM.
- SPEECH: three speech synthesisers and the LPC word banks, the largest engine left.
- Offer the PLAITS machine to Modded-Cycles as a pull request.
