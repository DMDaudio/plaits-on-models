# Plaits as a Model-TG Sampler mode (earlier version)

Before the PLAITS machine, the engines were added to Model-TG's Sampler machine as an eighth playback mode.
It works, but the engine sits behind Sampler > Settings + Preset, and the stock amp knobs are not on its page.
The PLAITS machine replaces it.

- `model-tg-plaits-mode.patch`: changes to Model-TG v1.1.0 (`70b39dd`): the mode, its menus and saving, the
  build of the C code into RAM at boot (0x46600000; `REGION_END` lowered to 0x4e600000).
- `macro_glue.c`: connects the Sampler's render to the engines (`plt_fill`).
- `test_macro_mode.py`, `test_plaits_ui.py`: the emulation tests used for it. They expect the local layout they
  were written in (Model-TG and Modded-Cycles checkouts side by side) and need editing to run elsewhere.

In this mode Start is the engine (zones of 8 values), End HARMONICS, Filter TIMBRE, Res MORPH, and the mode's
option (Settings + Punch) chooses OUT, MIX or AUX.
