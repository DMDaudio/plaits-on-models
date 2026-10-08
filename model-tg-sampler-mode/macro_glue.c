/* Model-TG - src/macro_glue.c
 * The Sampler's eighth mode, Macro: Plaits' engines (Emilie Gillet, MIT), as ported to 32-bit integer
 * arithmetic by digi1_mods' MACRO (src/macro/, MIT; see src/macro/MACRO.md for the engines and how each
 * was checked against Plaits). This file only connects it to the Sampler:
 *
 *   Start  -> ENGN  the engine, 16 zones of 8 (read at each note start, so a p-lock changes it on its trig)
 *   End    -> HARM  Plaits' HARMONICS
 *   Filter -> TIMB  Plaits' TIMBRE
 *   Res    -> MORP  Plaits' MORPH
 *   the mode's option OUT / MIX / AUX -> MACRO's F (Plaits' OUT, a crossfade, its AUX)
 *
 * The four dials stay the same parameters as in every other mode, so they p-lock, take LFOs and persist.
 * sampler_pre (sw_macro) publishes them each block; its trigger edge sets plt_trig; sampler_render calls
 * plt_fill in place of a sample fill, and the 2x buffer then goes through the stock decimator and amp stages.
 *
 * Every variable here is initialised data, never .bss: the blob is copied out of the image as one binary,
 * and whatever lay past its end would be the filesystem cache's.
 */
#include <stdint.h>
#include "macro/macro.h"

#define MC_TRACKS 7                 /* as loop_mode: six tracks and a spare */

struct macro_voice plt_voice[MC_TRACKS] = { { 0 } };
uint8_t  plt_init[MC_TRACKS] = { 0 };    /* the voice has been through macro_init                 */
uint8_t  plt_trig[MC_TRACKS] = { 0 };    /* sampler_pre: a note started on this track this block  */
uint8_t  plt_prm[MC_TRACKS][4] = { { 0 } };  /* ENGN HARM TIMB MORP, 0..127, from sw_macro          */
int32_t  plt_out[MC_TRACKS] = { 0 };     /* the option: 0 OUT, 1 MIX, 2 AUX (a long: the menus' tables) */

static const uint8_t aux_of[3] = { 0, 64, 127 };

/* plt_fill(track, step, buf): one block of the voice into the Sampler's 2x buffer (64 longs, each output
 * sample twice, as the PCM fill's zero-order hold), at the note's step - Q16, 0x10000 = note 60, the same
 * step Pluck and Wave turn into 261.6 Hz. */
void plt_fill(int32_t track, uint32_t step, int32_t *buf)
{
    struct macro_voice *m;
    uint8_t p[MACRO_PARAMS];
    int16_t out[32];
    uint32_t inc;
    int i;
    int32_t o;

    if ((uint32_t)track >= MC_TRACKS)
        return;
    m = &plt_voice[track];
    if (!plt_init[track]) {
        macro_init(m);
        plt_init[track] = 1;
    }
    for (i = 0; i < 4; i++)
        p[i] = plt_prm[track][i];
    o = plt_out[track];
    p[MACRO_P_AUX] = aux_of[(uint32_t)o < 3 ? o : 0];
    p[5] = p[6] = 0;
    if (plt_trig[track]) {
        plt_trig[track] = 0;
        macro_trig(m);
    }
    if (step == 0)
        step = 0x10000;
    /* phase per 48 kHz sample: step x 357.203 (261.626 Hz x 2^32 / 48000 / 65536), as wave_fill */
    inc = step * 357u + ((step * 52u) >> 8);
    macro_render(m, p, inc, out, 32);
    for (i = 0; i < 32; i++) {
        int32_t s = (int32_t)out[i] << 15;
        buf[2 * i] = s;
        buf[2 * i + 1] = s;
    }
}

/* the ENGN dial's value as the engine's name, for the page (0..127 -> "WSHAPE" ...) */
const char *plt_engine_name(int32_t v)
{
    return macro_engine_name[macro_engine_of(v < 0 ? 0 : v > 127 ? 127 : v)];
}
