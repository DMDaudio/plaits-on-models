#include "macro.c"
void t_ssynth(struct macro_ssynth *s, uint32_t inc, const int32_t *reg, int32_t gain, int32_t *out, int n)
{ ssynth_render(s, inc, reg, gain, out, n); }
void t_wtv(struct macro_wtv *w, uint32_t inc, int32_t amp, int32_t wave, int32_t *out, int n)
{ wtv_render(w, inc, amp, wave, out, n); }
void t_genv(struct macro_genv *e, uint32_t *rng, int32_t rate, int start, int32_t sr, int32_t *amp, int32_t *freq)
{ genv_step(e, rng, rate, start); *amp = genv_amp(e, sr); *freq = genv_freq(e, sr); }
void t_swarm(struct macro_voice *m, const uint8_t *p, uint32_t inc, int32_t *o, int32_t *a, int n)
{ swarm_render(m, p, inc, o, a, n, 1, 1); }
