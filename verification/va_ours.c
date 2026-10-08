#include "macro.c"
void t_vsaw(struct macro_vsaw *o, uint32_t f, int32_t pw31, int32_t ws, int32_t *out, int n) { vsaw_render(o, f, pw31, ws, out, n); }
void t_vsaw_init(struct macro_vsaw *o) { vsaw_init(o); }
void t_vso(struct macro_vso *o, int sync, uint32_t mf, uint32_t sf, int32_t pw31, int32_t ws, int32_t *out, int n) { vso_render(o, sync, mf, sf, pw31, ws, out, n); }
void t_vso_init(struct macro_vso *o, uint32_t mp) { vso_init(o, mp); }
