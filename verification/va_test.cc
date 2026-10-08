#include <cstring>
#include "plaits/dsp/oscillator/variable_saw_oscillator.h"
#include "plaits/dsp/oscillator/variable_shape_oscillator.h"
extern "C" {
#include "macro.h"
void t_vsaw(struct macro_vsaw *o, uint32_t f, int32_t pw31, int32_t ws, int32_t *out, int n);
void t_vsaw_init(struct macro_vsaw *o);
void t_vso(struct macro_vso *o, int sync, uint32_t mf, uint32_t sf, int32_t pw31, int32_t ws, int32_t *out, int n);
void t_vso_init(struct macro_vso *o, uint32_t mp);
}
static uint32_t U(double f) { return (uint32_t)llrint(f * 4294967296.0); }
int main(int argc, char **argv) {
  int verbose = argc > 1;
  double fs[] = {0.0007, 0.0055, 0.03, 0.11}, pws[] = {0.05, 0.3, 0.55, 0.9}, ws[] = {0.0, 0.4, 1.0};
  for (double f : fs) for (double pw : pws) for (double w : ws) {
    plaits::VariableSawOscillator o; o.Init();
    struct macro_vsaw q; t_vsaw_init(&q);
    double se = 0, sr = 0;
    for (int b = 0; b < 300; b++) {
      float out[32]; int32_t x[32];
      o.Render(f, pw, w, out, 32);
      t_vsaw(&q, U(f), (int32_t)llrint(pw * 2147483648.0), (int32_t)llrint(w * 65536), x, 32);
      for (int i = 0; i < 32; i++) { double a = x[i] / 268435456.0, y = out[i];
        if (b >= 20) { se += (a - y) * (a - y); sr += y * y; }
        if (verbose && b == 100 && i < 8 && f == 0.0055 && pw == 0.55 && w == 1.0) printf("   saw ours %+.5f ref %+.5f\n", a, y); }
    }
    printf("saw f %.4f pw %.2f shape %.1f: diff %6.1f dB\n", f, pw, w, 10 * log10(se / sr));
  }
  for (int sync = 0; sync < 2; sync++) for (double f : fs) for (double pw : pws) for (double w : ws) {
    plaits::VariableShapeOscillator o; o.Init();
    struct macro_vso q; t_vso_init(&q, 0);
    double se = 0, sr = 0, sf = f * 2.3;
    for (int b = 0; b < 300; b++) {
      float out[32]; int32_t x[32];
      if (sync) o.Render(f, sf, pw, w, out, 32); else o.Render(f, pw, w, out, 32);
      t_vso(&q, sync, sync ? U(f) : 0, U(sync ? sf : f), (int32_t)llrint(pw * 2147483648.0), (int32_t)llrint(w * 65536), x, 32);
      for (int i = 0; i < 32; i++) { double a = x[i] / 268435456.0, y = out[i];
        if (b >= 20) { se += (a - y) * (a - y); sr += y * y; } }
    }
    printf("vso sync %d f %.4f pw %.2f shape %.1f: diff %6.1f dB\n", sync, f, pw, w, 10 * log10(se / sr));
  }
}
