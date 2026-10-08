#include <cstdio>
#include <cstring>
#include <cmath>
#include "plaits/dsp/oscillator/string_synth_oscillator.h"
extern "C" {
#include "macro.h"
void t_ssynth(struct macro_ssynth *s, uint32_t inc, const int32_t *reg, int32_t gain, int32_t *out, int n);
}
static const float R[8][6] = {{0,1,0,0,0,0},{1,0,0,0,0,0},{.5f,0,.5f,0,0,0},{.33f,0,.33f,0,.33f,0},
  {.33f,0,0,.33f,0,.33f},{.5f,0,0,0,0,.5f},{0,.5f,0,0,0,.5f},{0,.1f,.1f,0,.2f,.6f}};
int main() {
  float fs[] = {0.0005f, 0.0021f, 0.007f, 0.02f, 0.05f, 0.09f};
  for (int r = 0; r < 8; r++) for (float f : fs) {
    float reg[7] = {R[r][0],R[r][1],R[r][2],R[r][3],R[r][4],R[r][5],0};
    plaits::StringSynthOscillator o; o.Init();
    struct macro_ssynth s; memset(&s, 0, sizeof s); s.inc = 536871;
    int32_t ireg[7]; for (int i = 0; i < 7; i++) ireg[i] = (int32_t)lrintf(reg[i] * 32768);
    uint32_t inc = (uint32_t)llrint(f * 4294967296.0);
    double se = 0, sr = 0, dc = 0; int cnt = 0;
    for (int b = 0; b < 300; b++) {
      float out[32] = {0}; int32_t q[32] = {0};
      o.Render(f, reg, 0.25f, out, 32);
      t_ssynth(&s, inc, ireg, 8192, q, 32);
      for (int i = 0; i < 32; i++) { double a = q[i] / 16777216.0, x = out[i];
        if (b >= 100) { se += (a - x) * (a - x); sr += x * x; dc += a - x; cnt++; } }
    }
    printf("reg %d f %.4f: diff %6.1f dB, mean offset %+.6f\n", r, f, 10 * log10(se / sr), dc / cnt);
  }
}
