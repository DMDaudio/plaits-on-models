#include <cstring>
#include "plaits/dsp/oscillator/wavetable_oscillator.h"
#include "plaits/resources.h"
extern "C" {
#include "macro.h"
void t_wtv(struct macro_wtv *w, uint32_t inc, int32_t amp, int32_t wave, int32_t *out, int n);
}
#define WAVE(bank, row, column) &plaits::wav_integrated_waves[(bank * 64 + row * 8 + column) * 132]
const int16_t* const wt[] = { WAVE(2, 6, 1), WAVE(2, 6, 6), WAVE(2, 6, 4), WAVE(0, 6, 0), WAVE(0, 6, 1), WAVE(0, 6, 2),
  WAVE(0, 6, 7), WAVE(2, 4, 7), WAVE(2, 4, 6), WAVE(2, 4, 5), WAVE(2, 4, 4), WAVE(2, 4, 3), WAVE(2, 4, 2), WAVE(2, 4, 1), WAVE(2, 4, 0) };
int main() {
  double fs[] = {0.0007, 0.003, 0.011, 0.04, 0.12}, ws[] = {0.0, 0.3, 0.5, 0.97};
  for (double f : fs) for (double wv : ws) {
    plaits::WavetableOscillator<128, 15> o; o.Init();
    struct macro_wtv w; memset(&w, 0, sizeof w);
    uint32_t inc = (uint32_t)llrint(f * 4294967296.0);
    double se = 0, sr = 0;
    for (int b = 0; b < 400; b++) {
      float out[32] = {0}; int32_t q[32] = {0};
      o.Render(f, 0.25, wv, wt, out, 32);
      t_wtv(&w, inc, 8192, (int32_t)lrint(wv * 65536), q, 32);
      for (int i = 0; i < 32; i++) { double a = q[i] / 16777216.0, x = out[i];
        if (b >= 50) { se += (a - x) * (a - x); sr += x * x; }
        if (b == 200 && i < 4 && f == 0.003 && wv == 0.3) printf("   ours %+.5f ref %+.5f\n", a, x); }
    }
    printf("f %.4f wave %.2f: diff %6.1f dB (ref rms %.4f)\n", f, wv, 10 * log10(se / sr), sqrt(sr / (350 * 32)));
  }
}
