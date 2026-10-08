#include <cstring>
#include "plaits/dsp/engine/swarm_engine.h"
#include "stmlib/utils/random.h"
extern "C" {
#include "macro.h"
void t_genv(struct macro_genv *e, uint32_t *rng, int32_t rate, int start, int32_t sr, int32_t *amp, int32_t *freq);
}
int main(int argc, char** argv) {
  double rate = atof(argv[1]), sr = atof(argv[2]);
  stmlib::Random::Seed(0x2545f491u); uint32_t rng = 0x2545f491u;
  plaits::GrainEnvelope g; g.Init();
  struct macro_genv e = { 0, 1 << 30, 1 << 28, 0, 1 << 29, 0, 0 };
  for (int b = 0; b < 40; b++) {
    g.Step(rate, true, b == 0);
    double a = g.amplitude(sr), f = g.frequency(sr);
    int32_t ia, ifr;
    t_genv(&e, &rng, (int32_t)llrint(rate * 1073741824.0), b == 0, (int32_t)llrint(sr * 65536), &ia, &ifr);
    if (b < 12 || b % 8 == 0)
      printf("b%2d ref amp %.5f freq %+.5f | ours amp %.5f freq %+.5f | ours phase %.4f fm %.3f\n", b, a, f,
             ia / 1073741824.0, ifr / 1073741824.0, e.phase / 268435456.0, e.fm / 67108864.0);
  }
}
