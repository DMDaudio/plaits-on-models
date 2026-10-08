// Reference: Plaits' own StringEngine (unchanged), 32-sample blocks, a trig (patched trigger input) at block 0
// and every `every` blocks after; accent 0.8 as Plaits' voice gives an unpatched LEVEL.
// OUT goes through Plaits' limiter, as in the voice.
// usage: modal_ref note harmonics timbre morph samples [every] -> stdout out, aux (doubles); stderr f0
// Plaits sizes temp_buffer_ for kMaxBlockSize (24) samples; these 32-sample blocks get their own.
#define private public
#include <cstdio>
#include <cstdlib>
#include "plaits/dsp/engine/string_engine.h"
#include "stmlib/dsp/limiter.h"
using namespace plaits;
char ram[65536];
int main(int argc, char** argv) {
  float note = atof(argv[1]), harm = atof(argv[2]), timb = atof(argv[3]), morph = atof(argv[4]);
  int total = atoi(argv[5]), every = argc > 6 ? atoi(argv[6]) : 1 << 30;
  stmlib::BufferAllocator alloc(ram, sizeof(ram));
  static StringEngine e;
  e.Init(&alloc);
  e.Reset();
  static float temp32[32];
  e.temp_buffer_ = temp32;
  fprintf(stderr, "%.9g\n", NoteToFrequency(note));
  stmlib::Limiter lim; lim.Init();     // Plaits' voice limits OUT (gain -1): limiter, x 0.8; undone below
  float out[32], aux[32];
  bool env;
  for (int done = 0, b = 0; done < total; done += 32, b++) {
    EngineParameters p = {b % every == 0 ? TRIGGER_RISING_EDGE : TRIGGER_LOW, note, timb, morph, harm, 0.8f};
    e.Render(p, out, aux, 32, &env);
#ifndef NOLIM
#ifndef NOLIM
    lim.Process(1.0f, out, 32);
    for (int i = 0; i < 32; ++i) out[i] /= 0.8f;
#endif
#endif
    for (int i = 0; i < 32; ++i) { double o = out[i], a = aux[i]; fwrite(&o, 8, 1, stdout); fwrite(&a, 8, 1, stdout); }
  }
}
