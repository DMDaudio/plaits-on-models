// Reference: Plaits' own WavetableEngine (unchanged), 32-sample blocks, the default wave map (no user data).
// usage: wt_ref note harmonics timbre morph samples -> stdout out, aux (doubles); stderr f0
#include <cstdio>
#include <cstdlib>
#include "plaits/dsp/engine/wavetable_engine.h"
using namespace plaits;
char ram[16384];
int main(int argc, char** argv) {
  float note = atof(argv[1]), harm = atof(argv[2]), timb = atof(argv[3]), morph = atof(argv[4]);
  int total = atoi(argv[5]);
  stmlib::BufferAllocator alloc(ram, sizeof(ram));
  static WavetableEngine e;
  e.Init(&alloc);
  e.LoadUserData(NULL);
  e.Reset();
  fprintf(stderr, "%.9g\n", NoteToFrequency(note));
  float out[32], aux[32];
  bool env;
  for (int done = 0; done < total; done += 32) {
    EngineParameters p = {0, note, timb, morph, harm, 0.8f};
    e.Render(p, out, aux, 32, &env);
    for (int i = 0; i < 32; ++i) { double o = out[i], a = aux[i]; fwrite(&o, 8, 1, stdout); fwrite(&a, 8, 1, stdout); }
  }
}
