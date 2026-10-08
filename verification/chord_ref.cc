// Reference: Plaits' own float ChordEngine (unchanged source), rendered in 32-sample blocks with fixed knobs.
// usage: chord_ref note harmonics timbre morph samples  -> stdout: out, aux interleaved (doubles); stderr: f0
// chord_ref_d: the same source with float compiled as double (dbl.h), free of float phase drift
#include <cstdio>
#include <cstdlib>
#include "plaits/dsp/engine/chord_engine.h"
using namespace plaits;
char ram[16384];
int main(int argc, char** argv) {
  float note = atof(argv[1]), harm = atof(argv[2]), timb = atof(argv[3]), morph = atof(argv[4]);
  int total = atoi(argv[5]);
  stmlib::BufferAllocator alloc(ram, sizeof(ram));
  static ChordEngine e;
  e.Init(&alloc);
  e.Reset();
  EngineParameters p = {0, note, timb, morph, harm, 0.8f};
  fprintf(stderr, "%.9g\n", NoteToFrequency(note));
  float out[32], aux[32];
  bool env;
  for (int done = 0; done < total; done += 32) {
    e.Render(p, out, aux, 32, &env);
    for (int i = 0; i < 32; ++i) { double o = out[i], a = aux[i]; fwrite(&o, 8, 1, stdout); fwrite(&a, 8, 1, stdout); }
  }
}
