// Reference: Plaits' own SwarmEngine (unchanged), 32-sample blocks, a trigger (rising edge, patched) at block 0,
// stmlib's Random seeded as MACRO's voice (0x2545f491). usage: swarm_ref note harm timb morph samples
#include <cstdio>
#include <cstdlib>
#include "plaits/dsp/engine/swarm_engine.h"
#include "stmlib/utils/random.h"
using namespace plaits;
char ram[16384];
int main(int argc, char** argv) {
  float note = atof(argv[1]), harm = atof(argv[2]), timb = atof(argv[3]), morph = atof(argv[4]);
  int total = atoi(argv[5]);
  stmlib::Random::Seed(0x2545f491u);
  stmlib::BufferAllocator alloc(ram, sizeof(ram));
  static SwarmEngine e;
  e.Init(&alloc);
  e.Reset();
  fprintf(stderr, "%.9g\n", NoteToFrequency(note));
  float out[32], aux[32];
  bool env;
  for (int done = 0; done < total; done += 32) {
    EngineParameters p = {done == 0 ? TRIGGER_RISING_EDGE : TRIGGER_LOW, note, timb, morph, harm, 0.8f};
    e.Render(p, out, aux, 32, &env);
    for (int i = 0; i < 32; ++i) { double o = out[i], a = aux[i]; fwrite(&o, 8, 1, stdout); fwrite(&a, 8, 1, stdout); }
  }
}
