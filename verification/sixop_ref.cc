// Reference: Plaits' own SixOpEngine (unchanged), bank 0..2 (syx_bank_N, as voice.cc loads them), 32-sample
// blocks with their own scratch buffers (Plaits sizes them for 24-sample blocks), a trig at block 0 and every
// `every` blocks, the gate high for `gate` blocks after each trig; accent 0.8.
// usage: sixop_ref bank note harmonics timbre morph samples [every [gate]] -> stdout out, aux (doubles); stderr f0
#define private public
#include <cstdio>
#include <cstdlib>
#include "plaits/dsp/engine2/six_op_engine.h"
#include "plaits/resources.h"
using namespace plaits;
char ram[65536];
int main(int argc, char** argv) {
  int bank = atoi(argv[1]);
  float note = atof(argv[2]), harm = atof(argv[3]), timb = atof(argv[4]), morph = atof(argv[5]);
  int total = atoi(argv[6]), every = argc > 7 ? atoi(argv[7]) : 1 << 30, gate = argc > 8 ? atoi(argv[8]) : 1;
  stmlib::BufferAllocator alloc(ram, sizeof(ram));
  static SixOpEngine e;
  e.Init(&alloc);
  static float temp[64 * 3], acc[32 * kNumSixOpVoices];
  e.temp_buffer_ = temp;
  e.acc_buffer_ = acc;
  e.LoadUserData(fm_patches_table[bank]);
  // Lfo::Init leaves frequency_ and delay_increment_ at 0.1 a sample until a patch is set: a voice that has not
  // played yet runs its LFO phase up without bound (and draws a random number every block), so its first note has a
  // garbage LFO. The port keeps an idle voice's LFO still; so does this reference.
  for (int i = 0; i < kNumSixOpVoices; ++i) {
    e.voice_[i].lfo_.frequency_ = 0.0f;
    e.voice_[i].lfo_.delay_increment_[0] = e.voice_[i].lfo_.delay_increment_[1] = 0.0f;
  }
  e.Reset();
  fprintf(stderr, "%.9g\n", NoteToFrequency(note));
  float out[32], aux[32];
  bool env;
  for (int done = 0, b = 0; done < total; done += 32, b++) {
    int k = b % every, t = (k == 0 ? TRIGGER_RISING_EDGE : 0) | (k < gate ? TRIGGER_HIGH : 0);
    EngineParameters p = {TriggerState(t), note, timb, morph, harm, 0.8f};
    e.Render(p, out, aux, 32, &env);
    for (int i = 0; i < 32; ++i) { double o = out[i], a = aux[i]; fwrite(&o, 8, 1, stdout); fwrite(&a, 8, 1, stdout); }
  }
}
