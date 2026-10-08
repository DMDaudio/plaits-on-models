// Plaits bench: renders the stock plaits::Voice on the ColdFire, driven by
// run.py through the bench_* globals. One call renders `blocks` blocks of
// kBlockSize samples of one engine; run.py counts instructions per block.
#include <cstddef>
#include <cstring>
#include "plaits/dsp/voice.h"

using namespace plaits;

extern "C" {
volatile int bench_engine = 8;        // Plaits engine index (voice.cc order)
volatile int bench_blocks = 400;
volatile int bench_trigger_every = 2000;  // blocks between triggers
volatile float bench_note = 48.0f;
volatile float bench_harmonics = 0.5f, bench_timbre = 0.5f, bench_morph = 0.5f;
Voice::Frame bench_out[48000];
volatile int bench_block_index;  // run.py watches writes to this
void* __dso_handle = 0;
int __cxa_atexit(void (*)(void*), void*, void*) { return 0; }
void __cxa_pure_virtual() { for (;;) {} }
__attribute__((noinline, used)) void bench_block_mark(int i) { bench_block_index = i; }
void bench_main();
}

static char ram_block[16384];
static Voice voice;

void bench_main() {
  stmlib::BufferAllocator allocator(ram_block, sizeof(ram_block));
  voice.Init(&allocator);
  bench_block_mark(-1);

  Patch patch;
  memset(&patch, 0, sizeof(patch));
  patch.engine = bench_engine;
  patch.note = bench_note;
  patch.harmonics = bench_harmonics;
  patch.timbre = bench_timbre;
  patch.morph = bench_morph;
  patch.decay = 0.5f;
  patch.lpg_colour = 0.5f;

  Modulations mod;
  memset(&mod, 0, sizeof(mod));
  mod.trigger_patched = true;

  for (int b = 0; b < bench_blocks; ++b) {
    mod.trigger = (b % bench_trigger_every) < 4 ? 1.0f : 0.0f;
    voice.Render(patch, mod, &bench_out[b * kBlockSize], kBlockSize);
    bench_block_mark(b);
  }
}
