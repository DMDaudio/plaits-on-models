#!/usr/bin/env python3
"""Run the Plaits bench (<build>/bench.bin) under Unicorn and count ColdFire
instructions per 12-sample block, per engine. Writes out/<engine>.wav.

usage: run.py [--blocks N] [--engines 0,8,21] [--note 48]
"""
import argparse, bisect, os, re, struct, subprocess, wave
from unicorn import Uc, UC_ARCH_M68K, UC_MODE_BIG_ENDIAN, UC_HOOK_BLOCK, UC_HOOK_CODE
from unicorn import m68k_const as mk

HERE = os.path.dirname(os.path.abspath(__file__))
ELF = BIN = None
BASE = 0x10000
ENGINES = ["VA-VCF", "PHASE-DIST", "6OP-A", "6OP-B", "6OP-C", "WAVE-TERRAIN",
           "STRING-MACH", "CHIPTUNE", "VA", "WAVESHAPE", "FM", "GRAIN",
           "ADDITIVE", "WAVETABLE", "CHORD", "SPEECH", "SWARM", "NOISE",
           "PARTICLE", "STRING", "MODAL", "BASS-DRUM", "SNARE", "HI-HAT"]
BLOCK = 12
CPU_HZ, SR = 250e6, 48000          # MCF54415 at 250 MHz, Plaits at 48 kHz
BUDGET = CPU_HZ / SR               # 5,208 cycles a sample for everything


def symbols():
    out = subprocess.run(["m68k-elf-nm", ELF], capture_output=True, text=True).stdout
    return {p[2]: int(p[0], 16) for p in (l.split() for l in out.splitlines()) if len(p) == 3}


def insn_starts():
    out = subprocess.run(["m68k-elf-objdump", "-d", ELF],
                         capture_output=True, text=True).stdout
    return sorted(int(m.group(1), 16) for m in re.finditer(r"^\s*([0-9a-f]+):\t", out, re.M))


def run(engine, blocks, note, sym, starts, timbre=0.5, harmonics=0.5, morph=0.5):
    uc = Uc(UC_ARCH_M68K, UC_MODE_BIG_ENDIAN)
    uc.ctl_set_cpu_model(CPU)
    uc.mem_map(0, 0x01100000)
    uc.mem_write(BASE, open(BIN, "rb").read())
    w32 = lambda n, v: uc.mem_write(sym[n], struct.pack(">i", v))
    wf = lambda n, v: uc.mem_write(sym[n], struct.pack(">f", v))
    w32("bench_engine", engine); w32("bench_blocks", blocks); w32("bench_trigger_every", 2000)
    wf("bench_note", note); wf("bench_harmonics", harmonics)
    wf("bench_timbre", timbre); wf("bench_morph", morph)

    count = [0]
    cache = {}

    def on_block(uc, addr, size, _):
        n = cache.get((addr, size))
        if n is None:
            n = cache[(addr, size)] = bisect.bisect_left(starts, addr + size) - bisect.bisect_left(starts, addr)
        count[0] += n

    marks = []
    uc.hook_add(UC_HOOK_BLOCK, on_block)
    uc.hook_add(UC_HOOK_CODE, lambda uc, a, s, _: marks.append(count[0]),
                begin=sym["bench_block_mark"], end=sym["bench_block_mark"])
    uc.emu_start(sym["_start"], sym["bench_done"])

    per_block = [b - a for a, b in zip(marks, marks[1:])]   # marks[0] = after Init
    raw = uc.mem_read(sym["bench_out"], blocks * BLOCK * 4)
    out = [struct.unpack(">h", raw[i:i + 2])[0] for i in range(0, len(raw), 4)]
    return marks[0], per_block, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--blocks", type=int, default=400)   # 100 ms
    ap.add_argument("--engines", default=",".join(map(str, range(24))))
    ap.add_argument("--note", type=float, default=48.0)
    ap.add_argument("--build", default="build", help="build (soft-float) or build-hf (FPU estimate)")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    global ELF, BIN, CPU
    ELF, BIN = f"{HERE}/{a.build}/bench.elf", f"{HERE}/{a.build}/bench.bin"
    CPU = mk.UC_CPU_M68K_CFV4E if a.build.endswith("hf") else mk.UC_CPU_M68K_ANY
    sym, starts = symbols(), insn_starts()
    os.makedirs(f"{HERE}/out", exist_ok=True)
    print(f"{'#':>2} {'engine':<13} {'init':>10} {'insn/smp avg':>12} {'peak':>7} "
          f"{'%CPU avg':>8} {'%CPU peak':>9} {'peak |x|':>8}")
    for e in map(int, a.engines.split(",")):
        init, pb, out = run(e, a.blocks, a.note, sym, starts)
        avg = sum(pb) / len(pb) / BLOCK
        peak = max(pb) / BLOCK
        print(f"{e:>2} {ENGINES[e]:<13} {init:>10,} {avg:>12,.0f} {peak:>7,.0f} "
              f"{100 * avg / BUDGET:>7.1f}% {100 * peak / BUDGET:>8.1f}% {max(map(abs, out)):>8}",
              flush=True)
        with wave.open(f"{HERE}/out/{e:02d}-{ENGINES[e]}{a.tag}.wav", "wb") as wf:
            wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(SR)
            wf.writeframes(struct.pack(f"<{len(out)}h", *out))


if __name__ == "__main__":
    main()
