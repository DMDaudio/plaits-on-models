#!/usr/bin/env python3
"""Emulation test of Model-TG's Macro mode (repos/Model-TG, branch `macro`).

Runs the real six-voice loop of the built MAIN OS (Modded-Cycles' mcengine: Unicorn, ColdFire, exact EMAC),
with track 1 on the Sampler in Macro mode, and checks for each engine:
  - the 2x buffer mc_fill wrote each block equals MACRO's own output (src/macro/macro.c built for this Mac,
    same knobs, same trig, same pitch), sample for sample: the glue hands the engine the right knobs, trig and
    pitch, and the C runs on the ColdFire as on the PC;
  - the track's output (after the decimator and amp stages) is sound, not silence or garbage;
  - no access outside emulated memory;
and measures the instructions a block costs.

    python3 test_macro_mode.py [--engines 0,1,..] [--blocks 120] [--wav DIR]
"""
import argparse, ctypes, os, pathlib, shutil, struct, subprocess, sys, tempfile
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
TG = ROOT / "repos" / "Model-TG"
EMU = ROOT / "repos" / "Modded-Cycles" / "tools" / "emu"
IMG = TG / "build" / "Model-TG.bin"         # the patched MAIN OS build.py writes
ELF = TG / "build" / "_b.elf"
ENGINES = ["WSHAPE", "FM", "NOISE", "PARTICLE", "BDRUM", "SNARE", "HIHAT", "GRAIN", "CHORDS", "SWARM",
           "WAVETABLE"]                                                                    # build.py's names
IDS = ["MACRO_WSH", "MACRO_FM", "MACRO_NOISE", "MACRO_PARTICLE", "MACRO_BD", "MACRO_SD", "MACRO_HH", "MACRO_GRAIN",
       "MACRO_CHORD", "MACRO_SWARM", "MACRO_WAVETABLE"]
SHOWN = {"FM": "2OP FM", "PARTICLE": "PARTCL", "WAVETABLE": "WAVES"}                # names on the knob
DEFAULT_SEL = "WSHAPE,FM,NOISE,BDRUM,GRAIN,CHORDS,SWARM,WAVETABLE"                                       # build.py's default
MC_MODE = 7
FAIL = []

# emac.py calls m68k-linux-gnu-objdump; Homebrew has m68k-elf-objdump
_shim = pathlib.Path(tempfile.mkdtemp())
(_shim / "m68k-linux-gnu-objdump").symlink_to(shutil.which("m68k-elf-objdump"))
os.environ["PATH"] = f"{_shim}:{os.environ['PATH']}"
sys.path.insert(0, str(EMU)); sys.path.insert(0, str(EMU.parent))
import emac                                 # noqa: E402
import mcengine as E                        # noqa: E402

# macro_render saves and restores ACCEXT01 around its filters; the exact model keeps each accumulator as one
# integer, so the extension word reads 0 and a write leaves the accumulators alone.
_slow = emac.EMAC._slow
def _slow_ext(self, uc, addr):
    sz, mn, ops = self.ops[addr]
    if mn == "movel" and "%accext" in ops:
        src, dst = [a.strip() for a in ops.split(",")]
        if src.startswith("%accext"):
            self.w(dst, 0)
        uc.reg_write(E.mk.UC_M68K_REG_PC, addr + sz)
        return
    _slow(self, uc, addr)
emac.EMAC._slow = _slow_ext
_inst = emac.EMAC.install
def _install(self, instrs):
    n = _inst(self, {a: v for a, v in instrs.items() if "%accext" not in v[2]})
    for a, v in instrs.items():
        if "%accext" in v[2]:
            self.ops[a] = v
            self.uc.hook_add(E.UC_HOOK_CODE, self._hook, begin=a, end=a)
            n += 1
    return n
emac.EMAC.install = _install


def check(ok, msg):
    print(("  ok    " if ok else "  FAIL  ") + msg, flush=True)
    if not ok:
        FAIL.append(msg)


def symbols():
    out = subprocess.run(["m68k-elf-nm", str(ELF)], capture_output=True, text=True, check=True).stdout
    return {p[2]: int(p[0], 16) for p in (l.split() for l in out.splitlines()) if len(p) == 3}


def host_macro(sel):
    """MACRO for this Mac (macro.c's portable arithmetic), with the same engines, as a ctypes library."""
    lib = pathlib.Path(tempfile.mkdtemp()) / "macro.so"
    subprocess.run(["cc", "-O2", "-w", "-shared", "-fPIC", "-I", str(TG / "src" / "macro"),
                    "-DMACRO_SEL=" + ",".join(IDS[ENGINES.index(e)] for e in sel),
                    "-DMACRO_MASK=%#x" % sum(1 << ENGINES.index(e) for e in sel),
                    str(TG / "src" / "macro" / "macro.c"), "-o", str(lib)], check=True)
    m = ctypes.CDLL(str(lib))
    m.macro_render.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32,
                               ctypes.POINTER(ctypes.c_int16), ctypes.c_int]
    return m


def inc_of(step):
    return (step * 357 + ((step * 52) >> 8)) & 0xffffffff


def reference(m, prm, trig_blocks, inc, blocks):
    v = ctypes.create_string_buffer(4096)
    m.macro_init(v)
    out = []
    buf = (ctypes.c_int16 * 32)()
    for b in range(blocks):
        if b in trig_blocks:
            m.macro_trig(v)
        m.macro_render(v, bytes(prm), inc[b] if isinstance(inc, list) else inc, buf, 32)
        out.append(np.array(buf[:], dtype=np.int64))
    return out


def run(engine_ix, blocks, sym, img, host, harm=64, timb=64, morph=64, note=60):
    e = E.Engine(img, extra_code=[(0x401ab750, sym["reserved_end"] - 0x401ab750)])
    # what boot_extra_hook does: the Plaits C, from its load image to plt_run; its EMAC instructions there
    pl = img[sym["plt_load"] - E.BASE:sym["plt_load"] - E.BASE + sym["plt_size"]]
    e.uc.mem_map(0x46000000, 0x01000000)
    e.uc.mem_write(sym["plt_run"], pl)
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as tf:
        tf.write(pl)
    e.emac.install(emac.disasm(tf.name, sym["plt_run"], sym["plt_run"], sym["plt_run"] + len(pl)))
    os.unlink(tf.name)
    e.uc.mem_map(0x40800000, 0x01800000)
    e.uc.mem_map(0x42400000, 0x00c00000)
    e.uc.mem_map(0x48000000, 0x08000000)
    e.uc.mem_map(0xfc078000, 0x1000)
    for t in range(6):
        e.uc.mem_write(0x40a78c08 + 4 * t, struct.pack(">I", 0x20000000))      # mixer gains: audible
        e.uc.mem_write(E.PARAMS + 0xe + t * 0x42 + 46, struct.pack(">hhh", 0, 32512, 0))
        e.set(t, machine=4, note=60, pitch=64, decay=60)
    engn = engine_ix * 8 + 4
    # track 0: the Sampler, Macro mode; Start/End/Filter/Res = color/shape/sweep/contour (+22..+28)
    e.set(0, machine=6, note=note, pitch=64, color=engn, shape=harm, sweep=timb, contour=morph,
          punch=0, gate=0, finetune=64, decay=110)
    e.uc.mem_write(sym["loop_mode"], struct.pack(">I", MC_MODE))
    e.solo(0)
    bufs, outs, cost, steps = [], [], [], []
    def on_fill(uc, a, s, u):           # mc_fill(track, step, buf): the step the render hands it
        sp = uc.reg_read(E.mk.UC_M68K_REG_A7)
        steps.append(struct.unpack(">I", bytes(uc.mem_read(sp + 8, 4)))[0])
    e.uc.hook_add(E.UC_HOOK_CODE, on_fill, begin=sym["plt_fill"], end=sym["plt_fill"])
    count = [0]
    e.uc.hook_add(E.UC_HOOK_CODE, lambda uc, a, s, u: count.__setitem__(0, count[0] + 1))
    for b in range(blocks):
        count[0] = 0
        o = e.block(1 if b == 1 else 0)[0]
        cost.append(count[0])
        raw = e.uc.mem_read(sym["sampler_buf"] + 32, 64 * 4)
        x = np.frombuffer(bytes(raw), dtype=">i4").astype(np.int64)
        bufs.append(x)
        outs.append(o)
    mode = struct.unpack(">I", e.uc.mem_read(sym["loop_mode"], 4))[0]
    prm = bytes(e.uc.mem_read(sym["plt_prm"], 4))
    return bufs, np.concatenate(outs), cost, e.unmapped, mode, prm, steps


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sel", default=DEFAULT_SEL, help="the build's --plaits-engines")
    ap.add_argument("--engines", help="knob zones to test (default: all of --sel)")
    ap.add_argument("--blocks", type=int, default=120)
    ap.add_argument("--wav")
    a = ap.parse_args()
    img = IMG.read_bytes()
    sym = symbols()
    sel = [x.strip().upper() for x in a.sel.split(",")]
    host = host_macro(sel)
    names = host.macro_engine_name
    host.macro_engine_of.restype = ctypes.c_int
    ename = (ctypes.c_char_p * 11).in_dll(host, "macro_engine_name")
    print(f"image {len(img):,} B, blob to 0x{sym['reserved_end']:08x}; plt_fill at 0x{sym['plt_fill']:08x}")
    for ei in (map(int, a.engines.split(",")) if a.engines else range(len(sel))):
        name = sel[ei]
        got_e = ename[host.macro_engine_of(ei * 8 + 4)].decode()
        check(got_e == SHOWN.get(name, name), f"knob zone {ei} ({ei * 8}-{ei * 8 + 7}) plays {got_e}")
        print(f"{name}")
        bufs, out, cost, unm, mode, prm, steps = run(ei, a.blocks, sym, img, host)
        check(len(steps) == a.blocks, f"mc_fill called every block ({len(steps)}); step {steps[0]:#x}"
              f"{' (constant)' if len(set(steps)) == 1 else f', {len(set(steps))} values'}")
        check(mode == MC_MODE, f"loop_mode stays Macro ({mode})")
        check(list(prm) == [ei * 8 + 4, 64, 64, 64], f"mc_prm from the dials: {list(prm)}")
        # the 2x buffer holds each sample twice, << 15
        got = [b[0::2] >> 15 for b in bufs]
        twice = all(np.array_equal(b[0::2], b[1::2]) for b in bufs)
        ref = reference(host, [ei * 8 + 4, 64, 64, 64, 0, 0, 0], {1}, [inc_of(x) for x in steps], a.blocks)
        # block 0 renders before the trig; the voice is rendered from block 0 on (Sampler not idle at boot)
        first_bad = next((b for b in range(a.blocks) if not np.array_equal(got[b], ref[b])), None)
        g, r = np.concatenate(got), np.concatenate(ref)
        dmax = int(np.abs(g - r).max())
        rpk = max(int(np.abs(r).max()), 1)
        check(twice and dmax * 3162 < rpk,      # within -70 dB of the PC's peak
              f"mc_fill's buffer vs MACRO on the PC, {a.blocks} blocks: "
              + ("identical" if first_bad is None else
                 f"max difference {dmax} LSB ({20 * np.log10(max(dmax, 1) / rpk):.0f} dB under its peak {rpk}), "
                 f"first at block {first_bad}"))
        peak = int(np.abs(out).max())
        check(peak > 1 << 20, f"track output: peak {peak:.3e}, rms {np.sqrt(np.mean(out.astype(float)**2)):.3e}")
        check(not unm, f"no access outside emulated memory" + ("" if not unm else f": {unm[:3]}"))
        play = cost[2:]
        print(f"        cost: {np.mean(play):,.0f} instructions a block avg, {max(play):,} peak "
              f"({100 * np.mean(play) / 166667:.1f}% / {100 * max(play) / 166667:.1f}% of 166,667 cycles), "
              f"the whole voice loop with one track")
        if a.wav:
            pathlib.Path(a.wav).mkdir(parents=True, exist_ok=True)
            E.wav(f"{a.wav}/macro-{ei}-{name.replace(' ', '')}.wav", out)
    print("\nALL OK" if not FAIL else f"\n{len(FAIL)} FAILURE(S)")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
