#!/usr/bin/env python3
"""Emulation test of the Plaits mode's UI and persistence in Model-TG (repos/Model-TG, branch `macro`).

Calls the blob's real functions in Unicorn, as Modded-Cycles' interface tests do. Every call into the stock OS
(0x40000400..0x401aa140) is a stub that records its arguments and returns a scripted value; Model-TG's own
code runs as built. sound_obj(track) returns a fake 256-byte sound record per track.

  1. boot: boot_extra_hook zeroes 0x4019b590..0x423380b0 except the blob, whose data (mc_voice, mc_prm,
     mc_out...) survives intact, and jumps on to 0x4000053a
  2. packed state: state_store / st_apply round-trip for every mode (Plaits with OUT, MIX, AUX), with the other
     settings random; Plaits words carry tag 6, the others tag 7, and the other fields are kept
  3. a word stored by Model-TG 1.1.0 (tag 7) still decodes to the same settings
  4. name token: name_store writes 'M' for Plaits
  5. mode menu: mm_render draws 8 markers, the 8th filled, and "Plaits"; the encoder steps 0 -> 7 and stops
  6. labels: gran_watch says 7 for a Plaits track, apply_names puts Engine/Harmonics/Timbre/Morph (ENGN..MORP,
     category Plaits) on the four ids, with plain (not fine) Start/End values
  7. options: mo_v_out names OUT/MIX/AUX; mo_r_out turns mc_out 0..2, clamped, and stores it in the sound
"""
import pathlib, random, struct, subprocess, sys
from unicorn import Uc, UC_ARCH_M68K, UC_MODE_BIG_ENDIAN, UC_HOOK_CODE, UC_HOOK_MEM_UNMAPPED
from unicorn import m68k_const as mk

HERE = pathlib.Path(__file__).resolve().parent
TG = HERE.parent / "repos" / "Model-TG"
IMG = (TG / "build" / "Model-TG.bin").read_bytes()
BASE, OS_END, BLOB = 0x40000400, 0x401aa140, 0x401ab750
SOUNDS, STACK, STOP = 0x41f00000, 0x90010000, 0x9000fff0
FAIL = []


def check(ok, msg):
    print(("  ok    " if ok else "  FAIL  ") + msg, flush=True)
    if not ok:
        FAIL.append(msg)


def symbols():
    out = subprocess.run(["m68k-elf-nm", str(TG / "build" / "_b.elf")], capture_output=True, text=True).stdout
    return {p[2]: int(p[0], 16) for p in (l.split() for l in out.splitlines()) if len(p) == 3}


S = symbols()


class M:
    def __init__(self, copied=True):
        uc = self.uc = Uc(UC_ARCH_M68K, UC_MODE_BIG_ENDIAN)
        uc.ctl_set_cpu_model(mk.UC_CPU_M68K_ANY)
        uc.mem_map(0x40000000, 0x02400000)
        uc.mem_map(0x80000000, 0x00020000)
        uc.mem_map(0x90000000, 0x00020000)
        uc.mem_map(0x91000000, 0x00400000)            # scratch objects handed to the functions (this, events, ctx)
        uc.mem_write(0x91000000, struct.pack(">I", 0x91200000) * 0x40000)   # every pointer field -> more scratch
        uc.mem_write(BASE, IMG)
        uc.mem_map(0x46000000, 0x01000000)            # the Plaits C runs from plt_run (copied at boot)
        if copied:
            uc.mem_write(S["plt_run"], IMG[S["plt_load"] - BASE:S["plt_load"] - BASE + S["plt_size"]])
        uc.mem_write(STOP, b"\x4e\x71\x4e\x71")
        self.calls, self.ret, self.bad = [], {}, []
        uc.hook_add(UC_HOOK_MEM_UNMAPPED, lambda u, a, ad, s, v, d: self.bad.append(ad) or False)
        uc.hook_add(UC_HOOK_CODE, self._os, begin=BASE, end=OS_END - 1)
        uc.hook_add(UC_HOOK_CODE, self._sound_obj, begin=S["sound_obj"], end=S["sound_obj"])

    def r32(self, a):
        return struct.unpack(">I", bytes(self.uc.mem_read(a, 4)))[0]

    def w32(self, a, v):
        self.uc.mem_write(a, struct.pack(">I", v & 0xffffffff))

    def cstr(self, a):
        b = bytes(self.uc.mem_read(a, 64))
        return b[:b.index(0)].decode("latin-1") if 0 in b else None

    def _ret(self, d0):
        sp = self.uc.reg_read(mk.UC_M68K_REG_A7)
        self.uc.reg_write(mk.UC_M68K_REG_D0, d0 & 0xffffffff)
        self.uc.reg_write(mk.UC_M68K_REG_PC, self.r32(sp))
        self.uc.reg_write(mk.UC_M68K_REG_A7, sp + 4)

    def _os(self, uc, addr, size, _):
        if addr == self.stop_at:
            uc.emu_stop()
            return
        sp = uc.reg_read(mk.UC_M68K_REG_A7)
        self.calls.append((addr, [self.r32(sp + 4 + 4 * i) for i in range(8)]))
        r = self.ret.get(addr, 0)
        self._ret(r() if callable(r) else r)

    def _sound_obj(self, uc, addr, size, _):
        t = uc.reg_read(mk.UC_M68K_REG_D0)
        self._ret(SOUNDS + 0x100 * t if t < 6 else 0)

    stop_at = None

    def call(self, fn, *args, regs=None):
        sp = STACK - 0x400
        self.uc.mem_write(sp, struct.pack(">I" + "I" * len(args), STOP, *[a & 0xffffffff for a in args]))
        self.uc.reg_write(mk.UC_M68K_REG_A7, sp)
        for r, v in (regs or {}).items():
            self.uc.reg_write(r, v & 0xffffffff)
        self.uc.emu_start(S[fn] if isinstance(fn, str) else fn, STOP, count=2_000_000)
        return self.uc.reg_read(mk.UC_M68K_REG_D0)


def fields(m):
    out, a = [], S["st_fields"]
    while m.r32(a):
        out.append((m.r32(a), m.r32(a + 12)))      # (table, largest value)
        a += 16
    return out


def boot():
    print("1. boot clear")
    m = M(copied=False)
    lo, hi, end = 0x4019b590, 0x423380b0, S["reserved_end"]
    load = IMG[S["plt_load"] - BASE:S["plt_load"] - BASE + S["plt_size"]]
    junk = b"\xa5" * 0x10000
    for a in range(BASE + len(IMG), hi, 0x10000):               # past the image: whatever RAM held
        m.uc.mem_write(a, junk[:min(0x10000, hi - a)])
    blob = bytes(m.uc.mem_read(BLOB, end - BLOB))
    m.stop_at = 0x4000053a
    m.uc.reg_write(mk.UC_M68K_REG_A7, STACK - 0x400)
    h = S["boot_extra_hook"] - BASE
    mv = S["boot_extra_hook"] + IMG[h:h + 128].index(b"\x4e\x7b")   # movec %d4,%acr1: no cache model here
    m.uc.hook_add(UC_HOOK_CODE, lambda u, a, s, d: u.reg_write(mk.UC_M68K_REG_PC, a + 4), begin=mv, end=mv)
    try:
        m.uc.emu_start(S["boot_extra_hook"], 0x4000053a, count=50_000_000)
    except Exception as ex:
        check(False, f"boot hook ran: {ex}")
        return
    pc = m.uc.reg_read(mk.UC_M68K_REG_PC)
    kept = bytes(m.uc.mem_read(BLOB, end - BLOB)) == blob
    below = bytes(m.uc.mem_read(lo, BLOB - lo)) == bytes(BLOB - lo)
    above = all(not any(m.uc.mem_read(a, min(0x100000, hi - a))) for a in range(end, hi, 0x100000))
    run = S["plt_run"]
    data = all(run <= S[n] < run + len(load) for n in ("plt_voice", "plt_prm", "plt_out", "plt_trig", "plt_init",
                                                      "plt_fill")) and bytes(m.uc.mem_read(run, len(load))) == load
    above = above and not any(m.uc.mem_read(end, S["plt_load"] + len(load) - end))   # the load image: cleared
    check(pc == 0x4000053a and kept and below and above and data and not m.bad,
          f"0x4019b590..0x423380b0 zeroed except the assembly 0x{BLOB:x}..0x{end:x}, kept byte for byte; the "
          f"Plaits C ({len(load):,} B) copied to 0x{run:08x} first; continues at 0x4000053a")


def state():
    print("2-3. packed state")
    m = M()
    fl = fields(m)
    lm, mo = S["loop_mode"], S["plt_out"]
    rnd = random.Random(1)
    bad, n = [], 0
    for mode in range(8):
        for out in range(3):
            for trial in range(4):
                t = rnd.randrange(6)
                want = {tab: rnd.randint(0, mx) for tab, mx in fl}
                want[lm] = mode
                for tab, v in want.items():
                    m.w32(tab + 4 * t, v)
                m.w32(mo + 4 * t, out)
                m.call("state_store", regs={mk.UC_M68K_REG_D0: t})
                word = m.r32(SOUNDS + 0x100 * t + 72)
                tag = word >> 28
                for tab, _ in fl:                       # scramble, then decode
                    m.w32(tab + 4 * t, 0x55)
                m.w32(mo + 4 * t, 0x55)
                ok = m.call("st_apply", regs={mk.UC_M68K_REG_D2: t, mk.UC_M68K_REG_A2: SOUNDS + 0x100 * t})
                got = {tab: m.r32(tab + 4 * t) for tab, _ in fl}
                gout = m.r32(mo + 4 * t)
                n += 1
                if not (ok == 1 and tag == (6 if mode == 7 else 7) and got == want
                        and (mode != 7 or gout == out)):
                    bad.append((mode, out, hex(word), got.get(lm), gout))
    check(not bad and not m.bad, f"{n} round trips (8 modes x OUT/MIX/AUX x 4, other settings random): tag 6 for "
                                 f"Plaits, 7 otherwise, every field back" + (f"; {bad[:3]}" if bad else ""))
    # a 1.1.0 word: the same encoder minus the Plaits case, computed here from st_fields
    t, want = 2, {tab: rnd.randint(0, mx) for tab, mx in fl}
    want[lm] = 6
    acc = 0
    for tab, mx in reversed(fl):
        acc = acc * (mx + 1) + want[tab]
    m.w32(SOUNDS + 0x100 * t + 72, 0x70000000 | acc)
    m.call("st_apply", regs={mk.UC_M68K_REG_D2: t, mk.UC_M68K_REG_A2: SOUNDS + 0x100 * t})
    check({tab: m.r32(tab + 4 * t) for tab, _ in fl} == want, "a Model-TG 1.1.0 word (tag 7) decodes as before")


def token():
    print("4. name token")
    m = M()
    m.w32(S["loop_mode"], 7)
    m.call("name_store", regs={mk.UC_M68K_REG_D0: 0, mk.UC_M68K_REG_D1: 0x1234ABCD})
    name = bytes(m.uc.mem_read(SOUNDS + 4, 12))
    check(name == b"SMP1234ABCDM", f"name_store: {name!r}")


def menu():
    print("5. mode menu")
    m = M()
    m.w32(S["mm_kind"], 0)
    m.w32(S["mm_track"], 0)
    m.w32(S["loop_mode"], 7)
    m.call("mm_render", 0x91000000, 0x92000000)
    filled = [c for c in m.calls if c[0] == 0x40070efc]
    outl = [c for c in m.calls if c[0] == 0x40070c4e]
    texts = [m.cstr(c[1][6]) for c in m.calls if c[0] == 0x40071a04]
    xs = sorted(c[1][1] for c in filled + outl)
    check(len(filled) == 1 and len(outl) == 7 and filled[0][1][1] == max(xs) and max(xs) + 8 < 128,
          f"8 markers, the 8th filled, the last at x={max(xs)} (inside 128 px)")
    check("Plaits" in texts, f"the panel names the mode: {texts}")
    m = M()
    m.w32(S["mm_kind"], 0)
    m.w32(S["mm_track"], 0)
    m.w32(S["loop_mode"], 0)
    m.ret = {0x4006f6ce: 1, 0x4006f73a: 1}          # a knob event, one click right
    ev = 0x91100000
    m.w32(ev + 12, 2)
    seen = []
    for _ in range(10):
        m.call("mm_enc", 0x91000000, ev)
        seen.append(m.r32(S["loop_mode"]))
    check(seen == [1, 2, 3, 4, 5, 6, 7, 7, 7, 7] and not m.bad, f"turning right: modes {seen}")
    check(m.r32(SOUNDS + 72) >> 28 == 6, "mode_set stored the Plaits state (tag 6)")


def labels():
    print("6. page labels")
    m = M()
    m.w32(S["loop_mode"], 7)
    m.call("gran_watch")
    ui = m.r32(S["ui_gran"])
    check(ui == 7, f"gran_watch: ui_gran = {ui} for a Plaits track")
    m.call("apply_names", regs={mk.UC_M68K_REG_D1: 1})
    got = []
    for off in (2576, 3472, 4088, 4144):
        e = 0x4010dce0 + off
        got.append(tuple(m.cstr(m.r32(e + k)) for k in (44, 48, 52)))
    want = [("Engine", "Plaits", "ENGN"), ("Harmonics", "Plaits", "HARM"),
            ("Timbre", "Plaits", "TIMB"), ("Morph", "Plaits", "MORP")]
    check(got == want, f"labels {got}")
    check(m.r32(0x40a7296c) == 0x400456c8 and m.r32(0x40a72978) == 0,
          "Start/End values plain (value>>8, no fine draw) on a Plaits page")
    m.w32(S["loop_mode"], 6)
    m.call("gran_watch")
    m.call("apply_names", regs={mk.UC_M68K_REG_D1: 1})
    check(m.r32(S["ui_gran"]) == 4 and m.cstr(m.r32(0x4010dce0 + 2576 + 44)) == "Wave Pos"
          and m.r32(0x40a7296c) == 0x4004a440, "Wave mode keeps its labels and fine Start/End")


def options():
    print("7. options row")
    m = M()
    m.w32(S["gm_track"], 1)
    m.w32(S["loop_mode"] + 4, 7)
    names = []
    for o in range(3):
        m.w32(S["plt_out"] + 4, o)
        m.calls.clear()
        m.call("mo_v_out", 0x91000000, 0x91000100, 0x91000200, 0x91000300, 0x91000400)
        strs = [m.cstr(a) for _, args in m.calls for a in args[:2] if 0x40000000 <= a < 0x42400000]
        names.append(next((s for s in strs if s in ("OUT", "MIX", "AUX")), None))
    check(names == ["OUT", "MIX", "AUX"], f"mo_v_out: {names}")
    m.w32(S["plt_out"] + 4, 0)
    m.ret[0x4000d0dc] = 0x91300000                  # the screen's view: vtable +0x10 = redraw, a stub too
    m.w32(0x91300000, 0x91300100)
    m.w32(0x91300110, 0x40000500)
    seq = []
    for turn in (1, 1, 1, -1, -1, -1):
        m.call("mo_r_out", 0x91000000, 0x91000100, turn)
        seq.append(m.r32(S["plt_out"] + 4))
    word = m.r32(SOUNDS + 0x100 + 72)
    check(seq == [1, 2, 2, 1, 0, 0] and word >> 28 == 6 and not m.bad,
          f"mo_r_out: {seq}, stored in the sound (word {word:#010x})")
    d = S["gm_descs"] + 4 * 7
    desc = m.r32(d)
    check(desc == S["gm_desc_mc"] and m.cstr(m.r32(desc)) == "Plaits" and m.r32(desc + 8) == 1,
          "gm_descs[7] = the Plaits list, one row")


if __name__ == "__main__":
    for f in (boot, state, token, menu, labels, options):
        try:
            f()
        except Exception as ex:
            check(False, f"{f.__name__}: {type(ex).__name__}: {ex}")
    print("\nALL OK" if not FAIL else f"\n{len(FAIL)} FAILURE(S)")
    sys.exit(1 if FAIL else 0)
