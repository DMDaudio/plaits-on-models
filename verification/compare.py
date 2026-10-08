#!/usr/bin/env python3
"""The fixed-point CHORD (Model-TG src/macro/macro.c) against Plaits' float ChordEngine (chord_ref), same knobs
(k/127), same f0, 32-sample blocks, after the knobs' one-pole has settled (0.5 s): the difference's level under
the reference's RMS, OUT and AUX.  usage: compare.py [--quick] [--wavetable | --va | --modal | --string] [--float]"""
import ctypes, itertools, pathlib, subprocess, sys, tempfile
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parent / "engines"
N = 48000
FALLBACK, CRASH = [], []
WT = "--wavetable" in sys.argv                                   # WAVETABLE instead of CHORD
VA = "--va" in sys.argv                                          # VA instead of CHORD
MODAL = "--modal" in sys.argv                                    # MODAL: a trig at 0 and at the half, compared after it
STRING = "--string" in sys.argv                                  # STRING: the same
SEL, MASK = (("MACRO_WAVETABLE", "0x400") if WT else ("MACRO_VA", "0x800") if VA else ("MACRO_MODAL", "0x1000") if MODAL
             else ("MACRO_STRING", "0x2000") if STRING else ("MACRO_CHORD", "0x100"))
EVERY = N // 64 if MODAL or STRING else 1 << 30                  # blocks between trigs
REF = ("wt_ref" if WT else "va_ref" if VA else "modal_ref" if MODAL else "string_ref" if STRING else "chord_ref") + ("" if "--float" in sys.argv else "_d")   # default: Plaits in double
lib = pathlib.Path(tempfile.mkdtemp()) / "m.so"
subprocess.run(["cc", "-O2", "-w", "-shared", "-fPIC", "-DMACRO_SEL=" + SEL, "-DMACRO_MASK=" + MASK, "-DCHORD_BLOCK=32", "-DCHORD_FROM_ZERO", "-DWT_BLOCK=32", "-DWT_FROM_ZERO", "-DMODAL_BLOCK=32", "-DMODAL_FROM_ZERO",
                str(SRC / "macro.c"), "-o", str(lib)], check=True)
m = ctypes.CDLL(str(lib))
m.macro_render.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.POINTER(ctypes.c_int16), ctypes.c_int]

def ours(inc, h, t, mo, aux):
    v = ctypes.create_string_buffer(65536)
    m.macro_init(v)
    ctypes.memmove(ctypes.addressof(v) + 4, (0x21).to_bytes(4, "little"), 4)   # the random numbers as Plaits' start
    buf = (ctypes.c_int16 * 32)()
    out = np.empty(N, np.int64)
    for b in range(0, N, 32):
        if (b // 32) % EVERY == 0:
            m.macro_trig(v)
        m.macro_render(v, bytes([4, h, t, mo, aux, 0, 0]), inc, buf, 32)
        out[b:b + 32] = buf[:]
    return out

def ref(note, h, t, mo):
    k16 = lambda k: repr(((k * 33026) >> 6) / 65536)      # exactly the knob value the port uses
    r = subprocess.run([str(HERE / REF), str(note), k16(h), k16(t), k16(mo), str(N)] + ([str(EVERY)] if MODAL or STRING else []), capture_output=True)
    if r.returncode:            # double Plaits reaching a grid position of exactly 7.0 reads past its wave bank
        r = subprocess.run([str(HERE / REF.replace("_d", "")), str(note), k16(h), k16(t), k16(mo), str(N)],
                           capture_output=True)
        if r.returncode:        # float Plaits too: the module reads out of its wave map here; nothing to compare
            CRASH.append((note, h, t, mo))
            return None
        FALLBACK.append((note, h, t, mo))
    x = np.frombuffer(r.stdout, dtype="<f8").reshape(-1, 2)
    return float(r.stderr), x[:, 0].astype(np.float64), x[:, 1].astype(np.float64)

quick = "--quick" in sys.argv
notes = [36, 60, 84] if quick else [24, 36, 48, 60, 72, 84, 96]
hs = [0, 64, 127] if quick else [0, 12, 30, 47, 64, 81, 100, 127]
ts = [0, 64, 127] if quick else [0, 20, 40, 64, 90, 127]
ms = [0, 40, 64, 70, 100, 127] if quick else [0, 20, 40, 55, 60, 62, 64, 66, 68, 70, 80, 100, 127]
worst, rows = [], []
for note, h, t, mo in itertools.product(notes, hs, ts, ms):
    got = ref(note, h, t, mo)
    if got is None:
        continue
    f0, ro, ra = got
    inc = int(round(f0 * 2 ** 32))
    for name, r, aux in (("OUT", ro, 0), ("AUX", ra, 127)):
        x = ours(inc, h, t, mo, aux)[N // 2:]
        y = np.clip(np.round(r[N // 2:] * (0.6 if WT else 0.8) * 32768), -32768, 32767)
        rms = np.sqrt(np.mean(y ** 2))
        if rms < 30:                     # silent (a voice above Nyquist, or AUX with nothing on it)
            continue
        d = np.sqrt(np.mean((x - y) ** 2))
        # phase-blind: the magnitude spectra (Hann, 8192 points, 3 frames), their difference's energy under the signal's
        def mag(z):
            return sum(np.abs(np.fft.rfft(z[k:k + 8192] * np.hanning(8192))) for k in (0, 7900, 15800))
        X, Y = mag(x), mag(y)
        spec = 20 * np.log10(max(np.sqrt(np.sum((X - Y) ** 2) / np.sum(Y ** 2)), 1e-9))
        rows.append((20 * np.log10(max(d, 1e-9) / rms), note, h, t, mo, name, rms, spec))
rows.sort()
db = np.array([r[0] for r in rows])
loud = [r for r in rows if r[6] > 3000]
ld = np.array([r[0] for r in loud])
print(f"louder than -21 dBFS RMS: {len(loud)} cases, median {np.median(ld):.1f} dB, worst {ld.max():.1f} dB, "
      f"worse than -40 dB: {(ld > -40).sum()}, worse than -30 dB: {(ld > -30).sum()}")
sp = np.array([r[7] for r in rows])
print(f"magnitude spectra (phase-blind): median {np.median(sp):.1f} dB, worst {sp.max():.1f} dB, worse than -30 dB: "
      f"{(sp > -30).sum()}, worse than -20 dB: {(sp > -20).sum()}")
for r in sorted(rows, key=lambda r: r[7])[-5:]:
    print("     spectrum %6.1f dB (waveform %5.1f)  note %d HARM %d TIMB %d MORP %d %s (rms %.0f)" % ((r[7], r[0]) + tuple(r[1:7])))
other = [r for r in rows if not (r[2] <= 12 and r[3] == 127)]
so = np.array([r[7] for r in other])
print(f"  excluding OCT/5 chords at the top inversion: {len(other)} cases, spectra median {np.median(so):.1f} dB, "
      f"worst {so.max():.1f} dB, worse than -30 dB: {(so > -30).sum()}")
for r in sorted(other, key=lambda r: r[7])[-6:]:
    print("     spectrum %6.1f dB (waveform %5.1f)  note %d HARM %d TIMB %d MORP %d %s (rms %.0f)" % ((r[7], r[0]) + tuple(r[1:7])))
if CRASH:
    print(f"  {len(CRASH)} settings skipped: Plaits itself (float and double) reads past its wave map, e.g. {CRASH[:3]}")
if FALLBACK:
    print(f"  {len(FALLBACK)} cases against float Plaits (double Plaits crashed): e.g. {FALLBACK[:2]}")
print(f"{len(rows)} cases: difference under the reference RMS: median {np.median(db):.1f} dB, "
      f"worst {db.max():.1f} dB, cases worse than -40 dB: {(db > -40).sum()}")
import collections
for key, idx in (("note", 1), ("MORP", 4), ("TIMB", 3)):
    g = collections.defaultdict(list)
    for r in rows:
        g[r[idx]].append(r[0])
    print(f"  by {key}: " + ", ".join(f"{k}: med {np.median(v):.0f} / worst {max(v):.0f} ({sum(x > -30 for x in v)} > -30)"
                                      for k, v in sorted(g.items())))
for r in rows[-8:]:
    print("  %6.1f dB  note %d HARM %d TIMB %d MORP %d %s (rms %.0f)" % r[:7])
