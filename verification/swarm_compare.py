#!/usr/bin/env python3
"""(Swarm is chaotic: a grain re-randomises when its phase wraps, so the tiniest timing difference changes which
random number goes where, and float Plaits and double Plaits part ways the same way. So: the first 2,000 samples'
waveform, then the long-term 1/3-octave band levels over 2 s, ours and float Plaits each against double Plaits.)
The fixed-point SWARM (Model-TG src/macro/macro.c) against Plaits' SwarmEngine compiled in double (swarm_ref_d),
same knobs (k/127), f0, trigger at block 0, the same random sequence, 32-sample blocks. OUT goes through the limiter
on both sides (x 3, Plaits' -3); AUX x 1. Waveform and magnitude-spectrum differences under the reference RMS."""
import ctypes, itertools, pathlib, subprocess, sys, tempfile
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parent / "engines"
N = 96000
lib = pathlib.Path(tempfile.mkdtemp()) / "m.so"
subprocess.run(["cc", "-O2", "-w", "-shared", "-fPIC", "-DMACRO_SEL=MACRO_SWARM", "-DMACRO_MASK=0x200",
                str(SRC / "macro.c"), "-o", str(lib)], check=True)
m = ctypes.CDLL(str(lib))
m.macro_render.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.POINTER(ctypes.c_int16), ctypes.c_int]

def ours(inc, h, t, mo, aux):
    v = ctypes.create_string_buffer(4096)
    m.macro_init(v)
    buf = (ctypes.c_int16 * 32)()
    out = np.empty(N, np.int64)
    for b in range(0, N, 32):
        m.macro_render(v, bytes([4, h, t, mo, aux, 0, 0]), inc, buf, 32)
        out[b:b + 32] = buf[:]
    return out

def limiter(x, pre):                                    # stmlib::Limiter, as Plaits' voice applies it
    peak, y = 0.5, np.empty_like(x)
    for i, s in enumerate(x * pre):
        e = abs(s) - peak
        peak += (0.05 if e > 0 else 0.00002) * e
        y[i] = s * (1.0 if peak <= 1 else 1 / peak) * 0.8
    return y

def ref(note, h, t, mo, exe="swarm_ref_d"):
    k16 = lambda k: repr(((k * 33026) >> 6) / 65536)      # exactly the knob value the port uses
    r = subprocess.run([str(HERE / exe), str(note), k16(h), k16(t), k16(mo), str(N)],
                       capture_output=True, check=True)
    x = np.frombuffer(r.stdout, dtype="<f8").reshape(-1, 2)
    return float(r.stderr), limiter(x[:, 0], 3.0), x[:, 1]

def mag(z):
    return sum(np.abs(np.fft.rfft(z[k:k + 8192] * np.hanning(8192))) for k in (0, 7900, 15800))

quick = "--quick" in sys.argv
notes = [36, 60, 84] if quick else [24, 48, 60, 72, 96]
hs = [0, 64, 127] if quick else [0, 20, 50, 80, 110, 127]
ts = [0, 64, 127] if quick else [0, 30, 64, 90, 127]
ms = [0, 64, 127] if quick else [0, 20, 45, 64, 80, 100, 127]
def bands(z):
    """1/3-octave band levels (dB) of the long-term spectrum (Welch, 2048-point Hann frames, half overlap)"""
    fr = [z[k:k + 2048] * np.hanning(2048) for k in range(0, len(z) - 2048, 1024)]
    P = np.mean([np.abs(np.fft.rfft(f)) ** 2 for f in fr], axis=0)
    f = np.fft.rfftfreq(2048, 1 / 48000)
    edges = 31.25 * 2 ** (np.arange(0, 30) / 3)
    return np.array([10 * np.log10(P[(f >= lo) & (f < hi)].sum() + 1e-9) for lo, hi in zip(edges, edges[1:])])

def band_diff(x, y):
    bx, by = bands(x), bands(y)
    live = by > by.max() - 30                       # the bands within 30 dB of the loudest
    return np.abs(bx - by)[live].max(), abs(10 * np.log10(np.mean(x ** 2) / np.mean(y ** 2)))

rows = []
for note, h, t, mo in itertools.product(notes, hs, ts, ms):
    f0, ro, ra = ref(note, h, t, mo)
    _, fo, fa = ref(note, h, t, mo, "swarm_ref")
    inc = int(round(f0 * 2 ** 32))
    for name, r, fl, aux in (("OUT", ro, fo, 0), ("AUX", ra, fa, 127)):
        x = ours(inc, h, t, mo, aux).astype(float)
        y = np.clip(np.round(r * 32768), -32768, 32767)
        z = np.clip(np.round(fl * 32768), -32768, 32767)
        rms = np.sqrt(np.mean(y ** 2))
        if rms < 30:
            continue
        early = 20 * np.log10(max(np.sqrt(np.mean((x[:2000] - y[:2000]) ** 2)), 1e-9) / max(np.sqrt(np.mean(y[:2000] ** 2)), 1))
        bo, lo = band_diff(x, y)
        bf, lf = band_diff(z, y)
        rows.append((bo, bf, lo, lf, early, note, h, t, mo, name, rms))
A = np.array([r[:5] for r in rows])
print(f"{len(rows)} cases (2 s each). Against Plaits in double:")
print(f"  ours:        worst 1/3-octave band {A[:,0].max():.1f} dB (median {np.median(A[:,0]):.1f}), level {A[:,2].max():.2f} dB worst")
print(f"  float Plaits: worst 1/3-octave band {A[:,1].max():.1f} dB (median {np.median(A[:,1]):.1f}), level {A[:,3].max():.2f} dB worst")
print(f"  first 2,000 samples, ours: median {np.median(A[:,4]):.1f} dB, worst {A[:,4].max():.1f} dB")
for r in sorted(rows, key=lambda r: r[0] - r[1])[-5:]:
    print("   ours %5.1f dB vs float %5.1f dB (bands), level %.2f/%.2f, early %6.1f  note %d HARM %d TIMB %d MORP %d %s (rms %.0f)" % r)
