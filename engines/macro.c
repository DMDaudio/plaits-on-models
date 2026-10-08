/* Digi Mono's MACRO machine: engines ported from Plaits, in 32-bit integer arithmetic.
 *
 * The synthesis is Plaits' (github.com/pichenettes/eurorack, plaits/dsp), by Emilie Gillet, under the MIT
 * licence below; this file restates it in fixed point for a CPU without an FPU. Each engine says which
 * Plaits file it follows. Numbers: a phase is a 32-bit fraction of a cycle; a signal is Q15 (32768 = 1)
 * unless named Q16; a knob is 0..127 as the SRC page gives it.
 *
 * Copyright 2016 Emilie Gillet (the synthesis); the fixed-point port, the digi1_mods authors.
 * Model-TG changes: MACRO_SEL, the engines built in and their knob order (macro_engine_of, HAS()).
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
 * following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
 * EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "macro.h"
#include "macro_tables.h"

/* code run once a block or less (coefficients, starts): built for size, the mod's RAM is short */
#define COLD __attribute__((noinline, optimize("Os")))

/* ---- the multiply-accumulate unit ------------------------------------------------------------------- *
 * The filters multiply on the ColdFire's EMAC: signed fractional, truncating (MACSR 0x20), ACC0 only. Two
 * 32-bit products summed come out as (floor(a x / 2^23) + floor(b y / 2^23)) >> 8, which a PC build
 * computes in 64-bit C (the tests compare the two). A third product, 2^15 x 2^15, adds half the last bit:
 * rounded, not truncated (truncation's bias builds up in a resonant filter's states). macro_render() saves MACSR, ACC0, ACC1
 * (MODAL's filters use it too) and ACCEXT01 and puts them back, as Digi EQ does: the firmware's own audio code uses the unit. */
#if defined(__mcoldfire__)
static inline int32_t fmac2(int32_t a, int32_t x, int32_t b, int32_t y)        /* (a x + b y) / 2^31, rounded */
{
    int32_t r, h = 32768;                                   /* + h h = 2^30: half the result's last bit */
    __asm__ volatile ("mac.l %1,%2,%%acc0\n\tmac.l %3,%4,%%acc0\n\tmac.l %5,%5,%%acc0\n\tmovclr.l %%acc0,%0"
                      : "=d"(r) : "r"(a), "r"(x), "r"(b), "r"(y), "r"(h));
    return r;
}
static inline int32_t fmac1(int32_t a, int32_t x)                              /* a x / 2^31, rounded */
{
    int32_t r, h = 32768;
    __asm__ volatile ("mac.l %1,%2,%%acc0\n\tmac.l %3,%3,%%acc0\n\tmovclr.l %%acc0,%0"
                      : "=d"(r) : "r"(a), "r"(x), "r"(h));
    return r;
}
static inline int32_t fmac1t(int32_t a, int32_t x)                             /* a x / 2^31, truncated */
{
    int32_t r;
    __asm__ volatile ("mac.l %1,%2,%%acc0\n\tmovclr.l %%acc0,%0" : "=d"(r) : "r"(a), "r"(x));
    return r;
}
struct emac_save { int32_t macsr, acc0, acc1, ext01; };
static inline void emac_enter(struct emac_save *e)
{
    int32_t z = 0;
    __asm__ volatile ("move.l %%macsr,%0\n\tmove.l %%acc0,%1\n\tmove.l %%acc1,%2\n\tmove.l %%accext01,%3\n\t"
                      "move.l #0x20,%%macsr\n\tmove.l %4,%%acc0\n\tmove.l %4,%%acc1\n\tmove.l %4,%%accext01"
                      : "=&d"(e->macsr), "=&d"(e->acc0), "=&d"(e->acc1), "=&d"(e->ext01) : "d"(z));
}
static inline void emac_leave(const struct emac_save *e)
{
    __asm__ volatile ("move.l %0,%%acc0\n\tmove.l %1,%%acc1\n\tmove.l %2,%%accext01\n\tmove.l %3,%%macsr"
                      : : "d"(e->acc0), "d"(e->acc1), "d"(e->ext01), "d"(e->macsr));
}
#else
static inline int32_t fmac2(int32_t a, int32_t x, int32_t b, int32_t y)
{
    return (int32_t)(((((int64_t)a * x) >> 23) + (((int64_t)b * y) >> 23) + 128) >> 8);
}
static inline int32_t fmac1(int32_t a, int32_t x)
{
    return (int32_t)(((((int64_t)a * x) >> 23) + 128) >> 8);
}
static inline int32_t fmac1t(int32_t a, int32_t x)
{
    return (int32_t)((((int64_t)a * x) >> 23) >> 8);
}
struct emac_save { int32_t unused; };
static inline void emac_enter(struct emac_save *e) { (void)e; }
static inline void emac_leave(const struct emac_save *e) { (void)e; }
#endif

#define INC_MAX 0x40000000u                 /* Plaits' kMaxFrequency, 0.25 of the sample rate */
#define INC_MIN 4295u                       /* kMinFrequency, 1e-6                            */

/* ---- shared pieces ------------------------------------------------------------------------------- */

/* 0..127 -> Q16 (127 -> 65535) and Q15 */
static inline int32_t k16(int x) { return (x * 33026) >> 6; }
static inline int32_t k15(int x) { return (x * 33026) >> 7; }

static inline int32_t iabs(int32_t x) { return x < 0 ? -x : x; }

/* stmlib's integrated polyBLEP, Q16 in and out: NextIntegratedBlepSample(t) */
static inline int32_t iblep_next(int32_t t)
{
    int32_t t1 = t >> 1, t2 = (t1 * t1) >> 16, t4 = (t2 * t2) >> 16;
    return 12288 - t1 + ((3 * t2) >> 1) - t4;
}

/* t = d / inc as a Q16 fraction of a sample, d < inc */
static inline int32_t sub_sample(uint32_t d, uint32_t inc)
{
    if (d >= inc)                       /* (a step a knob moved behind the phase: at most a sample) */
        return 65536;
    if (inc < (1u << 24))
        return (int32_t)((d << 8) / ((inc >> 8) | 1));
    return (int32_t)(d / ((inc >> 16) | 1));
}

/* stmlib's InterpolateHermite over a Q15 table, index Q15 in 0..1 scaled by 512 (Plaits' fold tables) */
static inline int32_t hermite512(const int16_t *table, int32_t index)
{
    int32_t i = index >> 6, f = (index & 63) << 6;      /* f: Q12 */
    const int16_t *t = table + i;                        /* table + 1 + i - 1 */
    int32_t xm1 = t[0], x0 = t[1], x1 = t[2], x2 = t[3];
    int32_t c = (x1 - xm1) >> 1, v = x0 - x1, w = c + v;
    int32_t a = w + v + ((x2 - x0) >> 1), b_neg = w + a;
    return ((((((a * f) >> 12) - b_neg) * f >> 12) + c) * f >> 12) + x0;
}

/* Plaits' Sine(phase) over lut_sine (512 points a cycle), phase Q15 in 0..1.25 */
static inline int32_t sine15(int32_t ph)
{
    int32_t i = ph >> 6, f = ph & 63;
    int32_t a = MACRO_SINE[i], b = MACRO_SINE[i + 1];
    return a + (((b - a) * f) >> 6);
}

/* waveshaping_engine.cc's Tame(): how much of a control to keep as the fundamental rises, Q15 */
static COLD int32_t tame(int32_t f0_q16, int32_t mult_q8, int order)
{
    int32_t f = (f0_q16 * mult_q8) >> 8, max_f = 32768 / order, denom = 32768 - max_f, a;
    if (f <= max_f)
        return 32768;
    if (f - max_f >= denom)
        return 0;
    a = 32768 - (int32_t)(((uint32_t)(f - max_f) << 15) / (uint32_t)denom);
    return (((a * a) >> 15) * a) >> 15;
}

/* A linear ramp from a block's start value to its target (stmlib's ParameterInterpolator). Q23.
 * RAMP_NEW in *prev (an engine just started): no ramp, the block starts at the target. */
#define RAMP_NEW ((int32_t)0x80000000)
struct ramp { int32_t v, d; };
static inline void ramp_init(struct ramp *r, int32_t *prev, int32_t target, int n)
{
    if (*prev == RAMP_NEW)
        *prev = target;
    r->v = *prev * 256;
    r->d = ((target - *prev) * 256) / n;
    *prev = target;
}
static inline int32_t ramp_next(struct ramp *r) { r->v += r->d; return r->v >> 8; }

/* 2^t for t in 0..1 (Q16), as Q16 (65536..131071): a quartic, within 5e-6 (0.01 cent) */
static inline uint32_t exp2_frac(uint32_t t)
{
    return 65536 + ((t * (45416 + ((t * (15831 + ((t * (3392 + ((t * 897) >> 16))) >> 16))) >> 16))) >> 16);
}

/* 2^x, x Q16 (any sign), as Q16 */
static uint32_t exp2_q16(int32_t x)
{
    int32_t e = x >> 16;
    uint32_t y = exp2_frac((uint32_t)x & 0xffff);
    if (e >= 0)
        return e > 14 ? 0x7fffffffu : y << e;
    return e < -16 ? 0 : y >> -e;
}

/* log2(x), x > 0, as Q16; a quartic on the mantissa, within 0.00015 */
static int32_t log2_q16(uint32_t x)
{
    int32_t e = 31, m, q;
    if (!x)
        return -(32 << 16);
    while (!(x & 0x80000000u)) {
        x <<= 1;
        e--;
    }
    m = (int32_t)((x >> 16) & 0x7fff);                   /* the mantissa's fraction, Q15 */
    q = 20775 + ((m * -5263) >> 15);
    q = -44221 + ((m * q) >> 15);
    q = 94245 + ((m * q) >> 15);
    return (e << 16) + (int32_t)(((uint32_t)m * (uint32_t)q) >> 15);
}

/* The MIDI note of a phase increment, Q8 (note 69 = 440 Hz at 48 kHz) */
static COLD int32_t note_q8(uint32_t inc)
{
    return 69 * 256 + (((log2_q16(inc) - 1653513) * 3) >> 6);   /* 12 x 256 / 65536 = 3 / 64 */
}

/* a x b / 65536 for a phase increment a and a ratio b (Q16, up to 2^19) */
static uint32_t mul_inc(uint32_t a, uint32_t b)
{
    return (a >> 16) * b + (((a & 0xffff) * (b >> 4)) >> 12);
}

/* The phase increment whose log2 is x (Q16): 2^x, saturating at 0xffffffff (a frequency of 1) */
static uint32_t inc_of_log2(int32_t x)
{
    int32_t e = x >> 16;
    uint32_t y;
    if (x >= (32 << 16))
        return 0xffffffffu;
    y = exp2_frac((uint32_t)x & 0xffff);
    if (e >= 16)
        return y << (e - 16);
    return e < -1 ? 0 : y >> (16 - e);
}

/* ---- coefficients and the state-variable filter ----------------------------------------------------- *
 * A filter coefficient is a float of sorts: m x 2^-s, the mantissa m in 16384..32767, 11 <= s <= 31 (a value
 * from about 2^-17 to 16, coef_fit()). cmul() multiplies a signal (|x| < 2^27) by it to full precision, in two 16 x 16
 * products: no 64-bit arithmetic. */
struct coef { int32_t m, s; };

/* leading zeros of v > 0 (in C, not the CPU's FF1, which the emulators the tests use do not know) */
static int clz32(uint32_t v)
{
    int z = 0;
    if (!(v & 0xffff0000u)) { z += 16; v <<= 16; }
    if (!(v & 0xff000000u)) { z += 8; v <<= 8; }
    if (!(v & 0xf0000000u)) { z += 4; v <<= 4; }
    if (!(v & 0xc0000000u)) { z += 2; v <<= 2; }
    if (!(v & 0x80000000u)) z += 1;
    return z;
}

static struct coef coef_norm(uint32_t v, int32_t s)    /* v x 2^-s, any v; any s (not yet for cmul) */
{
    struct coef c;
    int z;
    if (!v) {
        c.m = 0;
        c.s = 31;
        return c;
    }
    z = clz32(v);
    c.m = (int32_t)((v << z) >> 17);
    c.s = s + z - 17;
    return c;
}

static struct coef coef_fit(struct coef c)             /* into cmul()'s range: 11 <= s <= 31 */
{
    if (c.s < 11) {
        c.m = 32767;
        c.s = 11;
    } else if (c.s > 31) {
        c.m = c.s - 31 > 15 ? 0 : c.m >> (c.s - 31);
        c.s = 31;
    }
    return c;
}

static struct coef coef_of_log2(int32_t x)             /* 2^x, x Q16 */
{
    return coef_norm(exp2_frac((uint32_t)x & 0xffff), 16 - (x >> 16));
}

static struct coef coef_mul(struct coef a, struct coef b)
{
    return coef_norm((uint32_t)(a.m * b.m), a.s + b.s);
}

static inline int32_t cmul(int32_t m, int32_t s, int32_t x)
{
    return ((m * (x >> 11)) >> (s - 11)) + ((m * (x & 0x7ff)) >> s);
}

static int32_t coef_q(struct coef c, int q)                    /* the value, Qq (below 2^31) */
{
    if (c.s >= q)
        return c.s - q > 31 ? 0 : c.m >> (c.s - q);
    return c.m << (q - c.s);
}

/* stmlib's Svf, in A. Simper's form of the same trapezoidal filter, on the EMAC (Q31 coefficients):
 *   v3 = in - ic2;  bp = v1 = a1 ic1 + a2 v3;  lp = v2 = ic2 + a2 ic1 + a3 v3;  ic1 = 2 v1 - ic1;
 *   ic2 = 2 v2 - ic2;  hp = in - k bp - lp
 * a1 = 1 / (1 + g (g + k)), a2 = g a1, a3 = g a2 (k = r = 1/q). Signals Q24 (|x| < 128); the states ic1,
 * ic2 are struct macro_svf's s1, s2. */
struct svf_c { int32_t a1, a2, a3, k2; };                           /* Q31; k2 = k / 2 */

static int32_t shift_q(int32_t t, int sh)                     /* t x 2^sh, sh -31..31 */
{
    if (sh >= 0)
        return t * (1 << sh);
    return sh < -31 ? 0 : t >> -sh;
}

/* a1 = 1/D to 31 bits (a hardware division to 16, one Newton step on the EMAC), a2 = g a1, a3 = g a2 with
 * the same g: a filter as stable as the float one up to q 512 at the top of the band (where 1 - |pole|^2
 * is 5e-4 and a1, a2, a3 rounded to 15 bits each were not). */
static COLD void svf_coefs_g(struct svf_c *c, struct coef g, struct coef r)
{
    int32_t g27, k27, d23, rc, e;
    g = coef_fit(g);
    r = coef_fit(r);
    g27 = coef_q(g, 27);
    k27 = coef_q(r, 27);
    d23 = (1 << 23) + fmac1(g27, g27 + k27);                         /* D = 1 + g (g + k), Q23 */
    rc = (int32_t)(0x7fffffffu / (uint32_t)(d23 >> 8)) << 15;          /* 1/D, Q31, 16 bits */
    if (rc <= 0)
        rc = 0x7fffffff;
    e = (1 << 23) - fmac1(d23, rc);                                  /* 1 - D/D', Q23 */
    e = fmac1(rc, e * 256);
    rc = e > 0 && rc > 0x7fffffff - e ? 0x7fffffff : rc + e;
    c->a1 = rc;
    c->a2 = shift_q(fmac1(rc, g.m << 16), 15 - g.s);                   /* g / D */
    c->a3 = shift_q(fmac1(c->a2, g.m << 16), 15 - g.s);                /* g^2 / D */
    c->k2 = r.s <= 13 ? 0x7fffffff : coef_q(r, 30);                    /* k / 2, Q31 (k = 2 at most) */
}

/* g = tan(pi f), FREQUENCY_ACCURATE (Plaits' polynomial, tabulated as tan(pi f) / f), f up to 0.5 */
static COLD struct coef tan_accurate(uint32_t finc)
{
    uint32_t i, f, t;
    if (finc > 0x80000000u)
        finc = 0x80000000u;
    i = finc >> 25;
    f = (finc >> 9) & 0xffff;
    t = MACRO_SVF_TAN[i] + (uint32_t)((((int32_t)MACRO_SVF_TAN[i + (i < 64)] - MACRO_SVF_TAN[i]) * (int32_t)f) >> 16);
    return coef_mul(coef_norm(finc, 32), coef_norm(t, 12));
}

/* g = tan(pi f), FREQUENCY_DIRTY: f (pi + 3.736e-1 pi^3 f^2), f up to 0.25 */
static COLD struct coef tan_dirty(uint32_t finc)
{
    uint32_t f16;
    if (finc > 0x40000000u)
        finc = 0x40000000u;
    f16 = finc >> 16;                                                   /* Q16, up to 16384 */
    return coef_mul(coef_norm(finc, 32), coef_norm(12868 + ((47452 * ((f16 * f16) >> 16)) >> 16), 12));
}

/* stmlib's Svf::set_f_q<FREQUENCY_ACCURATE>: the frequency as a phase increment, the damping r = 1/q */
static COLD void svf_coefs(struct svf_c *k, uint32_t finc, struct coef r)
{
    svf_coefs_g(k, tan_accurate(finc), r);
}

/* One sample: BP and LP (Q24) */
#define SVF_STEP(F, K, IN, BP, LP) do {                                                             \
        int32_t v3_ = (IN) - (F).s2;                                                               \
        BP = fmac2((K).a1, (F).s1, (K).a2, v3_);                                                   \
        LP = (F).s2 + fmac2((K).a2, (F).s1, (K).a3, v3_);                                          \
        (F).s1 = BP + BP - (F).s1;                                                                 \
        (F).s2 = LP + LP - (F).s2;                                                                 \
    } while (0)
#define SVF_HP(K, IN, BP, LP) ((IN) - 2 * fmac1((K).k2, BP) - (LP))


/* After a block: the states kept within +-32 (Q24), so nothing runs away out of range (Plaits' float filter
 * never needs it at the levels its engines feed it) */
static void svf_guard(struct macro_svf *f)
{
    const int32_t lim = 32 << 24;
    f->s1 = f->s1 > lim ? lim : f->s1 < -lim ? -lim : f->s1;
    f->s2 = f->s2 > lim ? lim : f->s2 < -lim ? -lim : f->s2;
}

/* stmlib's Limiter (as Plaits' voice applies it to the engines registered with a negative gain): a peak
 * follower (attack 0.05, release 0.00002 a sample) and 1/peak above 1. x: Q17 in, Q15 out (the 0.8 after
 * it is in the machine's gain). The peak is Q24: in Q17 the release step rounded to 15 % too fast. */
static void limit(int32_t *peak, int32_t *x, int n)
{
    int32_t pk = *peak, i;
    for (i = 0; i < n; i++) {
        int32_t s = x[i], err;
        if (s > (31 << 17))
            s = 31 << 17;
        if (s < -(31 << 17))
            s = -(31 << 17);
        err = (iabs(s) << 7) - pk;
        pk += fmac1(err, err > 0 ? 107374182 : 42950);         /* SLOPE(peak, |s|, 0.05, 0.00002) */
        if (pk <= (1 << 24))
            s >>= 2;
        else {                                                  /* s / peak: 1/peak Q31 (below 1), Q17 -> Q15 */
            uint32_t rp = 0x80000000u / (uint32_t)(pk >> 8);
            s = fmac1(s, rp >= 32768 ? 0x7fffffff : (int32_t)(rp << 16)) >> 2;
        }
        x[i] = s > 65535 ? 65535 : s < -65535 ? -65535 : s;     /* (clipped later; keeps the gain in range) */
    }
    *peak = pk;
}

/* a random 32-bit word (stmlib's Random::GetWord()) */
static inline uint32_t rnd32(uint32_t *rng)
{
    *rng = *rng * 1664525u + 1013904223u;
    return *rng;
}

/* a random Q15 value, -1..1 (stmlib's Random::GetFloat() x 2 - 1) */
static inline int32_t rnd15(uint32_t *rng)
{
    *rng = *rng * 1664525u + 1013904223u;
    return (int32_t)(*rng >> 16) - 32768;
}

/* Plaits' SinePM: the sine at a 32-bit phase (512 points a cycle, linear), Q15 */
static inline int32_t sin32(uint32_t ph)
{
    int32_t i = (int32_t)(ph >> 23), f = (int32_t)(ph >> 8) & 0x7fff;
    int32_t a = MACRO_SINE[i], b = MACRO_SINE[i + 1];
    return a + (((b - a) * f) >> 15);
}

/* ---- the slope oscillator: plaits/dsp/oscillator/oscillator.h, OSCILLATOR_SHAPE_SLOPE ------------ */

static COLD void slope_init(struct macro_slope *o)
{
    o->phase = 0x80000000u;
    o->next = 0;
    o->high = 1;
}

/* pw: Q16, already kept in 2f..1-2f. out: Q15. */
static void slope_render(struct macro_slope *__restrict o, uint32_t inc, int32_t pw, int32_t *__restrict out, int n)
{
    uint32_t pw32 = (uint32_t)pw << 16;
    int32_t rup = (int32_t)(0x80000000u / (uint32_t)pw);            /* 1/pw, Q15       */
    int32_t rdown = (int32_t)(0x80000000u / (uint32_t)(65536 - pw)); /* 1/(1-pw), Q15  */
    int32_t disc = (((rup + rdown) >> 7) * (int32_t)(inc >> 16)) >> 12;   /* (up+down) f, Q12 */
    uint32_t phase = o->phase;
    int32_t next = o->next;
    int high = o->high;
    while (n--) {
        uint32_t old = phase;
        int32_t this_s = next, t, b;
        next = 0;
        phase += inc;
        if (high && (phase < old || phase >= pw32)) {               /* the slope turns down */
            t = sub_sample(phase - pw32, inc);
            b = iblep_next(65536 - t);
            this_s -= (b * disc) >> 12;
            next -= (iblep_next(t) * disc) >> 12;
            high = 0;
        }
        if (phase < old) {                                          /* a new cycle: up again */
            t = sub_sample(phase, inc);
            this_s += (iblep_next(65536 - t) * disc) >> 12;
            next += (iblep_next(t) * disc) >> 12;
            high = 1;
        }
        if (high)
            next += (int32_t)(((phase >> 16) * (uint32_t)rup) >> 15);
        else
            next += 65536 - (int32_t)((((phase - pw32) >> 16) * (uint32_t)rdown) >> 15);
        *out++ = this_s - 32768;
    }
    o->phase = phase;
    o->next = next;
    o->high = high;
}

/* ---- WSH: plaits/dsp/engine/waveshaping_engine.cc ----------------------------------------------- */

static const int16_t *const ws_table[6] = {
    MACRO_WS_INVERSE_TAN, MACRO_WS_INVERSE_SIN, MACRO_WS_LINEAR, MACRO_WS_BUMP, MACRO_WS_DOUBLE_BUMP,
    MACRO_WS_DOUBLE_BUMP,
};

static COLD void wsh_init(struct macro_wsh *w)
{
    slope_init(&w->slope);
    slope_init(&w->tri);
    w->prev_shape = RAMP_NEW;
    w->prev_gain = RAMP_NEW;
    w->prev_overtone = RAMP_NEW;
}

/* Keep a slope oscillator's phase running over n samples without rendering them (the AUX path of an
 * engine while the AUX knob is at 0): the next sample starts from the plain slope, without a blep. */
static COLD void slope_skip(struct macro_slope *o, uint32_t inc, int n)
{
    o->phase += inc * (uint32_t)n;
    o->high = o->phase < 0x80000000u;
    o->next = o->high ? (int32_t)(o->phase >> 15) : 131072 - (int32_t)(o->phase >> 15);
}

static void wsh_render(struct macro_wsh *__restrict w, const uint8_t *p, uint32_t inc, int32_t *__restrict out, int32_t *__restrict aux, int n,
                       int want_out, int want_aux)
{
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k15(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]);
    int32_t f0 = (int32_t)(inc >> 16), pw, slope, amount, sa_att, wf_att, inner, o1, i;
    int16_t index[32];
    struct ramp shape, gain, overtone;

    pw = 32768 + ((morph * 29491) >> 16);                       /* morph * 0.45 + 0.5 */
    if (pw < 2 * f0)
        pw = 2 * f0;
    if (pw > 65536 - 2 * f0)
        pw = 65536 - 2 * f0;
    slope_render(&w->slope, inc, pw, out, n);
    if (want_aux)
        slope_render(&w->tri, inc, 32768, aux, n);
    else
        slope_skip(&w->tri, inc, n);

    slope = 768 + ((iabs(morph - 32768) * 5) >> 8);             /* 3 + |morph - 0.5| * 5, Q8 */
    amount = iabs(harm - 32768);                                /* |harmonics - 0.5| * 2, Q15 */
    sa_att = tame(f0, slope, 16);
    inner = 98304 + 5 * ((amount * sa_att) >> 15);             /* 3 + amount * att * 5, Q15 */
    wf_att = tame(f0, (slope * inner) >> 15, 12);

    ramp_init(&shape, &w->prev_shape, 16384 + ((((harm - 32768) >> 1) * sa_att) >> 15), n);
    ramp_init(&gain, &w->prev_gain, 983 + ((((timb * wf_att) >> 15) * 15073) >> 15), n);
    o1 = (timb * (65536 - timb)) >> 15;                         /* t (2 - t) */
    ramp_init(&overtone, &w->prev_overtone, (o1 * (65536 - o1)) >> 15, n);

    /* the waveshaper: the slope through two shape tables, crossfaded, times the folder's gain -> the
     * folder's index, which OUT and AUX share. The usual case, knobs not moving: the tables, the
     * crossfade and the gain are the block's. */
    if (shape.d == 0 && gain.d == 0) {
        int32_t s = (shape.v >> 8) * 4, sf, g = gain.v >> 8;
        const int16_t *s1, *s2;
        if (s > 131071)
            s = 131071;
        if (s < 0)
            s = 0;
        sf = s & 32767;
        s1 = ws_table[s >> 15];
        s2 = ws_table[(s >> 15) + 1];
        for (i = 0; i < n; i++) {
            int32_t idx = 127 * out[i] + (128 << 15), wi = (idx >> 15) & 255, wf = idx & 32767, x, y, ix;
            x = s1[wi] + (((s1[wi + 1] - s1[wi]) * wf) >> 15);
            y = s2[wi] + (((s2[wi + 1] - s2[wi]) * wf) >> 15);
            ix = (((x + (((y - x) * sf) >> 15)) * g) >> 15) + 16384;
            index[i] = (int16_t)(ix < 0 ? 0 : ix > 32767 ? 32767 : ix);
        }
    } else
    for (i = 0; i < n; i++) {
        int32_t s = ramp_next(&shape) * 4, si, sf, idx, wi, wf, x, y, mix, ix;
        const int16_t *s1, *s2;
        if (s > 131071)
            s = 131071;
        if (s < 0)
            s = 0;
        si = s >> 15;
        sf = s & 32767;
        s1 = ws_table[si];
        s2 = ws_table[si + 1];
        idx = 127 * out[i] + (128 << 15);
        wi = (idx >> 15) & 255;
        wf = idx & 32767;
        x = s1[wi] + (((s1[wi + 1] - s1[wi]) * wf) >> 15);
        y = s2[wi] + (((s2[wi + 1] - s2[wi]) * wf) >> 15);
        mix = x + (((y - x) * sf) >> 15);
        ix = ((mix * ramp_next(&gain)) >> 15) + 16384;
        index[i] = (int16_t)(ix < 0 ? 0 : ix > 32767 ? 32767 : ix);
    }
    if (want_out)
        for (i = 0; i < n; i++)
            out[i] = hermite512(MACRO_FOLD, index[i]);
    if (want_aux)
        for (i = 0; i < n; i++) {
            int32_t sine = sine15((aux[i] >> 2) + 16384), fold2 = -hermite512(MACRO_FOLD_2, index[i]);
            aux[i] = sine + (((fold2 - sine) * ramp_next(&overtone)) >> 15);
        }
}

/* ---- FM: plaits/dsp/engine/fm_engine.cc ----------------------------------------------------------- *
 * Plaits runs it 4x oversampled with an 8-tap decimator; so does a note here that starts with feedback
 * (fm_render4: Plaits' own arithmetic, it matches Plaits within rounding). A note without feedback runs
 * 2x (fm_render2) with the half-band [-1 0 9 16 9 0 -1] / 32 (shifts and adds only): the aliasing measured
 * within 0.5 dB of Plaits' at notes 84-96 and high indices, the treble a little brighter (-1.2 dB at 15 kHz
 * where Plaits' is -2.8), for half the work. With feedback, 2x is not the same: the carrier's highest
 * partials alias inside the feedback loop and change it (measured: up to +30 % level at full MORPH). */

static void fm_init(struct macro_fm *f)
{
    int i;
    f->carrier = f->modulator = f->sub = 0;
    f->prev = 0;
    for (i = 0; i < 6; i++)
        f->hc[i] = f->hs[i] = 0;
    f->car_fir = f->sub_fir = 0;
    f->prev_amount = f->prev_feedback = RAMP_NEW;
    f->os4 = 0;
}

/* At a note start: 4x for a note with feedback (MORPH off its middle), 2x without. Chosen per note, so a
 * MORPH turned during a note keeps the note's rate (no switch, no click); a p-lock lands on a trig. */
static COLD void fm_trig(struct macro_fm *f, const uint8_t *p)
{
    int32_t fb = k16(p[MACRO_P_MORPH]) - 32768;
    f->os4 = fb > 1024 || fb < -1024;
}

/* the half-band decimator: x[] holds six samples of history and then 2n new ones; out[i] is centred on
 * x[2i + 4] */
static void halfband(const int32_t *__restrict x, int32_t *__restrict out, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        const int32_t *c = x + 2 * i + 4;
        int32_t s1 = c[-1] + c[1];
        out[i] = (c[0] * 16 + s1 * 9 - (c[-3] + c[3])) >> 5;
    }
}

static COLD uint32_t fm_controls(struct macro_fm *f, const uint8_t *p, uint32_t inc, uint32_t c_inc, struct ramp *amount,
                            struct ramp *feedback, int n);

/* fm_render2's usual case: no feedback, OUT only. A step at a time, so that its values fit the registers */
static void fm2_plain(uint32_t *carp, uint32_t *modp, int32_t *prevp, uint32_t c_inc, uint32_t m_inc,
                      struct ramp *amount, int32_t *__restrict pc, int n)
{
    uint32_t car = *carp, mod = *modp;
    int32_t prev = *prevp, v = amount->v, d = amount->d, amt = 0, j;
    for (j = 0; j < 2 * n; j++) {
        int32_t m_, c_;
        if (!(j & 1)) {
            v += d;
            amt = v >> 8;
        }
        mod += m_inc;
        car += c_inc;
        m_ = sin32(mod);
        c_ = sin32(car + ((uint32_t)(amt * m_) << 3));
        prev += ((c_ - prev) * 3195) >> 15;
        *pc++ = c_;
    }
    amount->v = v;
    *carp = car;
    *modp = mod;
    *prevp = prev;
}

static void fm_render2(struct macro_fm *__restrict f, const uint8_t *p, uint32_t inc, int32_t *__restrict out, int32_t *__restrict aux, int n,
                       int want_aux)
{
    uint32_t c_inc = inc >> 1, m_inc;                   /* 2x oversampled: half the step */
    uint32_t car = f->carrier, mod = f->modulator, sub = f->sub;
    int32_t prev = f->prev, xc[64 + 6], xs[64 + 6], *pc = xc + 6, *ps = xs + 6, i, fast;
    struct ramp amount, feedback;
    m_inc = fm_controls(f, p, inc, c_inc, &amount, &feedback, n);
    for (i = 0; i < 6; i++) {
        xc[i] = f->hc[i];
        xs[i] = f->hs[i];
    }

    /* one oversampled step: the modulator (phase feedback PFB or self modulation MFB), the carrier, the
     * feedback's one-pole (0.05 a step at 4x = 0.0975 at 2x), the sub (Plaits' AUX) if wanted */
#define FM_STEP(PFB, MFB, SUB) do {                                                                \
        int32_t m_, c_;                                                                            \
        if (PFB)                                                                                   \
            mod += m_inc + (uint32_t)((int32_t)(m_inc >> 15) * ((prev * pfb) >> 15));              \
        else                                                                                       \
            mod += m_inc;                                                                          \
        car += c_inc;                                                                              \
        m_ = (MFB) ? sin32(mod + ((uint32_t)(mfb * prev) << 2)) : sin32(mod);                      \
        c_ = sin32(car + ((uint32_t)(amt * m_) << 3));                                             \
        prev += ((c_ - prev) * 3195) >> 15;                                                        \
        *pc++ = c_;                                                                                \
        if (SUB) {                                                                                 \
            sub += c_inc >> 1;                                                                     \
            *ps++ = sin32(sub + ((uint32_t)(amt * c_) << 1));        /* amount x carrier x 0.25 */ \
        }                                                                                          \
    } while (0)
#define FM_LOOP(PFB, MFB, SUB) do {                                                                \
        FM_STEP(PFB, MFB, SUB);                                                                    \
        FM_STEP(PFB, MFB, SUB);                                                                    \
    } while (0)
    /* MORPH still at its middle, OUT only (the usual 2x note): no feedback, no sub; else the general step,
     * every term (zeros where off; the sub then runs anyway) */
    fast = feedback.d == 0 && !want_aux && (feedback.v >> 8) <= 362 && (feedback.v >> 8) > -256;   /* fb^2 terms 0 */
    if (fast) {
        fm2_plain(&car, &mod, &prev, c_inc, m_inc, &amount, pc, n);
    } else {
        for (i = 0; i < n; i++) {
            int32_t amt = ramp_next(&amount), fb = ramp_next(&feedback);
            int32_t pfb = fb < 0 ? (fb * fb) >> 16 : 0;             /* phase feedback, 0.5 fb^2 */
            int32_t mfb = fb > 0 ? (fb * fb) >> 17 : 0;             /* self modulation, 0.25 fb^2 */
            FM_LOOP(1, 1, 1);
        }
    }
#undef FM_LOOP
#undef FM_STEP
    halfband(xc, out, n);
    for (i = 0; i < 6; i++)
        f->hc[i] = xc[2 * n + i];
    if (want_aux) {
        halfband(xs, aux, n);
        for (i = 0; i < 6; i++)
            f->hs[i] = xs[2 * n + i];
    } else if (fast) {
        sub += (c_inc >> 1) * 2 * (uint32_t)n;
    }
    f->carrier = car;
    f->modulator = mod;
    f->sub = sub;
    f->prev = prev;
}

/* the controls both rates share: the modulator's increment (at c_inc's rate), the amount and feedback ramps */
static COLD uint32_t fm_controls(struct macro_fm *f, const uint8_t *p, uint32_t inc, uint32_t c_inc, struct ramp *amount,
                            struct ramp *feedback, int n)
{
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k15(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]);
    int32_t ratio, hf, t2, mn, ri = harm >> 9, rf = (harm << 7) & 0xffff;
    uint32_t m_inc;
    ratio = MACRO_FM_RATIO[ri] + (((MACRO_FM_RATIO[ri + 1] - MACRO_FM_RATIO[ri]) * rf) >> 16);
    m_inc = mul_inc(c_inc, exp2_q16((ratio * 21845) >> 10));
    if (m_inc > 0x80000000u)
        m_inc = 0x80000000u;
    mn = note_q8(inc) - 24 * 256 + ratio;
    hf = 32768 - (((mn - 72 * 256) * 13107) >> 12);
    hf = hf < 0 ? 0 : hf > 32768 ? 32768 : hf;
    hf = (hf * hf) >> 15;
    t2 = (timb * timb) >> 15;
    ramp_init(amount, &f->prev_amount, (t2 * hf) >> 15, n);
    ramp_init(feedback, &f->prev_feedback, morph - 32768, n);
    return m_inc;
}

static void fm_render4(struct macro_fm *__restrict f, const uint8_t *p, uint32_t inc, int32_t *__restrict out, int32_t *__restrict aux, int n,
                       int want_aux)
{
    uint32_t c_inc = inc >> 2, m_inc;                   /* 4x oversampled: a quarter of the step */
    const int32_t f0 = MACRO_FIR4X[0], f1 = MACRO_FIR4X[1], f2 = MACRO_FIR4X[2], f3 = MACRO_FIR4X[3];
    uint32_t car = f->carrier, mod = f->modulator, sub = f->sub;
    int32_t prev = f->prev, chead = f->car_fir, shead = f->sub_fir, i;
    struct ramp amount, feedback;
    m_inc = fm_controls(f, p, inc, c_inc, &amount, &feedback, n);
#define FM_STEP(CA, CB, PFB, MFB, SUB) do {                                                        \
        int32_t m_, c_;                                                                            \
        if (PFB)                                                                                   \
            mod += m_inc + (uint32_t)((int32_t)(m_inc >> 15) * ((prev * pfb) >> 15));              \
        else                                                                                       \
            mod += m_inc;                                                                          \
        car += c_inc;                                                                              \
        m_ = (MFB) ? sin32(mod + ((uint32_t)(mfb * prev) << 2)) : sin32(mod);                      \
        c_ = sin32(car + ((uint32_t)(amt * m_) << 3));                                             \
        prev += ((c_ - prev) * 1638) >> 15;                         /* ONE_POLE 0.05 */            \
        chead += c_ * (CA);                                                                        \
        ctail += c_ * (CB);                                                                        \
        if (SUB) {                                                                                 \
            int32_t s_;                                                                            \
            sub += c_inc >> 1;                                                                     \
            s_ = sin32(sub + ((uint32_t)(amt * c_) << 1));                                         \
            shead += s_ * (CA);                                                                    \
            stail += s_ * (CB);                                                                    \
        }                                                                                          \
    } while (0)
#define FM_SAMPLE(PFB, MFB, SUB) do {                                                              \
        int32_t ctail = 0, stail = 0;                                                              \
        FM_STEP(f3, f0, PFB, MFB, SUB);                                                            \
        FM_STEP(f2, f1, PFB, MFB, SUB);                                                            \
        FM_STEP(f1, f2, PFB, MFB, SUB);                                                            \
        FM_STEP(f0, f3, PFB, MFB, SUB);                                                            \
        out[i] = chead >> 15;                                                                      \
        chead = ctail;                                                                             \
        if (SUB) {                                                                                 \
            aux[i] = shead >> 15;                                                                  \
            shead = stail;                                                                         \
        }                                                                                          \
        (void)stail;                                                                               \
    } while (0)
    for (i = 0; i < n; i++) {
        int32_t amt = ramp_next(&amount), fb = ramp_next(&feedback);
        int32_t pfb = fb < 0 ? (fb * fb) >> 16 : 0;
        int32_t mfb = fb > 0 ? (fb * fb) >> 17 : 0;
        if (want_aux)                               /* the general step: the same sums, with zeros */
            FM_SAMPLE(1, 1, 1);
        else                                        /* OUT: phase feedback or self modulation (or neither) */
            FM_SAMPLE(1, 1, 0);
    }
#undef FM_SAMPLE
#undef FM_STEP
    if (!want_aux)
        sub += (c_inc >> 1) * 4 * (uint32_t)n;
    f->carrier = car;
    f->modulator = mod;
    f->sub = sub;
    f->prev = prev;
    f->car_fir = chead;
    f->sub_fir = shead;
}

static void fm_render(struct macro_fm *f, const uint8_t *p, uint32_t inc, int32_t *out, int32_t *aux, int n,
                      int want_aux)
{
    if (f->os4)
        fm_render4(f, p, inc, out, aux, n, want_aux);
    else
        fm_render2(f, p, inc, out, aux, n, want_aux);
}

/* ---- NOISE: plaits/dsp/engine/noise_engine.cc ------------------------------------------------------- *
 * Two clocked noises (plaits/dsp/noise/clocked_noise.h) at TIMBRE's clock, the second's clock HARMONICS
 * apart; OUT: the first through a filter at the note's pitch, LP (HARMONICS 0) to BP to HP (1), MORPH its
 * resonance; AUX: two band-passes, at the note and HARMONICS (-2..+2 octaves) from it. A note start is
 * Plaits' trigger: both clocks restart, TIMBRE spans its patched range (-24..128). The filters' controls
 * are the block's (Plaits glides them over the block). */

static void cnoise_render(struct macro_cnoise *__restrict c, uint32_t *__restrict rng, int sync, uint32_t finc, int32_t *__restrict out, int n)
{
    uint32_t phase = c->phase;
    int32_t sample = c->sample, next = c->next, raw_amount = 0, i;
    if (finc > 0x40000000u) {                              /* clocks over 1/4 the rate: some raw noise */
        raw_amount = (int32_t)((finc - 0x40000000u) >> 15);
        if (raw_amount > 32768)
            raw_amount = 32768;
    }
    if (sync)
        phase = 0;
    for (i = 0; i < n; i++) {
        int32_t this_s = next, raw = 0;
        uint32_t old = phase;
        next = 0;
        if (raw_amount)
            raw = rnd15(rng);
        phase += finc;
        if (phase < old || sync) {                          /* a new value: a band-limited step */
            int32_t t = sync ? 65536 : sub_sample(phase, finc), u = 65536 - t, disc;
            sync = 0;
            if (!raw_amount)
                raw = rnd15(rng);
            disc = raw - sample;
            this_s += (disc * (((t >> 1) * (t >> 1)) >> 16)) >> 15;
            next -= (disc * (((u >> 1) * (u >> 1)) >> 16)) >> 15;
            sample = raw;
        }
        next += sample;
        out[i] = raw_amount ? this_s + ((((raw - this_s) >> 1) * raw_amount) >> 14) : this_s;
    }
    c->phase = phase;
    c->sample = sample;
    c->next = next;
}

static void noise_init(struct macro_noise *z)
{
    int i;
    for (i = 0; i < 2; i++) {
        z->src[i].phase = 0;
        z->src[i].sample = z->src[i].next = 0;
    }
    z->mm.s1 = z->mm.s2 = z->bp.s1 = z->bp.s2 = 0;
    z->sync = 0;
}

static void noise_render(struct macro_noise *__restrict z, uint32_t *__restrict rng, const uint8_t *p, uint32_t inc, int32_t *__restrict out,
                         int32_t *__restrict aux, int n, int want_aux)
{
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]);
    int32_t l0 = log2_q16(inc), lc, lv, cb, cx, use_hp, gm, gsh, i;
    int32_t nz[32];
    struct coef r, gain;
    struct svf_c k0, k1;
    int sync = z->sync;
    z->sync = 0;

    /* the clock: note TIMBRE x 152 - 24 (1653513: log2 of note 69's increment, Q16) */
    lc = 1653513 + (timb * 152 - 93 * 65536) / 12;
    cnoise_render(&z->src[0], rng, sync, inc_of_log2(lc), nz, n);

    /* q = 0.5 x 2^(10 MORPH): r = 1/q; the input's gain 1/sqrt((0.5 + q) x 40 x f0), at most 16 */
    r = coef_fit(coef_of_log2(65536 - 10 * morph));
    lv = log2_q16(65536 + exp2_q16(10 * morph)) - 17 * 65536 + 348779 + l0 - 32 * 65536;   /* 348779: log2 40 */
    lv = -(lv >> 1);
    gain = coef_fit(coef_of_log2(lv > 4 * 65536 ? 4 * 65536 : lv));
    gm = gain.m;
    gsh = gain.s - 9;                                       /* Q15 noise -> Q24 */

    /* the LP-to-HP mode: HARMONICS 0 LP, 0.5 BP, 1 HP; Q8 */
    if (harm <= 32768) {
        cb = harm >> 7;
        cx = 256 - cb;
        use_hp = 0;
    } else {
        cb = 512 - (harm >> 7);
        cx = 256 - (harm >> 7);
        use_hp = 1;
    }

    svf_coefs(&k0, inc, r);
    if (use_hp)
        for (i = 0; i < n; i++) {
            int32_t in = (gm * nz[i]) >> gsh, bp, lp;
            SVF_STEP(z->mm, k0, in, bp, lp);
            out[i] = ((bp >> 10) * cb + (SVF_HP(k0, in, bp, lp) >> 10) * cx) >> 5;   /* Q17 */
            nz[i] = bp;
        }
    else
        for (i = 0; i < n; i++) {
            int32_t in = (gm * nz[i]) >> gsh, bp, lp;
            SVF_STEP(z->mm, k0, in, bp, lp);
            out[i] = ((bp >> 10) * cb + (lp >> 10) * cx) >> 5;
            nz[i] = bp;
        }
    svf_guard(&z->mm);
    if (!want_aux)
        return;
    cnoise_render(&z->src[1], rng, sync, inc_of_log2(lc + 4 * harm - 2 * 65536), aux, n);
    svf_coefs(&k1, inc_of_log2(l0 + 4 * harm - 2 * 65536), r);
    for (i = 0; i < n; i++) {
        int32_t in = (gm * aux[i]) >> gsh, bp, lp;
        SVF_STEP(z->bp, k1, in, bp, lp);
        aux[i] = (nz[i] + bp) >> 7;
        (void)lp;
    }
    svf_guard(&z->bp);
}

/* ---- PARTICLE: plaits/dsp/engine/particle_engine.cc, plaits/dsp/noise/particle.h ---------------------- *
 * Six particles; each draws an impulse a sample with the probability TIMBRE's density sets, its height
 * random (0..1), into its own resonant band-pass (stmlib's Svf, FREQUENCY_DIRTY) at a frequency HARMONICS
 * spreads at random around the note (+-4 octaves at most), drawn again at the first impulse of a block.
 * OUT: their sum through a low-pass at the note; AUX: the impulses themselves. A note start makes every
 * particle fire at once (Plaits' trigger). MORPH: above its middle the band-passes' resonance; below it
 * Plaits adds a reverb-like diffuser (16 KB of delay a voice), which this port leaves out: there the
 * band-passes keep the resonance at the middle.
 *
 * Here the impulses come from exponential waiting times (the same Bernoulli process, drawn once an
 * impulse instead of every sample), and a particle whose band-pass has rung out and has no impulse in
 * the block is skipped. */

static void particles_init(struct macro_particles *z)
{
    int i;
    for (i = 0; i < MACRO_PARTICLES; i++) {
        struct macro_particle *q = &z->p[i];
        q->e = 1 << 26;
        q->s1 = q->s2 = 0;
        q->a1 = q->a2 = q->a3 = q->c1m = q->c2m = 0;
        q->c1s = q->c2s = 31;
    }
    z->post.s1 = z->post.s2 = 0;
    z->sync = 0;
}

/* an exponential waiting time, Q26: -ln(u), u uniform in (0, 1] */
static int32_t exp_wait(uint32_t *rng)
{
    uint32_t u = rnd32(rng) | 1;
    return ((32 << 16) - log2_q16(u)) * 710;                  /* x ln 2 x 2^10 */
}

/* a particle's band-pass at f (log2, Q16, 2^-16..0.25), damping k = 1/q; its input's gain 2^lpre. The
 * filter is linear, so an impulse x adds to the step without it: bp and lp by a2 x and a3 x, s1 and s2 by
 * twice those (c1, c2: pre_gain a2, pre_gain a3). Pre_gain itself reaches the thousands at low
 * frequencies, c1 and c2 stay small. */
static COLD struct coef coef_cap(struct coef c)             /* c x (Q15) >> (s - 9) stays a shift of 0..31 */
{
    if (c.s < 9) {
        c.m = 32767;
        c.s = 9;
    } else if (c.s > 40) {
        c.m = c.s - 40 > 15 ? 0 : c.m >> (c.s - 40);
        c.s = 40;
    }
    return c;
}

static COLD void particle_coefs(struct macro_particle *q, int32_t lf, struct coef k, int32_t lpre)
{
    struct svf_c c;
    struct coef pre = coef_of_log2(lpre), c1, c2;
    svf_coefs_g(&c, coef_fit(tan_dirty(inc_of_log2(lf + 32 * 65536))), k);
    q->a1 = c.a1;
    q->a2 = c.a2;
    q->a3 = c.a3;
    c1 = coef_cap(coef_mul(pre, coef_norm((uint32_t)c.a2, 31)));             /* pre_gain g h */
    c2 = coef_cap(coef_mul(pre, coef_norm((uint32_t)c.a3, 31)));             /* pre_gain g^2 h */
    q->c1m = c1.m;
    q->c1s = c1.s;
    q->c2m = c2.m;
    q->c2s = c2.s;
}

static void particle_render(struct macro_voice *__restrict m, const uint8_t *p, uint32_t inc, int32_t *__restrict out, int32_t *__restrict aux,
                            int n, int want_aux)
{
    struct macro_particles *z = &m->e.part;
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]);
    int32_t lf0 = log2_q16(inc) - 32 * 65536, ld, lq, lpre_base, spread, t2, pd, i, j;
    int32_t acc[32];
    struct coef k;
    struct svf_c kp;
    int sync = z->sync;
    z->sync = 0;

    /* density: NoteToFrequency(60 + 72 TIMBRE^2)^2 / 6 an impulse a sample; pd: Q26 */
    t2 = (int32_t)(((uint32_t)timb * (uint32_t)timb) >> 16);
    ld = 2 * (1653513 - 32 * 65536 + ((72 * t2 - 9 * 65536) / 12)) - 169408;  /* 169408: log2 6 */
    pd = (int32_t)inc_of_log2(ld + 26 * 65536);
    if (pd < 1)
        pd = 1;
    /* q = 0.5 + 2^(20 (MORPH - 0.5)) above the middle, 1.5 below */
    lq = log2_q16(32768 + (morph > 32768 ? exp2_q16(20 * (morph - 32768)) : 65536)) - 16 * 65536;
    k = coef_fit(coef_of_log2(-lq));
    /* pre_gain = 0.5 / sqrt(q f sqrt(density)): its log2 without f's part */
    lpre_base = -65536 - ((lq + (ld >> 1)) >> 1);
    spread = (int32_t)(((uint32_t)harm * (uint32_t)harm) >> 14);           /* 4 HARMONICS^2 octaves, Q16 */

    for (i = 0; i < n; i++)
        acc[i] = 0;
    if (want_aux)
        for (i = 0; i < n; i++)
            aux[i] = 0;
    for (j = 0; j < MACRO_PARTICLES; j++) {
        struct macro_particle *q = &z->p[j];
        struct macro_svf f;
        struct svf_c c;
        int32_t at, s;
        int fresh = 1;
        if (sync)
            q->e = 0;
        /* the first impulse this block, if any */
        at = q->e < pd * n ? q->e / pd : n;
        if (at >= n) {
            q->e -= pd * n;
            if (!q->s1 && !q->s2)
                continue;                                       /* silent: nothing to compute */
        }
        f.s1 = q->s1;
        f.s2 = q->s2;
        c.a1 = q->a1;
        c.a2 = q->a2;
        c.a3 = q->a3;
        i = 0;
        for (;;) {
            int32_t bp, lp, *a = acc + i, e = at < n ? at : n;
            for (; i < e; i++) {                                /* ringing, up to the next impulse */
                SVF_STEP(f, c, 0, bp, lp);
                *a++ += bp;
                (void)lp;
            }
            if (i >= n)
                break;
            /* an impulse */
            s = (sync && fresh) ? 32768 : (int32_t)(rnd32(&m->rng) >> 17);
            if (fresh) {                                        /* the band-pass's frequency for this block */
                int32_t u = (int32_t)(rnd32(&m->rng) >> 16) - 32768;      /* -1..1, Q15 */
                int32_t lf = lf0 + (((spread >> 2) * u) >> 13);
                lf = lf > -2 * 65536 ? -2 * 65536 : lf < -16 * 65536 ? -16 * 65536 : lf;   /* f: 2^-16..0.25 */
                particle_coefs(q, lf, k, lpre_base - (lf >> 1));
                c.a1 = q->a1;
                c.a2 = q->a2;
                c.a3 = q->a3;
                fresh = 0;
            }
            if (want_aux)
                aux[i] += s;
            q->e = exp_wait(&m->rng);
            at = q->e < pd * (n - i - 1) ? i + 1 + q->e / pd : n;
            if (at >= n)
                q->e -= pd * (n - i - 1);
            SVF_STEP(f, c, 0, bp, lp);
            lp = (q->c1m * s) >> (q->c1s - 9);                  /* Q15 -> Q24 */
            bp += lp;
            f.s1 += 2 * lp;
            lp = (q->c2m * s) >> (q->c2s - 9);
            f.s2 += 2 * lp;
            acc[i] += bp;
            i++;
        }
        svf_guard(&f);
        if (iabs(f.s1) < 512 && iabs(f.s2) < 512)
            f.s1 = f.s2 = 0;                                    /* rung out (-90 dB) */
        q->s1 = f.s1;
        q->s2 = f.s2;
    }
    if (want_aux)
        for (i = 0; i < n; i++)
            aux[i] = aux[i] > 65535 ? 65535 : aux[i];          /* (clipped later; keeps the gain in range) */

    /* the low-pass at the note, q 0.5 (FREQUENCY_DIRTY, f at most 0.49) */
    svf_coefs_g(&kp, tan_dirty(inc), coef_norm(2, 0));
    for (i = 0; i < n; i++) {
        int32_t bp, lp, x = acc[i];
        x = x > (64 << 24) ? 64 << 24 : x < -(64 << 24) ? -(64 << 24) : x;
        SVF_STEP(z->post, kp, x, bp, lp);
        out[i] = lp >> 6;                                       /* Q24 x 2 (Plaits' pre-gain) -> Q17 */
        (void)bp;
    }
    svf_guard(&z->post);
}

/* ---- shared by the drums ------------------------------------------------------------------------------ */

#define ONE_POLE(X, IN, C) ((X) += fmac1t((IN) - (X), (C)))        /* C: Q31 (truncated: a smoother) */
#define Q31(x) ((int32_t)((x) * 2147483648.0 + 0.5))                 /* a constant 0..1 as Q31 (compile time) */
#define Q24(x) ((int32_t)((x) * 16777216.0 + ((x) < 0 ? -0.5 : 0.5)))

/* y / (1 + |y|), Q24 in and out: tables (MACRO_RSAT_*), a division past |y| = 16 (rsat_far). Inline: it
 * runs a sample in the drums' loops. */
static COLD int32_t rsat_far(int32_t u)
{
    return 32768 - (int32_t)(0x7fffffffu / (uint32_t)(((1 << 24) + (u > 0x7e000000 ? 0x7e000000 : u)) >> 8));
}

static int32_t rsat24(int32_t y)
{
    int32_t u = iabs(y), r, i, f;
    if (u < (1 << 24)) {
        i = u >> 18;
        f = (u >> 2) & 0xffff;
        r = MACRO_RSAT_FINE[i] + (((MACRO_RSAT_FINE[i + 1] - MACRO_RSAT_FINE[i]) * f) >> 16);
    } else if (u < (16 << 24)) {
        u -= 1 << 24;
        i = u >> 20;
        f = (u >> 4) & 0xffff;
        r = MACRO_RSAT_COARSE[i] + (((MACRO_RSAT_COARSE[i + 1] - MACRO_RSAT_COARSE[i]) * f) >> 16);
    } else {
        r = rsat_far(u);
    }
    r <<= 9;
    return y < 0 ? -r : r;
}

/* stmlib's SoftClip, Q24 (inline: a sample in the drums' loops) */
static inline __attribute__((always_inline)) int32_t softclip24(int32_t x)
{
    int32_t u = iabs(x), r, i, f;
    if (u >= (3 << 24))
        r = 1 << 24;
    else {
        u = fmac1(u, Q31(1.0 / 3.0)) << 7;                  /* x / 3, Q31 */
        i = u >> 24;
        f = (u >> 8) & 0xffff;
        r = (MACRO_SOFTCLIP[i] + (((MACRO_SOFTCLIP[i + 1] - MACRO_SOFTCLIP[i]) * f) >> 16)) << 9;
    }
    return x < 0 ? -r : r;
}

/* A drum's sound has died away when every state that feeds its output is below -96 dB (Q24): from then on
 * until the next trig its output is silence and nothing of it is computed. */
#define TINY(x) ((uint32_t)((x) + 256) < 512u)
/* (a resonator rounding to a fixed point a few hundred LSBs off zero at a low frequency: -72 dB counts) */
#define TINYR(x) ((uint32_t)((x) + 4096) < 8192u)

/* AnalogBassDrum's Diode(): x, or 0.7 x 2x / (1 + |2x|) below 0 (Q24) */
static inline int32_t diode24(int32_t x)
{
    if (x >= 0)
        return x;
    return fmac1(rsat24(x < -(60 << 24) ? -(120 << 24) : 2 * x), Q31(0.7));
}

/* g = tan(pi f) for f in Q31 (<= 0.5), as Q27: FREQUENCY_DIRTY f (pi + 3.736e-1 pi^3 f^2) and FREQUENCY_FAST
 * f (pi + f^2 (3.260e-1 pi^3 + 1.823e-1 pi^5 f^2)) */
static int32_t tan_dirty27(int32_t f31)
{
    return fmac1(f31, 421657428 + fmac1(1554770775, fmac1(f31, f31)));
}

static int32_t tan_fast27(int32_t f31)
{
    int32_t f2 = fmac1(f31, f31);
    int32_t inner26 = 678339498 + (fmac1(f2, 1871914135) << 1);              /* 10.108 + 55.787 f^2, Q26 */
    return fmac1(f31, 421657428 + (fmac1(f2, inner26) << 1));
}

/* 1/x for x >= 1 (Qq, q 16..27) as Q31: a hardware division to 16 bits and a Newton step to 31 */
static int32_t recip_q(int32_t x, int q)
{
    uint32_t rr = (0x7fffffffu / (uint32_t)(x >> (q - 15))) << 15;
    int32_t r = rr > 0x7fffffffu ? 0x7fffffff : (int32_t)rr, e;
    e = (1 << q) - fmac1(x, r);
    e = fmac1(r, e * (1 << (31 - q)));
    return e > 0 && r > 0x7fffffff - e ? 0x7fffffff : r + e;
}
#define recip27(x) recip_q((x), 27)

/* An Svf's coefficients (struct svf_c, k2 left out) from g and g k (Q27): a1 = 1/D to 31 bits (the margin to instability, 2 g k / D, is 4e-5 at the longest
 * decays), a2 = g a1, a3 = g a2. */
static COLD void svf_coefs_gk(int32_t g27, int32_t gk27, int32_t *a1, int32_t *a2, int32_t *a3)
{
    int32_t r = recip_q((1 << 24) + (fmac1(g27, g27) << 1) + (gk27 >> 3), 24);   /* 1 / (1 + g^2 + g k), D < 128 */
    *a1 = r;
    *a2 = fmac1(r, g27) << 4;
    *a3 = fmac1(*a2, g27) << 4;
}

/* k = 1 / (1 + q f) (qf: q f, Q8), Q31 */
static COLD int32_t k_of_qf(int32_t qf8)
{
    int32_t den = 256 + (qf8 < 0 || qf8 > 0x3fffffff ? 0x3fffffff : qf8), z = clz32((uint32_t)den);
    uint32_t kk = 0x7fffffffu / ((uint32_t)(den << (z - 1)) >> 15);
    kk = z >= 8 ? kk << (z - 8) : kk >> (8 - z);
    return kk > 0x7fffffffu ? 0x7fffffff : (int32_t)kk;
}

/* A resonator: FREQUENCY_DIRTY at f (Q31, <= 0.4), q = 1 + q f */
static void reso_coefs(int32_t f31, int32_t qf8, int32_t *a1, int32_t *a2, int32_t *a3)
{
    int32_t g27 = tan_dirty27(f31);
    svf_coefs_gk(g27, fmac1(g27, k_of_qf(qf8)), a1, a2, a3);
}

/* stmlib's OnePole (FREQUENCY_FAST at f): G = g / (1 + g), Q31; lp = s + G (in - s), s = 2 lp - s */
static COLD int32_t onepole_G(int32_t f31)
{
    return 0x7fffffff - recip27((1 << 27) + tan_fast27(f31));
}
#define ONEPOLE_LP(S, G, IN, LP) do {                                                              \
        LP = (S) + fmac1((IN) - (S), (G));                                                         \
        (S) = LP + LP - (S);                                                                       \
    } while (0)

/* ---- BD: plaits/dsp/engine/bass_drum_engine.cc, drums/analog_bass_drum.h, drums/synthetic_bass_drum.h --- *
 * OUT: the analog-style drum (a pulse into a resonator that its own output and an attack pulse bend in
 * pitch, then the overdrive); AUX: the synthetic one (a distorted sine with a pitch envelope, a click and
 * noise). HARM: attack FM, self FM, drive; TIMB: tone; MORP: decay. Accent is Plaits' unpatched 0.8; the
 * drums are triggered (Plaits' patched trigger), never free-running. The resonator's pitch is updated
 * every sample during the attack's pitch sweep (the first 7 ms), then every 8 samples (Plaits: every
 * sample; measured: the decay's level within 0.3 dB); its other arithmetic is Plaits', in Q24. */

static void bd_init(struct macro_bd *d)
{
    int32_t *w = &d->trig, *end = (int32_t *)(d + 1);
    while (w < end)
        *w++ = 0;
}

#ifndef BD_EVERY
#define BD_EVERY 7                                    /* the resonator's pitch: every 8 samples after the attack */
#endif
static void bd_analog(struct macro_bd *__restrict d, int32_t harm, int32_t timb, int32_t morph, uint32_t inc, int32_t *__restrict out,
                      int n)
{
    int32_t f0 = (int32_t)(inc >> 1), lf0 = log2_q16(inc) - 32 * 65536, afm27, sfm31, q8, scale, tone_f, leak, i;
    struct svf_c c;
    /* HARMONICS: attack FM 1.7 min(4h, 1) (Q27), self FM 0.08 clamp(4h - 1, 0, 1) (Q31) */
    afm27 = harm >= 16384 ? Q31(0.85) >> 3 : fmac1(harm << 17, Q31(0.85)) >> 3;
    sfm31 = harm <= 16384 ? 0 : harm >= 32768 ? Q31(0.08) : fmac1((harm - 16384) << 17, Q31(0.08));
    /* q = 1500 x 2^(80 MORPH / 12), x 256 */
    q8 = (int32_t)inc_of_log2(691454 + (morph * 20) / 3 + 8 * 65536);           /* 691454: log2 1500 */
    /* scale = 0.001 / f0, at most 5, Q28 */
    scale = (int32_t)inc_of_log2(28 * 65536 - 653118 - lf0);                  /* 653118: -log2 0.001 */
    if (scale > (5 << 28) || scale <= 0)
        scale = 5 << 28;
    /* tone: min(4 f0 2^(9 TIMBRE), 1), Q31; the exciter's leak 0.08 (TIMBRE + 0.25), Q31 */
    {
        uint32_t t = inc_of_log2(lf0 + 33 * 65536 + 9 * timb);
        tone_f = t > 0x7fffffffu ? 0x7fffffff : (int32_t)t;
    }
    leak = (timb + 16384) * 2621;
    if (d->trig) {
        d->pulse_left = 48;                                 /* 1 ms */
        d->fm_left = 288;                                   /* 6 ms */
        d->lp_out = 0;
    }
    c.a1 = d->a1;
    c.a2 = d->a2;
    c.a3 = d->a3;
    {
        int32_t xin[32], xleak[32], fml[32], active, sweep = 0, fixed = 0;
        int32_t pl = d->pulse_left, fl = d->fm_left, pu = d->pulse, plp = d->pulse_lp, rt = d->retrig, fmlp = d->fm_lp;
        int32_t s1 = d->res.s1, s2 = d->res.s2, lpo = d->lp_out, tl = d->tone_lp, bp, lp;
        /* the trigger pulse, the FM pulse and the retrigger pulse (while they last) */
        active = pl || fl || !TINY(pu) || !TINY(plp) || rt || fmlp > 16;
        if (active && !pl && !fl && TINY(pu) && TINY(plp) && fmlp <= 16) {
            /* only the retrigger pulse's slow tail: the input alone */
            pu = plp = fmlp = 0;
            for (i = 0; i < n; i++) {
                rt = (uint32_t)(rt + 4096) < 8192u ? 0 : fmac1(rt, Q31(1.0 - 1.0 / 2400.0));
                xin[i] = fmac1(-fmac1(rt, Q31(0.2)), scale) << 3;
                xleak[i] = 0;
                fml[i] = 0;
            }
        } else if (active) {
            for (i = 0; i < n; i++) {
                int32_t pulse, fm_pulse = 0;
                if (pl) {
                    pl--;
                    pulse = pl ? Q24(8.6) : Q24(7.6);          /* 3 + 7 accent, accent 0.8 */
                    pu = pulse;
                } else {
                    pu = fmac1t(pu, Q31(1.0 - 1.0 / 9.6));
                    pulse = pu;
                }
                ONE_POLE(plp, pulse, Q31(1.0 / 4.8));
                pulse = diode24(pulse - plp + fmac1(pulse, Q31(0.044)));
                if (fl) {
                    fl--;
                    fm_pulse = 0x7fffff00 >> 7;                 /* 1, Q24 */
                    rt = fl ? 0 : -Q24(0.8);
                } else
                    rt = (uint32_t)(rt + 4096) < 8192u ? 0 : fmac1(rt, Q31(1.0 - 1.0 / 2400.0));
                ONE_POLE(fmlp, fm_pulse, Q31(1.0 / 4.8));
                fml[i] = fmlp;
                xin[i] = fmac1(pulse - fmac1(rt, Q31(0.2)), scale) << 3;
                xin[i] = xin[i] > Q24(100.0) ? Q24(100.0) : xin[i] < -Q24(100.0) ? -Q24(100.0) : xin[i];
                xleak[i] = fmac1(pulse, leak);
            }
            sweep = fmlp > Q24(0.004) || fl;
        } else
            pu = plp = rt = fmlp = 0;
        d->pulse_left = pl;
        d->fm_left = fl;
        d->pulse = pu;
        d->pulse_lp = plp;
        d->retrig = rt;
        d->fm_lp = fmlp;
        /* the resonator's pitch: from the attack FM and the self FM (its own output); without self FM and
         * after the attack, the block's */
        if (!sfm31 && !active) {
            int32_t f27 = f0 >> 4;
            f27 = f27 > (Q31(0.4) >> 4) ? Q31(0.4) >> 4 : f27 < 16 ? 16 : f27;
            reso_coefs(f27 << 4, fmac1(f27 << 4, q8), &c.a1, &c.a2, &c.a3);
            fixed = 1;
        }
        for (i = 0; i < n; i++) {
            if (!fixed && (!(i & BD_EVERY) || (sweep && !(i & 1) && fml[i] > Q24(0.004)))) {
                int32_t lo = lpo > Q24(12.0) ? Q24(12.0) : lpo < -Q24(12.0) ? -Q24(12.0) : lpo;
                int32_t punch = Q24(0.7) + diode24(10 * lo - (1 << 24));
                int32_t m27, f27;
                if (punch > (100 << 24))
                    punch = 100 << 24;
                m27 = (active ? fmac1(fml[i] << 7, afm27) : 0) + (fmac1(punch, sfm31) << 3);
                f27 = (f0 >> 4) + fmac1(f0, m27);
                f27 = f27 > (Q31(0.4) >> 4) ? Q31(0.4) >> 4 : f27 < 16 ? 16 : f27;
                reso_coefs(f27 << 4, fmac1(f27 << 4, q8), &c.a1, &c.a2, &c.a3);
            }
            {
                struct macro_svf f;
                f.s1 = s1;
                f.s2 = s2;
                SVF_STEP(f, c, active ? xin[i] : 0, bp, lp);
                s1 = f.s1;
                s2 = f.s2;
            }
            lpo = lp;
            ONE_POLE(tl, (active ? xleak[i] : 0) + bp, tone_f);
            out[i] = tl;
        }
        d->res.s1 = s1;
        d->res.s2 = s2;
        d->lp_out = lpo;
        d->tone_lp = tl;
    }
    svf_guard(&d->res);
    d->a1 = c.a1;
    d->a2 = c.a2;
    d->a3 = c.a3;
}

/* SyntheticBassDrum's DistortedSine: a triangle bent by t / (1 + |t|), towards a clean sine as
 * dirtiness falls; the phase jittered by the phase noise. Q24 */
static int32_t distorted_sine(uint32_t phase, int32_t pnoise, int32_t dirt31)
{
    int32_t t, tri, sine, clean;
    phase += (uint32_t)(fmac1(pnoise, dirt31) * 256);
    t = (int32_t)(phase < 0x80000000u ? phase : 0u - phase);     /* 0..0.5, Q32 */
    tri = (int32_t)((uint32_t)t >> 6) - (1 << 24);                /* 4 t - 1, Q24 */
    sine = 2 * rsat24(tri);
    clean = sin32(phase + 0xc0000000u) << 9;
    return sine + fmac1(clean - sine, 0x7fffffff - dirt31);
}

static void bd_synthetic(struct macro_bd *__restrict d, uint32_t *__restrict rng, int32_t harm, int32_t timb, int32_t morph, uint32_t inc,
                         int32_t *__restrict aux, int n)
{
    int32_t f0 = (int32_t)(inc >> 1), lf0 = log2_q16(inc) - 32 * 65536, m2, dirt31, fm_amt, fmd, fm_decay, body_decay;
    int32_t tone_f, tone15, i, df;
    uint32_t t;
    m2 = (int32_t)(((uint32_t)morph * (uint32_t)morph) >> 16);               /* decay^2, Q16 */
    /* dirtiness (0.4 - 0.25 MORPH^2) max(1 - 8 f0, 0), Q31 */
    df = 0x7fffffff - (f0 > (0x7fffffff >> 3) ? 0x7fffffff : f0 * 8);
    dirt31 = fmac1((Q31(0.4) >> 0) - (m2 * 8192), df);
    /* the FM envelope: amount min(2h, 1) x 3.5 (Q27), decay max(2h - 1, 0)^2 */
    fm_amt = harm >= 32768 ? Q31(0.4375) : fmac1(harm << 16, Q31(0.4375));   /* 3.5/8: Q28 of 3.5 x amount */
    fmd = harm <= 32768 ? 0 : (harm - 32768) * 2;                             /* Q16 */
    fmd = (int32_t)(((uint32_t)fmd * (uint32_t)fmd) >> 16);
    /* 1 - 1 / (384 (1 + 4 fmd^2)) */
    fm_decay = 0x7fffffff - (int32_t)(0x7fffffffu / (uint32_t)(384 + ((384 * 4 * fmd) >> 16)));
    /* 1 - 2^(-5 MORPH^2) / 960 */
    body_decay = 0x7fffffff - fmac1((int32_t)exp2_q16(-5 * m2) << 14, 4473924);   /* 4473924: 2^32 / 960 */
    t = inc_of_log2(lf0 + 33 * 65536 + 9 * timb);
    tone_f = t > 0x7fffffffu ? 0x7fffffff : (int32_t)t;
    tone15 = timb << 15;                                                      /* transient level, Q31 */
    if (d->trig) {
        d->fm = 0x7fffff00 >> 7;
        d->body = d->trans = Q24(0.86);
        d->body_pw = 48;
        d->fm_pw = 62;
    }
    for (i = 0; i < n; i++) {
        int32_t body, transient, mix, x, c_in, bp, lp;
        struct svf_c ck;
        ONE_POLE(d->pnoise, (int32_t)(rnd32(rng) >> 8) - Q24(0.5), Q31(0.002));
        if (d->fm_pw) {
            d->fm_pw--;
            d->phase = 0x40000000u;
        } else {
            uint32_t step;
            d->fm = fmac1t(d->fm, fm_decay);
            /* min(f0 (1 + 3.5 amount fm_lp), 0.5) */
            step = (uint32_t)(f0 >> 3) + (uint32_t)fmac1(f0, fmac1(d->fm_lp2 << 7, fm_amt));   /* Q28 */
            d->phase += step > 0x08000000u ? 0x80000000u : step << 4;
        }
        if (d->body_pw)
            d->body_pw--;
        else {
            d->body = fmac1t(d->body, body_decay);
            d->trans = fmac1t(d->trans, Q31(1.0 - 1.0 / 240.0));
        }
        ONE_POLE(d->body_lp, d->body, Q31(0.1));
        ONE_POLE(d->trans_lp, d->trans, Q31(0.1));
        ONE_POLE(d->fm_lp2, d->fm, Q31(0.1));
        body = distorted_sine(d->phase, d->pnoise, dirt31);
        /* the click: SLOPE (0.5 up, 0.1 down), a one-pole high-pass (0.04), a low-pass at 5 kHz, q 2 */
        c_in = d->body_pw ? 0 : 0x7fffff00 >> 7;
        x = c_in - d->click_lp;
        d->click_lp += fmac1(x, x > 0 ? Q31(0.5) : Q31(0.1));
        ONE_POLE(d->click_hp, d->click_lp, Q31(0.04));
        ck.a1 = 1671397352;
        ck.a2 = 567202663;
        ck.a3 = 192484965;
        SVF_STEP(d->click, ck, d->click_lp - d->click_hp, bp, lp);
        (void)bp;
        /* the noise: band-limited by two one-poles */
        ONE_POLE(d->noise_lp, (int32_t)(rnd32(rng) >> 8), Q31(0.05));
        ONE_POLE(d->noise_hp, d->noise_lp, Q31(0.005));
        transient = lp + d->noise_lp - d->noise_hp;
        /* TransistorVCA: s = (body - 0.6) gain; 3 s / (2 + |s|) + 0.3 gain */
        x = fmac1(body - Q24(0.6), d->body_lp << 7);
        mix = -(3 * rsat24(x >> 1) + fmac1(d->body_lp, Q31(0.3)));
        mix -= fmac1(fmac1(transient, d->trans_lp << 7), tone15);
        ONE_POLE(d->tone_lp2, mix, tone_f);
        aux[i] = d->tone_lp2;
    }
    svf_guard(&d->click);
}

/* Plaits' Overdrive (on OUT): drive 0.5 + 0.5 max(2h - 1, 0) max(1 - 16 f0, 0) */
static void bd_overdrive(int32_t harm, uint32_t inc, int32_t *__restrict x, int n)
{
    int32_t f0 = (int32_t)(inc >> 1), dv, d2, pa, pb, pre, sq, arg, sc, post, i;
    int32_t lim = f0 > (0x7fffffff >> 4) ? 0 : 0x7fffffff - f0 * 16;
    dv = harm <= 32768 ? 0 : fmac1((harm - 32768) << 9, lim);                 /* Q24 */
    dv = Q24(0.5) + (dv >> 1);
    d2 = fmac1(dv << 7, dv << 7);                                             /* Q31 */
    pa = dv >> 1;                                                             /* Q24 */
    pb = fmac1(fmac1(d2, d2), dv) * 24;                                       /* 24 dv^5, Q24 */
    pre = pa + fmac1(pb - pa, d2);
    sq = fmac1(dv << 7, (Q24(2.0) - dv) << 6) >> 6;                           /* dv (2 - dv), Q24 */
    arg = Q24(0.33) + fmac1(sq << 6, pre - Q24(0.33)) * 2;
    sc = softclip24(arg);
    post = (int32_t)(0x7fffffffu / (uint32_t)(sc >> 7)) << 10;                /* 1/sc, Q24 */
    for (i = 0; i < n; i++) {
        int32_t v = fmac1(x[i], pre << 2);                                    /* pre x, Q19 */
        v = softclip24((v > (7 << 18) ? 7 << 18 : v < -(7 << 18) ? -(7 << 18) : v) << 5);
        x[i] = fmac1(v, post << 5) << 2;
    }
}

static void bd_render(struct macro_voice *m, const uint8_t *p, uint32_t inc, int32_t *out, int32_t *aux, int n,
                      int want_out, int want_aux)
{
    struct macro_bd *d = &m->e.bd;
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]), i;
    if (d->trig)
        d->idle[0] = d->idle[1] = 0;
    if (want_out) {
        if (d->idle[0])
            for (i = 0; i < n; i++)
                out[i] = 0;
        else {
            bd_analog(d, harm, timb, morph, inc, out, n);
            bd_overdrive(harm, inc, out, n);
            for (i = 0; i < n; i++)
                out[i] >>= 9;                               /* Q15 */
            d->idle[0] = !d->pulse_left && !d->fm_left && TINY(d->pulse) && TINY(d->pulse_lp) && TINY(d->retrig)
                         && TINYR(d->res.s1) && TINYR(d->res.s2) && TINYR(d->tone_lp);
        }
    }
    if (want_aux) {
        if (d->idle[1])
            for (i = 0; i < n; i++)
                aux[i] = 0;
        else {
            bd_synthetic(d, &m->rng, harm, timb, morph, inc, aux, n);
            for (i = 0; i < n; i++)
                aux[i] >>= 9;
            d->idle[1] = !d->body_pw && TINY(d->body) && TINY(d->body_lp) && TINY(d->trans_lp) && TINY(d->tone_lp2);
        }
    }
    d->trig = 0;
}

/* ---- SD: plaits/dsp/engine/snare_drum_engine.cc, drums/analog_snare_drum.h, drums/synthetic_snare_drum.h ---- *
 * OUT: the analog-style snare (a pulse into five resonant modes, 808-like up to TIMBRE 2/3 then more modes,
 * soft-clipped, plus band-passed noise); AUX: the synthetic one (two coupled distorted sines and filtered
 * noise with their own envelopes). HARM: snappy (noise against shell); TIMB: tone (OUT), FM (AUX); MORP:
 * decay. Accent 0.8, triggered. */

static void sd_init(struct macro_sd *d)
{
    int32_t *w = &d->trig, *end = (int32_t *)(d + 1);
    while (w < end)
        *w++ = 0;
    d->key_knobs = -1;
}

static const int32_t sd_ratio28[5] = {268435456, 536870912, 853624750, 1116691497, 1508608262};   /* 1, 2, 3.18, 4.16, 5.62 */

static void sd_analog(struct macro_sd *__restrict d, uint32_t *__restrict rng, int32_t harm, int32_t timb, int32_t morph, uint32_t inc,
                      int32_t *__restrict out, int n)
{
    int32_t f0 = (int32_t)(inc >> 1), dxt, snappy, leak31, gain28[5], one_minus_snappy, ned, i, j, nm;
    /* decay_xt = d (1 + d (d - 1)), Q16 */
    dxt = (int32_t)(((uint32_t)morph * (uint32_t)(65536 + (int32_t)(((uint32_t)morph * (uint32_t)morph) >> 16) - morph)) >> 16);
    if (d->key_inc != inc || d->key_knobs != morph) {                     /* the resonators' coefficients */
        int32_t q8 = (int32_t)inc_of_log2(718654 + (dxt * 7) + 8 * 65536);   /* 2000 x 2^(84 dxt / 12), x 256 */
        int32_t fn;
        d->key_inc = inc;
        d->key_knobs = morph;
        for (j = 0; j < 5; j++) {
            int32_t f = fmac1(f0, sd_ratio28[j]), g27;                     /* Q28 */
            f = f > (Q31(0.499) >> 3) ? Q31(0.499) : f << 3;
            g27 = tan_fast27(f);
            svf_coefs_gk(g27, fmac1(g27, k_of_qf(fmac1(f, j ? q8 >> 2 : q8))), &d->ma1[j], &d->ma2[j], &d->ma3[j]);
        }
        fn = f0 > Q31(0.499) / 16 ? Q31(0.499) : f0 * 16;
        {
            int32_t g27 = tan_fast27(fn);
            svf_coefs_gk(g27, fmac1(g27, k_of_qf(fmac1(fn, 384))), &d->na1, &d->na2, &d->na3);
        }
    }
    /* noise envelope: 1 - 0.0017 x 2^(-MORPH (50 + 10 HARM) / 12) */
    ned = 0x7fffffff - fmac1((int32_t)exp2_q16(-(int32_t)(((uint32_t)(morph >> 4) * (uint32_t)((50 * 65536 + 10 * harm) / 12)) >> 12)) << 14,
                             7301444);                                       /* 0.0017 x 2^32 */
    /* exciter leak: snappy (2 - snappy) 0.1 (the raw HARMONICS) */
    leak31 = fmac1(harm << 15, (131072 - harm) * 1638) * 2;
    /* snappy = clamp(1.1 HARM - 0.05, 0, 1), Q31 */
    {
        int32_t sn = harm + ((harm * 3277) >> 15) - 3277;                        /* Q16 */
        sn = sn < 0 ? 0 : sn > 65535 ? 65535 : sn;
        snappy = sn << 15;
    }
    one_minus_snappy = 0x7fffffff - snappy;
    /* the modes' gains, Q28 */
    if (timb < 43691) {                                                    /* 808-style: two modes */
        int32_t t = (int32_t)(((uint32_t)timb * 3) >> 1);                    /* tone x 1.5, Q16 */
        int32_t u = 65536 - t;
        gain28[0] = (3 << 27) + (int32_t)((((uint32_t)u >> 1) * ((uint32_t)u >> 1) >> 14) * 18432);   /* 1.5 + 4.5 (1 - t)^2 */
        gain28[1] = (t << 13) + (int32_t)(0.15 * 268435456.0);              /* 2 t + 0.15 */
        gain28[2] = gain28[3] = gain28[4] = 0;
        nm = 2;
    } else {
        int32_t t = (timb - 43691) * 3;                                     /* Q16 */
        t = t > 65535 ? 65535 : t;
        gain28[0] = (3 << 27) - (t << 11);                                  /* 1.5 - 0.5 t */
        gain28[1] = (int32_t)(2.15 * 268435456.0) - fmac1(t << 15, (int32_t)(0.7 * 268435456.0) );
        gain28[2] = t << 12;
        t = (int32_t)(((uint32_t)t * (uint32_t)t) >> 16);
        gain28[3] = t << 12;
        t = (int32_t)(((uint32_t)t * (uint32_t)t) >> 16);
        gain28[4] = t << 12;
        nm = 5;
    }
    if (d->trig) {
        d->pulse_left = 48;
        d->noise_env = Q24(2.0);
    }
    /* the excitation (while the trigger pulse and its tail last) */
    {
        int32_t pl = d->pulse_left, pu = d->pulse, plp = d->pulse_lp, active = pl || !TINY(pu) || !TINY(plp);
        int32_t x0[32], x1[32], shell[32];
        for (i = 0; i < n; i++)
            shell[i] = 0;
        if (active) {
            for (i = 0; i < n; i++) {
                int32_t pulse;
                if (pl) {
                    pl--;
                    pulse = pl ? Q24(8.6) : Q24(7.6);
                    pu = pulse;
                } else {
                    pu = fmac1t(pu, Q31(1.0 - 1.0 / 4.8));
                    pulse = pu;
                }
                ONE_POLE(plp, pulse, Q31(0.75));
                x0[i] = pulse - plp + fmac1(pulse, Q31(0.006));
                x1[i] = fmac1(pulse, Q31(0.026));
            }
            d->pulse_left = pl;
            d->pulse = pu;
            d->pulse_lp = plp;
        } else {
            d->pulse = d->pulse_lp = 0;
        }
        /* the shell: each mode over the block (its states and coefficients in registers) */
        for (j = 0; j < nm; j++) {
            struct macro_svf f = d->mode[j];
            struct svf_c c;
            int32_t g = gain28[j], bp, lp;
            const int32_t *ex = j ? x1 : x0;
            c.a1 = d->ma1[j];
            c.a2 = d->ma2[j];
            c.a3 = d->ma3[j];
            if (active)
                for (i = 0; i < n; i++) {
                    SVF_STEP(f, c, ex[i], bp, lp);
                    shell[i] += fmac1(bp + fmac1(ex[i], leak31), g);          /* Q21 */
                }
            else if (!TINY(f.s1) || !TINY(f.s2))
                for (i = 0; i < n; i++) {
                    SVF_STEP(f, c, 0, bp, lp);
                    shell[i] += fmac1(bp, g);
                }
            (void)lp;
            svf_guard(&f);
            d->mode[j] = f;
        }
        for (i = 0; i < n; i++) {
            int32_t s21 = shell[i];
            s21 = s21 > (7 << 20) ? 7 << 20 : s21 < -(7 << 20) ? -(7 << 20) : s21;
            out[i] = fmac1(softclip24(s21 << 3), one_minus_snappy);
        }
    }
    /* the noise: 2u - 1 kept above 0, the envelope, snappy x 2, a band-pass at 16 f0 (none without snappy) */
    if (snappy || !TINY(d->nf.s1) || !TINY(d->nf.s2)) {
        struct macro_svf f = d->nf;
        struct svf_c c;
        int32_t env = d->noise_env, bp, lp;
        uint32_t r = *rng;
        c.a1 = d->na1;
        c.a2 = d->na2;
        c.a3 = d->na3;
        for (i = 0; i < n; i++) {
            int32_t noise;
            r = r * 1664525u + 1013904223u;
            noise = (int32_t)(r >> 7) - (1 << 24);
            if (noise < 0)
                noise = 0;
            env = fmac1t(env, ned);
            noise = fmac1(fmac1(noise, env << 5) << 2, snappy) << 1;   /* x the envelope (Q29: up to 2) x snappy x 2 */
            SVF_STEP(f, c, noise, bp, lp);
            out[i] += bp;
        }
        (void)lp;
        *rng = r;
        d->noise_env = env;
        svf_guard(&f);
        d->nf = f;
    } else
        for (i = 0; i < n; i++)
            d->noise_env = fmac1t(d->noise_env, ned);
}

/* SyntheticSnareDrum's DistortedSine: t = 4 p - 1.3 (p mirrored past 0.5), 2 t / (1 + |t|); p Q28 */
static inline int32_t sd_dsine(int32_t p28)
{
    int32_t t = (p28 < (1 << 27) ? p28 : (1 << 28) - p28) >> 4;            /* Q24 */
    return 2 * rsat24(4 * t - Q24(1.3));
}

static void sd_synthetic(struct macro_sd *__restrict d, uint32_t *__restrict rng, int32_t harm, int32_t timb, int32_t morph, uint32_t inc,
                         int32_t *__restrict aux, int n)
{
    int32_t f0 = (int32_t)(inc >> 1), dxt, fm2, drum_decay, snare_decay, sn, drum_level, snare_level, rna, i;
    int32_t g_hp, g_dlp, f, fmin, fmax, a1, a2, a3, k31, f28, fm4;
    struct svf_c c;
    dxt = (int32_t)(((uint32_t)morph * (uint32_t)(65536 + (int32_t)(((uint32_t)morph * (uint32_t)morph) >> 16) - morph)) >> 16);
    fm2 = (int32_t)(((uint32_t)timb * (uint32_t)timb) >> 16);               /* fm_amount^2, Q16 */
    /* drum: 1 - 2^((-72 dxt - 12 fm^2 + 7 HARM) / 12) / 720; snare: 1 - 2^((-60 MORPH - 7 HARM) / 12) / 480 */
    drum_decay = 0x7fffffff - fmac1((int32_t)exp2_q16(-6 * dxt - fm2 + (7 * harm) / 12) << 13, 11930465); /* 2^33/720 */
    snare_decay = 0x7fffffff - fmac1((int32_t)exp2_q16(-5 * morph - (7 * harm) / 12) << 14, 8947849);     /* 2^32/480 */
    sn = harm + ((harm * 3277) >> 15) - 3277;
    sn = sn < 0 ? 0 : sn > 65535 ? 65535 : sn;                             /* snappy, Q16 */
    drum_level = sn >= 65535 ? 0 : (int32_t)exp2_q16((log2_q16((uint32_t)(65536 - sn)) - 16 * 65536) >> 1) << 15;
    snare_level = sn <= 0 ? 0 : (int32_t)exp2_q16((log2_q16((uint32_t)sn) - 16 * 65536) >> 1) << 15;
    if (drum_level < 0)
        drum_level = 0x7fffffff;
    if (snare_level < 0)
        snare_level = 0x7fffffff;
    /* the oscillators' reset noise: ((0.125 - f0) 8)^2 clamped, x fm^2 */
    rna = f0 >= Q31(0.125) ? 0 : (Q31(0.125) - f0) * 8;
    rna = fmac1(fmac1(rna, rna), fm2 << 15);                               /* Q31 */
    /* filters */
    fmin = f0 > Q31(0.05) ? Q31(0.5) : f0 * 10;
    fmax = f0 > Q31(0.5 / 35) ? Q31(0.5) : f0 * 35;
    g_hp = onepole_G(fmin);
    f = f0 > Q31(0.5 / 3) ? Q31(0.5) : f0 * 3;
    g_dlp = onepole_G(f);
    {
        int32_t g27 = tan_fast27(fmax);
        /* k = 1 / q, q = 0.5 + 2 snappy: g k = 2 g / (1 + 4 snappy) */
        k31 = recip27((1 << 27) + (sn << 13));
        svf_coefs_gk(g27, fmac1(g27, k31) << 1, &a1, &a2, &a3);
    }
    c.a1 = a1;
    c.a2 = a2;
    c.a3 = a3;
    fm4 = fm2 << 13;                                                       /* 4 fm^2, Q27 */
    if (d->trig) {
        d->snare_amp = d->drum_amp = Q24(0.86);
        d->fm = 0x7fffff00 >> 7;
        d->ph0 = d->ph1 = 0;
        d->hold = 1920 + ((1440 * morph) >> 16);
    }
    for (i = 0; i < n; i++) {
        int32_t rn, drum, noise, snare, lp, bp;
        if (d->drum_amp > Q24(0.03) || !((n - 1 - i) & 1))
            d->drum_amp = fmac1t(d->drum_amp, drum_decay);
        if (d->hold)
            d->hold--;
        else
            d->snare_amp = fmac1t(d->snare_amp, snare_decay);
        d->fm = fmac1t(d->fm, Q31(1.0 - 1.0 / 336.0));
        rn = (d->ph0 > (1 << 27) ? -1 : 1) + (d->ph1 > (1 << 27) ? -1 : 1);
        rn = fmac1(rn << 28, fmac1(rna, Q31(0.025)));                      /* Q28 */
        f28 = (f0 >> 3) + (fmac1(f0, fmac1(d->fm << 7, fm4)) << 1);        /* f0 (1 + 4 fm^2 fm), Q28 */
        d->ph0 += f28;
        d->ph1 += fmac1(f28, Q31(0.735)) * 2;
        if (rna > Q31(0.1)) {
            if (d->ph0 >= (1 << 28) + rn)
                d->ph0 = (1 << 28) - d->ph0;
            if (d->ph1 >= (1 << 28) + rn)
                d->ph1 = (1 << 28) - d->ph1;
        } else {
            if (d->ph0 >= (1 << 28))
                d->ph0 -= 1 << 28;
            if (d->ph1 >= (1 << 28))
                d->ph1 -= 1 << 28;
        }
        /* (past a step of 1 a sample, at the top notes with FM, Plaits' phases run away; kept here in -1..2) */
        d->ph0 = d->ph0 > (2 << 28) ? d->ph0 - (2 << 28) : d->ph0 < -(1 << 28) ? -(1 << 28) : d->ph0;
        d->ph1 = d->ph1 > (2 << 28) ? d->ph1 - (2 << 28) : d->ph1 < -(1 << 28) ? -(1 << 28) : d->ph1;
        drum = -Q24(0.1) + fmac1(sd_dsine(d->ph0), Q31(0.6)) + fmac1(sd_dsine(d->ph1), Q31(0.25));
        drum = fmac1(fmac1(drum, d->drum_amp << 7), drum_level);
        ONEPOLE_LP(d->drum_lp, g_dlp, drum, drum);
        noise = (int32_t)(rnd32(rng) >> 8);
        SVF_STEP(d->snare_lp, c, noise, bp, lp);
        (void)bp;
        ONEPOLE_LP(d->snare_hp, g_hp, lp, snare);
        snare = lp - snare;                                                 /* the high-pass */
        snare = fmac1(fmac1(snare + Q24(0.1), (d->snare_amp + d->fm) << 6) << 1, snare_level);
        aux[i] = snare + drum;
    }
    svf_guard(&d->snare_lp);
}

static void sd_render(struct macro_voice *m, const uint8_t *p, uint32_t inc, int32_t *out, int32_t *aux, int n,
                      int want_out, int want_aux)
{
    struct macro_sd *d = &m->e.sd;
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]), i;
    if (d->trig)
        d->idle[0] = d->idle[1] = 0;
    if (want_out) {
        if (d->idle[0])
            for (i = 0; i < n; i++)
                out[i] = 0;
        else {
            int j, q = 1;
            sd_analog(d, &m->rng, harm, timb, morph, inc, out, n);
            for (i = 0; i < n; i++)
                out[i] >>= 9;
            for (j = 0; j < 5; j++)
                q &= TINYR(d->mode[j].s1) && TINYR(d->mode[j].s2);
            d->idle[0] = q && !d->pulse_left && TINY(d->pulse) && TINY(d->pulse_lp) && TINY(d->noise_env)
                         && TINY(d->nf.s1) && TINY(d->nf.s2);
        }
    }
    if (want_aux) {
        if (d->idle[1])
            for (i = 0; i < n; i++)
                aux[i] = 0;
        else {
            sd_synthetic(d, &m->rng, harm, timb, morph, inc, aux, n);
            for (i = 0; i < n; i++)
                aux[i] >>= 9;
            d->idle[1] = !d->hold && TINY(d->drum_amp) && TINY(d->snare_amp) && TINY(d->fm) && TINY(d->drum_lp);
        }
    }
    d->trig = 0;
}

/* ---- HH: plaits/dsp/engine/hi_hat_engine.cc, drums/hi_hat.h ------------------------------------------- *
 * OUT: 808-style metallic noise (six square oscillators), a resonant band-pass, clocked noise mixed in by
 * HARMONICS, a "swing" VCA and a high-pass; AUX: three ring-modulated square x saw pairs, a band-pass, a
 * linear VCA with a two-stage envelope, a high-pass. TIMB: tone (the filters' frequency); MORP: decay. */

static void hh_init(struct macro_hh *d)
{
    int32_t *w = &d->trig, *end = (int32_t *)(d + 1), i;
    while (w < end)
        *w++ = 0;
    for (i = 0; i < 6; i++) {
        d->ph[i] = 0x80000000u;                             /* Oscillator::Init: phase 0.5, high */
        d->high[i] = 1;
    }
    d->key_timb = -1;
}

static const uint32_t hh_sq_ratio28[6] = {268435456, 350039835, 393526378, 479694160, 518617301, 680752316};
static const uint32_t hh_rm_inc[6] = {17895697, 673772995, 45634028, 722538769, 65319294, 939524096};

/* Plaits' Oscillator, SQUARE (pw 0.5) or SAW, one sample, Q15 (band-limited steps: polyBLEP) */
static inline int32_t blep_q15(int32_t t)                    /* 0.5 t^2, t Q16 */
{
    return ((t >> 1) * (t >> 1)) >> 16;
}

static void hh_render(struct macro_voice *__restrict m, const uint8_t *p, uint32_t inc, int32_t *__restrict out, int32_t *__restrict aux, int n,
                      int want_out, int want_aux)
{
    struct macro_hh *d = &m->e.hh;
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]);
    int32_t f0 = (int32_t)(inc >> 1), noisiness, nf25, env_decay, cut_decay, i, j, h;
    uint32_t cutoff, nclk_inc;
    struct svf_c kbp, khp;
    noisiness = (int32_t)(((uint32_t)harm * (uint32_t)harm) >> 16) << 15;   /* HARMONICS^2, Q31 */
    /* 1 - 0.003 x 2^(-7 MORPH), 1 - 0.0025 x 2^(-3 MORPH) */
    env_decay = 0x7fffffff - fmac1((int32_t)exp2_q16(-7 * morph) << 14, 12884902);
    cut_decay = 0x7fffffff - fmac1((int32_t)exp2_q16(-3 * morph) << 14, 10737418);
    /* the filters at 150 Hz x 2^(6 TIMBRE), 16 kHz at most */
    if (d->key_timb != timb) {                             /* (computed again only when TIMBRE moves) */
        struct svf_c k;
        cutoff = inc_of_log2(1551766 + 6 * timb);
        if (cutoff > 0x55555555u)
            cutoff = 0x55555555u;
        svf_coefs(&k, cutoff, coef_fit(coef_norm((uint32_t)recip_q((3 << 24) + timb * 768, 24), 31)));   /* q 3 + 3 TIMBRE */
        d->kc[0][0] = k.a1; d->kc[0][1] = k.a2; d->kc[0][2] = k.a3; d->kc[0][3] = k.k2;
        svf_coefs(&k, cutoff, coef_norm(1, 0));                                                          /* q 1 */
        d->kc[1][0] = k.a1; d->kc[1][1] = k.a2; d->kc[1][2] = k.a3; d->kc[1][3] = k.k2;
        svf_coefs(&k, cutoff, coef_norm(2, 0));                                                          /* q 0.5 */
        d->kc[2][0] = k.a1; d->kc[2][1] = k.a2; d->kc[2][2] = k.a3; d->kc[2][3] = k.k2;
        d->key_timb = timb;
    }
    khp.a1 = d->kc[2][0]; khp.a2 = d->kc[2][1]; khp.a3 = d->kc[2][2]; khp.k2 = d->kc[2][3];
    /* the clocked noise: f0 (32 - 16 HARMONICS^2), at most 0.5 */
    {
        nf25 = fmac1(f0, (32 << 25) - ((noisiness >> 15) << 13));           /* Q25 */
        nclk_inc = nf25 >= (1 << 24) ? 0x80000000u : (uint32_t)nf25 << 7;
    }
    for (h = 0; h < 2; h++) {
        int32_t *x = h ? aux : out;
        if (!(h ? want_aux : want_out))
            continue;
        if (d->trig)
            d->idle[h] = 0;
        if (d->idle[h]) {
            for (i = 0; i < n; i++)
                x[i] = 0;
            continue;
        }
        if (d->trig)                                       /* (1.5 + 0.5 (1 - MORPH)) x 0.86 */
            d->env[h] = Q24(0.86 * 2.0) - (fmac1(morph << 15, Q31(0.43)) >> 7);
        if (h == 0) {                                      /* SquareNoise at 2 f0 */
            uint32_t sinc[6];
            for (j = 0; j < 6; j++) {
                uint32_t f = (uint32_t)fmac1(f0, (int32_t)hh_sq_ratio28[j]);       /* Q28 of f0 r */
                sinc[j] = f >= (Q31(0.499) >> 4) ? (uint32_t)Q31(0.499) << 1 : f << 5;  /* 2 f0 r, Q32 */
            }
            if ((sinc[0] | sinc[1] | sinc[2] | sinc[3] | sinc[4] | sinc[5]) < (1u << 26)) {
                /* Up to ~700 Hz the squares turn over at most twice a block each: the count of squares up
                 * changes only there, so it is built from those turns (a division each) rather than six
                 * phases a sample. The same samples as the loop below. */
                int32_t dc[32], c = 0;
                for (i = 0; i < n; i++)
                    dc[i] = 0;
                for (j = 0; j < 6; j++) {
                    uint32_t q = d->sq[j], si = sinc[j];
                    int m = 0;
                    c += (int32_t)(q >> 31);
                    if (si)
                        for (;;) {
                            uint32_t dist = (q & 0x80000000u) ? 0u - q : 0x80000000u - q;   /* to the next half */
                            uint32_t k;
                            if (dist > (uint32_t)(n - m) * si)
                                break;
                            k = (dist - 1) / si + 1;          /* steps until the top bit turns */
                            m += (int)k;
                            dc[m - 1] += (q & 0x80000000u) ? -1 : 1;
                            q += k * si;
                        }
                    d->sq[j] += (uint32_t)n * si;
                }
                for (i = 0; i < n; i++) {
                    c += dc[i];
                    x[i] = c * Q24(0.33) - Q24(1.0);
                }
            } else {
                uint32_t p0 = d->sq[0], p1 = d->sq[1], p2 = d->sq[2], p3 = d->sq[3], p4 = d->sq[4], p5 = d->sq[5];
                for (i = 0; i < n; i++) {
                    int32_t c;
                    p0 += sinc[0];
                    p1 += sinc[1];
                    p2 += sinc[2];
                    p3 += sinc[3];
                    p4 += sinc[4];
                    p5 += sinc[5];
                    c = (int32_t)((p0 >> 31) + (p1 >> 31) + (p2 >> 31) + (p3 >> 31) + (p4 >> 31) + (p5 >> 31));
                    x[i] = c * Q24(0.33) - Q24(1.0);
                }
                d->sq[0] = p0;
                d->sq[1] = p1;
                d->sq[2] = p2;
                d->sq[3] = p3;
                d->sq[4] = p4;
                d->sq[5] = p5;
            }
            kbp.a1 = d->kc[0][0]; kbp.a2 = d->kc[0][1]; kbp.a3 = d->kc[0][2]; kbp.k2 = d->kc[0][3];
        } else {                                           /* RingModNoise at 2 f0: ratio 2f0 / (0.01 + 2f0) */
            /* 1 - 0.01 / (0.01 + 2 f0) = 1 - 1 / (1 + 200 f0) */
            int32_t ratio = 0x7fffffff - recip_q((1 << 24) + (f0 >> 7) * 200, 24);
            uint32_t rinc[6];
            for (j = 0; j < 6; j++) {
                uint32_t f = (uint32_t)fmac1((int32_t)(hh_rm_inc[j] >> 1), ratio) << 1;
                if (f > 0x40000000u)
                    f = 0x40000000u;                       /* kMaxFrequency 0.25 */
                rinc[j] = f;
            }
            for (i = 0; i < n; i++)
                x[i] = 0;
            for (j = 0; j < 6; j += 2) {                   /* a pair over the block: square x saw */
                uint32_t pa = d->ph[j], pb = d->ph[j + 1], ia = rinc[j], ib = rinc[j + 1];
                int32_t na = d->next[j], nb = d->next[j + 1], hi = d->high[j];
                for (i = 0; i < n; i++) {
                    int32_t ta = na, tb = nb, above, t;
                    uint32_t old = pa;
                    na = 0;
                    pa += ia;
                    above = pa < old || pa >= 0x80000000u;
                    if (hi ^ above) {                      /* the square up */
                        t = sub_sample(pa - 0x80000000u, ia);
                        ta += blep_q15(t);
                        na -= blep_q15(65536 - t);
                        hi = above;
                    }
                    if (pa < old) {                        /* and down: a new cycle */
                        t = sub_sample(pa, ia);
                        ta -= blep_q15(t);
                        na += blep_q15(65536 - t);
                        hi = 0;
                    }
                    na += pa >= 0x80000000u ? 32768 : 0;
                    old = pb;
                    nb = 0;
                    pb += ib;
                    if (pb < old) {                        /* the saw's reset */
                        t = sub_sample(pb, ib);
                        tb -= blep_q15(t);
                        nb += blep_q15(65536 - t);
                    }
                    nb += (int32_t)(pb >> 17);
                    x[i] += ((2 * ta - 32768) * (2 * tb - 32768)) >> 6;    /* Q30 -> Q24 */
                }
                d->ph[j] = pa;
                d->ph[j + 1] = pb;
                d->next[j] = na;
                d->next[j + 1] = nb;
                d->high[j] = hi;
            }
            kbp.a1 = d->kc[1][0]; kbp.a2 = d->kc[1][1]; kbp.a3 = d->kc[1][2]; kbp.k2 = d->kc[1][3];
        }
        {
            struct macro_svf fb = d->bp[h], fh = d->hp[h];
            uint32_t clk = d->nclk[h], r = m->rng;
            int32_t smp = d->nsmp[h], env = d->env[h], bp, lp, s;
            for (i = 0; i < n; i++) {
                SVF_STEP(fb, kbp, x[i], bp, lp);
                if (noisiness) {                           /* the clocked noise, mixed in by HARMONICS^2 */
                    uint32_t old = clk;
                    clk += nclk_inc;
                    if (clk < old) {
                        r = r * 1664525u + 1013904223u;
                        smp = (int32_t)(r >> 8) - Q24(0.5);
                    }
                    s = bp + fmac1(smp - bp, noisiness);
                } else
                    s = bp;
                if (h == 0) {                              /* SwingVCA, one-stage envelope */
                    env = fmac1t(env, env_decay);
                    s = s > 0 ? (s > Q24(30.0) ? Q24(120.0) : s * 4) : fmac1(s, Q31(0.1));
                    s = fmac1(rsat24(s) + Q24(0.1), env << 6) << 1;
                } else {                                   /* LinearVCA, two stages */
                    env = fmac1t(env, env > Q24(0.5) ? env_decay : cut_decay);
                    s = fmac1(s, env << 6) << 1;
                }
                SVF_STEP(fh, khp, s, bp, lp);
                x[i] = SVF_HP(khp, s, bp, lp) >> 9;        /* Q15 */
            }
            d->bp[h] = fb;
            d->hp[h] = fh;
            d->nclk[h] = clk;
            d->nsmp[h] = smp;
            d->env[h] = env;
            m->rng = r;
        }
        svf_guard(&d->bp[h]);
        svf_guard(&d->hp[h]);
        d->idle[h] = TINY(d->env[h]) && TINY(d->hp[h].s1) && TINY(d->hp[h].s2);
    }
    d->trig = 0;
}

/* ---- GRAIN: plaits/dsp/engine/grain_engine.cc, oscillator/grainlet_oscillator.h, z_oscillator.h ------ *
 * OUT: two "grainlets" (a sine formant hard-synced to a shaped carrier), the second's formant HARMONICS
 * away (-2..+2 octaves), summed, DC-blocked; AUX: the Z oscillator (a formant with a sine discontinuity).
 * TIMB: formant frequency; MORP: carrier shape; HARM: formant ratio and carrier bleed (OUT), the
 * discontinuity's shape (AUX). Controls are the block's (Plaits glides them over the block). */

static void grain_init(struct macro_grain *z)
{
    int32_t *w = (int32_t *)z, *end = (int32_t *)(z + 1);
    while (w < end)
        *w++ = 0;
}

/* a sine of a Q32 phase, Q24 */
static inline int32_t sine24(uint32_t ph)
{
    return sin32(ph) << 9;
}

struct carrier_c { int32_t seg, m27, bp31, a22, b31; };

/* GrainletOscillator::Carrier for a block's shape: the warped phase's parameters */
static void carrier_coefs(struct carrier_c *c, int32_t shape16)
{
    int32_t s3 = shape16 * 3, fr = s3 & 0xffff, t = 65536 - fr, t3;
    c->m27 = c->bp31 = c->a22 = c->b31 = 0;
    c->seg = s3 >> 16;
    if (c->seg >= 2)
        t = fr;                                             /* (segment 2: t = 1 - t) */
    t = t > 65535 ? 65535 : t;
    t3 = (int32_t)(((((uint32_t)t * (uint32_t)t) >> 16) * (uint32_t)t) >> 16);   /* Q16 */
    if (c->seg == 0)
        c->m27 = (1 << 27) + t3 * 15 * 2048;                /* 1 + 15 t^3 */
    else if (c->seg == 1) {
        int32_t bp = Q31(0.001) + fmac1(t3 << 15, Q31(0.499));
        c->bp31 = bp;
        {
            uint32_t q = (0x7fffffffu / ((uint32_t)bp >> 11)) << 10;           /* 0.5 / bp, Q22 */
            c->a22 = q > 0x7fffffffu ? 0x7fffffff : (int32_t)q;
        }
        c->b31 = (int32_t)(0x40000000u / ((uint32_t)(0x80000000u - (uint32_t)bp) >> 16)) << 15;   /* 0.5 / (1 - bp) */
        if (c->b31 < 0)
            c->b31 = 0x7fffffff;
    } else
        c->m27 = (1 << 26) + t3 * 29 * 1024;                /* 0.5 + 14.5 t^3 */
}

/* the carrier, Q24: (Sine(warped phase) + 1) / 4; p: Q31, 0..1 inclusive */
static inline int32_t carrier24(const struct carrier_c *c, uint32_t p)
{
    uint32_t w;
    if (c->seg == 0) {
        int32_t v = fmac1((int32_t)(p >> 1), c->m27);       /* Q26 */
        w = (v >= (1 << 26) ? 0x80000000u : (uint32_t)v << 5) * 2 + 0xc0000000u;
    } else if (c->seg == 1) {
        if (p < (uint32_t)c->bp31)
            w = (uint32_t)fmac1((int32_t)p, c->a22) << 10;                        /* p 0.5 / bp, Q32 */
        else
            w = 0x80000000u + ((uint32_t)fmac1((int32_t)(p - (uint32_t)c->bp31), c->b31) << 1);
        w += 0xc0000000u;
    } else {
        int32_t v = fmac1((int32_t)(p >> 1), c->m27);       /* Q26 */
        w = v >= (1 << 25) ? 0xc0000000u : 0x40000000u + ((uint32_t)v << 6);
    }
    return (sine24(w) + (1 << 24)) >> 2;
}

static void grain_render(struct macro_grain *__restrict z, const uint8_t *p, uint32_t inc, int32_t *__restrict out, int32_t *__restrict aux, int n,
                         int want_out, int want_aux)
{
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]), i, j;
    int32_t lf0 = log2_q16(inc), G;
    uint32_t c_inc = inc > 0x20000000u ? 0x20000000u : inc;      /* the carrier: at most 0.125 */
    {
        uint32_t f = inc >> 1;                                     /* 0.3 f0: the DC blockers */
        G = 0x7fffffff - recip27((1 << 27) + tan_dirty27(fmac1((int32_t)f, Q31(0.3))));
    }
    if (want_out) {
        struct carrier_c cc;
        int32_t bleed, inv, shape, lim;
        uint32_t f_inc[2];
        /* f1 = NoteToFrequency(24 + 84 TIMBRE); the second x 2^((48 HARMONICS - 24) / 12) */
        f_inc[0] = inc_of_log2(1653513 + ((84 * timb - 45 * 65536) / 12));
        f_inc[1] = inc_of_log2(1653513 + ((84 * timb - 45 * 65536) / 12) + 4 * harm - 2 * 65536);
        for (j = 0; j < 2; j++)
            if (f_inc[j] > 0x40000000u)
                f_inc[j] = 0x40000000u;
        /* bleed: cb (2 - cb), cb = 1 - 2 HARMONICS below the middle; the grainlet's 1 / (1 + bleed) */
        bleed = harm < 32768 ? (32768 - harm) * 2 - (harm == 0) : 0;          /* cb, Q16 (65535 at most) */
        bleed = (int32_t)(((uint32_t)bleed * (uint32_t)(131072 - bleed)) >> 16) << 8;   /* Q24 */
        inv = recip_q((1 << 24) + bleed, 24);
        /* shape 0.33 + (MORPH - 0.33) max(1 - 24 f0, 0) */
        lim = inc >= 0x0aaaaaaau ? 0 : 65536 - (int32_t)((inc >> 12) * 24 >> 4);
        shape = 21627 + (((morph - 21627) * (lim >> 1)) >> 15);
        carrier_coefs(&cc, shape < 0 ? 0 : shape > 65535 ? 65535 : shape);
        for (i = 0; i < n; i++)
            out[i] = 0;
        for (j = 0; j < 2; j++) {
            uint32_t fi = f_inc[j], gc = z->gc[j], gf = z->gf[j];   /* (in locals: registers) */
            int32_t gn = z->gnext[j];
            for (i = 0; i < n; i++) {
                int32_t this_s = gn, next = 0, g;
                uint32_t old = gc;
                gc += c_inc;
                if (gc < old) {                                  /* the carrier restarts: the formant too */
                    int32_t rt = sub_sample(gc, c_inc), before, after, disc;
                    before = fmac1(carrier24(&cc, 0x80000000u) << 7,
                                   sine24(gf + mul_inc(fi, (uint32_t)(65536 - rt))) + bleed);
                    after = fmac1(carrier24(&cc, 0) << 7, bleed);
                    disc = fmac1(after - before, inv);
                    this_s += fmac1(disc, blep_q15(rt) << 16);
                    next -= fmac1(disc, blep_q15(65536 - rt) << 16);
                    gf = mul_inc(fi, (uint32_t)rt);
                } else
                    gf += fi;
                g = fmac1(carrier24(&cc, gc >> 1) << 7, sine24(gf) + bleed);
                next += fmac1(g, inv);
                gn = next;
                out[i] += this_s;
            }
            z->gc[j] = gc;
            z->gf[j] = gf;
            z->gnext[j] = gn;
        }
        for (i = 0; i < n; i++) {                               /* the DC blocker (a one-pole high-pass) */
            int32_t lp;
            ONEPOLE_LP(z->dc[0], G, out[i], lp);
            out[i] = (out[i] - lp) >> 9;
        }
    }
    if (want_aux) {
        /* the formant: NoteToFrequency(note + 96 TIMBRE), at most 0.25; the shape MORPH; the mode HARMONICS */
        uint32_t fi = inc_of_log2(lf0 + 8 * timb), ps;
        int32_t offset, s2 = 0, lowshape = morph < 32768, i2;
        if (fi > 0x40000000u)
            fi = 0x40000000u;
        if (harm < 21823) {                                     /* 0.333 */
            offset = 1 << 24;
            ps = 0x40000000u + (uint32_t)harm * 98304u;          /* 0.25 + 1.5 mode */
        } else {
            ps = 0xbfdf3b64u - (uint32_t)(harm - 21627) * 49152u;   /* 0.7495 - 0.75 (mode - 0.33) */
            offset = harm < 43647 ? -sine24(ps) : Q24(0.001);
        }
        if (lowshape)
            s2 = morph << 16;                                     /* 2 shape, Q31 */
#define ZFN(C, D, F, R) do {                                                                       \
            int32_t rd_ = (sine24(((uint32_t)(D)) + 0x40000000u) + (1 << 24)) >> 1;               \
            int32_t ct_;                                                                           \
            if (lowshape) {                                                                        \
                if ((C) >= 0x40000000u)                                                            \
                    rd_ = fmac1(rd_, s2);                                                          \
                ct_ = (1 << 24) + fmac1(sine24(((uint32_t)(C) << 1) + 0x40000000u) - (1 << 24), s2); \
            } else                                                                                 \
                ct_ = sine24(((uint32_t)(C) << 1) + ((uint32_t)morph << 15));                     \
            R = fmac1((fmac1(rd_ << 7, offset + sine24((F) + ps)) - offset), ct_ << 6) << 1;       \
        } while (0)
        for (i = 0; i < n; i++) {
            int32_t this_s = z->znext, next = 0, v;
            uint32_t zi = c_inc >> 1;                             /* f0, Q31 */
            z->zd += c_inc;                                       /* 2 f0 */
            z->zc += zi;
            if (z->zd >= 0x80000000u) {
                int32_t rt, before, after, disc;
                uint32_t cb, ca;
                z->zd -= 0x80000000u;
                rt = sub_sample(z->zd, c_inc);
                cb = z->zc >= 0x80000000u ? 0x80000000u : 0x40000000u;
                ca = z->zc >= 0x80000000u ? 0u : 0x40000000u;
                ZFN(cb, 0x80000000u, z->zf + mul_inc(fi, (uint32_t)(65536 - rt)), before);
                ZFN(ca, 0u, 0u, after);
                disc = after - before;
                this_s += fmac1(disc, blep_q15(rt) << 16);
                next -= fmac1(disc, blep_q15(65536 - rt) << 16);
                z->zf = mul_inc(fi, (uint32_t)rt);
                if (z->zc > 0x80000000u)
                    z->zc = z->zd >> 1;
            } else
                z->zf += fi;
            if (z->zc >= 0x80000000u)
                z->zc -= 0x80000000u;
            ZFN(z->zc, z->zd, z->zf, v);
            next += v;
            z->znext = next;
            aux[i] = this_s;
        }
#undef ZFN
        for (i2 = 0; i2 < n; i2++) {
            int32_t lp;
            ONEPOLE_LP(z->dc[1], G, aux[i2], lp);
            aux[i2] = (aux[i2] - lp) >> 9;
        }
    }
}

/* ---- the machine ---------------------------------------------------------------------------------- */

/* ---- CHORD (Model-TG): plaits/dsp/engine/chord_engine.cc, chords/chord_bank.cc,
 * oscillator/string_synth_oscillator.h, oscillator/wavetable_oscillator.h ------------------------------ *
 * Four chord notes on five voices (an inversion moves a note an octave up, crossfading it between two
 * voices); each voice a divide-down "string machine" organ (four octave saws on one phase) crossfaded, as
 * MORPH passes the voice's fade point, into a wavetable voice (15 of Plaits' integrated waves). HARM: the
 * chord (11, with Plaits' hysteresis); TIMB: the inversion; MORP: the organ's registration, then the
 * waves. OUT: every voice; AUX: the root's voice(s) alone, x 3. MORPH and TIMBRE go through Plaits' one-pole
 * (0.1 a 12-sample block, rescaled to n samples). Accumulated in Q24. */

#include "macro_chord_tables.h"

static const int32_t chord_fade[MACRO_CHORD_VOICES] = { 36045, 30802, 32113, 33423, 34734 };   /* Q16 */

/* One-pole smoothing of MORPH and TIMBRE: Plaits' 0.1 a CHORD_BLOCK-sample block. The tests set CHORD_BLOCK to
 * their own block size, and CHORD_FROM_ZERO to start both at 0 as Plaits' Init does; built for the Cycles, they
 * start at the knobs, so a p-lock that switches to CHORDS does not sweep through the inversions first. */
#ifndef CHORD_BLOCK
#define CHORD_BLOCK 12
#endif
#define CHORD_LP_NEW (-1)

static COLD void chord_init(struct macro_chord *c)
{
    int32_t *w = (int32_t *)c, *end = (int32_t *)(c + 1);
    int i;
    while (w < end)
        *w++ = 0;
    for (i = 0; i < MACRO_CHORD_VOICES; i++)
        c->dd[i].inc = 536871;                  /* StringSynthOscillator::Init: frequency_ 0.001 (x 8) */
#ifndef CHORD_FROM_ZERO
    c->morph_lp = c->timbre_lp = CHORD_LP_NEW;
#endif
}

/* the high 32 bits of a x b, exactly */
static inline uint32_t mulhi(uint32_t a, uint32_t b)
{
    uint32_t ah = a >> 16, al = a & 0xffff, bh = b >> 16, bl = b & 0xffff;
    uint32_t m1 = ah * bl, m2 = al * bh, lo = al * bl;
    return ah * bh + (m1 >> 16) + (m2 >> 16) + (((m1 & 0xffff) + (m2 & 0xffff) + (lo >> 16)) >> 16);
}

/* x << s, saturating at 0xffffffff */
static inline uint32_t sat_shl(uint32_t x, int s)
{
    return s <= 0 ? x >> -s : x > (0xffffffffu >> s) ? 0xffffffffu : x << s;
}

/* StringSynthOscillator::Render: inc the voice's phase increment (f x 2^32), reg[7] the registration (Q15),
 * gain Q15; adds 2 x the voice into out (Q24) */
static void ssynth_render(struct macro_ssynth *__restrict s, uint32_t inc, const int32_t *reg_u, int32_t gain,
                          int32_t *__restrict out, int n)
{
    int32_t reg[7], g[4], d[4], fd, next = s->next, seg = s->seg;
    uint32_t f, ph = s->phase;
    int shift = 0, i;
    /* the phase spans the 8 segments of a cycle: f x 8 segments a sample is f x 2^32. Above 1/16 (8f > 0.5)
     * everything moves down two registration steps and an octave, as Plaits does */
    while (inc > 0x10000000u) {
        shift += 2;
        inc >>= 1;
    }
    if (shift >= 8)
        return;
    for (i = 0; i < 7; i++)
        reg[i] = i < shift ? 0 : reg_u[i - shift];
    g[0] = ((reg[0] + 2 * reg[1]) * gain) >> 7;                  /* Q15 x Q15 >> 7: Q23 */
    g[1] = ((reg[2] - reg[1] + 2 * reg[3]) * gain) >> 7;
    g[2] = ((reg[4] - reg[3] + 2 * reg[5]) * gain) >> 7;
    g[3] = ((reg[6] - reg[5]) * gain) >> 7;
    for (i = 0; i < 4; i++) {                                    /* ParameterInterpolator: start, step */
        d[i] = (g[i] - s->g[i]) / n;
        g[i] = s->g[i];
    }
    f = s->inc;
    fd = ((int32_t)inc - (int32_t)f) / n;
    for (i = 0; i < n; i++) {
        uint32_t old = ph;
        int32_t this_s = next, nseg;
        f += (uint32_t)fd;
        g[0] += d[0]; g[1] += d[1]; g[2] += d[2]; g[3] += d[3];
        ph += f;
        nseg = (int32_t)(ph >> 29);
        next = 0;
        if (nseg != seg) {
            int32_t disc = 0;
            if (ph < old)                                        /* segment 8: the cycle restarts */
                disc -= g[0];
            if ((nseg & 3) == 0)
                disc -= g[1];
            if ((nseg & 1) == 0)
                disc -= g[2];
            disc -= g[3];
            if (disc) {
                uint32_t t = (uint32_t)sub_sample(ph & 0x1fffffffu, f), u;
                if (t > 65535)
                    t = 65535;
                u = 65536 - t;
                if (u > 65535)
                    u = 65535;
                this_s += fmac1(disc << 1, (int32_t)((t * t) >> 2));     /* ThisBlepSample: 0.5 t^2  */
                next -= fmac1(disc << 1, (int32_t)((u * u) >> 2));      /* NextBlepSample: -0.5 (1-t)^2 */
            }
        }
        seg = nseg;
        next += fmac1((int32_t)(ph ^ 0x80000000u), g[0])            /* saws in -0.5..0.5, Q32 x Q23: Q24 */
              + fmac1((int32_t)((ph << 1) ^ 0x80000000u), g[1])
              + fmac1((int32_t)((ph << 2) ^ 0x80000000u), g[2])
              + fmac1((int32_t)((ph << 3) ^ 0x80000000u), g[3]);
        out[i] += this_s << 1;
    }
    s->phase = ph;
    s->next = next;
    s->seg = seg;
    s->inc = inc;
    for (i = 0; i < 4; i++)
        s->g[i] = g[i];
}

/* WavetableOscillator<128, 15>::Render over chord_wave: inc f x 2^32, amp Q15 (before Plaits' 1 - 2f and
 * 1 / (131072 f)), wave the wave index Q16 (0..1); adds into out (Q24). The integrated wave is differentiated,
 * scaled by 1 / f once a block (the target f: Plaits glides that factor with f), and low-passed twice at 128 f */
static void wtv_render(struct macro_wtv *__restrict w, uint32_t inc, int32_t amp, int32_t wave, int32_t *__restrict out,
                       int n)
{
    uint32_t f = w->inc, ph = w->phase, in, r16;
    int32_t fd, ad, wd, a = w->amp, wf = w->wf, prev = w->prev, lpd = w->lpd, lp = w->lp, z, sh, i;
    if (inc < 429u)                                     /* CONSTRAIN(f, 1e-7, kMaxFrequency) */
        inc = 429u;
    if (inc > 0x40000000u)
        inc = 0x40000000u;
    amp = (amp * (32768 - (int32_t)(inc >> 16))) >> 15; /* 1 - 2f */
    wave = wave * 14 - (wave >> 13);                    /* x (15 - 1.0001) */
    fd = ((int32_t)inc - (int32_t)f) / n;
    ad = (amp - a) / n;
    wd = (wave - wf) / n;
    /* y = d / (128 f), table units a table step, Q8: d x 2^25 / inc = d x r16 x 2^(z - 23), r16 = 2^48 / (inc << z) */
    for (z = 0, in = inc; !(in & 0x80000000u); z++)
        in <<= 1;
    r16 = 0xffffffffu / (in >> 16);
    sh = z - 12;
    for (i = 0; i < n; i++) {
        int32_t wi, wfr, idx, pf, x0, x1, x, dd, y, c;
        const int16_t *t0, *t1;
        f += (uint32_t)fd;
        a += ad;
        wf += wd;
        ph += f;
        wi = wf >> 16;
        wfr = wf & 0xffff;
        idx = (int32_t)(ph >> 25);
        pf = (int32_t)((ph >> 10) & 0x7fff);                         /* Q15 */
        t0 = chord_wave[wi] + idx;
        t1 = chord_wave[wi + 1] + idx;
        x0 = (t0[0] << 8) + (((t0[1] - t0[0]) * pf) >> 7);          /* Q8 */
        x1 = (t1[0] << 8) + (((t1[1] - t1[0]) * pf) >> 7);
        x = x0 + fmac1(x1 - x0, wfr << 15);
        dd = x - prev;
        prev = x;
        y = fmac1(dd << 6, (int32_t)(r16 << 14));
        y = sh >= 0 ? y << sh : y >> -sh;
        c = f >= (1u << 25) ? 0x7fffffff : (int32_t)(f << 6);       /* min(128 f, 1), Q31 */
        lpd += fmac1(y - lpd, c);
        lp += fmac1(lpd - lp, c);
        out[i] += fmac1(lp << 7, a << 15);                           /* x amp / 1024: Q24 */
    }
    w->phase = ph;
    w->inc = inc;
    w->amp = amp;
    w->wf = wave;
    w->prev = prev;
    w->lpd = lpd;
    w->lp = lp;
}

static void chord_render(struct macro_chord *__restrict c, const uint8_t *p, uint32_t inc, int32_t *__restrict out,
                         int32_t *__restrict aux, int n)
{
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]);
    int32_t cp, v, h, q, reg, rq, ri, rf, harmonics[7], amps[MACRO_CHORD_VOICES];
    int32_t rnum[MACRO_CHORD_VOICES], rsh[MACRO_CHORD_VOICES];      /* a voice: chord note, octave shift */
    int32_t inv, ii, fr, rot, rnote, mask = 0, waveform, i, k;
    uint32_t f0;
    /* ONE_POLE(x_lp, x, 0.1) once a 12-sample block: 1 - 0.9^(n / 12) for n samples */
    cp = 65536 - (int32_t)exp2_q16(-9962 * n / CHORD_BLOCK);
    if (c->morph_lp == CHORD_LP_NEW) {
        c->morph_lp = morph << 8;
        c->timbre_lp = timb << 8;
    }
    c->morph_lp += (((morph << 8) - c->morph_lp) >> 8) * (cp >> 2) >> 6;       /* Q24 state */
    c->timbre_lp += (((timb << 8) - c->timbre_lp) >> 8) * (cp >> 2) >> 6;
    morph = c->morph_lp >> 8;                                    /* from here: the smoothed values, Q16 */
    timb = c->timbre_lp >> 8;
    /* ChordBank::set_chord: HysteresisQuantizer2(11, 0.075, asymmetric).Process(HARMONICS x 1.02) */
    v = ((harm * 11489) >> 10) - 32768;                          /* x 1.02 x 11 - 0.5, Q16 */
    h = v > (c->chord << 16) ? -4915 : 4915;
    q = v + h + 32768;
    q = q < 0 ? 0 : q >> 16;
    c->chord = q > CHORD_N - 1 ? CHORD_N - 1 : q;
    /* registration max(1 - 2.15 MORPH, 0) over the 8 rows (x 6.999) */
    reg = 65536 - (int32_t)(((uint32_t)morph * 35226u) >> 14);
    if (reg < 0)
        reg = 0;
    rq = reg * 7 - ((reg * 131) >> 17);
    ri = rq >> 16;
    rf = rq & 0xffff;
    for (k = 0; k < 6; k++) {
        int32_t ra = chord_reg[ri][k], rb = chord_reg[ri < 7 ? ri + 1 : 7][k];
        harmonics[k] = ra + (((rb - ra) * (rf >> 1)) >> 15);
    }
    harmonics[6] = 0;
    /* ChordBank::ComputeChordInversion(TIMBRE): ratios Q16, amplitudes Q15 (0.25 each) */
    inv = timb * 20;
    ii = inv >> 16;
    fr = inv & 0xffff;
    rot = ii >> 2;
    rnote = ii & 3;
    for (k = 0; k < 4; k++) {
        int32_t e = ((3 + ii - k) >> 2) - 2;                     /* transposition 0.25 x 2^((3 + ii - k) / 4) */
        int32_t tv = (k - rot + MACRO_CHORD_VOICES) % MACRO_CHORD_VOICES;
        int32_t pv = (tv - 1 + MACRO_CHORD_VOICES) % MACRO_CHORD_VOICES;
        if (k == rnote) {
            rnum[tv] = rnum[pv] = k;
            rsh[tv] = e;
            rsh[pv] = e + 1;
            amps[pv] = (8192 * fr) >> 16;
            amps[tv] = 8192 - amps[pv];
        } else if (k < rnote) {
            rnum[pv] = k;
            rsh[pv] = e;
            amps[pv] = 8192;
        } else {
            rnum[tv] = k;
            rsh[tv] = e;
            amps[tv] = 8192;
        }
        if (k == 0) {
            if (k >= rnote)
                mask |= 1 << tv;
            if (k <= rnote)
                mask |= 1 << pv;
        }
    }
    for (i = 0; i < n; i++)
        out[i] = aux[i] = 0;
    f0 = mulhi(inc, 4286377353u);                                /* x 0.998 */
    waveform = ((morph - 35062) * 35226) >> 14;            /* max((MORPH - 0.535) 2.15, 0) */
    if (waveform < 0)
        waveform = 0;
    for (k = 0; k < MACRO_CHORD_VOICES; k++) {
        uint32_t nf = sat_shl(mulhi(f0, chord_ratio[c->chord][rnum[k]]), 2 + rsh[k]);   /* Q30 ratio */
        int32_t wta = 50 * (morph - chord_fade[k]), dda, ddg;
        int32_t *dst = (mask >> k) & 1 ? aux : out;
        wta = wta < 0 ? 0 : wta > 65536 ? 65536 : wta;
        dda = 65536 - wta;
        ddg = 4 * 65536 - (int32_t)(nf >> 11);                   /* 4 - 32 f */
        ddg = ddg < 0 ? 0 : ddg > 65536 ? 65536 : ddg;
        dda = (int32_t)(((uint32_t)dda * (uint32_t)(ddg >> 1)) >> 15);    /* (65536 x 65536 would wrap) */
        if (wta)
            wtv_render(&c->wt[k], sat_shl(mulhi(nf, 2156073583u), 1), (amps[k] * wta) >> 16, waveform, dst, n);   /* x 1.004 */
        if (dda)
            ssynth_render(&c->dd[k], nf, harmonics, (amps[k] * dda) >> 16, dst, n);
    }
    for (i = 0; i < n; i++) {                                    /* out += aux; aux x 3; Q24 -> Q15 */
        int32_t o = out[i] + aux[i], x = aux[i] * 3;
        out[i] = (o + 256) >> 9;
        aux[i] = (x + 256) >> 9;
    }
}

/* ---- SWARM (Model-TG): plaits/dsp/engine/swarm_engine.cc ------------------------------------------- *
 * Eight voices, each a grain envelope (re-randomised as its phase wraps, once a block) shaping a saw (OUT)
 * and a sine (AUX) at the note spread over +-4 octaves by HARM^3 and the voice's rank. TIMB: grain density
 * (NoteToFrequency(120 TIMBRE) x 0.025 a sample); MORP: grain size (below a size of 1 the grains glide, from
 * one random pitch to the next). A trig is Plaits' trigger, patched: a burst. OUT goes through the limiter at
 * x 3, as Plaits registers the engine with -3. Accumulated in Q24. */

#define NOTE_L0 1653765                     /* log2 of NoteToFrequency(69) x 2^32, at Plaits' 47872.34 Hz, Q16 */

/* rank (rank + 0.01) / 4, Q30. Plaits' rank is (i - n) / n with n = (8 - 1) / 2 in integer arithmetic: 3, so the
 * ranks run -1 .. 1.33, not -1 .. 1 */
static const int32_t swarm_lin[MACRO_SWARM_VOICES] = {
    265751101, 117515077, 28931377, 0, 30720947, 121094217, 271119811, 480797728
};

static COLD void swarm_init(struct macro_swarm *s)
{
    int i;
    for (i = 0; i < MACRO_SWARM_VOICES; i++) {
        struct macro_genv *e = &s->env[i];
        e->from = 0;
        e->interval = 1 << 30;
        e->phase = 1 << 28;
        e->fm = 0;
        e->amp = 1 << 29;
        e->big = 0;
        e->fc = 0;
        s->saw[i].phase = 0;
        s->saw[i].next = 0;
        s->saw[i].inc = 42949673u;                      /* 0.01 */
        s->saw[i].gain = 0;
        s->sine[i].x = 1 << 30;
        s->sine[i].y = 0;
        s->sine[i].eps = 0;
        s->sine[i].amp = 0;
    }
    s->trig = 0;
}

/* GrainEnvelope::Step (rate Q30), burst mode (the trigger is patched) */
static void genv_step(struct macro_genv *e, uint32_t *rng, int32_t rate, int start)
{
    int randomize = 0;
    if (start) {
        e->phase = 1 << 27;
        e->fm = 1 << 30;                                /* 16 */
        randomize = 1;
    } else {
        e->phase += fmac1(rate, e->fm) << 3;
        if (e->phase >= (1 << 28)) {
            e->phase &= (1 << 28) - 1;
            randomize = 1;
        }
    }
    if (randomize) {
        e->from += e->interval;
        e->interval = (int32_t)(rnd32(rng) >> 2) - e->from;
        e->fm = fmac1(e->fm, (int32_t)(1717986918u + ((rnd32(rng) >> 2) / 5u * 2u)));   /* x (0.8 + 0.2 r), Q31 */
    }
}

/* the voice's sine shaped amplitude and its pitch offset (GrainEnvelope::amplitude, frequency); sr Q16 */
static int32_t genv_amp(struct macro_genv *e, int32_t sr)
{
    int32_t target = 1 << 30, big = sr >= 65536;
    if (big) {
        int32_t p = fmac1((e->phase - (1 << 27)) << 2, sr << 9);      /* (phase - 0.5) size, Q24 */
        p = p > (1 << 24) ? 1 << 24 : p < -(1 << 24) ? -(1 << 24) : p;
        target = (sin32((uint32_t)(p << 7) + 0x40000000u) + 32768) << 14;   /* (Sine(p / 2 + 1.25) + 1) / 2 */
    }
    if (big != e->big)
        e->fc = 1 << 29;
    e->fc = fmac1(e->fc, 2040109466);                  /* x 0.95 */
    e->big = big;
    e->amp += fmac1(target - e->amp, ((1 << 29) - e->fc) << 1);
    return e->amp;
}

static int32_t genv_freq(const struct macro_genv *e, int32_t sr)
{
    if (sr < 65536)                                     /* 2 (from + interval phase) - 1 */
        return (e->from + (fmac1(e->interval, e->phase << 2) << 1) - (1 << 29)) * 2;
    return e->from;
}

/* AdditiveSawOscillator::Render, gain Q26, adds into out (Q24) */
static void asaw_render(struct macro_asaw *__restrict o, uint32_t inc, int32_t gain, int32_t *__restrict out, int n)
{
    uint32_t f = o->inc, ph = o->phase;
    int32_t fd, g = o->gain, gd, next = o->next, i;
    if (inc > 0x40000000u)
        inc = 0x40000000u;
    fd = ((int32_t)inc - (int32_t)f) / n;
    gd = (gain - g) / n;
    for (i = 0; i < n; i++) {
        uint32_t old = ph;
        int32_t this_s = next;
        next = 0;
        f += (uint32_t)fd;
        ph += f;
        if (ph < old) {
            uint32_t t = (uint32_t)sub_sample(ph, f), u;
            t = t > 65535 ? 65535 : t;
            u = 65536 - t;
            u = u > 65535 ? 65535 : u;
            this_s -= (int32_t)((t * t) >> 3);          /* ThisBlepSample, Q30 */
            next += (int32_t)((u * u) >> 3);            /* - NextBlepSample   */
        }
        next += (int32_t)(ph >> 2);
        g += gd;
        out[i] += fmac1(this_s - (1 << 29), g);        /* (2 this - 1) x gain */
    }
    o->phase = ph;
    o->inc = inc;
    o->next = next;
    o->gain = gain;
}

static COLD uint32_t isqrt32(uint32_t v)
{
    uint32_t r = 0, b = 1u << 30;
    while (b > v)
        b >>= 2;
    while (b) {
        if (v >= r + b) {
            v -= r + b;
            r = (r >> 1) + b;
        } else
            r >>= 1;
        b >>= 2;
    }
    return r;
}

/* FastSineOscillator::Render, ADDITIVE: amp Q25 (before 1 - 4f), adds into out (Q24) */
static void fsin_render(struct macro_fsin *__restrict o, uint32_t inc, int32_t amp, int32_t *__restrict out, int n)
{
    int32_t x = o->x, y = o->y, e = o->eps, a = o->amp, ed, ad, fp, fp2, eps, nq, i;
    if (inc >= 0x40000000u) {
        inc = 0x40000000u;
        amp = 0;
    } else
        amp = fmac1(amp, (int32_t)(0x7fffffffu - (inc << 1))) ;   /* x (1 - 4f) */
    fp = (int32_t)mulhi(inc, 3373259426u);                       /* f pi, Q30 */
    fp2 = fmac1(fp, fp) << 1;
    eps = fmac1(fp, (1 << 30) - (fmac1(fp2, 687194767) >> 1)) << 2;   /* f pi (2 - 0.32 (f pi)^2) */
    ed = (eps - e) / n;
    ad = (amp - a) / n;
    nq = (fmac1(x, x) >> 1) + (fmac1(y, y) >> 1);               /* x^2 + y^2, Q28 */
    if (nq <= (1 << 27) || nq >= (1 << 29)) {
        uint32_t r = nq >= (1 << 30) ? 0 : isqrt32((uint32_t)nq << 2);   /* sqrt, Q15 */
        if (r <= 16500 || nq >= (1 << 30)) {                     /* (1 / sqrt would not fit: start again) */
            x = 1 << 30;
            y = 0;
        } else {
            int32_t inv = (int32_t)((0x7fffffffu / r) << 14);   /* 1 / sqrt, Q30 */
            x = fmac1(x, inv) << 1;
            y = fmac1(y, inv) << 1;
        }
    }
    for (i = 0; i < n; i++) {
        e += ed;
        x += fmac1(e, y) << 1;
        y -= fmac1(e, x) << 1;
        a += ad;
        out[i] += fmac1(x, a);
    }
    o->x = x;
    o->y = y;
    o->eps = eps;
    o->amp = amp;
}

static void swarm_render(struct macro_voice *m, const uint8_t *p, uint32_t inc, int32_t *__restrict out,
                         int32_t *__restrict aux, int n, int want_out, int want_aux)
{
    struct macro_swarm *s = &m->e.swarm;
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]);
    int32_t spread, sr, rate, i, k, start = s->trig;
    s->trig = 0;
    /* density: NoteToFrequency(120 TIMBRE) x 0.025 x n, Q30 */
    {   /* NoteToFrequency reads SemitonesToRatio's tables: the semitones above -128 floored to 1/256 */
        int32_t semis = ((120 * timb - (9 << 16) + (128 << 16)) & ~0xff) - (128 << 16);   /* note - 9, Q16 */
        rate = (int32_t)mulhi(inc_of_log2(NOTE_L0 + (semis - (60 << 16)) / 12), (uint32_t)n * 26843546u);
    }
    spread = (int32_t)(((((uint32_t)harm * (uint32_t)harm) >> 16) * (uint32_t)harm) >> 2);   /* HARM^3, Q30 */
    sr = (int32_t)exp2_q16(7 * (65536 - morph) - 131072);       /* 0.25 x 2^(7 (1 - MORPH)), Q16 */
    for (i = 0; i < n; i++)
        out[i] = aux[i] = 0;
    for (k = 0; k < MACRO_SWARM_VOICES; k++) {
        struct macro_genv *e = &s->env[k];
        int32_t amp, expo, oct;
        uint32_t f;
        genv_step(e, &m->rng, rate, start);
        amp = genv_amp(e, sr);                                   /* / 8 below */
        expo = genv_freq(e, sr);
        oct = ((fmac1(expo, spread) >> 13) * (4 * (k - 3))) / 3;       /* 4 expo spread rank, octaves Q16 */
        f = sat_shl(mulhi(inc, exp2_q16(oct) << 10), 6);        /* x 2^oct (Q16 -> Q26: up to 2^5.33) */
        f = mulhi(f, 0x80000000u + ((uint32_t)fmac1(swarm_lin[k], spread) << 2)) << 1;   /* x (1 + linear) */
        if (want_out)
            asaw_render(&s->saw[k], f, amp >> 7, out, n);
        if (want_aux)
            fsin_render(&s->sine[k], f, amp >> 8, aux, n);
        sr -= ((sr >> 2) * 1966) >> 14;                          /* x 0.97 (sr x 63570 would wrap) */
    }
    for (i = 0; i < n; i++) {
        out[i] = (out[i] * 3 + 64) >> 7;                         /* x 3 (the limiter's pre-gain), Q17 */
        aux[i] = (aux[i] + 256) >> 9;                            /* Q15 */
    }
}

/* ---- WAVETABLE (Model-TG): plaits/dsp/engine/wavetable_engine.cc ----------------------------------- *
 * 192 integrated waves in 4 banks of 8 x 8 (the fourth Plaits' reshuffle of the others). TIMB: X, MORP: Y,
 * HARM: the bank (Z, mirrored 0 1 2 3 3 2 1 0 over its range; above 3/7 the positions snap towards whole waves).
 * Eight waves read with Hermite interpolation and blended trilinearly, differentiated (the waves are stored
 * integrated), scaled by 1 / f once a block and low-passed at 128 f. AUX: OUT truncated to steps of 1/32.
 * X Y Z go through Plaits' one-poles (0.2, 0.2, 0.05 a 12-sample block, rescaled; WT_BLOCK for the tests), then
 * a per-sample one-pole at 2 f (4 - 3 quantization), 0.01..0.1. On the box they start at the knobs. */

#include "macro_wavetable.h"

#ifndef WT_BLOCK
#define WT_BLOCK 12
#endif
#define WT_NEW (-1)

static COLD void wt_init(struct macro_wavetable *w)
{
    int32_t *p = (int32_t *)w, *end = (int32_t *)(w + 1);
    while (p < end)
        *p++ = 0;
    w->f = 4934440u;                                    /* previous_f0_ = a0 */
#ifndef WT_FROM_ZERO
    w->pre[0] = WT_NEW;
#endif
}

/* InterpolateWaveHermite over a wave (Q8 out), at sample i (0..127) and fraction f (Q31) */
static inline int32_t wt_hermite(const int16_t *t, int32_t i, int32_t f)
{
    int32_t xm1 = t[i] << 8, x0 = t[i + 1] << 8, x1 = t[i + 2] << 8, x2 = t[i + 3] << 8;
    int32_t c = (x1 - xm1) >> 1, v = x0 - x1, w = c + v, a = w + v + ((x2 - x0) >> 1), bn = w + a;
    return fmac1(fmac1(fmac1(a, f) - bn, f) + c, f) + x0;
}

static inline const int16_t *wt_wave(int x, int y, int z)
{
    int i = x + y * 8;
    return wt_waves + 132 * (z < 3 ? z * 64 + i : wt_bank3[i]);
}

/* Clamp(): a fraction (Q24) pushed towards 0 or 1, (f - 0.5) x 16 + 0.5 within 0..1 */
static inline int32_t wt_snap(int32_t f)
{
    int32_t v = ((f - (1 << 23)) << 4) + (1 << 23);
    return v < 0 ? 0 : v > (1 << 24) ? 1 << 24 : v;
}

static void wt_render(struct macro_wavetable *__restrict w, const uint8_t *p, uint32_t inc, int32_t *__restrict out,
                      int32_t *__restrict aux, int n)
{
    static const int32_t lg[3] = { -21098, -21098, -4850 };          /* log2 (1 - 0.2), (1 - 0.2), (1 - 0.05), Q16 */
    int32_t knob[3], tg[3], d[3], cur[3], q, lpc, i, k, sh, z;
    uint32_t f = w->f, ph = w->phase, in, r16;
    int32_t fd, prev = w->prev, dlp = w->dlp;
    knob[0] = k16(p[MACRO_P_TIMB]) * 7 << 8;                       /* x 6.9999, Q24 */
    knob[1] = k16(p[MACRO_P_MORPH]) * 7 << 8;
    knob[2] = k16(p[MACRO_P_HARM]) * 7 << 8;
    for (k = 0; k < 3; k++)
        knob[k] -= knob[k] >> 16;                                   /* (7 - 0.0001) */
    if (w->pre[0] == WT_NEW)                                        /* the engine just started: no sweep */
        for (k = 0; k < 3; k++)
            w->pre[k] = w->tgt[k] = w->lp[k] = knob[k];
    for (k = 0; k < 3; k++) {
        int32_t cp = 65536 - (int32_t)exp2_q16(lg[k] * n / WT_BLOCK);
        w->pre[k] += ((knob[k] - w->pre[k]) >> 8) * (cp >> 2) >> 6;
    }
    q = w->pre[2] - (3 << 24);                                      /* quantization: Z - 3 in 0..1, Q24 */
    q = q < 0 ? 0 : q > (1 << 24) ? 1 << 24 : q;
    {   /* 2 f0 (4 - 3 quantization) in 0.01 .. 0.1, Q31; f0 = inc / 2^32 */
        uint32_t m = (4u << 24) - 3u * (uint32_t)q;                 /* Q24, 1..4 */
        uint32_t c = mulhi(inc, m << 5) << 3;                       /* f0 m x 2, Q31 (m << 6 would wrap at 4) */
        lpc = c < 21474836u ? 21474836 : c > 214748365u ? 214748365 : (int32_t)c;
    }
    for (k = 0; k < 3; k++) {                                       /* the target, its fraction snapped */
        int32_t ip = w->pre[k] & ~0xffffff, fr = w->pre[k] & 0xffffff;
        fr += fmac1(wt_snap(fr) - fr, q >= (1 << 24) ? 0x7fffffff : q << 7);   /* (1.0 << 7 would wrap) */
        tg[k] = ip + fr;
        d[k] = (tg[k] - w->tgt[k]) / n;
        cur[k] = w->tgt[k];
        w->tgt[k] = tg[k];
    }
    fd = ((int32_t)inc - (int32_t)f) / n;
    /* y = d / (128 f), table units a table step, Q8: d x 2^25 / inc (as wtv_render) */
    for (z = 0, in = inc; !(in & 0x80000000u); z++)
        in <<= 1;
    r16 = 0xffffffffu / (in >> 16);
    sh = z - 12;
    {
        int32_t g31 = 2040109466 - (int32_t)(inc >> 1);             /* 0.95 - f0, Q31 */
        for (i = 0; i < n; i++) {
            int32_t xi, yi, zi, xf, yf, zf, pi, pf, z0, z1, a0, a1, b0, b1, m0, m1, mix, dd, y, c;
            f += (uint32_t)fd;
            for (k = 0; k < 3; k++) {
                cur[k] += d[k];
                w->lp[k] += fmac1(cur[k] - w->lp[k], lpc);
            }
            xi = w->lp[0] >> 24; xf = (w->lp[0] & 0xffffff) << 7;
            yi = w->lp[1] >> 24; yf = (w->lp[1] & 0xffffff) << 7;
            zi = w->lp[2] >> 24; zf = (w->lp[2] & 0xffffff) << 7;
            if (xi > 6) { xi = 6; xf = 0x7fffffff; }                /* 7.0 itself: the last wave, never past it */
            if (yi > 6) { yi = 6; yf = 0x7fffffff; }                /* (Plaits would read past its bank there) */
            if (zi > 6) { zi = 6; zf = 0x7fffffff; }
            ph += f;
            pi = (int32_t)(ph >> 25);
            pf = (int32_t)((ph << 7) >> 1);                         /* the fraction of a table step, Q31 */
            z0 = zi >= 4 ? 7 - zi : zi;
            z1 = zi + 1 >= 4 ? 6 - zi : zi + 1;
            a0 = wt_hermite(wt_wave(xi, yi, z0), pi, pf);
            a1 = wt_hermite(wt_wave(xi + 1, yi, z0), pi, pf);
            a0 += fmac1(a1 - a0, xf);
            b0 = wt_hermite(wt_wave(xi, yi + 1, z0), pi, pf);
            b1 = wt_hermite(wt_wave(xi + 1, yi + 1, z0), pi, pf);
            b0 += fmac1(b1 - b0, xf);
            m0 = a0 + fmac1(b0 - a0, yf);
            a0 = wt_hermite(wt_wave(xi, yi, z1), pi, pf);
            a1 = wt_hermite(wt_wave(xi + 1, yi, z1), pi, pf);
            a0 += fmac1(a1 - a0, xf);
            b0 = wt_hermite(wt_wave(xi, yi + 1, z1), pi, pf);
            b1 = wt_hermite(wt_wave(xi + 1, yi + 1, z1), pi, pf);
            b0 += fmac1(b1 - b0, xf);
            m1 = a0 + fmac1(b0 - a0, yf);
            mix = m0 + fmac1(m1 - m0, zf);                          /* Q8 */
            dd = mix - prev;
            prev = mix;
            y = fmac1(dd << 6, (int32_t)(r16 << 14));
            y = sh >= 0 ? y << sh : y >> -sh;
            c = f >= (1u << 25) ? 0x7fffffff : (int32_t)(f << 6);   /* min(128 f, 1) */
            dlp += fmac1(y - dlp, c);
            out[i] = fmac1(dlp << 2, g31) >> 5;                     /* x (0.95 - f) / 1024, Q15 */
            aux[i] = (out[i] / 1024) * 1024;                        /* int(x 32) / 32 */
        }
    }
    w->f = inc;
    w->phase = ph;
    w->prev = prev;
    w->dlp = dlp;
}

/* Model-TG / Modded-Cycles: the phase increment (at 48 kHz) of a MIDI note, Q16 (note 69 = 440 Hz) */
uint32_t macro_inc_of_note(int32_t note_q16)
{
    return inc_of_log2(1653513 + (note_q16 - (69 << 16)) / 12);
}

/* ---- VA (Model-TG): plaits/dsp/engine/virtual_analog_engine.cc (VA_VARIANT 2),
 * oscillator/variable_shape_oscillator.h, variable_saw_oscillator.h ------------------------------------- *
 * OUT: a variable-width square (TIMBRE: width, then hard sync above the middle) plus a variable saw (MORPH: notch
 * to triangle), detuned by HARMONICS (unison, fifth, octave, ... both ways); AUX: two variable-shape oscillators
 * (MORPH: triangle, saw, square, width) in "monster sync" (TIMBRE). Signals Q28 inside, the edges corrected with
 * polyBLEP and integrated polyBLEP as in Plaits; the slopes' reciprocals are the block's. */

static inline int32_t q28mul(int32_t a, int32_t b) { return fmac1(a, b) << 3; }          /* Q28 x Q28 */
static inline int32_t amt(int32_t x, int32_t a16) { return fmac1(x, a16 >= 65536 ? 0x7fffffff : a16 << 15); }   /* x a, a Q16 */
static inline int32_t bthis(uint32_t t)                    /* ThisBlepSample: 0.5 t^2, t Q16, Q28 */
{
    t = t > 65535 ? 65535 : t;
    return (int32_t)((t * t) >> 5);
}
static inline int32_t bnext(uint32_t t)                    /* NextBlepSample: -0.5 (1 - t)^2 */
{
    uint32_t u = t >= 65536 ? 0 : 65536 - t;
    u = u > 65535 ? 65535 : u;
    return -(int32_t)((u * u) >> 5);
}
static inline int32_t ithis(uint32_t t) { return iblep_next(65536 - (int32_t)(t > 65536 ? 65536 : t)) << 12; }
static inline int32_t inext(uint32_t t) { return iblep_next((int32_t)(t > 65536 ? 65536 : t)) << 12; }
static inline uint32_t mulq16(uint32_t a, uint32_t t16)    /* a x t, t Q16 (<= 1) */
{
    return (a >> 16) * t16 + (((a & 0xffff) * t16) >> 16);
}

/* 1 / pw and 1 / (1 - pw), Q15, for a pulse width pw (Q31) */
static inline void slopes(int32_t pw31, uint32_t *up, uint32_t *down)
{
    uint32_t q = (uint32_t)pw31 >> 15;
    q = q < 1 ? 1 : q > 65535 ? 65535 : q;
    *up = 0x80000000u / q;
    *down = 0x80000000u / (65536 - q);
}

/* the naive variable shape (saw, square, triangle) at phase ph, Q28; sq, tri: Q16 amounts */
static inline int32_t vso_naive(uint32_t ph, uint32_t pw32, uint32_t up, uint32_t down, int32_t tri, int32_t sq)
{
    int32_t saw = (int32_t)(ph >> 4), square = ph < pw32 ? 0 : 1 << 28, triangle;
    if (ph < pw32)
        triangle = (int32_t)(((ph >> 16) * up) >> 3);
    else
        triangle = (1 << 28) - (int32_t)((((ph - pw32) >> 16) * down) >> 3);
    saw += amt(square - saw, sq);
    saw += amt(triangle - saw, tri);
    return saw;
}

static void vso_init(struct macro_vso *o, uint32_t master_phase)
{
    o->mph = master_phase;
    o->sph = 0;
    o->mf = 0;
    o->sf = 42949673u;                                       /* 0.01 */
    o->pw31 = o->ppw31 = 1 << 30;                            /* 0.5 */
    o->ws = 0;
    o->next = 0;
    o->high = 0;
}

/* VariableShapeOscillator::Render (sync: master mf, slave sf); writes Q28 into out */
static void vso_render(struct macro_vso *__restrict o, int sync, uint32_t mf, uint32_t sf, int32_t pw31, int32_t ws,
                       int32_t *__restrict out, int n)
{
    uint32_t up, down, mph = o->mph, sph = o->sph, m_f = o->mf, s_f = o->sf;
    int32_t mfd, sfd, pwd, wsd, pw = o->pw31, ppw = o->ppw31, w = o->ws, next = o->next, high = o->high, i;
    if (mf > 0x40000000u)
        mf = 0x40000000u;
    if (sf > 0x40000000u)
        sf = 0x40000000u;
    if (sf >= 0x40000000u)
        pw31 = 1 << 30;
    else if (pw31 < (int32_t)sf)                              /* within 2f .. 1 - 2f */
        pw31 = (int32_t)sf;
    else if (pw31 > 0x7fffffff - (int32_t)sf)
        pw31 = 0x7fffffff - (int32_t)sf;
    mfd = ((int32_t)mf - (int32_t)m_f) / n;
    sfd = ((int32_t)sf - (int32_t)s_f) / n;
    pwd = (pw31 - pw) / n;
    wsd = (ws - w) / n;
    slopes(pw31, &up, &down);
    for (i = 0; i < n; i++) {
        int32_t this_s = next, sq, tri, reset = 0, trans = 0, step;
        uint32_t rt = 0, pw32, old;
        int wrapped;
        next = 0;
        m_f += (uint32_t)mfd;
        s_f += (uint32_t)sfd;
        pw += pwd;
        w += wsd;
        sq = w > 32768 ? (w - 32768) * 2 : 0;
        tri = w < 32768 ? 65536 - 2 * w : 0;
        pw32 = (uint32_t)pw << 1;
        step = amt((int32_t)((((up + down) >> 1) * (s_f >> 16)) >> 2), tri);   /* (up + down) f tri, Q28 */
        if (sync) {
            old = mph;
            mph += m_f;
            if (mph < old) {                                     /* the master restarts: reset */
                uint32_t at, a0 = sph;
                int32_t value;
                rt = (uint32_t)sub_sample(mph, m_f);
                at = sph + mulq16(s_f, 65536 - rt);
                reset = 1;
                if (at < a0)
                    trans = 1;
                if (!high && at >= pw32)
                    trans = 1;
                value = vso_naive(at, pw32, up, down, tri, sq);
                this_s -= q28mul(value, bthis(rt));
                next -= q28mul(value, bnext(rt));
            }
        }
        old = sph;
        sph += s_f;
        wrapped = sph < old;
        if (trans || !reset) {
            for (;;) {
                if (!high) {
                    uint32_t t;
                    if (!wrapped && sph < pw32)
                        break;
                    t = (uint32_t)sub_sample(sph - pw32, (uint32_t)((int32_t)(((uint32_t)ppw << 1) - pw32) + (int32_t)s_f));
                    this_s += amt(bthis(t), sq);
                    next += amt(bnext(t), sq);
                    this_s -= q28mul(step, ithis(t));
                    next -= q28mul(step, inext(t));
                    high = 1;
                }
                if (high) {
                    uint32_t t;
                    if (!wrapped)
                        break;
                    wrapped = 0;
                    t = (uint32_t)sub_sample(sph, s_f);
                    this_s -= amt(bthis(t), 65536 - tri);
                    next -= amt(bnext(t), 65536 - tri);
                    this_s += q28mul(step, ithis(t));
                    next += q28mul(step, inext(t));
                    high = 0;
                }
            }
        }
        if (sync && reset) {
            sph = mulq16(s_f, rt);
            high = 0;
        }
        next += vso_naive(sph, pw32, up, down, tri, sq);
        ppw = pw;
        out[i] = (this_s << 1) - (1 << 28);
    }
    o->mph = mph;
    o->sph = sph;
    o->mf = mf;
    o->sf = sf;
    o->pw31 = pw31;
    o->ppw31 = ppw;
    o->ws = ws;
    o->next = next;
    o->high = high;
}

#define NOTCH (53687091)                                     /* kVariableSawNotchDepth 0.2, Q28 */

static void vsaw_init(struct macro_vsaw *o)
{
    o->ph = 0;
    o->f = 42949673u;
    o->pw31 = o->ppw31 = 1 << 30;
    o->ws = 0;
    o->next = 0;
    o->high = 0;
}

/* VariableSawOscillator::Render: writes Q28 into out */
static void vsaw_render(struct macro_vsaw *__restrict o, uint32_t f, int32_t pw31, int32_t ws, int32_t *__restrict out, int n)
{
    uint32_t up, down, ph = o->ph, fr = o->f;
    int32_t fd, pwd, wsd, pw = o->pw31, ppw = o->ppw31, w = o->ws, next = o->next, high = o->high, i;
    if (f > 0x40000000u)
        f = 0x40000000u;
    if (f >= 0x40000000u)
        pw31 = 1 << 30;
    else if (pw31 < (int32_t)f)
        pw31 = (int32_t)f;
    else if (pw31 > 0x7fffffff - (int32_t)f)
        pw31 = 0x7fffffff - (int32_t)f;
    fd = ((int32_t)f - (int32_t)fr) / n;
    pwd = (pw31 - pw) / n;
    wsd = (ws - w) / n;
    slopes(pw31, &up, &down);
    for (i = 0; i < n; i++) {
        int32_t this_s = next, tri_a, notch_a, step, v;
        uint32_t pw32, old;
        int wrapped;
        next = 0;
        fr += (uint32_t)fd;
        pw += pwd;
        w += wsd;
        tri_a = w;
        notch_a = 65536 - w;
        pw32 = (uint32_t)pw << 1;
        step = amt((int32_t)((((up + down) >> 1) * (fr >> 16)) >> 2), tri_a);
        old = ph;
        ph += fr;
        wrapped = ph < old;
        if (!high && (wrapped || ph >= pw32)) {
            int32_t notch = amt((1 << 28) + NOTCH - (int32_t)(pw32 >> 4), notch_a);
            uint32_t t = (uint32_t)sub_sample(ph - pw32, (uint32_t)((int32_t)(((uint32_t)ppw << 1) - pw32) + (int32_t)fr));
            this_s += q28mul(notch, bthis(t));
            next += q28mul(notch, bnext(t));
            this_s -= q28mul(step, ithis(t));
            next -= q28mul(step, inext(t));
            high = 1;
        }
        if (high && wrapped) {
            int32_t notch = amt((1 << 28) + NOTCH, notch_a);
            uint32_t t = (uint32_t)sub_sample(ph, fr);
            this_s -= q28mul(notch, bthis(t));
            next -= q28mul(notch, bnext(t));
            this_s += q28mul(step, ithis(t));
            next += q28mul(step, inext(t));
            high = 0;
        }
        if (ph < pw32)
            v = amt((int32_t)(ph >> 4), notch_a) + amt((int32_t)(((ph >> 16) * up) >> 3), tri_a);
        else
            v = amt((1 << 28) + NOTCH, notch_a) + amt((1 << 28) - (int32_t)((((ph - pw32) >> 16) * down) >> 3), tri_a);
        next += v;
        ppw = pw;
        out[i] = fmac1((this_s << 1) - (1 << 28), 1789569707);   /* / (1 + 0.2) */
    }
    o->ph = ph;
    o->f = f;
    o->pw31 = pw31;
    o->ppw31 = ppw;
    o->ws = ws;
    o->next = next;
    o->high = high;
}

static COLD void va_init(struct macro_va *v)
{
    vso_init(&v->primary, 0);
    vso_init(&v->auxiliary, 0x40000000u);                  /* set_master_phase(0.25) */
    vso_init(&v->sync, 0);
    vsaw_init(&v->saw);
    v->sq_gain = v->saw_gain = 0;
}

/* SemitonesToRatio's tables, Q31: 2^(k / 12) and 2^(k / 256 / 12) */
static const uint32_t pitch_semi[12] = {
    2147483648u, 2275179671u, 2410468894u, 2553802834u, 2705659852u, 2866546760u,
    3037000500u, 3217589947u, 3408917802u, 3611622603u, 3826380858u, 4053909305u,
};
static const uint32_t pitch_fine[256] = {
    2147483648u, 2147968248u, 2148452957u, 2148937775u, 2149422703u, 2149907740u,
    2150392887u, 2150878143u, 2151363509u, 2151848984u, 2152334569u, 2152820263u,
    2153306067u, 2153791980u, 2154278004u, 2154764136u, 2155250379u, 2155736731u,
    2156223193u, 2156709765u, 2157196447u, 2157683238u, 2158170140u, 2158657151u,
    2159144272u, 2159631503u, 2160118844u, 2160606295u, 2161093856u, 2161581527u,
    2162069308u, 2162557199u, 2163045200u, 2163533311u, 2164021532u, 2164509864u,
    2164998306u, 2165486858u, 2165975520u, 2166464293u, 2166953175u, 2167442169u,
    2167931272u, 2168420486u, 2168909810u, 2169399245u, 2169888790u, 2170378446u,
    2170868212u, 2171358088u, 2171848076u, 2172338173u, 2172828382u, 2173318700u,
    2173809130u, 2174299670u, 2174790321u, 2175281083u, 2175771955u, 2176262939u,
    2176754033u, 2177245237u, 2177736553u, 2178227980u, 2178719517u, 2179211165u,
    2179702925u, 2180194795u, 2180686776u, 2181178868u, 2181671072u, 2182163386u,
    2182655811u, 2183148348u, 2183640996u, 2184133755u, 2184626625u, 2185119606u,
    2185612699u, 2186105903u, 2186599218u, 2187092644u, 2187586182u, 2188079831u,
    2188573592u, 2189067464u, 2189561447u, 2190055542u, 2190549748u, 2191044066u,
    2191538496u, 2192033037u, 2192527690u, 2193022454u, 2193517330u, 2194012317u,
    2194507417u, 2195002628u, 2195497950u, 2195993385u, 2196488931u, 2196984590u,
    2197480360u, 2197976241u, 2198472235u, 2198968341u, 2199464559u, 2199960888u,
    2200457330u, 2200953884u, 2201450549u, 2201947327u, 2202444217u, 2202941219u,
    2203438333u, 2203935560u, 2204432898u, 2204930349u, 2205427912u, 2205925587u,
    2206423375u, 2206921275u, 2207419287u, 2207917412u, 2208415649u, 2208913999u,
    2209412461u, 2209911035u, 2210409722u, 2210908522u, 2211407434u, 2211906458u,
    2212405596u, 2212904845u, 2213404208u, 2213903683u, 2214403271u, 2214902972u,
    2215402785u, 2215902712u, 2216402751u, 2216902903u, 2217403167u, 2217903545u,
    2218404036u, 2218904639u, 2219405356u, 2219906185u, 2220407128u, 2220908183u,
    2221409352u, 2221910633u, 2222412028u, 2222913536u, 2223415157u, 2223916892u,
    2224418739u, 2224920700u, 2225422774u, 2225924961u, 2226427262u, 2226929676u,
    2227432204u, 2227934844u, 2228437599u, 2228940466u, 2229443447u, 2229946542u,
    2230449750u, 2230953072u, 2231456507u, 2231960056u, 2232463719u, 2232967495u,
    2233471385u, 2233975388u, 2234479506u, 2234983737u, 2235488082u, 2235992540u,
    2236497113u, 2237001799u, 2237506600u, 2238011514u, 2238516542u, 2239021684u,
    2239526940u, 2240032310u, 2240537794u, 2241043393u, 2241549105u, 2242054931u,
    2242560872u, 2243066927u, 2243573095u, 2244079379u, 2244585776u, 2245092288u,
    2245598914u, 2246105654u, 2246612509u, 2247119478u, 2247626561u, 2248133759u,
    2248641071u, 2249148498u, 2249656039u, 2250163695u, 2250671465u, 2251179350u,
    2251687350u, 2252195464u, 2252703693u, 2253212037u, 2253720495u, 2254229068u,
    2254737756u, 2255246558u, 2255755475u, 2256264508u, 2256773655u, 2257282917u,
    2257792294u, 2258301786u, 2258811392u, 2259321114u, 2259830951u, 2260340903u,
    2260850970u, 2261361152u, 2261871449u, 2262381861u, 2262892389u, 2263403032u,
    2263913790u, 2264424663u, 2264935651u, 2265446755u, 2265957974u, 2266469309u,
    2266980759u, 2267492324u, 2268004005u, 2268515801u, 2269027713u, 2269539740u,
    2270051883u, 2270564141u, 2271076515u, 2271589004u, 2272101610u, 2272614330u,
    2273127167u, 2273640119u, 2274153187u, 2274666371u,
};

/* inc x 2^(semitones / 12), semitones Q16, read in steps of 1/256 semitone as Plaits' NoteToFrequency does */
static uint32_t va_freq(uint32_t inc, int32_t semis)
{
    int32_t q = semis >> 8, oct = (q + 3072 * 16) / 3072 - 16, r = q - oct * 3072;
    return sat_shl(mulhi(inc, mulhi(pitch_semi[r >> 8], pitch_fine[r & 255])), oct + 2);
}

static void va_render(struct macro_va *__restrict v, const uint8_t *p, uint32_t inc, int32_t *__restrict out,
                      int32_t *__restrict aux, int n, int want_out, int want_aux)
{
    static const int32_t iv[5] = { 0, 459407, 787087, 1245839, 1573519 };   /* 0, 7.01, 12.01, 19.01, 24.01 */
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]);
    int32_t d, sign, di, df, sq, det, sync_amount, shape, pw, i;
    int32_t a[32], b[32];
    uint32_t aux_f;
    /* ComputeDetuning(HARMONICS): -24 .. 24 semitones, flat on the intervals */
    d = harm * 2 + ((harm * 3277) >> 16) - 67174;                 /* 2.05 HARMONICS - 1.025 */
    d = d < -65536 ? -65536 : d > 65536 ? 65536 : d;
    sign = d < 0 ? -1 : 1;
    d *= sign;
    d = d * 4 - d / 10000;                                       /* x 3.9999 */
    di = d >> 16;
    df = d & 0xffff;
    sq = (int32_t)((((uint32_t)df * (uint32_t)df) >> 16) * (uint32_t)(196608 - 2 * df) >> 16);   /* Squash */
    sq = (int32_t)((((uint32_t)sq * (uint32_t)sq) >> 16) * (uint32_t)(196608 - 2 * sq) >> 16);
    det = (iv[di] + fmac1(iv[di + 1] - iv[di], sq >= 65536 ? 0x7fffffff : sq << 15)) * sign;
    aux_f = va_freq(inc, det);
    sync_amount = (int32_t)(((uint32_t)timb * (uint32_t)timb) >> 16);
    shape = (morph * 3) >> 1;
    shape = shape > 65535 ? 65535 : shape;
    pw = (1 << 30) + ((((morph - 43254) * 47841) >> 16) << 16);   /* 0.5 + (MORPH - 0.66) 1.46, Q31 */
    pw = pw < (1 << 30) ? 1 << 30 : pw > 2136746229 ? 2136746229 : pw;   /* 0.5 .. 0.995 */
    for (i = 0; i < n; i++)
        out[i] = aux[i] = 0;
    if (want_aux) {                                              /* monster sync */
        uint32_t ps = va_freq(inc, sync_amount * 48), as = va_freq(inc, det + sync_amount * 48);
        vso_render(&v->primary, 1, inc, ps, pw, shape, a, n);
        vso_render(&v->auxiliary, 1, aux_f, as, pw, shape, b, n);
        for (i = 0; i < n; i++)
            aux[i] = (((b[i] - a[i]) >> 1) + 4096) >> 13;       /* Q15 */
    }
    if (want_out) {
        int32_t spw, ratio, sg, swpw, swsh, swg, m, st, wt, sgd, wgd;
        spw = timb + ((timb * 19661) >> 16) - 9830;            /* 1.3 TIMBRE - 0.15, Q16 */
        spw = spw < 328 ? 328 : spw > 32768 ? 32768 : spw;    /* 0.005 .. 0.5 */
        ratio = timb < 32768 ? 0 : (int32_t)((((uint32_t)(timb - 32768) * (uint32_t)(timb - 32768)) >> 16) * 192);   /* (T - .5)^2 4 48 */
        sg = timb * 8 > 65536 ? 65536 : timb * 8;
        swpw = morph < 32768 ? morph + 32768 : 65536 - (morph - 32768) * 2;
        swpw += (swpw * 6554) >> 16;                           /* x 1.1 */
        swpw = swpw < 328 ? 328 : swpw > 65535 ? 65535 : swpw;
        swsh = 655360 - 21 * morph;                            /* 10 - 21 MORPH */
        swsh = swsh < 0 ? 0 : swsh > 65536 ? 65536 : swsh;
        swg = 8 * (65536 - morph);
        swg = swg < 1311 ? 1311 : swg > 65536 ? 65536 : swg;  /* 0.02 .. 1 */
        vso_render(&v->sync, 1, inc, va_freq(inc, ratio), spw << 15, 65536, a, n);
        vsaw_render(&v->saw, aux_f, swpw >= 65535 ? 0x7fffffff : swpw << 15, swsh, b, n);
        m = sg > swg ? sg : swg;
        st = (int32_t)(((uint32_t)sg * 19661u / (uint32_t)m) << 8);   /* square gain 0.3 / max, Q24 */
        wt = (int32_t)(((uint32_t)swg * 32768u / (uint32_t)m) << 8);  /* saw gain 0.5 / max */
        sgd = (st - v->sq_gain) / n;
        wgd = (wt - v->saw_gain) / n;
        for (i = 0; i < n; i++) {
            v->sq_gain += sgd;
            v->saw_gain += wgd;
            out[i] = ((fmac1(b[i], v->saw_gain << 7) + fmac1(a[i], v->sq_gain << 7)) + 4096) >> 13;
        }
        v->sq_gain = st;
        v->saw_gain = wt;
    }
}

/* ---- MODAL (Model-TG): plaits/dsp/engine/modal_engine.cc, physical_modelling/modal_voice.cc, resonator.cc ---------- *
 * A struck resonator: 24 band-pass modes, their spacing set by HARMONICS (stiffness: from bell-like through the
 * harmonic series to stretched), excited on each trig by a click filtered by TIMBRE (brightness); MORPH is the
 * damping. OUT: the modes (Plaits limits it); AUX: the click. The modes' Q goes into the tens of thousands, where the
 * Q23 denominator of svf_coefs_g cannot tell the damping apart, so their coefficients are worked out with a 32-bit
 * mantissa and an exponent (struct uf), and the filters run in svf_hq's form of the same trapezoidal SVF. */

#ifndef MODAL_BLOCK
#define MODAL_BLOCK 12
#endif
#define MODAL_LP_NEW (-1)

/* x = m 2^(e - 30), m in [2^30, 2^31), or m = 0 for 0: 31-bit mantissas, so a product is one fmac1 on the EMAC */

static inline struct uf uf_q(uint32_t v, int32_t q)               /* v / 2^q */
{
    struct uf r;
    int z;
    if (!v) {
        r.m = 0;
        r.e = -128;
        return r;
    }
    z = clz32(v);
    if (z) {
        r.m = (int32_t)(v << (z - 1));
        r.e = 31 - z - q;
    } else {
        r.m = (int32_t)(v >> 1);
        r.e = 31 - q;
    }
    return r;
}

static inline struct uf uf_mul(struct uf a, struct uf b)
{
    struct uf r;
    int32_t p;
    if (!a.m || !b.m)
        return uf_q(0, 0);
    p = fmac1(a.m, b.m);                                            /* [2^29, 2^31) */
    if (p & 0x40000000) {
        r.m = p;
        r.e = a.e + b.e + 1;
    } else {
        r.m = p << 1;
        r.e = a.e + b.e;
    }
    return r;
}

static inline struct uf uf_add(struct uf a, struct uf b)            /* a, b >= 0 */
{
    int32_t d;
    uint32_t s;
    if (!b.m)
        return a;
    if (!a.m)
        return b;
    if (a.e < b.e) {
        struct uf t = a;
        a = b;
        b = t;
    }
    d = a.e - b.e;
    s = (uint32_t)a.m + (d > 30 ? 0 : (uint32_t)b.m >> d);
    if (s & 0x80000000u) {
        a.m = (int32_t)(s >> 1);
        a.e++;
    } else
        a.m = (int32_t)s;
    return a;
}

/* 1 / a: a 16-bit quotient, then two Newton steps */
static struct uf uf_recip(struct uf a)
{
    struct uf o;
    int32_t r, d, k;
    o.e = -a.e - 1;
    if (a.m == 0x40000000) {
        o.m = a.m;
        o.e = -a.e;
        return o;
    }
    r = (int32_t)((0x40000000u / (uint32_t)(a.m >> 15)) << 16);     /* 2^61 / m, in (2^30, 2^31) */
    if (r <= 0)
        r = 0x7fffffff;
    for (k = 0; k < 2; k++) {
        d = 0x40000000 - fmac1(a.m, r);                             /* (1 - m r / 2^61) 2^30 */
        r += fmac1(r, d << 1);
        if (r <= 0)
            r = 0x7fffffff;
    }
    o.m = r;
    return o;
}

static inline uint32_t uf_to(struct uf a, int32_t q)               /* a 2^q, saturated */
{
    int32_t sh = a.e - 30 + q;
    if (!a.m)
        return 0;
    if (sh > 1)
        return 0xffffffffu;
    if (sh == 1)
        return (uint32_t)a.m << 1;
    return sh < -31 ? 0 : (uint32_t)a.m >> -sh;
}

/* 2^(semitones / 12), semitones Q16, floored to 1/256 semitone as SemitonesToRatio reads its tables */
static struct uf uf_semis(int32_t semis)
{
    int32_t q = semis >> 8, oct = (q + 3072 * 16) / 3072 - 16, r = q - oct * 3072;
    struct uf x = uf_q(mulhi(pitch_semi[r >> 8], pitch_fine[r & 255]), 30);
    x.e += oct;
    return x;
}

/* OnePole::tan<FREQUENCY_FAST>: f (pi + f^2 (a + b f^2)), f a phase increment up to 0.499 (below 2^31) */
static struct uf tan_fast(uint32_t finc)
{
    int32_t f2 = fmac1((int32_t)finc, (int32_t)finc) >> 1;          /* Q32, up to 0.25 */
    int32_t p = 678339520 + fmac1(f2, 1871914112);            /* a + b f^2, Q26 */
    p = 421657428 + fmac1(f2, p);                       /* Q27 */
    return uf_mul(uf_q(finc, 32), uf_q((uint32_t)p, 27));
}

/* a trapezoidal SVF's a1 = 1 / D, a2 = g / D, a3 = g^2 / D, D = 1 + g (g + 1/q), as q / (q (1 + g^2) + g) */
static void svf_hq(struct svf_c *c, struct uf g, struct uf q)
{
    const struct uf one = {0x40000000, 0};
    struct uf a1, a2, a3;
    uint32_t v;
    a1 = uf_mul(q, uf_recip(uf_add(uf_mul(q, uf_add(one, uf_mul(g, g))), g)));
    a2 = uf_mul(g, a1);
    a3 = uf_mul(g, a2);
    v = uf_to(a1, 31);
    c->a1 = v > 0x7fffffffu ? 0x7fffffff : (int32_t)v;
    v = uf_to(a2, 31);
    c->a2 = v > 0x7fffffffu ? 0x7fffffff : (int32_t)v;
    v = uf_to(a3, 31);
    c->a3 = v > 0x7fffffffu ? 0x7fffffff : (int32_t)v;
    c->k2 = 0;
}

static const int32_t modal_stiffness[65] = {                       /* lut_stiffness, Q24 */
    -1048576, -983040, -917504, -851968, -786432, -720896, -655360, -589824,
    -524288, -458752, -393216, -327680, -262144, -196608, -131072, -65536,
    0, 0, 0, 0, 16938, 40535, 67147, 97158,
    131003, 169172, 212218, 260762, 315508, 377248, 446875, 525398,
    613951, 713818, 826443, 953456, 1096695, 1258232, 1440407, 1645855,
    1877549, 2138843, 2433517, 2765837, 3140610, 3563262, 4039909, 4577448,
    5183658, 5867313, 6638307, 7507796, 8488364, 9594201, 10841311, 12247741,
    13833846, 15622578, 16777848, 16871825, 17582612, 19864067, 24446104, 33554432,
    33554432,
};
static const int32_t modal_amp[MACRO_MODAL_MODES] = {               /* Resonator::Init(0.015): cosine x 0.25, Q31 */
    536870912, 535904512, 533012352, 528215168, 521547521, 513057472,
    502806079, 490867199, 477326785, 462282304, 445842112, 428124608,
    409257280, 389376031, 368623999, 347150593, 325110431, 302662207,
    279967553, 257189855, 234493120, 212040784, 189994496, 168513008,
};

/* one mode over the block: SVF_STEP, its band-pass x gain added into out. On the ColdFire, by hand: the two
 * sums on ACC0 and ACC1 side by side, the same products and rounding as fmac2 / fmac1 (so the same samples). */
static inline void modal_mode(struct macro_svf *f, const int32_t *mc, const int32_t *in, int32_t *out, int n)
{
    int32_t s1 = f->s1, s2 = f->s2;
#if defined(__mcoldfire__)
    int32_t v3, bp, lp, h = 32768, cnt = n;
    __asm__ volatile ("1:\n\t"
                      "move.l (%[in])+,%[v3]\n\t"
                      "sub.l %[s2],%[v3]\n\t"
                      "mac.l %[a1],%[s1],%%acc0\n\t"
                      "mac.l %[a2],%[s1],%%acc1\n\t"
                      "mac.l %[a2],%[v3],%%acc0\n\t"
                      "mac.l %[a3],%[v3],%%acc1\n\t"
                      "mac.l %[h],%[h],%%acc0\n\t"
                      "mac.l %[h],%[h],%%acc1\n\t"
                      "movclr.l %%acc0,%[bp]\n\t"
                      "movclr.l %%acc1,%[lp]\n\t"
                      "mac.l %[g],%[bp],%%acc0\n\t"
                      "add.l %[s2],%[lp]\n\t"
                      "neg.l %[s1]\n\t"
                      "add.l %[bp],%[s1]\n\t"
                      "add.l %[bp],%[s1]\n\t"
                      "neg.l %[s2]\n\t"
                      "add.l %[lp],%[s2]\n\t"
                      "add.l %[lp],%[s2]\n\t"
                      "mac.l %[h],%[h],%%acc0\n\t"
                      "movclr.l %%acc0,%[v3]\n\t"
                      "add.l %[v3],(%[out])+\n\t"
                      "subq.l #1,%[cnt]\n\t"
                      "bne.s 1b"
                      : [s1] "+d"(s1), [s2] "+d"(s2), [v3] "=&d"(v3), [bp] "=&d"(bp), [lp] "=&d"(lp), [cnt] "+d"(cnt),
                        [in] "+a"(in), [out] "+a"(out)
                      : [a1] "a"(mc[0]), [a2] "a"(mc[1]), [a3] "a"(mc[2]), [g] "a"(mc[3]), [h] "d"(h)
                      : "cc", "memory");
#else
    int32_t a1 = mc[0], a2 = mc[1], a3 = mc[2], gain = mc[3], i;
    for (i = 0; i < n; i++) {
        int32_t v3 = in[i] - s2, bp = fmac2(a1, s1, a2, v3), lp = s2 + fmac2(a2, s1, a3, v3);
        s1 = bp + bp - s1;
        s2 = lp + lp - s2;
        out[i] += fmac1(gain, bp);
    }
#endif
    f->s1 = s1;
    f->s2 = s2;
}

static COLD void modal_init(struct macro_modal *md)
{
    int32_t *w = (int32_t *)md, *end = (int32_t *)(md + 1);
    while (w < end)
        *w++ = 0;
#ifndef MODAL_FROM_ZERO
    md->harm_lp = MODAL_LP_NEW;
#endif
}

static void modal_render(struct macro_modal *__restrict md, const uint8_t *p, uint32_t inc, int32_t *__restrict out,
                         int32_t *__restrict aux, int n)
{
    const struct uf one = {0x40000000, 0};
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]);
    int32_t cp, s, st, sf, stiff, bright, damp, bb, b2, i, k, exc[32];
    uint32_t cut, mf;
    struct uf f0, harmonic, q, qloss, mq;
    struct svf_c c;
    /* ONE_POLE(harmonics_lp_, HARMONICS, 0.01) once a 12-sample block: 1 - 0.99^(n / 12) */
    cp = n == MODAL_BLOCK ? 21474836 : (65536 - (int32_t)exp2_q16(-950 * n / MODAL_BLOCK)) << 15;   /* Q31 */
    if (md->harm_lp == MODAL_LP_NEW)
        md->harm_lp = harm << 8;
    {                                                             /* rounded, and onto the target at the end */
        int32_t d = (harm << 8) - md->harm_lp, step = fmac1(d, cp);
        md->harm_lp = step ? md->harm_lp + step : harm << 8;
    }
    s = (md->harm_lp + 128) >> 8;                                 /* structure, Q16 */
    /* ModalVoice: accent 0.8 */
    bright = timb + (65536 - timb) / 5;
    damp = morph + (65536 - morph) / 5;
    bright = bright > 65535 ? 65535 : bright;
    damp = damp > 65535 ? 65535 : damp;
    bb = (int32_t)(((uint32_t)bright * (uint32_t)(131072 - bright)) >> 16);
    cut = va_freq(inc << 1, (bb - 32768) * 60);
    cut = cut > 2143188679u ? 2143188679u : cut;                     /* 0.499 */
    /* the click, through ResonatorSvf<1> (low-pass, q 1.5) */
    if (cut != (uint32_t)md->key_cut) {
        md->key_cut = (int32_t)cut;
        svf_hq(&c, tan_fast(cut), uf_q(3, 1));
        md->exc_c[0] = c.a1;
        md->exc_c[1] = c.a2;
        md->exc_c[2] = c.a3;
    }
    c.a1 = md->exc_c[0];
    c.a2 = md->exc_c[1];
    c.a3 = md->exc_c[2];
    {
        struct macro_svf f = md->exc;
        int32_t bp, lp;
        i = 0;
        if (md->trig) {                                            /* (0.12 + 0.08 accent) (1 - damping / 2) 2^(24 c^2) / c */
            int32_t imp, v3;
            struct uf a = uf_mul(uf_q((uint32_t)(65536 - (damp >> 1)), 16), uf_q(395136630u, 31));   /* 0.184 */
            a = uf_mul(uf_mul(a, uf_semis((int32_t)(mulhi(cut, cut) >> 16) * 24)), uf_recip(uf_q(cut, 32)));
            imp = (int32_t)uf_to(a, 18);                            /* Q18 */
            imp = imp < 0 ? 0x7fffffff : imp;
            v3 = imp - (f.s2 >> 6);
            bp = fmac1(c.a1, f.s1) + (fmac1(c.a2, v3) << 6);
            lp = f.s2 + fmac1(c.a2, f.s1) + (fmac1(c.a3, v3) << 6);
            f.s1 = bp + bp - f.s1;
            f.s2 = lp + lp - f.s2;
            exc[0] = lp;
            i = 1;
            md->trig = 0;
        }
        for (; i < n; i++) {
            SVF_STEP(f, c, 0, bp, lp);
            exc[i] = lp;
        }
        md->exc = f;
    }
    for (i = 0; i < n; i++) {
        aux[i] = (exc[i] + 256) >> 9;                               /* Q15 */
        out[i] = 0;
    }
    /* the modes' coefficients: worked out again only when the pitch or a knob has moved */
    if (inc != md->key[0] || s != md->key[1] || bright != md->key[2] || damp != md->key[3]) {
        md->key[0] = inc;
        md->key[1] = s;
        md->key[2] = bright;
        md->key[3] = damp;
        /* Resonator::Process */
        st = (s >> 10) > 63 ? 63 : s >> 10;
        stiff = modal_stiffness[st] + fmac1(modal_stiffness[st + 1] - modal_stiffness[st], (s & 1023) << 21);
        sf = (1 << 24) + stiff + fmac1(stiff, stiff < 0 ? 1997159793 : 2104533975);   /* NthHarmonicCompensation(3) */
        f0 = uf_mul(uf_q(inc, 32), uf_recip(uf_q((uint32_t)sf, 24)));
        harmonic = f0;
        q = uf_semis(damp * 79 + (int32_t)(((uint32_t)damp * 45875u) >> 16));              /* q_sqrt = 2^(79.7 damping / 12) */
        q = uf_mul(uf_mul(q, q), uf_q(500, 0));
        b2 = (int32_t)(((uint32_t)bright * (uint32_t)(65536 - ((s * 19661) >> 16))) >> 16);   /* x (1 - 0.3 structure) */
        b2 = (int32_t)(((uint32_t)b2 * (uint32_t)(65536 - ((damp * 19661) >> 16))) >> 16);   /* x (1 - 0.3 damping) */
        qloss = uf_q(((((uint32_t)b2 * (uint32_t)(131072 - b2)) >> 16) * 55706u >> 16) + 9830, 16);   /* b (2 - b) 0.85 + 0.15 */
        sf = 1 << 24;
        for (k = 0; k < MACRO_MODAL_MODES; k++) {
            struct uf mfu = uf_mul(harmonic, uf_q((uint32_t)sf, 24));
            int32_t *mc = md->coef[k];
            mf = uf_to(mfu, 32);
            if (mf > 2143188679u) {
                mf = 2143188679u;
                mfu = uf_q(mf, 32);
            }
            mq = uf_add(one, uf_mul(mfu, q));
            svf_hq(&c, tan_fast(mf), mq);
            mc[0] = c.a1;
            mc[1] = c.a2;
            mc[2] = c.a3;
            mc[3] = fmac1(modal_amp[k], (int32_t)(0x80000000u - mf));     /* gain x (1 - 2 f) */
            sf += stiff;
            stiff = fmac1(stiff, stiff < 0 ? 1997159793 : 2104533975);   /* x 0.93, x 0.98 */
            harmonic = uf_add(harmonic, f0);
            q = uf_mul(q, qloss);
        }
    }
    for (k = 0; k < MACRO_MODAL_MODES; k++)
        modal_mode(&md->mode[k], md->coef[k], exc, out, n);
    for (i = 0; i < n; i++)
        out[i] = (out[i] + 64) >> 7;                                /* Q17, for the limiter */
}

/* ---- STRING (Model-TG): plaits/dsp/engine/string_engine.cc, physical_modelling/string_voice.cc, string.cc ------ *
 * Three Karplus-Strong strings in turn (each trig plucks the next, the others ring on): a burst of filtered noise one
 * period long (TIMBRE: brightness), a delay line with a damping low-pass (MORPH: decay, endless at the top) and, from
 * HARMONICS, a curved bridge (below the middle) or dispersion through an allpass (above). OUT: the strings (Plaits
 * limits it); AUX: the bursts. Signals Q24, delays Q20 (samples; up to 2048, as dispersion and the curved bridge stretch them past 1020). */

static const int32_t string_shift[129] = {                         /* lut_svf_shift[0..128], Q31 */
    1610612736, 1630343906, 1650009481, 1669544948, 1688887922, 1707979097,
    1726763091, 1745189139, 1763211629, 1780790470, 1797891299, 1814485525,
    1830550242, 1846068024, 1861026615, 1875418564, 1889240793, 1902494149,
    1915182937, 1927314462, 1938898579, 1949947278, 1960474287, 1970494717,
    1980024741, 1989081306, 1997681885, 2005844259, 2013586331, 2020925971,
    2027880884, 2034468504, 2040705906, 2046609740, 2052196174, 2057480857,
    2062478892, 2067204814, 2071672582, 2075895575, 2079886593, 2083657863,
    2087221051, 2090587270, 2093767101, 2096770602, 2099607332, 2102286367,
    2104816317, 2107205347, 2109461196, 2111591196, 2113602289, 2115501047,
    2117293690, 2118986101, 2120583846, 2122092186, 2123516097, 2124860281,
    2126129183, 2127327001, 2128457705, 2129525044, 2130532557, 2131483591,
    2132381305, 2133228681, 2134028538, 2134783536, 2135496188, 2136168866,
    2136803810, 2137403133, 2137968834, 2138502796, 2139006799, 2139482524,
    2139931555, 2140355389, 2140755441, 2141133044, 2141489457, 2141825870,
    2142143403, 2142443117, 2142726011, 2142993028, 2143245061, 2143482949,
    2143707486, 2143919422, 2144119463, 2144308278, 2144486495, 2144654710,
    2144813485, 2144963348, 2145104800, 2145238314, 2145364334, 2145483281,
    2145595553, 2145701523, 2145801545, 2145895954, 2145985064, 2146069173,
    2146148561, 2146223494, 2146294221, 2146360978, 2146423989, 2146483462,
    2146539599, 2146592584, 2146642595, 2146689800, 2146734355, 2146776410,
    2146816104, 2146853570, 2146888934, 2146922313, 2146953818, 2146983555,
    2147011623, 2147038116, 2147063122,
};

/* OnePole::tan<FREQUENCY_DIRTY>: f (pi + a f^2) */
static struct uf tan_dirty_uf(uint32_t finc)
{
    int32_t f2 = fmac1((int32_t)finc, (int32_t)finc) >> 1;          /* Q32 */
    int32_t p = 421657428 + fmac1(f2, 777385408);           /* Q27 */
    return uf_mul(uf_q(finc, 32), uf_q((uint32_t)p, 27));
}

static void svf_store(int32_t *c3, struct uf g, struct uf q)
{
    struct svf_c c;
    svf_hq(&c, g, q);
    c3[0] = c.a1;
    c3[1] = c.a2;
    c3[2] = c.a3;
}

static COLD void string_init(struct macro_string *st)
{
    int32_t *w = (int32_t *)st, *end = (int32_t *)(st + 1), i;
    while (w < end)
        *w++ = 0;
    st->active = MACRO_STRINGS - 1;
    for (i = 0; i < MACRO_STRINGS; i++) {
        st->s[i].f0 = 42949673u;                                    /* f0_ 0.01 */
        st->s[i].delay = 100 << 20;                                 /* delay_ 100 */
    }
}

/* the delay line read between taps (Read), and with Hermite interpolation (ReadHermite); d Q20 */
static inline int32_t dl_read(const int32_t *line, int32_t wp, int32_t d)
{
    int32_t i = d >> 20, a = line[(wp + i) & 1023], b = line[(wp + i + 1) & 1023];
    return a + fmac1(b - a, (d & 0xfffff) << 11);
}

static inline int32_t dl_hermite(const int32_t *line, int32_t wp, int32_t d)
{
    int32_t t = wp + (d >> 20) + 1024, f = (d & 0xfffff) << 11;
    int32_t xm1 = line[(t - 1) & 1023], x0 = line[t & 1023], x1 = line[(t + 1) & 1023], x2 = line[(t + 2) & 1023];
    int32_t c = (x1 - xm1) >> 1, v = x0 - x1, w = c + v, a = w + v + ((x2 - x0) >> 1), bn = w + a;
    return fmac1(fmac1(fmac1(a, f) - bn, f) + c, f) + x0;
}

/* one string over the block: StringVoice::Render (no sustain) and String::Process */
static void string_one(struct macro_strv *__restrict v, uint32_t *rng, int trig, int32_t structure, int32_t b30,
                       int32_t damp, int32_t *__restrict out, int32_t *__restrict aux, int n)
{
    int32_t tmp[32], i, nl, curved, dc, ratio, delay, target, comp, sp, sc, na, nf, bc, apg, dinc, bb, bright, d30;
    int32_t wp = v->wp, steps = 0;
    uint32_t inc = v->f0, df, phase;
    struct macro_svf ef = v->exc, lf = v->damp;
    struct svf_c c;
    /* accent 0.8: brightness (TIMBRE^2) and damping in Q30, then Q16 for what needs no more */
    b30 += (0x40000000 - b30) / 5;
    d30 = (damp << 14) + (0x40000000 - (damp << 14)) / 5;
    bright = b30 >> 14;
    damp = d30 >> 14;
    if (trig) {                                                     /* the burst: one period of noise, filtered */
        uint32_t cut;
        bb = (int32_t)(((uint32_t)bright * (uint32_t)(131072 - bright)) >> 16);
        cut = va_freq(inc, (bb - 32768) * 72 + (24 << 16));        /* 4 f0 2^((b (2 - b) - 0.5) 72 / 12) */
        cut = cut > 2143188679u ? 2143188679u : cut;
        svf_store(v->exc_c, tan_dirty_uf(cut), uf_q(1, 1));
        v->remaining = (int32_t)uf_to(uf_recip(uf_q(inc, 32)), 0);
    }
    for (i = 0; i < n; i++)
        tmp[i] = 0;
    for (i = 0; i < n && v->remaining > 0; i++, v->remaining--)
        tmp[i] = (int32_t)(rnd32(rng) - 0x80000000u) >> 7;          /* 2 GetFloat() - 1, Q24 */
    c.a1 = v->exc_c[0];
    c.a2 = v->exc_c[1];
    c.a3 = v->exc_c[2];
    for (i = 0; i < n; i++) {
        int32_t bp, lp;
        SVF_STEP(ef, c, tmp[i], bp, lp);
        (void)bp;
        tmp[i] = lp;
        aux[i] += lp;
    }
    v->exc = ef;
    /* String::Process */
    {                                                               /* the non-linearity, Q30 */
        int32_t s30 = structure << 14;
        if (s30 < 257698038) {                                      /* (s - 0.24) 4.166 */
            int32_t x = 257698038 - s30;
            nl = x * 4 + fmac1(x, 356482286);
            curved = 1;
        } else if (s30 > 279172874) {                               /* (s - 0.26) 1.35135 */
            int32_t x = s30 - 279172874;
            nl = x + fmac1(x, 754518380);
            curved = 0;
        } else {
            nl = 0;
            curved = 1;
        }
        nl = nl > 0x40000000 ? 0x40000000 : nl;
    }
    if (!inc)
        delay = 1020 << 20;
    else {
        uint32_t d = uf_to(uf_recip(uf_q(inc, 32)), 20);
        delay = d < (4u << 20) ? 4 << 20 : d > (1020u << 20) ? 1020 << 20 : (int32_t)d;
    }
    ratio = (int32_t)mulhi((uint32_t)delay << 2, inc);              /* delay f0, Q22 */
    ratio = ratio >= 4193885 ? 1 << 30 : ratio << 8;                /* >= 0.9999: 1 (and the phase at 1), Q30 */
    phase = ratio == 1 << 30 ? 1u << 30 : v->src_phase;
    dc = 786432 + (int32_t)((((uint32_t)damp * (uint32_t)damp) >> 16) * 60u) + bright * 24;   /* 12 + 60 d^2 + 24 b, Q16 */
    dc = dc > 84 << 16 ? 84 << 16 : dc;
    df = va_freq(inc, dc);
    df = df > 2143188679u ? 2143188679u : df;
    if (d30 >= 1020054733) {                                        /* to endless decay, from 0.95 */
        int32_t t = (d30 - 1020054733) * 40;                        /* 20 (d - 0.95), Q31 */
        t = t < 0 ? 0x7fffffff : t;
        b30 += fmac1(0x40000000 - b30, t);
        bright = b30 >> 14;
        df += (uint32_t)fmac1((int32_t)(2147054152u - df), t);     /* += t (0.4999 - f) */
        dc += fmac1((128 << 16) - dc, t);
    }
    svf_hq(&c, tan_fast(df), uf_q(1, 1));
    {
        int32_t ci = dc >> 16, cf = dc & 0xffff;
        comp = string_shift[ci] + fmac1(string_shift[ci + 1 > 128 ? 128 : ci + 1] - string_shift[ci], cf << 15);
    }
    target = fmac1(delay, comp);
    dinc = (target - v->delay) / n;
    sp = fmac1(fmac1(nl, 0x40000000 - (nl >> 1)), 483183821) << 3;   /* nl (2 - nl) 0.225, Q31 */
    sc = (delay >> 4) / 300;                                        /* 160 / 48000 x delay, Q16 */
    sc = sc < 65536 ? 65536 : sc > 137626 ? 137626 : sc;
    na = nl > 805306368 ? (nl - 805306368) * 4 : 0;                /* 4 (nl - 0.75), Q30 */
    na = fmac1(fmac1(na, na), 214748365) << 2;                      /* 0.1 nas^2, Q31 */
    nf = fmac1(b30, b30);                                           /* b^2, Q29 */
    nf = 128849019 + fmac1(nf, 2018634629) * 4;                    /* 0.06 + 0.94 b^2, Q31 */
    nf = nf < 0 ? 0x7fffffff : nf;
    bc = fmac1(fmac1(nl, nl), 21474836) << 2;                       /* 0.01 nl^2, Q31 */
    apg = nl ? -fmac1((int32_t)uf_to(uf_mul(uf_q((uint32_t)nl, 30), uf_recip(uf_q((uint32_t)nl + 161061274u, 30))), 31),
                     1327144894) : 0;                               /* -0.618 nl / (0.15 + nl), Q31 */
    for (i = 0; i < n; i++) {
        phase += (uint32_t)ratio;
        if (phase > 1u << 30) {
            int32_t d, s;
            phase -= 1u << 30;
            v->delay += dinc;
            steps++;
            d = v->delay;
            if (!curved) {
                int32_t noise = (int32_t)(rnd32(rng) - 0x80000000u) >> 2;   /* GetFloat() - 0.5, Q30 */
                v->disp += fmac1(noise - v->disp, nf);
                d += fmac1(d, fmac1(v->disp << 1, na));
            } else
                d -= fmac1(d, fmac1(v->curve, bc) << 7);
            if (!curved) {
                int32_t ap = fmac1(d, sp);                    /* delay x stretch_point */
                int32_t md = d - fmac1(fmac1(ap, 876173886 - fmac1(sp, 661424964)) << 2, sc << 13);
                if (ap >= 4 << 20 && md >= 4 << 20) {
                    int32_t r, wv;
                    s = dl_read(v->line, wp, md);
                    r = v->stretch[(v->swp + (ap >> 20)) & 255];
                    wv = s + fmac1(apg, r);
                    v->stretch[v->swp] = wv;
                    v->swp = (v->swp - 1) & 255;
                    s = r - fmac1(wv, apg);
                } else
                    s = dl_hermite(v->line, wp, d);
            } else {
                int32_t a_, val;
                s = dl_hermite(v->line, wp, d);
                a_ = s < 0 ? -s : s;
                val = a_ - 419430;                                  /* |s| - 0.025 */
                v->curve = val > 0 ? (s > 0 ? val * 2 : -val * 3) : 0;
            }
            s += tmp[i];
            s = s > 20 << 24 ? 20 << 24 : s < -(20 << 24) ? -(20 << 24) : s;
            v->dc_y = fmac1(v->dc_y, 2146588863) + s - v->dc_x;     /* DCBlocker, pole 1 - 20 / 48000 */
            v->dc_x = s;
            s = v->dc_y;
            {
                int32_t bp, lp;
                SVF_STEP(lf, c, s, bp, lp);
                (void)bp;
                s = lp;
            }
            v->line[wp] = s;
            wp = (wp - 1) & 1023;
            v->out1 = v->out0;
            v->out0 = s;
        }
        out[i] += phase >= 1u << 30 ? v->out0 : v->out1 + fmac1(v->out0 - v->out1, (int32_t)(phase << 1));   /* Crossfade */
    }
    if (steps == n)                                                 /* the ramp's end, exactly (as in float) */
        v->delay = target;
    v->wp = wp;
    v->damp = lf;
    v->src_phase = phase;
}

static void string_render(struct macro_string *__restrict st, uint32_t *rng, const uint8_t *p, uint32_t inc,
                          int32_t *__restrict out, int32_t *__restrict aux, int n)
{
    int32_t harm = k16(p[MACRO_P_HARM]), timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]), i, trig = st->trig;
    st->trig = 0;
    if (trig) {                                                     /* the string let go keeps its pitch of 14 blocks ago */
        st->s[st->active].f0 = st->f0_hist[(st->f0_wp + 14) & 15];
        st->active = (st->active + 1) % MACRO_STRINGS;
    }
    st->s[st->active].f0 = inc;
    st->f0_hist[st->f0_wp] = inc;
    st->f0_wp = (st->f0_wp - 1) & 15;
    for (i = 0; i < n; i++)
        out[i] = aux[i] = 0;
    for (i = 0; i < MACRO_STRINGS; i++)
        string_one(&st->s[i], rng, trig && i == st->active, harm, (int32_t)(((uint32_t)timb * (uint32_t)timb) >> 2), morph,
                   out, aux, n);
    for (i = 0; i < n; i++) {
        out[i] = (out[i] + 64) >> 7;                                /* Q17, for the limiter */
        aux[i] = (aux[i] + 256) >> 9;                               /* Q15 */
    }
}

/* ---- 6-OP (Model-TG): plaits/dsp/engine2/six_op_engine.cc, plaits/dsp/fm/ (voice.h, operator.h, envelope.h,
 * lfo.h, dx_units.h, algorithms.cc) ---------------------------------------------------------------------------- *
 * Plaits' DX7 engine with its three factory banks as one: HARMONICS picks one of 96 patches, TIMBRE brightens the
 * modulators, MORPH scales the envelope times. Two voices take the trigs in turn, rendered one per block over two
 * blocks (Plaits' staggered rendering). The gate is a fixed pulse from each trig (DX7_GATE_BLOCKS).
 * Operator outputs are phase offsets in turns, Q26 (Plaits quantizes them to 2^-26 too); amplitudes Q27; envelope
 * levels Q24 (log2 units of 1/8); envelope phases Q30. */

#include "macro_dx7.h"

#ifndef DX7_GATE_BLOCKS
#define DX7_GATE_BLOCKS 188                                         /* 125 ms: a 16th at 120 BPM (10 ms left the slow */
#endif                                                              /* pads and strings of bank C nearly silent) */
#define DX7_PREVIOUS (-0x7fffffff)                                  /* Envelope's PREVIOUS_LEVEL */
#define DX7_LEVEL_67 112407347                                      /* 6.7, Q24 */

/* Pow2Fast<1> and Pow2Fast<2> (the float tricks Plaits' FM code uses), x Q16 */
static struct uf dx_pow2_1(int32_t x)
{
    struct uf r;
    r.m = 0x40000000 + ((x & 0xffff) << 14);
    r.e = x >> 16;
    return r;
}

static struct uf dx_pow2_2(int32_t x)
{
    struct uf r;
    int32_t f = (x & 0xffff) << 14;                                 /* Q30 */
    r.m = 0x40000000 + fmac1(f, 1409889502 + (fmac1(f, 737673478) << 1));   /* 1 + f (0.6565 + 0.3435 f), Q30 */
    r.e = x >> 16;
    if (r.m < 0) {                                                  /* (f = 1: 2) */
        r.m = 0x40000000;
        r.e++;
    }
    return r;
}

static void dx_unpack(struct macro_dxpatch *p, int index)
{
    const uint8_t *d = &dx7_patches[index * 128];
    int i, j;
#define DXMIN(x, m) ((x) > (m) ? (m) : (x))
    for (i = 0; i < 6; i++) {
        const uint8_t *o = &d[i * 17];
        struct macro_dxop_p *q = &p->op[i];
        for (j = 0; j < 4; j++) {
            q->rate[j] = DXMIN(o[j] & 0x7f, 99);
            q->level[j] = DXMIN(o[4 + j] & 0x7f, 99);
        }
        q->bp = DXMIN(o[8] & 0x7f, 99);
        q->ld = DXMIN(o[9] & 0x7f, 99);
        q->rd = DXMIN(o[10] & 0x7f, 99);
        q->lc = o[11] & 3;
        q->rc = (o[11] >> 2) & 3;
        q->rs = o[12] & 7;
        q->ams = o[13] & 3;
        q->vs = (o[13] >> 2) & 7;
        q->out = DXMIN(o[14] & 0x7f, 99);
        q->mode = o[15] & 1;
        q->coarse = (o[15] >> 1) & 0x1f;
        q->fine = DXMIN(o[16] & 0x7f, 99);
        q->detune = DXMIN((o[12] >> 3) & 0xf, 14);
    }
    for (j = 0; j < 4; j++) {
        p->prate[j] = DXMIN(d[102 + j] & 0x7f, 99);
        p->plevel[j] = DXMIN(d[106 + j] & 0x7f, 99);
    }
    p->algorithm = d[110] & 0x1f;
    p->feedback = d[111] & 7;
    p->reset_phase = (d[111] >> 3) & 1;
    p->lrate = DXMIN(d[112] & 0x7f, 99);
    p->ldelay = DXMIN(d[113] & 0x7f, 99);
    p->lpmd = DXMIN(d[114] & 0x7f, 99);
    p->lamd = DXMIN(d[115] & 0x7f, 99);
    p->lreset = d[116] & 1;
    p->lwave = DXMIN((d[116] >> 1) & 7, 5);
    p->lpms = d[116] >> 4;
#undef DXMIN
}

static int dx_op_level(int level)                                   /* OperatorLevel */
{
    if (level < 20)
        return level < 15 ? (level * (36 - level)) >> 3 : 27 + level;
    return level + 28;
}

/* Envelope::Init(scale): the levels and increments Voice::Init leaves before a patch is set up */
static void dx_env_init(struct macro_dxenv *e)
{
    int i;
    e->stage = 3;
    e->phase = 0x40000000;
    e->start = 0;
    for (i = 0; i < 4; i++) {
        e->inc[i] = uf_q(1099512, 40);                              /* 0.001 (Voice::Render sets a patch up first) */
        e->level[i] = (1 << 24) >> i;
    }
    e->level[3] = 0;
}

static int32_t dx_env_value(const struct macro_dxenv *e, int reshape)
{
    int32_t from = e->start == DX7_PREVIOUS ? e->level[(e->stage + 3) & 3] : e->start, to = e->level[e->stage];
    int32_t ph = e->phase;
    if (reshape && from < to) {
        from = from < DX7_LEVEL_67 ? DX7_LEVEL_67 : from;
        to = to < DX7_LEVEL_67 ? DX7_LEVEL_67 : to;
        ph = fmac1(fmac1(ph, (5 << 28) - (ph >> 1)), 1431656448) << 2;   /* phase (2.5 - phase) 0.666667 */
    }
    return (fmac1(to - from, ph) << 1) + from;
}

/* Envelope::Render(gate, rate, ad_scale, release_scale) */
static int32_t dx_env_render(struct macro_dxenv *e, int gate, struct uf rate, struct uf ad, struct uf rel, int reshape)
{
    uint32_t d;
    if (gate) {
        if (e->stage == 3) {
            e->start = dx_env_value(e, reshape);
            e->stage = 0;
            e->phase = 0;
        }
    } else if (e->stage != 3) {
        e->start = dx_env_value(e, reshape);
        e->stage = 3;
        e->phase = 0;
    }
    d = uf_to(uf_mul(uf_mul(e->inc[e->stage], rate), e->stage == 3 ? rel : ad), 30);
    if (d >= (uint32_t)(0x40000000 - e->phase)) {
        if (e->stage >= 2)
            e->phase = 0x40000000;
        else {
            e->phase = 0;
            e->stage++;
        }
        e->start = DX7_PREVIOUS;
    } else
        e->phase += (int32_t)d;
    return dx_env_value(e, reshape);
}

/* PitchEnvelopeIncrement(rate): (1 + 192 r (r^4 + 0.3333)) / (21.3 x 44100), r = rate / 100 */
static struct uf dx_pitch_inc(int rate)
{
    int32_t r = rate * 10737418;                                    /* Q30 */
    int32_t r2 = fmac1(r, r) << 1, r4 = fmac1(r2, r2) << 1;
    int32_t t = fmac1(r, r4 + 357878150) << 1;                     /* r (r^4 + 0.3333), Q30 */
    struct uf v = uf_add(uf_q(1, 0), uf_mul(uf_q((uint32_t)t, 30), uf_q(192, 0)));
    return uf_mul(v, uf_recip(uf_q(939330, 0)));
}

/* the envelopes, the ratios and the LFO for a patch (Voice::Setup and Lfo::Set) */
static void dx_setup(struct macro_dxvoice *v)
{
    const struct macro_dxpatch *p = &v->p;
    const struct uf scale = uf_mul(uf_q(44100, 0), uf_recip(uf_q(3063830, 6)));   /* 44100 / 47872.34 */
    int i, j;
    for (i = 0; i < 4; i++) {                                       /* PitchEnvelope::Set: PitchEnvelopeLevel */
        int32_t l = (p->plevel[i] - 50) << 19, a = l + 335544, tail, x;   /* (level - 50) / 32, |l + 0.02|, Q24 */
        a = a < 0 ? -a : a;
        tail = a > (1 << 24) ? a - (1 << 24) : 0;
        x = fmac1(fmac1(tail << 3, tail << 4), 1424211155) << 3;   /* 5.3056 tail^2 */
        v->penv.level[i] = l + (fmac1(l << 4, x << 4) >> 1);       /* l (1 + 5.3056 tail^2) */
    }
    for (i = 0; i < 4; i++) {
        int32_t from = v->penv.level[(i + 3) & 3], to = v->penv.level[i];
        struct uf inc = dx_pitch_inc(p->prate[i]);
        if (from != to) {
            int32_t d = from > to ? from - to : to - from;
            inc = uf_mul(inc, uf_recip(uf_q((uint32_t)d, 24)));
        } else if (i != 3)
            inc = uf_q(13421773, 26);                               /* 0.2 */
        v->penv.inc[i] = uf_mul(inc, scale);
    }
    for (i = 0; i < 6; i++) {
        const struct macro_dxop_p *o = &p->op[i];
        struct macro_dxenv *e = &v->env[i];
        int global = dx_op_level(o->out);
        int32_t base;
        for (j = 0; j < 4; j++) {                                   /* OperatorEnvelope::Set */
            int ls = (dx_op_level(o->level[j]) & ~1) + global - 133;
            e->level[j] = ls < 1 ? 1 << 20 : ls << 21;               /* 0.125 x (ls < 1 ? 0.5 : ls) */
        }
        for (j = 0; j < 4; j++) {
            int rs = (o->rate[j] * 41) >> 6;
            struct uf inc = uf_q((uint32_t)((4 + (rs & 3)) << (2 + (rs >> 2))), 24);
            int32_t from = e->level[(j + 3) & 3], to = e->level[j];
            if (from == to) {
                inc = uf_mul(inc, uf_q(644245094, 30));             /* x 0.6 */
                if (j == 0 && !o->level[j])
                    inc = uf_mul(inc, uf_q(20, 0));
            } else if (from < to) {
                from = from < DX7_LEVEL_67 ? DX7_LEVEL_67 : from;
                to = to < DX7_LEVEL_67 ? DX7_LEVEL_67 : to;
                if (from == to)
                    inc = uf_q(1, 0);
                else
                    inc = uf_mul(inc, uf_mul(uf_q(1932735283u, 28), uf_recip(uf_q((uint32_t)(to - from), 24))));
            } else
                inc = uf_mul(inc, uf_recip(uf_q((uint32_t)(from - to), 24)));
            e->inc[j] = uf_mul(inc, scale);
        }
        v->headroom[i] = 127 - global;
        /* FrequencyRatio */
        if (o->mode == 0) {
            base = dx7_coarse[o->coarse] + ((o->detune - 7) * 98304) / 100;
            v->ratio[i] = uf_semis(base);
            if (o->fine)
                v->ratio[i] = uf_mul(v->ratio[i], uf_mul(uf_q(100 + o->fine, 0), uf_recip(uf_q(100, 0))));
        } else {
            base = ((o->coarse & 3) * 100 + o->fine) * 26125 + ((o->detune - 7) * 98304) / 100;   /* x 0.39864, Q16 */
            v->ratio[i] = uf_mul(uf_semis(base), uf_recip(uf_q(3063830, 6)));   /* Hz / 47872.34 (fixed) */
        }
    }
    /* Lfo::Set */
    {
        int rs = p->lrate == 0 ? 1 : (p->lrate * 165) >> 6, d;
        rs *= rs < 160 ? 11 : 11 + ((rs - 160) >> 4);
        v->lfreq = uf_to(uf_mul(uf_q((uint32_t)rs, 0), uf_q(8621109, 46)), 32);   /* x 0.005865 / 47872.34, Q32 */
        if (p->ldelay == 0)
            v->ldinc[0] = v->ldinc[1] = 0x7fffffff;
        else {
            d = 99 - p->ldelay;
            d = (16 + (d & 15)) << (1 + (d >> 4));
            v->ldinc[0] = uf_to(uf_mul(uf_q((uint32_t)d, 0), uf_q(8621109, 46)), 31);
            v->ldinc[1] = uf_to(uf_mul(uf_q((uint32_t)(d & 0xff80 ? d & 0xff80 : 0x80), 0), uf_q(8621109, 46)), 31);
        }
        v->amd = p->lamd * 10737418;                                /* Q30 */
        v->pmd = fmac1(p->lpmd * 21474836, dx7_pms[p->lpms]);       /* amd 0.01 x pms, Q29 */
    }
}

/* Lfo::Step(scale) */
static void dx_lfo_step(struct macro_dxvoice *v, uint32_t *rng, int scale)
{
    uint32_t old = v->lph, ph, dp;
    int32_t val;
    v->lph += v->lfreq * (uint32_t)scale;
    if (v->lph < old)
        v->lrand = (int32_t)(rnd32(rng) >> 2);                      /* GetFloat(), Q30 */
    ph = v->lph;
    switch (v->p.lwave) {
    case 0: val = (int32_t)((ph < 0x80000000u ? 0x80000000u - ph : ph - 0x80000000u) >> 1); break;
    case 1: val = (int32_t)((0u - ph) >> 2) + (ph ? 0 : 0x40000000); break;
    case 2: val = (int32_t)(ph >> 2); break;
    case 3: val = ph < 0x80000000u ? 0 : 0x40000000; break;
    case 4: {
        uint32_t q = ph + 0x80000000u, i = q >> 23;
        val = (0x40000000 + dx7_sine[i] + fmac1(dx7_sine[i + 1] - dx7_sine[i], (int32_t)((q << 9) >> 1))) >> 1;
        break;
    }
    default: val = v->lrand; break;
    }
    v->lval = val;
    dp = v->ldinc[v->ldp < 0x40000000u ? 0 : 1];
    v->ldp = dp >= (0x80000000u - v->ldp) / (uint32_t)scale ? 0x80000000u : v->ldp + dp * (uint32_t)scale;
}

/* (pitch_mod, Q27; amp_mod, Q28) from a voice's LFO */
static void dx_lfo_mods(const struct macro_dxvoice *l, int32_t *pm, int32_t *am)
{
    int32_t ramp = l->ldp < 0x40000000u ? 0 : (int32_t)(l->ldp - 0x40000000u);   /* Q30 */
    *pm = fmac1(fmac1(l->lval - 0x20000000, ramp), l->pmd);
    *am = fmac1(fmac1(0x40000000 - l->lval, ramp), l->amd);
}

/* RenderOperators<1, src, add>, one operator: src -2 the input buffer, -1 none, 0 its own feedback. Written for
 * constant src and add, so each call site below gets its own loop. */
static inline __attribute__((always_inline)) void dx_op1(struct macro_dxop *op, uint32_t f, int32_t a, const int src,
                                                       const int add, int32_t *fb, int fbamt, const int32_t *in,
                                                       int32_t *out, int size)
{
    uint32_t ph = op->phase;
    int32_t amp = op->amp, ai, p0 = fb[0], p1 = fb[1], i, sh = 9 - fbamt;
    ai = ((a > 1 << 29 ? 1 << 29 : a) - amp) / size;
    for (i = 0; i < size; i++) {
        int32_t pm = src == -2 ? in[i] : src == 0 ? (fbamt ? (p0 + p1) >> sh : 0) : 0, j, s;
        uint32_t x;
        ph += f;
        x = ph + ((uint32_t)pm << 6);
        j = (int32_t)(x >> 23);
        s = dx7_sine[j] + fmac1(dx7_sine[j + 1] - dx7_sine[j], (int32_t)((x << 9) >> 1));
        pm = fmac1(s, amp);
        amp += ai;
        if (src == 0) {
            p1 = p0;
            p0 = pm;
        }
        out[i] = add ? out[i] + pm : pm;
    }
    op->phase = ph;
    op->amp = amp;
    if (src == 0) {
        fb[0] = p0;
        fb[1] = p1;
    }
}

/* RenderOperators<n, modulation_source, additive> */
static void dx_ops(struct macro_dxop *ops, const uint32_t *f, const int32_t *a, int n, int src, int add, int32_t *fb,
                   int fbamt, const int32_t *in, int32_t *out, int size)
{
    uint32_t ph[3], fr[3];
    int32_t amp[3], ai[3], p0 = fb[0], p1 = fb[1], i, k;
    for (k = 0; k < n; k++) {
        int32_t t = a[k] > 1 << 29 ? 1 << 29 : a[k];
        fr[k] = f[k];
        ph[k] = ops[k].phase;
        amp[k] = ops[k].amp;
        ai[k] = (t - amp[k]) / size;
    }
    for (i = 0; i < size; i++) {
        int32_t pm = src >= 0 ? (fbamt ? (p0 + p1) >> (9 - fbamt) : 0) : src == -2 ? in[i] : 0;
        for (k = 0; k < n; k++) {
            uint32_t x;
            int32_t j, s;
            ph[k] += fr[k];
            x = ph[k] + ((uint32_t)pm << 6);                        /* SinePM: the phase plus pm turns */
            j = (int32_t)(x >> 23);
            s = dx7_sine[j] + fmac1(dx7_sine[j + 1] - dx7_sine[j], (int32_t)((x << 9) >> 1));
            pm = fmac1(s, amp[k]);                                  /* Q30 x Q27 -> Q26 */
            amp[k] += ai[k];
            if (k == src) {
                p1 = p0;
                p0 = pm;
            }
        }
        out[i] = add ? out[i] + pm : pm;
    }
    for (k = 0; k < n; k++) {
        ops[k].phase = ph[k];
        ops[k].amp = amp[k];
    }
    if (src >= 0) {
        fb[0] = p0;
        fb[1] = p1;
    }
}

/* KeyboardScaling(note, ks), note Q16; level units Q16 */
static int32_t dx_kb(const struct macro_dxop_p *o, int32_t note)
{
    int32_t x = note - ((o->bp + 15) << 16), curve = x > 0 ? o->rc : o->lc, t = x < 0 ? -x : x;
    if (curve == 1 || curve == 2) {
        t = fmac1(t, 22478137);                                     /* x 0.010467 */
        t = t > 65536 ? 65536 : t;
        t = (int32_t)(((((uint32_t)t * (uint32_t)t) >> 16) * (uint32_t)t) >> 16) * 96;
    }
    if (curve < 2)
        t = -t;
    return fmac1(t * (x > 0 ? o->rd : o->ld), 57488186);            /* x depth x 0.02677 */
}

/* Voice::Render over size samples into the three buffers b (out, then two scratch) */
static void dx_voice_render(struct macro_dxvoice *v, int32_t *b, int size)
{
    const struct macro_dxpatch *p = &v->p;
    const struct dx7_call *c = dx7_algs[p->algorithm];
    struct uf rate = uf_q((uint32_t)size, 0), ad, rel, f0;
    int32_t ec = v->ectl, pitch, semis, i, note;
    uint32_t f[6];
    int32_t a[6];
    int32_t *buf[4];
    if (v->patch < 0)
        return;
    if (v->dirty) {                                                 /* Setup(): the block it runs in stays silent */
        dx_setup(v);
        v->dirty = 0;
        return;
    }
    ad = dx_pow2_1((32768 - ec) * 8);
    rel = dx_pow2_1(-(ec > 19661 ? ec - 19661 : 19661 - ec) * 8);
    pitch = dx_env_render(&v->penv, v->gate, rate, ad, rel, 0) + (v->pitch_mod >> 3);   /* Q24 */
    semis = (int32_t)(((pitch >> 4) * 12) >> 4);                    /* x 12, Q16 */
    f0 = uf_q(va_freq(v->inc, semis), 32);
    if (v->gate && !v->gate_) {
        v->note = (69 << 16) + (log2_q16(v->inc) - NOTE_L0) * 12;   /* Q16, on Plaits' pitch scale */
        v->nvel = DX7_NVEL;
        if (p->reset_phase)
            for (i = 0; i < 6; i++)
                v->op[i].phase = 0;
    }
    v->gate_ = v->gate;
    note = v->note;
    for (i = 0; i < 6; i++) {
        const struct macro_dxop_p *o = &p->op[i];
        int32_t level, sum, lm, x;
        uint32_t t;
        t = uf_to(o->mode == 0 ? uf_mul(v->ratio[i], f0) : v->ratio[i], 32);
        f[i] = t > 0x80000000u ? 0x80000000u : t;
        level = dx_env_render(&v->env[i], v->gate,
                              uf_mul(rate, dx_pow2_1((o->rs * ((note / 3) - (7 << 16))) >> 5)), ad, rel, 1);
        sum = dx_kb(o, note) + v->nvel * o->vs + ((dx7_modulators[p->algorithm] >> i) & 1 ? (v->bright - 32768) * 32 : 0);
        level += (sum < v->headroom[i] << 16 ? sum : v->headroom[i] << 16) << 5;
        lm = (fmac1(dx7_ams[o->ams], v->amp_mod) >> 11) - 65536;   /* sensitivity x amp_mod - 1, Q16 */
        lm = 0x40000000 - (int32_t)uf_to(dx_pow2_2(lm * 64 / 10), 30);   /* 1 - 2^(6.4 x that), Q30 */
        x = -(14 << 16) + (fmac1(level, lm) >> 7);                  /* -14 + level x level_mod, Q16 */
        t = uf_to(dx_pow2_2(x), 27);
        a[i] = t > 1u << 29 ? 1 << 29 : (int32_t)t;
    }
    buf[0] = b;
    buf[1] = b + size;
    buf[2] = buf[3] = b + 2 * size;
    for (i = 0; i < 6 && c[i].op >= 0; i++) {
        int k = c[i].op;
        int32_t *o = buf[c[i].out];
        const int32_t *in = buf[c[i].in];
        if (c[i].n == 1) {                                          /* (the usual case) */
            int sel = (c[i].src + 2) * 2 + c[i].add;
            switch (sel) {
            case 0: dx_op1(&v->op[k], f[k], a[k], -2, 0, v->fb, p->feedback, in, o, size); break;
            case 1: dx_op1(&v->op[k], f[k], a[k], -2, 1, v->fb, p->feedback, in, o, size); break;
            case 2: dx_op1(&v->op[k], f[k], a[k], -1, 0, v->fb, p->feedback, in, o, size); break;
            case 3: dx_op1(&v->op[k], f[k], a[k], -1, 1, v->fb, p->feedback, in, o, size); break;
            case 4: dx_op1(&v->op[k], f[k], a[k], 0, 0, v->fb, p->feedback, in, o, size); break;
            default: dx_op1(&v->op[k], f[k], a[k], 0, 1, v->fb, p->feedback, in, o, size); break;
            }
        } else
            dx_ops(&v->op[k], &f[k], &a[k], c[i].n, c[i].src, c[i].add, v->fb, p->feedback, in, o, size);
    }
}

static COLD void dx_voice_init(struct macro_dxvoice *v)
{
    int i;
    for (i = 0; i < 6; i++)
        dx_env_init(&v->env[i]);
    dx_env_init(&v->penv);
    v->patch = -1;
    v->note = 48 << 16;
    v->inc = 0;
    v->bright = v->ectl = 32768;
    v->nvel = 10 << 16;                                             /* Voice::Init's normalized_velocity_ */
}

static COLD void sixop_init(struct macro_sixop *x)
{
    int32_t *w = (int32_t *)x, *end = (int32_t *)(x + 1);
    while (w < end)
        *w++ = 0;
    dx_voice_init(&x->v[0]);
    dx_voice_init(&x->v[1]);
    x->active = 1;
}

static void sixop_render(struct macro_sixop *__restrict x, uint32_t *rng, const uint8_t *p, uint32_t inc,
                         int32_t *__restrict out, int32_t *__restrict aux, int n)
{
    int32_t timb = k16(p[MACRO_P_TIMB]), morph = k16(p[MACRO_P_MORPH]), idx, i, gate;
    struct macro_dxvoice *act;
#ifdef DX7_BANK                                                     /* tests: one bank, Plaits' quantizer */
    {
        int32_t v = ((k16(p[MACRO_P_HARM]) * 32) * 51 / 50) - 32768;   /* HARMONICS x 1.02 x 32 - 0.5, Q16 */
        int32_t q = (v + (v > x->quant << 16 ? -328 : 328) + 32768) >> 16;
        x->quant = q < 0 ? 0 : q > 31 ? 31 : q;
        idx = DX7_BANK * 32 + x->quant;
    }
#else
    idx = (p[MACRO_P_HARM] * 3) >> 2;                               /* 96 patches over 0..127 */
#endif
    if (x->trig) {
        x->trig = 0;
        x->active ^= 1;
        act = &x->v[x->active];
        if (act->patch != idx) {                                    /* FMVoice::LoadPatch */
            act->patch = idx;
            dx_unpack(&act->p, idx);
            act->dirty = 1;
            dx_setup(act);                                          /* (for the LFO's Set; Setup() redoes the rest) */
            act->dirty = 1;
        }
        if (act->p.lreset)                                          /* Lfo::Reset */
            act->lph = 0;
        act->ldp = 0;
        x->gate = DX7_GATE_BLOCKS;
    }
    act = &x->v[x->active];
    gate = x->gate > 0;
    if (x->gate > 0)
        x->gate--;
    act->inc = inc;
    act->ectl = morph;
    dx_lfo_step(act, rng, n);
    for (i = 0; i < 2; i++) {
        struct macro_dxvoice *v = &x->v[i];
        v->bright = timb;
        v->gate = gate && v == act;
        if (v->patch != act->patch) {
            dx_lfo_step(v, rng, n);
            dx_lfo_mods(v, &v->pitch_mod, &v->amp_mod);
        } else
            dx_lfo_mods(act, &v->pitch_mod, &v->amp_mod);
    }
    /* staggered: one voice a block, over this block and the next */
    for (i = 0; i < n; i++) {
        x->tmp[i] = x->acc[i];
        x->tmp[n + i] = 0;
    }
    x->rendered ^= 1;
    dx_voice_render(&x->v[x->rendered], x->tmp, 2 * n);
    for (i = 0; i < n; i++) {                                       /* SoftClip(x / 4), Q15 */
        int32_t s = x->tmp[i] >> 4, x2, num, den, r;                /* Q24 */
        if (s > 3 << 24)
            s = 1 << 24;
        else if (s < -(3 << 24))
            s = -(1 << 24);
        else {
            x2 = fmac1(s << 3, s << 3);                             /* Q23 */
            num = (27 << 23) + x2;
            den = (27 << 23) + 9 * x2;
            r = (int32_t)(((uint32_t)num << 3) / ((uint32_t)den >> 13));   /* (27 + x^2) / (27 + 9 x^2), Q16 */
            s = fmac1(s, r >= 65536 ? 0x7fffffff : r << 15);
        }
        out[i] = aux[i] = (s + 256) >> 9;
        x->acc[i] = x->tmp[n + i];
    }
}

/* the gains Plaits' voice gives each engine's OUT and AUX (voice.cc, RegisterInstance), Q15. An engine
 * Plaits registers with a negative gain goes through its limiter (limit()) and then 0.8. */
static const int16_t gain_out[MACRO_ENGINES] = {22938, 19661, 26214, 26214, 26214, 26214, 26214, 22938, 26214, 26214, 19661, 26214, 26214, 26214, 32767};  /* WSH .7, FM .6, NOISE, PART lim; drums .8; GRAIN .7 */
static const int16_t gain_aux[MACRO_ENGINES] = {19661, 19661, 26214, 32767, 26214, 26214, 26214, 19661, 26214, 32767, 19661, 26214, 26214, 26214, 32767};  /* WSH .6, FM .6, NOISE lim, PART 1; drums .8; GRAIN .6 */

const char *const macro_engine_name[MACRO_ENGINES] = {"WSHAPE", "2OP FM", "NOISE", "PARTCL", "BDRUM", "SNARE", "HIHAT", "GRAIN", "CHORDS", "SWARM", "WAVES", "VA", "MODAL", "STRING", "6-OP"};

/* Model-TG: which engines are built in, and their order on the ENGN knob. MACRO_SEL is a list of MACRO_*
 * ids (build.py --plaits-engines); an engine not in it is never called, so its code and tables are not
 * linked. Knob zone k (8 values each) plays the k-th engine of the list; past the last, the last. */
#ifndef MACRO_SEL
#define MACRO_SEL MACRO_WSH, MACRO_FM, MACRO_NOISE, MACRO_PARTICLE, MACRO_BD, MACRO_SD, MACRO_HH, MACRO_GRAIN, \
                  MACRO_CHORD, MACRO_SWARM, MACRO_WAVETABLE, MACRO_VA, MACRO_MODAL, MACRO_STRING, MACRO_SIXOP
#endif
#ifndef MACRO_MASK                     /* bit e set: engine e is in MACRO_SEL (build.py passes both) */
#define MACRO_MASK 0x7fff
#endif
static const uint8_t macro_sel[] = { MACRO_SEL };
#define MACRO_NSEL ((int)sizeof macro_sel)
#define HAS(e) ((MACRO_MASK >> (e)) & 1)

int macro_engine_of(int b)
{
    int z = b >> MACRO_ZONE_SHIFT;
    return macro_sel[z < MACRO_NSEL ? z : MACRO_NSEL - 1];
}

static void engine_init(struct macro_voice *m)
{
    m->lim_out = m->lim_aux = 1 << 23;                     /* the limiters start at a peak of 0.5 (Q24) */
    switch (m->engine) {
    case MACRO_WSH:   if (HAS(MACRO_WSH)) wsh_init(&m->e.wsh); break;
    case MACRO_FM:    if (HAS(MACRO_FM)) fm_init(&m->e.fm); break;
    case MACRO_NOISE: if (HAS(MACRO_NOISE)) noise_init(&m->e.noise); break;
    case MACRO_PARTICLE: if (HAS(MACRO_PARTICLE)) particles_init(&m->e.part); break;
    case MACRO_BD:    if (HAS(MACRO_BD)) bd_init(&m->e.bd); break;
    case MACRO_SD:    if (HAS(MACRO_SD)) sd_init(&m->e.sd); break;
    case MACRO_HH:    if (HAS(MACRO_HH)) hh_init(&m->e.hh); break;
    case MACRO_GRAIN: if (HAS(MACRO_GRAIN)) grain_init(&m->e.grain); break;
    case MACRO_CHORD: if (HAS(MACRO_CHORD)) chord_init(&m->e.chord); break;
    case MACRO_SWARM: if (HAS(MACRO_SWARM)) swarm_init(&m->e.swarm); break;
    case MACRO_WAVETABLE: if (HAS(MACRO_WAVETABLE)) wt_init(&m->e.wt); break;
    case MACRO_VA: if (HAS(MACRO_VA)) va_init(&m->e.va); break;
    case MACRO_MODAL: if (HAS(MACRO_MODAL)) modal_init(&m->e.modal); break;
    case MACRO_STRING: if (HAS(MACRO_STRING)) string_init(&m->e.string); break;
    case MACRO_SIXOP: if (HAS(MACRO_SIXOP)) sixop_init(&m->e.sixop); break;
    default: break;
    }
}

void macro_init(struct macro_voice *m)
{
    m->engine = macro_sel[0];
    m->latch = 1;
    m->pad[0] = m->pad[1] = 0;
    m->rng = 0x2545f491u;
    engine_init(m);
}

void macro_trig(struct macro_voice *m)
{
    m->latch = 1;
}

static void macro_render_e(struct macro_voice *m, const uint8_t *p, uint32_t inc, int16_t *out, int n);

void macro_render(struct macro_voice *m, const uint8_t *p, uint32_t inc, int16_t *out, int n)
{
    struct emac_save e;
    emac_enter(&e);
    macro_render_e(m, p, inc, out, n);
    emac_leave(&e);
}

#define CL2(v) ((v) > 65535 ? 65535 : (v) < -65535 ? -65535 : (v))     /* +-2 in Q15: clipped below anyway */

static void macro_render_e(struct macro_voice *m, const uint8_t *p, uint32_t inc, int16_t *out, int n)
{
    int32_t o[32], a[32], go, ga, mix;
    int i;
    if (m->latch) {                                 /* a note start: the engine, and its per-note choices */
        int e = macro_engine_of(p[MACRO_P_ENGINE]);
        m->latch = 0;
        if (e != m->engine) {
            m->engine = (uint8_t)e;
            engine_init(m);
        }
        if (HAS(MACRO_FM) && m->engine == MACRO_FM)
            fm_trig(&m->e.fm, p);
        if (m->engine == MACRO_NOISE)
            m->e.noise.sync = 1;
        if (m->engine == MACRO_PARTICLE)
            m->e.part.sync = 1;
        if (m->engine == MACRO_BD)
            m->e.bd.trig = 1;
        if (m->engine == MACRO_SD)
            m->e.sd.trig = 1;
        if (m->engine == MACRO_HH)
            m->e.hh.trig = 1;
        if (HAS(MACRO_SWARM) && m->engine == MACRO_SWARM)
            m->e.swarm.trig = 1;
        if (HAS(MACRO_MODAL) && m->engine == MACRO_MODAL)
            m->e.modal.trig = 1;
        if (HAS(MACRO_STRING) && m->engine == MACRO_STRING)
            m->e.string.trig = 1;
        if (HAS(MACRO_SIXOP) && m->engine == MACRO_SIXOP)
            m->e.sixop.trig = 1;
    }
    if (inc > INC_MAX)
        inc = INC_MAX;
    if (inc < INC_MIN)
        inc = INC_MIN;
    /* F: OUT up to 55, AUX from 72, a crossfade between (only there are both outputs computed) */
    mix = p[MACRO_P_AUX] <= 55 ? 0 : p[MACRO_P_AUX] >= 72 ? 32767 : (p[MACRO_P_AUX] - 55) * 1927;
    switch (m->engine) {
    case MACRO_WSH: if (!HAS(MACRO_WSH)) goto none; wsh_render(&m->e.wsh, p, inc, o, a, n, mix < 32767, mix > 0); break;
    case MACRO_FM:  if (!HAS(MACRO_FM)) goto none; fm_render(&m->e.fm, p, inc, o, a, n, mix > 0); break;
    case MACRO_NOISE:
        if (!HAS(MACRO_NOISE)) goto none;
        noise_render(&m->e.noise, &m->rng, p, inc, o, a, n, mix > 0);
        if (mix < 32767)
            limit(&m->lim_out, o, n);
        if (mix > 0)
            limit(&m->lim_aux, a, n);
        break;
    case MACRO_BD:
        if (!HAS(MACRO_BD)) goto none;
        bd_render(m, p, inc, o, a, n, mix < 32767, mix > 0);
        break;
    case MACRO_SD:
        if (!HAS(MACRO_SD)) goto none;
        sd_render(m, p, inc, o, a, n, mix < 32767, mix > 0);
        break;
    case MACRO_HH:
        if (!HAS(MACRO_HH)) goto none;
        hh_render(m, p, inc, o, a, n, mix < 32767, mix > 0);
        break;
    case MACRO_GRAIN:
        if (!HAS(MACRO_GRAIN)) goto none;
        grain_render(&m->e.grain, p, inc, o, a, n, mix < 32767, mix > 0);
        break;
    case MACRO_SIXOP:
        if (!HAS(MACRO_SIXOP)) goto none;
        sixop_render(&m->e.sixop, &m->rng, p, inc, o, a, n);
        break;
    case MACRO_STRING:
        if (!HAS(MACRO_STRING)) goto none;
        string_render(&m->e.string, &m->rng, p, inc, o, a, n);
        if (mix < 32767)
            limit(&m->lim_out, o, n);
        break;
    case MACRO_MODAL:
        if (!HAS(MACRO_MODAL)) goto none;
        modal_render(&m->e.modal, p, inc, o, a, n);
        if (mix < 32767)
            limit(&m->lim_out, o, n);
        break;
    case MACRO_VA:
        if (!HAS(MACRO_VA)) goto none;
        va_render(&m->e.va, p, inc, o, a, n, mix < 32767, mix > 0);
        break;
    case MACRO_WAVETABLE:
        if (!HAS(MACRO_WAVETABLE)) goto none;
        wt_render(&m->e.wt, p, inc, o, a, n);
        break;
    case MACRO_SWARM:
        if (!HAS(MACRO_SWARM)) goto none;
        swarm_render(m, p, inc, o, a, n, mix < 32767, mix > 0);
        if (mix < 32767)
            limit(&m->lim_out, o, n);
        break;
    case MACRO_CHORD:
        if (!HAS(MACRO_CHORD)) goto none;
        chord_render(&m->e.chord, p, inc, o, a, n);
        break;
    case MACRO_PARTICLE:
        if (!HAS(MACRO_PARTICLE)) goto none;
        particle_render(m, p, inc, o, a, n, mix > 0);
        if (mix < 32767)
            limit(&m->lim_out, o, n);
        break;
    default:
    none:
        for (i = 0; i < n; i++)
            out[i] = 0;
        return;
    }
    go = gain_out[m->engine];
    ga = gain_aux[m->engine];
    if (mix == 0 || mix >= 32767) {                 /* OUT only (the default) or AUX only */
        /* x g / 2^15 on the EMAC: no product overflows, so no clip before it (the gains are over 0.5:
         * whatever CL2 would clip lands past +-1 either way) */
        const int32_t *x = mix ? a : o;
        int32_t g = (mix ? ga : go) << 16;
        for (i = 0; i < n; i++) {
            int32_t y = fmac1t(x[i], g);
            out[i] = (int16_t)(y > 32767 ? 32767 : y < -32768 ? -32768 : y);
        }
        return;
    }
    for (i = 0; i < n; i++) {
        int32_t x = (CL2(o[i]) * go) >> 15, y = (CL2(a[i]) * ga) >> 15;
        x = x > 32767 ? 32767 : x < -32768 ? -32768 : x;
        y = y > 32767 ? 32767 : y < -32768 ? -32768 : y;
        x += ((y - x) * mix) >> 15;
        out[i] = (int16_t)(x > 32767 ? 32767 : x < -32768 ? -32768 : x);
    }
}
