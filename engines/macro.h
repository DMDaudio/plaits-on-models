/* Digi Mono's MACRO machine: synthesis engines ported from Plaits (Emilie Gillet, MIT licence), in 32-bit
 * integer arithmetic for the ColdFire (no FPU, no 64-bit maths, no library calls). See macro.c for the
 * licence text and mods/digimono/MACRO.md for the engines and how each was checked against the original.
 *
 * One machine, several engines: knob B picks the engine. It is read at each note start (and while a voice
 * has not started yet), so a p-lock on B changes the engine on that trig and a turning knob on the next.
 */
#ifndef MACRO_H
#define MACRO_H

#include <stdint.h>

enum {
    MACRO_WSH = 0,          /* waveshaping: a slope oscillator through a waveshaper and a wavefolder */
    MACRO_FM  = 1,          /* 2-operator FM with feedback, 4x oversampled (2x without feedback)    */
    MACRO_NOISE = 2,        /* clocked noise through a LP-to-HP filter (OUT), two band-passes (AUX)  */
    MACRO_PARTICLE = 3,     /* random impulses through resonant band-passes (OUT), the impulses (AUX) */
    MACRO_BD = 4,           /* bass drums: an analog-style one (OUT), a synthetic one (AUX)            */
    MACRO_SD = 5,           /* snare drums: an analog-style one (OUT), a synthetic one (AUX)           */
    MACRO_HH = 6,           /* hi-hats: six square oscillators (OUT), three ring-modulated pairs (AUX) */
    MACRO_GRAIN = 7,        /* granular formants: two grainlets (OUT), a Z oscillator (AUX)            */
    MACRO_CHORD = 8,        /* Model-TG: chords, five divide-down/wavetable voices (OUT), the root (AUX) */
    MACRO_SWARM = 9,        /* Model-TG: eight grains or glissandi of saws (OUT) and sines (AUX)        */
    MACRO_WAVETABLE = 10,   /* Model-TG: 8 x 8 x 4 waves, trilinear (OUT), bit-crushed to 1/32 (AUX)    */
    MACRO_VA = 11,          /* Model-TG: virtual analog, sync square + detuned saw (OUT), monster sync (AUX) */
    MACRO_MODAL = 12,       /* Model-TG: modal resonator, 24 modes (OUT), the click exciting it (AUX)              */
    MACRO_STRING = 13,      /* Model-TG: inharmonic string, three Karplus-Strong strings (OUT), their bursts (AUX)  */
    MACRO_SIXOP = 14,       /* Model-TG: 6-operator FM, the 96 DX7 patches of Plaits' three banks (OUT = AUX)        */
    MACRO_ENGINES
};

/* Knob B has 16 zones of 8 values, one an engine, so a saved ENGN value keeps its engine as engines are
 * added; the zones past the last engine play the last one. */
#define MACRO_ZONE_SHIFT 3

/* The parameters, raw 0..127: */
enum {
    MACRO_P_ENGINE = 0,     /* B  ENGN  the engine (MACRO_ENGINES zones over 0..127)               */
    MACRO_P_HARM   = 1,     /* C  HARM  Plaits' HARMONICS                                          */
    MACRO_P_TIMB   = 2,     /* D  TIMB  Plaits' TIMBRE                                             */
    MACRO_P_MORPH  = 3,     /* E  MORP  Plaits' MORPH                                              */
    MACRO_P_AUX    = 4,     /* F  AUX   0 = Plaits' OUT .. 127 = its AUX, a crossfade              */
    MACRO_PARAMS   = 7
};

/* Every engine's state below is made of 32-bit words only (the tests turn a voice's state round between
 * the ColdFire's byte order and a PC's word by word). */

/* The slope oscillator (Plaits' Oscillator, shape SLOPE), Q16 samples. */
struct macro_slope {
    uint32_t phase;
    int32_t  next;                  /* the next sample's value so far, Q16 (0..1 = 0..65536)          */
    int32_t  high;                  /* 1 while on the rising slope                                   */
};

struct macro_wsh {
    struct macro_slope slope, tri;
    int32_t prev_shape, prev_gain, prev_overtone;   /* Q15, the values the last block ended on        */
};

struct macro_fm {
    uint32_t carrier, modulator, sub;               /* phases                                        */
    int32_t  prev;                                  /* the carrier, one-pole filtered (the feedback)  */
    int32_t  hc[6], hs[6];                          /* 2x: the last six samples of carrier and sub    */
    int32_t  car_fir, sub_fir;                      /* 4x: the downsamplers' carried half, Q30        */
    int32_t  prev_amount, prev_feedback;            /* Q15 (amount: Q14)                              */
    int32_t  os4;                                   /* this note runs 4x (it started with feedback)   */
};

/* Plaits' ClockedNoise: random values held at a clock's rate, band-limited steps (polyBLEP) */
struct macro_cnoise {
    uint32_t phase;
    int32_t  sample, next;                          /* Q15                                           */
};

/* stmlib's Svf (a trapezoidal state-variable filter), states Q24 */
struct macro_svf {
    int32_t s1, s2;
};

struct macro_noise {
    struct macro_cnoise src[2];
    struct macro_svf mm, bp;                        /* OUT's LP-to-HP filter; AUX's second band-pass  */
    int32_t sync;                                   /* a note started: restart both clocks           */
};

/* Plaits' Particle: impulses at random times, each particle's band-pass (stmlib's Svf, Q24) */
#define MACRO_PARTICLES 6
struct macro_particle {
    int32_t e;                                      /* the time to the next impulse, in exponential units, Q26 */
    int32_t s1, s2;                                 /* the band-pass's states                          */
    int32_t a1, a2, a3;                             /* its coefficients (struct svf_c in macro.c)      */
    int32_t c1m, c1s, c2m, c2s;                     /* an impulse's step into bp (and s1), lp (and s2)  */
};

struct macro_particles {
    struct macro_particle p[MACRO_PARTICLES];
    struct macro_svf post;                          /* the low-pass after them                         */
    int32_t sync;
};

/* Plaits' bass drum engine: AnalogBassDrum (OUT, Q24) and SyntheticBassDrum (AUX, Q24) */
struct macro_bd {
    int32_t trig;
    int32_t idle[2];                                /* OUT's, AUX's drum has died away: nothing computed */
    /* analog */
    int32_t pulse_left, fm_left, pulse, pulse_lp, fm_lp, retrig, lp_out, tone_lp;
    struct macro_svf res;
    int32_t a1, a2, a3;                             /* the resonator's coefficients now               */
    /* synthetic */
    uint32_t phase;
    int32_t pnoise, fm, fm_lp2, body, body_lp, trans, trans_lp, tone_lp2;
    int32_t click_lp, click_hp, noise_lp, noise_hp, body_pw, fm_pw;
    struct macro_svf click;
};

/* Plaits' snare drum engine: AnalogSnareDrum (OUT) and SyntheticSnareDrum (AUX), Q24 */
struct macro_sd {
    int32_t trig;
    int32_t idle[2];
    /* analog: five modes' resonators, the noise's band-pass; coefficients kept for the block's knobs */
    int32_t pulse_left, pulse, pulse_lp, noise_env;
    struct macro_svf mode[5], nf;
    int32_t ma1[5], ma2[5], ma3[5], na1, na2, na3;
    uint32_t key_inc;
    int32_t key_knobs;
    /* synthetic */
    int32_t ph0, ph1, drum_amp, snare_amp, fm, hold;
    int32_t drum_lp, snare_hp;
    struct macro_svf snare_lp;
};

/* Plaits' hi-hat engine: two HiHats (OUT: SquareNoise, AUX: RingModNoise), Q24 */
struct macro_hh {
    int32_t trig;
    int32_t idle[2];
    int32_t env[2];
    uint32_t nclk[2];
    int32_t nsmp[2];
    struct macro_svf bp[2], hp[2];
    uint32_t sq[6];                                 /* SquareNoise's phases                             */
    uint32_t ph[6];                                 /* RingModNoise's oscillators: phase, next sample,  */
    int32_t next[6], high[6];                       /* and the square's state                          */
    int32_t key_timb;                               /* the filters' coefficients for this TIMBRE:      */
    int32_t kc[3][4];                               /* OUT's band-pass, AUX's, the high-pass (svf_c)    */
};

/* Plaits' grain engine: GrainletOscillator x 2 (OUT), ZOscillator (AUX), Q24 samples */
struct macro_grain {
    uint32_t gc[2], gf[2];                          /* the grainlets' carrier and formant phases (Q32)  */
    int32_t gnext[2];
    uint32_t zc, zd, zf;                            /* Z: carrier, discontinuity (Q31), formant (Q32)  */
    int32_t znext;
    int32_t dc[2];                                  /* the DC blockers' states                         */
};

/* Model-TG: Plaits' chord engine (chord_engine.cc), its five voices */
#define MACRO_CHORD_VOICES 5
struct macro_ssynth {                               /* StringSynthOscillator: 4 saws on one phase      */
    uint32_t phase, inc;                            /* phase: 8 segments a cycle; inc: what it ramps from */
    int32_t  next, seg;
    int32_t  g[4];                                  /* the 8', 4', 2', 1' saws' gains, Q23              */
};
struct macro_wtv {                                  /* WavetableOscillator<128, 15>                    */
    uint32_t phase, inc;
    int32_t  amp, wf;                               /* Q15; the wave index, Q16 (0..14)                */
    int32_t  prev, lpd, lp;                         /* the differentiator and the low-pass, Q8         */
};
struct macro_chord {
    struct macro_ssynth dd[MACRO_CHORD_VOICES];
    struct macro_wtv wt[MACRO_CHORD_VOICES];
    int32_t morph_lp, timbre_lp;                    /* Q24                                             */
    int32_t chord;                                  /* the quantized chord, 0..10                      */
};

/* Model-TG: Plaits' swarm engine (swarm_engine.cc), its eight voices */
#define MACRO_SWARM_VOICES 8
struct macro_genv {                                 /* GrainEnvelope                                   */
    int32_t from, interval;                         /* Q30                                             */
    int32_t phase, fm;                              /* Q28; Q26                                        */
    int32_t amp, big, fc;                           /* Q30; size_ratio >= 1 last block; Q30            */
};
struct macro_asaw { uint32_t phase, inc; int32_t next, gain; };   /* AdditiveSawOscillator: Q30; Q26 */
struct macro_fsin { int32_t x, y, eps, amp; };                    /* FastSineOscillator: Q30 x 3; Q25 */
struct macro_swarm {
    struct macro_genv env[MACRO_SWARM_VOICES];
    struct macro_asaw saw[MACRO_SWARM_VOICES];
    struct macro_fsin sine[MACRO_SWARM_VOICES];
    int32_t trig;                                   /* a note started: a burst                         */
};

/* Model-TG: Plaits' wavetable engine (wavetable_engine.cc) */
struct macro_wavetable {
    uint32_t phase, f;                              /* f: what the pitch glides from                  */
    int32_t  pre[3];                                /* X Y Z (TIMBRE MORPH HARMONICS x 7), one-poled, Q24 */
    int32_t  tgt[3];                                /* what the per-sample glide starts from, Q24       */
    int32_t  lp[3];                                 /* the per-sample one-pole of X Y Z, Q24            */
    int32_t  prev, dlp;                             /* the differentiator, Q8 table units              */
};

/* Model-TG: Plaits' virtual analog engine (virtual_analog_engine.cc, VA_VARIANT 2) */
struct macro_vso {                                  /* VariableShapeOscillator                         */
    uint32_t mph, sph;                              /* master, slave phase                             */
    uint32_t mf, sf;                                /* what the frequencies glide from                 */
    int32_t  pw31, ppw31;                           /* pulse width, previous one (Q31)                 */
    int32_t  ws;                                    /* waveshape, Q16                                  */
    int32_t  next, high;                            /* Q28                                             */
};
struct macro_vsaw {                                 /* VariableSawOscillator                           */
    uint32_t ph, f;
    int32_t  pw31, ppw31, ws, next, high;
};
struct macro_va {
    struct macro_vso primary, auxiliary, sync;
    struct macro_vsaw saw;
    int32_t sq_gain, saw_gain;                      /* auxiliary_amount_, xmod_amount_: Q24            */
};

/* a value as a 31-bit mantissa and an exponent: m 2^(e - 30), m in [2^30, 2^31) (macro.c's MODAL and 6-OP) */
struct uf { int32_t m, e; };

/* Model-TG: Plaits' 6-op FM engine (engine2/six_op_engine.cc, fm/) */
struct macro_dxop_p {                               /* a patch's operator, unpacked (fm::Patch)        */
    uint8_t rate[4], level[4], bp, ld, rd, lc, rc, rs, ams, vs, out, mode, coarse, fine, detune;
};
struct macro_dxpatch {
    struct macro_dxop_p op[6];
    uint8_t prate[4], plevel[4], algorithm, feedback, reset_phase;
    uint8_t lrate, ldelay, lpmd, lamd, lreset, lwave, lpms;
};
struct macro_dxop { uint32_t phase; int32_t amp; };          /* amplitude Q27                         */
struct macro_dxenv {                                /* fm::Envelope: levels Q24, phase Q30             */
    int32_t stage, phase, start;
    struct uf inc[4];
    int32_t level[4];
};
struct macro_dxvoice {                              /* FMVoice: the patch, fm::Voice, its Lfo          */
    struct macro_dxpatch p;
    struct macro_dxop op[6];
    struct macro_dxenv env[6], penv;
    struct uf ratio[6];
    int32_t headroom[6], fb[2];
    int32_t patch, dirty, gate, gate_;
    int32_t note, nvel, inc, bright, ectl, pitch_mod, amp_mod;
    uint32_t lph, lfreq, ldp, ldinc[2];
    int32_t lval, lrand, amd, pmd;
};
struct macro_sixop {
    struct macro_dxvoice v[2];
    int32_t tmp[3 * 64], acc[32];                   /* temp_buffer_, acc_buffer_ (Q26)                 */
    int32_t active, rendered, gate, trig, quant;
};

/* Model-TG: Plaits' string engine (string_engine.cc, string_voice.cc, string.cc) */
#define MACRO_STRINGS 3
struct macro_strv {
    int32_t line[1024];                             /* string_, Q24                                    */
    int32_t stretch[256];                           /* stretch_ (the dispersion allpass), Q24          */
    int32_t wp, swp;                                /* their write positions                           */
    struct macro_svf exc, damp;                     /* the burst's low-pass, the damping low-pass      */
    int32_t exc_c[3];                               /* the burst filter's a1, a2, a3 (set on a trig)   */
    int32_t remaining;                              /* noise samples left in the burst                 */
    int32_t dc_x, dc_y;                             /* DCBlocker                                       */
    int32_t disp, curve;                            /* dispersion_noise_ (Q30), curved_bridge_ (Q24)   */
    int32_t out0, out1;                             /* out_sample_, Q24                                */
    uint32_t src_phase;                             /* Q30                                             */
    int32_t delay;                                  /* delay_, Q20                                     */
    uint32_t f0;                                    /* f0_ of this string, a phase increment           */
};
struct macro_string {
    struct macro_strv s[MACRO_STRINGS];
    uint32_t f0_hist[16];                           /* f0_delay_                                       */
    int32_t f0_wp, active, trig;
};

/* Model-TG: Plaits' modal engine (modal_engine.cc, modal_voice.cc, resonator.cc) */
#define MACRO_MODAL_MODES 24
struct macro_modal {
    struct macro_svf exc;                           /* the click's low-pass, Q24                       */
    struct macro_svf mode[MACRO_MODAL_MODES];       /* Q24                                             */
    int32_t harm_lp;                                /* HARMONICS smoothed, Q24                         */
    int32_t trig;
    int32_t key[4];                                 /* inc, structure, brightness, damping of coef     */
    int32_t key_cut, exc_c[3];                      /* the click filter's cutoff and a1, a2, a3        */
    int32_t coef[MACRO_MODAL_MODES][4];             /* the modes' a1, a2, a3 (Q31) and gain (Q31)      */
};

struct macro_voice {
    uint8_t engine;                 /* the engine playing                                            */
    uint8_t latch;                  /* 1: take the engine from knob B at the next block              */
    uint8_t pad[2];                 /* (four bytes, then 32-bit words)                               */
    uint32_t rng;                   /* the voice's random numbers (stmlib's Random)                  */
    int32_t lim_out, lim_aux;       /* the limiters' peaks (engines Plaits limits), Q24              */
    union {
        struct macro_wsh wsh;
        struct macro_fm fm;
        struct macro_noise noise;
        struct macro_particles part;
        struct macro_bd bd;
        struct macro_sd sd;
        struct macro_hh hh;
        struct macro_grain grain;
        struct macro_chord chord;
        struct macro_swarm swarm;
        struct macro_wavetable wt;
        struct macro_va va;
        struct macro_modal modal;
        struct macro_string string;
        struct macro_sixop sixop;
    } e;
};

void macro_init(struct macro_voice *m);
void macro_trig(struct macro_voice *m);
/* Render n samples (n <= 32) of 16-bit mono with parameters p[0..6] at phase increment inc (a 32-bit
 * phase step per 48 kHz sample). */
void macro_render(struct macro_voice *m, const uint8_t *p, uint32_t inc, int16_t *out, int n);
/* The engine knob B's value -> its engine, and the engines' names as the knob shows them. */
int macro_engine_of(int b);
/* the phase increment (at 48 kHz) of a MIDI note, Q16 (note 69 = 440 Hz) */
uint32_t macro_inc_of_note(int32_t note_q16);
extern const char *const macro_engine_name[MACRO_ENGINES];
/* one cycle of a sine in 512 steps, Q15 (+ 128 more: a cosine reads it a quarter on); MONO SIN, POLY SIN
 * and VO read it too */
extern const int16_t MACRO_SINE[641];

#endif
