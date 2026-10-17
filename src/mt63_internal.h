/*
 * fldigi-mt63-core internals: Pawel Jalocha's MT63 DSP (fldigi 4.2.13 src/mt63/dsp.cxx,
 * src/include/dsp.h) and the receiver/transmitter state (src/include/mt63base.h,
 * src/include/mt63.h).
 *
 * Every helper keeps fldigi's statements and operation order: with -ffp-contract=on the
 * compiler fuses a * b + c inside one statement exactly where fldigi's reference build
 * (Apple clang -O2) fuses it, and never across statements, which is why fldigi's inline
 * functions stay functions here.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef MT63_INTERNAL_H
#define MT63_INTERNAL_H

#include "fldigi_mt63.h"

#include <math.h>
#include <stddef.h>

/* symbol.dat:24-26 */
#define MT63_SYMBOL_LEN 512     /* the shape's length = FFT length */
#define MT63_SYMBOL_SEPAR 200   /* distance in samples between symbols on a carrier */
#define MT63_CARR_SEPAR 4       /* carriers are 4 FFT bins apart */
#define MT63_CARRIERS 64        /* DataCarriers (mt63base.cxx:117, 807) */
#define MT63_SYMBOL_DIV 4       /* SymbolDiv (mt63base.cxx:824) */
#define MT63_SCAN_MARGIN 8      /* ScanMargin, DataScanMargin (mt63base.cxx:825, 838) */
#define MT63_BLOCK 512          /* fldigi's sound-card block (trx.cxx, the bench) */

extern const double mt63_symbol_shape[MT63_SYMBOL_LEN];
extern const int mt63_short_intlv_patt[64];
extern const int mt63_long_intlv_patt[64];

typedef struct {
    double re, im;
} mt63_cmpx;              /* Cdspcmpx<double> (dsp.h:32-34) */

/* ---------------------------------------------------------------- small helpers (dsp.h) */

/* dspPower (dsp.h:287-297) */
static inline double mt63_power(mt63_cmpx X)
{
    return X.re * X.re + X.im * X.im;
}

static inline double mt63_power2(double I, double Q)
{
    return I * I + Q * Q;
}

/* dspAmpl (dsp.h:313-319) */
static inline double mt63_ampl(mt63_cmpx X)
{
    return sqrt(X.re * X.re + X.im * X.im);
}

/* dspScalProd (dsp.h:63-70) */
static inline double mt63_scal_prod(mt63_cmpx A, mt63_cmpx B)
{
    return A.re * B.re + A.im * B.im;
}

static inline double mt63_scal_prod_iq(double Ia, double Qa, mt63_cmpx B)
{
    return Ia * B.re + Qa * B.im;
}

/* CdspcmpxMultAxB (dsp.h:74-84) */
static inline mt63_cmpx mt63_mult(mt63_cmpx A, mt63_cmpx B)
{
    mt63_cmpx Dst;
    Dst.re = A.re * B.re - A.im * B.im;
    Dst.im = A.re * B.im + A.im * B.re;
    return Dst;
}

/* CdspcmpxMultAxBs (dsp.h:87-91) */
static inline mt63_cmpx mt63_mult_s(mt63_cmpx A, mt63_cmpx B)
{
    mt63_cmpx Dst;
    Dst.re = A.re * B.re + A.im * B.im;
    Dst.im = A.im * B.re - A.re * B.im;
    return Dst;
}

/* dspLowPass2Coeff (dsp.h:707-712) */
static inline void mt63_lowpass2_coeff(int IntegLen, double *W1, double *W2, double *W5)
{
    *W1 = 1.0 / IntegLen;
    *W2 = 2.0 / IntegLen;
    *W5 = 5.0 / IntegLen;
}

/* dspLowPass2 for a real input (dsp.h:722-734) */
static inline void mt63_lowpass2(double Inp, double *Mid, double *Out, double W1, double W2,
                                 double W5)
{
    double Sum, Diff;
    Sum = *Mid + *Out;
    Diff = *Mid - *Out;
    *Mid += W2 * Inp - W1 * Sum;
    *Out += W5 * Diff;
}

/* ---------------------------------------------------------------- mt63_dsp.c */

void mt63_lowpass2_cmpx(const mt63_cmpx *Inp, mt63_cmpx *Mid, mt63_cmpx *Out, double W1,
                        double W2, double W5);
double mt63_rms(const double *Data, int Len);
double mt63_find_min(const double *Data, int Len, int *MinPos);
double mt63_find_max(const double *Data, int Len, int *MaxPos);
double mt63_find_max_power(const mt63_cmpx *Data, int Len, int *MaxPos);
void mt63_sel_fit_aver(const double *Data, int Len, double SelThres, int Loops, double *Aver,
                       double *Rms);
void mt63_sel_fit_aver_cmpx(const mt63_cmpx *Data, int Len, double SelThres, int Loops,
                            mt63_cmpx *Aver, double *Rms);
double mt63_window_blackman3(double phase);
void mt63_win_fir_i(double LowOmega, double UppOmega, double *Shape, int Len);
void mt63_win_fir_q(double LowOmega, double UppOmega, double *Shape, int Len);
void mt63_walsh(double *Data, int Len);
void mt63_walsh_inv(double *Data, int Len);

/* dsp_r2FFT (dsp.h, dsp.cxx:2005-2147) */
typedef struct {
    int Size;
    int *BitRevIdx;
    mt63_cmpx *Twiddle;
} mt63_fft;
int mt63_fft_preset(mt63_fft *fft, int size);
void mt63_fft_free(mt63_fft *fft);
void mt63_fft_core(const mt63_fft *fft, mt63_cmpx *x);

/* dspQuadrSplit (dsp.cxx:792-884): real input -> complex, band pass, decimation. */
typedef struct {
    int Len, Rate;
    double *ShapeI, *ShapeQ;
    double *Tap;
    int TapLen, TapCap;
} mt63_split;
int mt63_split_preset(mt63_split *s, int FilterLen, int DecimateRate, int max_input);
void mt63_split_compute_shape(mt63_split *s, double LowOmega, double UppOmega);
void mt63_split_clear(mt63_split *s);
void mt63_split_free(mt63_split *s);
int mt63_split_process(mt63_split *s, const double *Inp, int InpLen, mt63_cmpx *Out);

/* dspQuadrComb (dsp.cxx:890-998): complex -> real, interpolation, band pass. */
typedef struct {
    int Len, Rate;
    double *ShapeI, *ShapeQ;
    double *Tap;
    int TapPtr;
} mt63_comb;
int mt63_comb_preset(mt63_comb *c, int FilterLen, int DecimateRate);
void mt63_comb_compute_shape(mt63_comb *c, double LowOmega, double UppOmega);
void mt63_comb_free(mt63_comb *c);
int mt63_comb_process(mt63_comb *c, const mt63_cmpx *Inp, int InpLen, double *Out);

/* dspCmpxOverlapWindow (dsp.cxx:1530-1657) */
typedef struct {
    int Len, Dist;
    const double *Window;
    mt63_cmpx *Buff;
} mt63_overlap;
int mt63_overlap_preset(mt63_overlap *w, int WindowLen, int SlideDist, const double *Shape);
void mt63_overlap_free(mt63_overlap *w);
int mt63_overlap_process(mt63_overlap *w, const mt63_cmpx *Inp, int InpLen, mt63_cmpx *Out);

/* dspDelayLine<dspCmpx> (dsp.h:862-923) */
typedef struct {
    mt63_cmpx *Line;
    int Delay, LineSize, DataLen;
    int InpPos;            /* InpPtr - Line */
    int InpLen;
} mt63_delay;
int mt63_delay_preset(mt63_delay *d, int MaxDelay);
void mt63_delay_clear(mt63_delay *d);
void mt63_delay_free(mt63_delay *d);
int mt63_delay_process(mt63_delay *d, const mt63_cmpx *Inp, int Len);

/* ---------------------------------------------------------------- submodes */

const fmt63_submode *mt63__submode(const char *name);
/* MT63rx::Preset / MT63tx::Preset: FirstDataCarr (mt63base.cxx:101-121, 761-777) */
int mt63__first_carrier(int bandwidth, double freq);
/* carrier inside bandwidth/2 .. 4000 - bandwidth/2 */
int mt63__carrier_ok(int bandwidth, double freq);

/* The C library rand() of fldigi's reference build (macOS libc: Park-Miller minimal
 * standard, x = 16807 x mod (2^31 - 1), state 1 at start). */
static inline int mt63_rand(unsigned long *state)
{
    *state = (unsigned long)(((unsigned long long)*state * 16807ULL) % 2147483647ULL);
    return (int)*state;
}

#endif /* MT63_INTERNAL_H */
