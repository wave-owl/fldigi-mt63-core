/*
 * fldigi-mt63-core -- Pawel Jalocha's MT63 DSP building blocks, vendored from fldigi 4.2.13
 * src/mt63/dsp.cxx and src/include/dsp.h (MT63-specific, not part of fldigi-dsp-core,
 * wave-owl R-920.4). Each function names the fldigi lines it ports and keeps their
 * statements and operation order.
 *
 * Copyright (C) 2026 Gerolf Ziegenhain
 * Ported from fldigi 4.2.13 (tag v4.2.13, commit e7c3ab37); (c) 1999-2004 Pawel Jalocha
 * SP9VRC, 2007-2011 Dave Freese W1HKJ.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "mt63_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* dspPower(double X) (dsp.h:287-289): a function, so that a caller's "Sum += dspPower(x)"
 * is not fused, as in fldigi. */
static double power_d(double X)
{
    return X * X;
}

/* dspLowPass2 for a complex input (dsp.cxx:468-481) */
void mt63_lowpass2_cmpx(const mt63_cmpx *Inp, mt63_cmpx *Mid, mt63_cmpx *Out, double W1,
                        double W2, double W5)
{
    double Sum, Diff;
    Sum = Mid->re + Out->re;
    Diff = Mid->re - Out->re;
    Mid->re += W2 * Inp->re - W1 * Sum;
    Out->re += W5 * Diff;
    Sum = Mid->im + Out->im;
    Diff = Mid->im - Out->im;
    Mid->im += W2 * Inp->im - W1 * Sum;
    Out->im += W5 * Diff;
}

/* dspPower(double *X, int Len) (dsp.cxx:39-45) */
static double power_array(const double *X, int Len)
{
    double Sum;
    for (Sum = 0.0; Len; Len--, X++)
        Sum += (*X) * (*X);
    return Sum;
}

/* dspRMS(double *Data, int Len) (dsp.h:371-373) */
double mt63_rms(const double *Data, int Len)
{
    return sqrt(power_array(Data, Len) / Len);
}

/* dspFindMin(type *Data, int Len, int &MinPos) (dsp.h:396-408) */
double mt63_find_min(const double *Data, int Len, int *MinPos)
{
    double Min;
    int i, pos;
    Min = Data[0];
    pos = 0;
    for (i = 1; i < Len; i++)
        if (Data[i] < Min) {
            Min = Data[i];
            pos = i;
        }
    *MinPos = pos;
    return Min;
}

/* dspFindMax(type *Data, int Len, int &MaxPos) (dsp.h:420-432) */
double mt63_find_max(const double *Data, int Len, int *MaxPos)
{
    double Max;
    int i, pos;
    Max = Data[0];
    pos = 0;
    for (i = 1; i < Len; i++)
        if (Data[i] > Max) {
            Max = Data[i];
            pos = i;
        }
    *MaxPos = pos;
    return Max;
}

/* dspFindMaxdspPower(dspCmpx *Data, int Len, int &MaxPos) (dsp.cxx:96-110) */
double mt63_find_max_power(const mt63_cmpx *Data, int Len, int *MaxPos)
{
    double Max, Pwr;
    int i, pos;
    Max = mt63_power(Data[0]);
    pos = 0;
    for (i = 1; i < Len; i++) {
        Pwr = mt63_power(Data[i]);
        if (Pwr > Max) {
            Max = Pwr;
            pos = i;
        }
    }
    *MaxPos = pos;
    return Max;
}

/* dspSelFitAver for real data (dsp.h:444-492). With no sample inside the threshold fldigi
 * divides by Incl = 0; the IEEE result (NaN) is kept as in fldigi. */
void mt63_sel_fit_aver(const double *Data, int Len, double SelThres, int Loops, double *Aver,
                       double *Rms)
{
    int i, loop, Incl;
    double Sum, ErrSum, Lev, dLev, Diff, Thres;
    for (ErrSum = Sum = 0.0, i = 0; i < Len; i++) {
        Sum += Data[i];
        ErrSum += power_d(Data[i]);
    }
    Lev = Sum / Len;
    ErrSum /= Len;
    ErrSum -= Lev * Lev;
    Incl = 0;
    for (loop = 0; loop < Loops; loop++) {
        Thres = SelThres * SelThres * ErrSum;
        for (ErrSum = Sum = 0.0, Incl = 0, i = 0; i < Len; i++) {
            Diff = power_d(Data[i] - Lev);
            if (Diff <= Thres) {
                Sum += Data[i];
                ErrSum += Diff;
                Incl += 1;
            }
        }
        Sum /= Incl;
        dLev = Sum - Lev;
        ErrSum /= Incl;
        ErrSum -= dLev * dLev;
        Lev += dLev;
        ErrSum = fabs(ErrSum);
    }
    *Aver = Lev;
    *Rms = sqrt(ErrSum);
}

/* dspSelFitAver for complex data (dsp.h:494-548) */
void mt63_sel_fit_aver_cmpx(const mt63_cmpx *Data, int Len, double SelThres, int Loops,
                            mt63_cmpx *Aver, double *Rms)
{
    int i, loop, Incl;
    mt63_cmpx Sum, Lev, dLev;
    double ErrSum, Diff, Thres;
    for (ErrSum = 0.0, Sum.re = Sum.im = 0.0, i = 0; i < Len; i++) {
        Sum.re += Data[i].re;
        Sum.im += Data[i].im;
        ErrSum += mt63_power(Data[i]);
    }
    Lev.re = Sum.re / Len;
    Lev.im = Sum.im / Len;
    ErrSum /= Len;
    ErrSum -= mt63_power(Lev);
    Incl = 0;
    for (loop = 0; loop < Loops; loop++) {
        Thres = 0.5 * SelThres * SelThres * ErrSum;
        for (ErrSum = 0.0, Sum.re = Sum.im = 0.0, Incl = 0, i = 0; i < Len; i++) {
            Diff = mt63_power2(Data[i].re - Lev.re, Data[i].im - Lev.im);
            if (Diff <= Thres) {
                Sum.re += Data[i].re;
                Sum.im += Data[i].im;
                ErrSum += Diff;
                Incl += 1;
            }
        }
        Sum.re /= Incl;
        Sum.im /= Incl;
        dLev.re = Sum.re - Lev.re;
        dLev.im = Sum.im - Lev.im;
        ErrSum /= Incl;
        ErrSum -= mt63_power(dLev);
        ErrSum = fabs(ErrSum);
        Lev.re += dLev.re;
        Lev.im += dLev.im;
    }
    *Aver = Lev;
    *Rms = sqrt(ErrSum);
}

/* dspWindowBlackman3 (dsp.cxx:168-171) */
double mt63_window_blackman3(double phase)
{
    return 0.35875 + 0.48829 * cos(phase) + 0.14128 * cos(2 * phase) +
           0.01168 * cos(3 * phase);
}

/* The window reaches the FIR shapes through a pointer in fldigi (another translation
 * unit): keep it out of line here too. */
static double (*volatile const window_fn)(double) = mt63_window_blackman3;

/* dspWinFirI with shift 0 (dsp.cxx:176-192) */
void mt63_win_fir_i(double LowOmega, double UppOmega, double *Shape, int Len)
{
    int i;
    double time, phase, shape;
    const double shift = 0.0;
    double (*Window)(double) = window_fn;
    for (i = 0; i < Len; i++) {
        time = i + (1.0 - shift) - (double)Len / 2;
        phase = 2 * M_PI * time / Len;
        if (time == 0)
            shape = UppOmega - LowOmega;
        else
            shape = (sin(UppOmega * time) - sin(LowOmega * time)) / time;
        Shape[i] = shape * (*Window)(phase) / M_PI;
    }
}

/* WinFirQ with shift 0 (dsp.cxx:194-212): minus for the Q part, the shapes are applied in
 * reverse order. */
void mt63_win_fir_q(double LowOmega, double UppOmega, double *Shape, int Len)
{
    int i;
    double time, phase, shape;
    const double shift = 0.0;
    double (*Window)(double) = window_fn;
    for (i = 0; i < Len; i++) {
        time = i + (1.0 - shift) - (double)Len / 2;
        phase = 2 * M_PI * time / Len;
        if (time == 0)
            shape = 0.0;
        else
            shape = (-cos(UppOmega * time) + cos(LowOmega * time)) / time;
        Shape[i] = (-shape) * (*Window)(phase) / M_PI;
    }
}

/* dspWalshTrans (dsp.cxx:2589-2604) */
void mt63_walsh(double *Data, int Len)
{
    int step, ptr, ptr2;
    double bit1, bit2;
    for (step = 1; step < Len; step *= 2) {
        for (ptr = 0; ptr < Len; ptr += 2 * step) {
            for (ptr2 = ptr; (ptr2 - ptr) < step; ptr2 += 1) {
                bit1 = Data[ptr2];
                bit2 = Data[ptr2 + step];
                Data[ptr2] = (bit1 + bit2);
                Data[ptr2 + step] = (bit2 - bit1);
            }
        }
    }
}

/* dspWalshInvTrans (dsp.cxx:2606-2621) */
void mt63_walsh_inv(double *Data, int Len)
{
    int step, ptr, ptr2;
    double bit1, bit2;
    for (step = Len / 2; step; step /= 2) {
        for (ptr = 0; ptr < Len; ptr += 2 * step) {
            for (ptr2 = ptr; (ptr2 - ptr) < step; ptr2 += 1) {
                bit1 = Data[ptr2];
                bit2 = Data[ptr2 + step];
                Data[ptr2] = (bit1 - bit2);
                Data[ptr2 + step] = (bit1 + bit2);
            }
        }
    }
}

/* ---------------------------------------------------------------- dsp_r2FFT */

/* dsp_r2FFT::Preset (dsp.cxx:2092-2128): twiddles and the bit-reverse table. */
int mt63_fft_preset(mt63_fft *fft, int size)
{
    int idx, ridx, mask, rmask;
    double phase;
    if (size <= 0 || (size & (size - 1)))
        return -1;
    fft->Size = size;
    fft->BitRevIdx = (int *)malloc((size_t)size * sizeof(int));
    fft->Twiddle = (mt63_cmpx *)malloc((size_t)size * sizeof(mt63_cmpx));
    if (!fft->BitRevIdx || !fft->Twiddle) {
        mt63_fft_free(fft);
        return -1;
    }
    for (idx = 0; idx < size; idx++) {
        phase = (2 * M_PI * idx) / size;
        fft->Twiddle[idx].re = cos(phase);
        fft->Twiddle[idx].im = sin(phase);
    }
    for (ridx = 0, idx = 0; idx < size; idx++) {
        for (ridx = 0, mask = size / 2, rmask = 1; mask; mask >>= 1, rmask <<= 1) {
            if (idx & mask)
                ridx |= rmask;
        }
        fft->BitRevIdx[idx] = ridx;
    }
    return 0;
}

/* dsp_r2FFT::Free (dsp.cxx:2022-2028) */
void mt63_fft_free(mt63_fft *fft)
{
    free(fft->BitRevIdx);
    fft->BitRevIdx = NULL;
    free(fft->Twiddle);
    fft->Twiddle = NULL;
}

/* dsp_r2FFT::FFTbf, the radix-2 butterfly (dsp.cxx:2033-2042) */
static void fft_bf(mt63_cmpx *x0, mt63_cmpx *x1, const mt63_cmpx *W)
{
    mt63_cmpx x1W;
    x1W.re = x1->re * W->re + x1->im * W->im;
    x1W.im = (-x1->re * W->im) + x1->im * W->re;
    x1->re = x0->re - x1W.re;
    x1->im = x0->im - x1W.im;
    x0->re = x0->re + x1W.re;
    x0->im = x0->im + x1W.im;
}

/* dsp_r2FFT::FFT2, the 2-point FFT (dsp.cxx:2045-2054) */
static void fft2(mt63_cmpx *x0, mt63_cmpx *x1)
{
    mt63_cmpx x1W;
    x1W.re = x1->re;
    x1W.im = x1->im;
    x1->re = x0->re - x1->re;
    x1->im = x0->im - x1->im;
    x0->re += x1W.re;
    x0->im += x1W.im;
}

/* dsp_r2FFT::CoreProc (dsp.cxx:2133-2147): input already bit-reversed. */
void mt63_fft_core(const mt63_fft *fft, mt63_cmpx *x)
{
    int Groups, GroupHalfSize, Group, Bf, TwidIdx;
    int Size = fft->Size;
    int HalfSize = Size / 2;
    for (Bf = 0; Bf < Size; Bf += 2)
        fft2(&x[Bf], &x[Bf + 1]);
    for (Groups = HalfSize / 2, GroupHalfSize = 2; Groups; Groups >>= 1, GroupHalfSize <<= 1)
        for (Group = 0, Bf = 0; Group < Groups; Group++, Bf += GroupHalfSize)
            for (TwidIdx = 0; TwidIdx < HalfSize; TwidIdx += Groups, Bf++)
                fft_bf(&x[Bf], &x[Bf + GroupHalfSize], &fft->Twiddle[TwidIdx]);
}

/* ---------------------------------------------------------------- dspQuadrSplit */

/* dspQuadrSplit::Preset (dsp.cxx:817-835): Len zero taps; max_input bounds one call. */
int mt63_split_preset(mt63_split *s, int FilterLen, int DecimateRate, int max_input)
{
    s->Len = FilterLen;
    s->Rate = DecimateRate;
    s->TapCap = FilterLen + max_input + DecimateRate;
    s->ShapeI = (double *)calloc((size_t)FilterLen, sizeof(double));
    s->ShapeQ = (double *)calloc((size_t)FilterLen, sizeof(double));
    s->Tap = (double *)calloc((size_t)s->TapCap, sizeof(double));
    if (!s->ShapeI || !s->ShapeQ || !s->Tap) {
        mt63_split_free(s);
        return -1;
    }
    mt63_split_clear(s);
    return 0;
}

/* The state of dspQuadrSplit::Preset: Len zeros in the tap (dsp.cxx:829-832). */
void mt63_split_clear(mt63_split *s)
{
    s->TapLen = s->Len;
    memset(s->Tap, 0, (size_t)s->TapCap * sizeof(double));
}

/* dspQuadrSplit::ComputeShape with dspWindowBlackman3 (dsp.cxx:837-850) */
void mt63_split_compute_shape(mt63_split *s, double LowOmega, double UppOmega)
{
    mt63_win_fir_i(LowOmega, UppOmega, s->ShapeI, s->Len);
    mt63_win_fir_q(LowOmega, UppOmega, s->ShapeQ, s->Len);
}

/* dspQuadrSplit::Free (dsp.cxx:805-815) */
void mt63_split_free(mt63_split *s)
{
    free(s->ShapeI);
    free(s->ShapeQ);
    free(s->Tap);
    s->ShapeI = s->ShapeQ = s->Tap = NULL;
}

/* dspQuadrSplit::Process (dsp.cxx:852-884); InpLen <= max_input of the preset. */
int mt63_split_process(mt63_split *s, const double *Inp_, int InpLen, mt63_cmpx *Out)
{
    int i, t, o, l, k;
    double SumI, SumQ;
    const double *Inp;
    memcpy(s->Tap + s->TapLen, Inp_, (size_t)InpLen * sizeof(double));
    s->TapLen += InpLen;
    Inp = s->Tap;
    for (l = s->TapLen - s->Len, o = 0, i = 0; i < l; i += s->Rate) {
        for (SumI = SumQ = 0.0, k = i, t = 0; t < s->Len; t++, k++) {
            SumI += Inp[k] * s->ShapeI[t];
            SumQ += Inp[k] * s->ShapeQ[t];
        }
        Out[o].re = SumI;
        Out[o++].im = SumQ;
    }
    s->TapLen -= i;
    memmove(s->Tap, s->Tap + i, (size_t)s->TapLen * sizeof(double));
    return o;
}

/* ---------------------------------------------------------------- dspQuadrComb */

/* dspQuadrComb::Preset (dsp.cxx:918-938) */
int mt63_comb_preset(mt63_comb *c, int FilterLen, int DecimateRate)
{
    c->Len = FilterLen;
    c->Rate = DecimateRate;
    c->TapPtr = 0;
    c->ShapeI = (double *)calloc((size_t)FilterLen, sizeof(double));
    c->ShapeQ = (double *)calloc((size_t)FilterLen, sizeof(double));
    c->Tap = (double *)calloc((size_t)FilterLen, sizeof(double));
    if (!c->ShapeI || !c->ShapeQ || !c->Tap) {
        mt63_comb_free(c);
        return -1;
    }
    return 0;
}

/* dspQuadrComb::ComputeShape with dspWindowBlackman3 (dsp.cxx:940-955) */
void mt63_comb_compute_shape(mt63_comb *c, double LowOmega, double UppOmega)
{
    mt63_win_fir_i(LowOmega, UppOmega, c->ShapeI, c->Len);
    mt63_win_fir_q(LowOmega, UppOmega, c->ShapeQ, c->Len);
}

/* dspQuadrComb::Free (dsp.cxx:905-916) */
void mt63_comb_free(mt63_comb *c)
{
    free(c->ShapeI);
    free(c->ShapeQ);
    free(c->Tap);
    c->ShapeI = c->ShapeQ = c->Tap = NULL;
}

/* dspQuadrComb::Process (dsp.cxx:957-998): writes InpLen * Rate samples. */
int mt63_comb_process(mt63_comb *c, const mt63_cmpx *Inp, int InpLen, double *Out)
{
    int i, o, r, t, len;
    double I, Q;
    int Len = c->Len, Rate = c->Rate;
    double *Tap = c->Tap;
    for (o = 0, i = 0; i < InpLen; i++) {
        I = Inp[i].re;
        Q = Inp[i].im;
        for (r = 0, t = c->TapPtr; t < Len; t++, r++)
            Tap[t] += I * c->ShapeI[r] + Q * c->ShapeQ[r];
        for (t = 0; t < c->TapPtr; t++, r++)
            Tap[t] += I * c->ShapeI[r] + Q * c->ShapeQ[r];
        len = Len - c->TapPtr;
        if (len < Rate) {
            for (r = 0; r < len; r++) {
                Out[o++] = Tap[c->TapPtr];
                Tap[c->TapPtr++] = 0.0;
            }
            c->TapPtr = 0;
            for (; r < Rate; r++) {
                Out[o++] = Tap[c->TapPtr];
                Tap[c->TapPtr++] = 0.0;
            }
        } else {
            for (r = 0; r < Rate; r++) {
                Out[o++] = Tap[c->TapPtr];
                Tap[c->TapPtr++] = 0.0;
            }
        }
    }
    return o;
}

/* ---------------------------------------------------------------- dspCmpxOverlapWindow */

/* dspCmpxOverlapWindow::Preset (dsp.cxx:1553-1567) */
int mt63_overlap_preset(mt63_overlap *w, int WindowLen, int SlideDist, const double *Shape)
{
    if (SlideDist > WindowLen)
        return -1;
    w->Len = WindowLen;
    w->Dist = SlideDist;
    w->Window = Shape;
    w->Buff = (mt63_cmpx *)calloc((size_t)WindowLen, sizeof(mt63_cmpx));
    return w->Buff ? 0 : -1;
}

/* dspCmpxOverlapWindow::Free (dsp.cxx:1544-1551) */
void mt63_overlap_free(mt63_overlap *w)
{
    free(w->Buff);
    w->Buff = NULL;
}

/* dspCmpxOverlapWindow::Process(dspCmpx *Inp, dspCmpx *Out) with a window
 * (dsp.cxx:1612-1641) */
static void overlap_one(mt63_overlap *w, const mt63_cmpx *Inp, mt63_cmpx *Out)
{
    int i;
    mt63_cmpx *Buff = w->Buff;
    const double *Window = w->Window;
    int Len = w->Len, Dist = w->Dist;
    for (i = 0; i < Dist; i++) {
        Out[i].re = Buff[i].re + Inp[i].re * Window[i];
        Out[i].im = Buff[i].im + Inp[i].im * Window[i];
    }
    for (; i < Len - Dist; i++) {
        Buff[i - Dist].re = Buff[i].re + Inp[i].re * Window[i];
        Buff[i - Dist].im = Buff[i].im + Inp[i].im * Window[i];
    }
    for (; i < Len; i++) {
        Buff[i - Dist].re = Inp[i].re * Window[i];
        Buff[i - Dist].im = Inp[i].im * Window[i];
    }
}

/* dspCmpxOverlapWindow::Process(dspCmpx_buff *Input) (dsp.cxx:1589-1600): one slide of
 * Dist output samples per Len input samples; returns the output length. */
int mt63_overlap_process(mt63_overlap *w, const mt63_cmpx *Inp, int InpLen, mt63_cmpx *Out)
{
    int i, n = 0;
    for (i = 0; i < InpLen; i += w->Len) {
        overlap_one(w, Inp + i, Out + n);
        n += w->Dist;
    }
    return n;
}

/* ---------------------------------------------------------------- dspDelayLine */

/* dspDelayLine::Preset with MaxSize 0 (dsp.h:894-907) */
int mt63_delay_preset(mt63_delay *d, int MaxDelay)
{
    d->LineSize = 2 * MaxDelay;
    d->Delay = MaxDelay;
    d->Line = (mt63_cmpx *)calloc((size_t)d->LineSize, sizeof(mt63_cmpx));
    if (!d->Line)
        return -1;
    mt63_delay_clear(d);
    return 0;
}

/* The state after dspDelayLine::Preset (dsp.h:899-906) */
void mt63_delay_clear(mt63_delay *d)
{
    memset(d->Line, 0, (size_t)d->LineSize * sizeof(mt63_cmpx));
    d->DataLen = d->Delay;
    d->InpPos = d->DataLen;
    d->InpLen = 0;
}

/* dspDelayLine::Free (dsp.h:888-892) */
void mt63_delay_free(mt63_delay *d)
{
    free(d->Line);
    d->Line = NULL;
}

/* dspDelayLine::Process (dsp.h:909-922) */
int mt63_delay_process(mt63_delay *d, const mt63_cmpx *Inp, int Len)
{
    if ((d->DataLen + Len) > d->LineSize) {
        memmove(d->Line, d->Line + d->DataLen - d->Delay, (size_t)d->Delay * sizeof(mt63_cmpx));
        d->DataLen = d->Delay;
    }
    if ((d->DataLen + Len) > d->LineSize)
        return -1;
    memcpy(d->Line + d->DataLen, Inp, (size_t)Len * sizeof(mt63_cmpx));
    d->InpPos = d->DataLen;
    d->InpLen = Len;
    d->DataLen += Len;
    return 0;
}
