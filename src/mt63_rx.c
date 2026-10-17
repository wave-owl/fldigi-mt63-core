/*
 * fldigi-mt63-core -- the MT63 receiver: MT63rx and MT63decoder of fldigi 4.2.13
 * src/mt63/mt63base.cxx and the receive side of mt63::rx_process / rx_flush
 * (src/mt63/mt63.cxx). Every function names the fldigi lines it ports.
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

#define CODE_TAP 4096
#define DATA_SCAN_LEN (MT63_CARRIERS + 2 * MT63_SCAN_MARGIN)          /* 80 */
#define SCAN_LEN ((MT63_CARRIERS + 2 * MT63_SCAN_MARGIN) * MT63_CARR_SEPAR) /* 320 */
#define FIT_LEN (2 * MT63_SCAN_MARGIN * MT63_CARR_SEPAR)                /* 64 */
#define DEC_SCAN_LEN (2 * MT63_SCAN_MARGIN + 1)                         /* 17 */
#define MAX_INTEG 32

/* MT63decoder (mt63base.h:117-146) */
typedef struct {
    int IntlvLen, IntlvSize, IntlvPtr;
    int IntlvPatt[MT63_CARRIERS];
    double *IntlvPipe;
    double WalshBuff[MT63_CARRIERS];
    int ScanSize;
    double DecodeSnrMid[DEC_SCAN_LEN], DecodeSnrOut[DEC_SCAN_LEN];
    double W1, W2, W5;
    unsigned char DecodePipe[(MAX_INTEG / 2) * DEC_SCAN_LEN];
    int DecodeLen, DecodeSize, DecodePtr;
    unsigned char Output;
    double SignalToNoise;
    int CarrOfs;
} decoder;

struct fmt63_rx {
    const fmt63_submode *sub;
    fmt63_rx_config cfg;
    double carrier;
    /* MT63rx (mt63base.h:240-374) */
    mt63_split InpSplit;
    mt63_delay ProcLine;
    mt63_cmpx *SplitOut;
    int ProcDelay, SyncProcPtr, DataProcPtr;
    mt63_fft FFT;
    int WindowLen, WindowLenMask;
    int DecimateRatio, DataInterleave;
    const int *InterleavePattern;
    int FirstDataCarr, IntegLen, SyncStep, ScanFirst;
    mt63_cmpx FFTbuff[MT63_SYMBOL_LEN], FFTbuff2[MT63_SYMBOL_LEN];
    mt63_cmpx SyncPipe[MT63_SYMBOL_DIV][SCAN_LEN];
    int SyncPtr, SymbPtr;
    mt63_cmpx SyncPhCorr[SCAN_LEN];
    mt63_cmpx CorrelMid[MT63_SYMBOL_DIV][SCAN_LEN], CorrelOut[MT63_SYMBOL_DIV][SCAN_LEN];
    double PowerMid[SCAN_LEN], PowerOut[SCAN_LEN];
    mt63_cmpx CorrelNorm[MT63_SYMBOL_DIV][SCAN_LEN];
    double W1, W2, W5, W1p, W2p, W5p;
    mt63_cmpx CorrelAver[MT63_SYMBOL_DIV][FIT_LEN];
    mt63_cmpx SymbFit[FIT_LEN];
    int SymbFitPos;
    double FreqPipe[MAX_INTEG];
    mt63_cmpx SymbPipe[MAX_INTEG];
    int TrackPipeLen, TrackPipePtr;
    double AverFreq;
    mt63_cmpx AverSymb;
    double SyncLockThres, SyncHoldThres;
    int SyncLocked;
    double SyncSymbConf, SyncFreqOfs, SyncFreqDev, SyncSymbShift;
    int DataScanFirst;
    mt63_cmpx RefDataSlice[DATA_SCAN_LEN], DataVect[DATA_SCAN_LEN];
    int DataPipeLen, DataPipePtr;
    mt63_cmpx DataPipe[MAX_INTEG / 2][DATA_SCAN_LEN];
    double DataPwrMid[DATA_SCAN_LEN], DataPwrOut[DATA_SCAN_LEN];
    double dW1, dW2, dW5;
    double DataPhase[DATA_SCAN_LEN];
    decoder Decoder;
    /* mt63 (mt63.h): escape state, the current block's output, test tap */
    int escape;
    unsigned char block_out[8];
    int block_len;
    double snr;              /* FEC_SNR after the last block (mt63.cxx:197) */
    size_t dropped;
    unsigned char codes[CODE_TAP];
    size_t ncodes;
};

/* ---------------------------------------------------------------- MT63decoder */

/* MT63decoder::Preset (mt63base.cxx:475-519) */
static void decoder_preset(decoder *d, int Intlv, const int *Pattern, int Integ)
{
    int i, p;
    d->ScanSize = MT63_CARRIERS + 2 * MT63_SCAN_MARGIN;
    mt63_lowpass2_coeff(Integ, &d->W1, &d->W2, &d->W5);
    d->DecodeLen = Integ / 2;
    d->DecodeSize = d->DecodeLen * DEC_SCAN_LEN;
    memset(d->DecodePipe, 0, sizeof d->DecodePipe);
    d->DecodePtr = 0;
    d->IntlvLen = Intlv;
    for (p = 0, i = 0; i < MT63_CARRIERS; i++) {
        d->IntlvPatt[i] = p * d->ScanSize;
        p += Pattern[i];
        if (p >= d->IntlvLen)
            p -= d->IntlvLen;
    }
    d->IntlvSize = (d->IntlvLen + 1) * d->ScanSize;
    memset(d->IntlvPipe, 0, (size_t)d->IntlvSize * sizeof(double));
    d->IntlvPtr = 0;
    memset(d->DecodeSnrMid, 0, sizeof d->DecodeSnrMid);
    memset(d->DecodeSnrOut, 0, sizeof d->DecodeSnrOut);
    d->SignalToNoise = 0.0;
    d->CarrOfs = 0;
    d->Output = 0;
}

/* MT63decoder::Process (mt63base.cxx:521-579): de-interleave, Walsh transform for every
 * carrier offset, best s/n wins, delayed by Integ / 2 symbols. */
static void decoder_process(decoder *d, const double *data)
{
    int s, i, k;
    double Min, Max, Sig, Noise, SNR;
    int MinPos, MaxPos, code;
    int ScanSize = d->ScanSize, IntlvSize = d->IntlvSize;

    memcpy(d->IntlvPipe + d->IntlvPtr, data, (size_t)ScanSize * sizeof(double));
    for (s = 0; s < DEC_SCAN_LEN; s++) {
        for (i = 0; i < MT63_CARRIERS; i++) {
            k = d->IntlvPtr - ScanSize - d->IntlvPatt[i];
            if (k < 0)
                k += IntlvSize;
            if ((s & 1) && (i & 1)) {
                k += ScanSize;
                if (k >= IntlvSize)
                    k -= IntlvSize;
            }
            d->WalshBuff[i] = d->IntlvPipe[k + s + i];
        }
        mt63_walsh(d->WalshBuff, MT63_CARRIERS);
        Min = mt63_find_min(d->WalshBuff, MT63_CARRIERS, &MinPos);
        Max = mt63_find_max(d->WalshBuff, MT63_CARRIERS, &MaxPos);
        if (fabs(Max) > fabs(Min)) {
            code = MaxPos + MT63_CARRIERS;
            Sig = fabs(Max);
            d->WalshBuff[MaxPos] = 0.0;
        } else {
            code = MinPos;
            Sig = fabs(Min);
            d->WalshBuff[MinPos] = 0.0;
        }
        Noise = mt63_rms(d->WalshBuff, MT63_CARRIERS);
        if (Noise > 0.0)
            SNR = Sig / Noise;
        else
            SNR = 0.0;
        mt63_lowpass2(SNR, &d->DecodeSnrMid[s], &d->DecodeSnrOut[s], d->W1, d->W2, d->W5);
        d->DecodePipe[d->DecodePtr + s] = (unsigned char)code;
    }
    d->IntlvPtr += ScanSize;
    if (d->IntlvPtr >= IntlvSize)
        d->IntlvPtr = 0;
    d->DecodePtr += DEC_SCAN_LEN;
    if (d->DecodePtr >= d->DecodeSize)
        d->DecodePtr = 0;
    Max = mt63_find_max(d->DecodeSnrOut, DEC_SCAN_LEN, &MaxPos);
    d->Output = d->DecodePipe[d->DecodePtr + MaxPos];
    d->SignalToNoise = Max;
    d->CarrOfs = MaxPos - (DEC_SCAN_LEN - 1) / 2;
}

/* ---------------------------------------------------------------- MT63rx::Preset */

/* MT63rx::Preset (mt63base.cxx:743-942) for the configured carrier; the arrays are
 * allocated once in fmt63_rx_create, so this only (re)computes and clears. */
static void rx_preset(fmt63_rx *rx)
{
    int i, c;
    int BandWidth = rx->sub->bandwidth;
    double freq = rx->carrier;
    double hbw = 1.5 * BandWidth / 2;
    double omega_low = (freq - hbw);
    double omega_high = (freq + hbw);
    if (omega_low < 100)
        omega_low = 100;
    if (omega_high > 4000)
        omega_high = 4000;
    omega_low *= (M_PI / 4000);
    omega_high *= (M_PI / 4000);

    rx->FirstDataCarr = mt63__first_carrier(BandWidth, freq);
    rx->DecimateRatio = rx->sub->decimate;
    rx->WindowLen = MT63_SYMBOL_LEN;
    rx->IntegLen = rx->cfg.long_integration ? 32 : 16;   /* mt63.cxx:52-55 */
    rx->SyncStep = MT63_SYMBOL_SEPAR / MT63_SYMBOL_DIV;
    rx->ProcDelay = rx->IntegLen * MT63_SYMBOL_SEPAR;
    rx->TrackPipeLen = rx->IntegLen;
    if (rx->sub->interleave == 64) {
        rx->DataInterleave = 64;
        rx->InterleavePattern = mt63_long_intlv_patt;
    } else {
        rx->DataInterleave = 32;
        rx->InterleavePattern = mt63_short_intlv_patt;
    }
    rx->WindowLenMask = rx->WindowLen - 1;

    mt63_split_clear(&rx->InpSplit);
    mt63_split_compute_shape(&rx->InpSplit, omega_low, omega_high);
    mt63_delay_clear(&rx->ProcLine);
    rx->SyncProcPtr = 0;

    rx->ScanFirst = rx->FirstDataCarr - MT63_SCAN_MARGIN * MT63_CARR_SEPAR;
    if (rx->ScanFirst < 0)
        rx->ScanFirst += rx->WindowLen;
    memset(rx->SyncPipe, 0, sizeof rx->SyncPipe);
    rx->SyncPtr = 0;

    for (c = (rx->ScanFirst * MT63_SYMBOL_SEPAR) & rx->WindowLenMask, i = 0; i < SCAN_LEN;
         i++) {
        const mt63_cmpx *T = &rx->FFT.Twiddle[c];
        rx->SyncPhCorr[i].re = T->re * T->re - T->im * T->im;
        rx->SyncPhCorr[i].im = 2 * T->re * T->im;
        c = (c + MT63_SYMBOL_SEPAR) & rx->WindowLenMask;
    }
    memset(rx->CorrelMid, 0, sizeof rx->CorrelMid);
    memset(rx->CorrelOut, 0, sizeof rx->CorrelOut);
    mt63_lowpass2_coeff(rx->IntegLen, &rx->W1, &rx->W2, &rx->W5);
    memset(rx->PowerMid, 0, sizeof rx->PowerMid);
    memset(rx->PowerOut, 0, sizeof rx->PowerOut);
    mt63_lowpass2_coeff(rx->IntegLen * MT63_SYMBOL_DIV, &rx->W1p, &rx->W2p, &rx->W5p);
    memset(rx->CorrelNorm, 0, sizeof rx->CorrelNorm);
    memset(rx->CorrelAver, 0, sizeof rx->CorrelAver);
    memset(rx->SymbFit, 0, sizeof rx->SymbFit);
    memset(rx->SymbPipe, 0, sizeof rx->SymbPipe);
    memset(rx->FreqPipe, 0, sizeof rx->FreqPipe);
    rx->TrackPipePtr = 0;
    rx->AverFreq = 0.0;
    rx->AverSymb.re = rx->AverSymb.im = 0.0;

    rx->SymbFitPos = MT63_SCAN_MARGIN * MT63_CARR_SEPAR;
    rx->SyncLocked = 0;
    rx->SyncSymbConf = 0.0;
    rx->SyncFreqOfs = 0.0;
    rx->SyncFreqDev = 0.0;
    rx->SymbPtr = 0;
    rx->SyncSymbShift = 0.0;

    rx->SyncHoldThres = 1.5 * sqrt(1.0 / (rx->IntegLen * MT63_CARRIERS));
    rx->SyncLockThres = 1.5 * rx->SyncHoldThres;

    rx->DataProcPtr = (-rx->ProcDelay);
    rx->DataScanFirst = rx->FirstDataCarr - MT63_SCAN_MARGIN * MT63_CARR_SEPAR;
    memset(rx->RefDataSlice, 0, sizeof rx->RefDataSlice);
    rx->DataPipeLen = rx->IntegLen / 2;
    mt63_lowpass2_coeff(rx->IntegLen, &rx->dW1, &rx->dW2, &rx->dW5);
    memset(rx->DataPipe, 0, sizeof rx->DataPipe);
    rx->DataPipePtr = 0;
    memset(rx->DataPwrMid, 0, sizeof rx->DataPwrMid);
    memset(rx->DataPwrOut, 0, sizeof rx->DataPwrOut);

    decoder_preset(&rx->Decoder, rx->DataInterleave, rx->InterleavePattern, rx->IntegLen);
    /* mt63::rx_init (mt63.cxx:48-58) */
    rx->escape = 0;
    rx->snr = 0.0;
}

/* ---------------------------------------------------------------- SyncProcess */

/* MT63rx::DoCorrelSum (mt63base.cxx:988-1016), including fldigi's sign of the imaginary
 * part of the samples entering the sliding sum. */
static void do_correl_sum(const mt63_cmpx *Correl1, const mt63_cmpx *Correl2, mt63_cmpx *Aver)
{
    mt63_cmpx sx;
    int i, s, d;
    s = 2 * MT63_CARR_SEPAR;
    d = MT63_CARRIERS * MT63_CARR_SEPAR;
    sx.re = sx.im = 0.0;
    for (i = 0; i < d; i += s) {
        sx.re += Correl1[i].re;
        sx.im += Correl1[i].im;
        sx.re += Correl2[i].re;
        sx.im += Correl2[i].im;
    }
    Aver[0].re = sx.re / MT63_CARRIERS;
    Aver[0].im = sx.im / MT63_CARRIERS;
    for (i = 0; i < (FIT_LEN - s);) {
        sx.re -= Correl1[i].re;
        sx.im -= Correl1[i].im;
        sx.re -= Correl2[i].re;
        sx.im -= Correl2[i].im;
        sx.re += Correl1[i + d].re;
        sx.im -= Correl1[i + d].im;
        sx.re += Correl2[i + d].re;
        sx.im -= Correl2[i + d].im;
        i += s;
        Aver[i].re = sx.re / MT63_CARRIERS;
        Aver[i].im = sx.im / MT63_CARRIERS;
    }
}

/* dspPhase (dsp.h:323-329) */
static double phase_of(double I, double Q)
{
    return atan2(Q, I);
}

/* dspAmpl of a SymbFit entry (dsp.h:317-319), a function as in fldigi */
static double ampl(mt63_cmpx X)
{
    return mt63_ampl(X);
}

/* dspScalProd(I, Q, B) (dsp.h:68-70) */
static double sprod(double I, double Q, mt63_cmpx B)
{
    return mt63_scal_prod_iq(I, Q, B);
}

/* MT63rx::SyncProcess (mt63base.cxx:1018-1296): FFT of the windowed slice, per-bin
 * squared-phase correlation with the slice one symbol earlier, two-pole integrators; once
 * per symbol the carrier-pattern sums, the symbol-time and frequency fit, selective
 * averaging and the lock hysteresis. */
static void sync_process(fmt63_rx *rx, const mt63_cmpx *Slice)
{
    int i, j, k, r, s, s2;
    double pI, pQ;
    mt63_cmpx Correl;
    mt63_cmpx *PrevSlice;
    double I, Q, dI, dQ, P, A, w0, w1, Fl, F0, Fu;
    mt63_cmpx SymbTime;
    double SymbConf, SymbShift, FreqOfs, Rms;
    const int WindowLen = rx->WindowLen, mask = rx->WindowLenMask;
    const int SymbolDiv = MT63_SYMBOL_DIV, DataCarrSepar = MT63_CARR_SEPAR;
    const int SymbolSepar = MT63_SYMBOL_SEPAR;

    rx->SyncPtr = (rx->SyncPtr + 1) & (SymbolDiv - 1);

    for (i = 0; i < WindowLen; i++) {
        r = rx->FFT.BitRevIdx[i];
        rx->FFTbuff[r].re = Slice[i].re * mt63_symbol_shape[i];
        rx->FFTbuff[r].im = Slice[i].im * mt63_symbol_shape[i];
    }
    mt63_fft_core(&rx->FFT, rx->FFTbuff);

    PrevSlice = rx->SyncPipe[rx->SyncPtr];
    for (i = 0; i < SCAN_LEN; i++) {
        k = (rx->ScanFirst + i) & mask;
        I = rx->FFTbuff[k].re;
        Q = rx->FFTbuff[k].im;
        P = I * I + Q * Q;
        A = sqrt(P);
        if (P > 0.0) {
            dI = (I * I - Q * Q) / A;
            dQ = (2 * I * Q) / A;
        } else {
            dI = dQ = 0.0;
        }
        mt63_lowpass2(P, &rx->PowerMid[i], &rx->PowerOut[i], rx->W1p, rx->W2p, rx->W5p);
        pI = PrevSlice[i].re * rx->SyncPhCorr[i].re - PrevSlice[i].im * rx->SyncPhCorr[i].im;
        pQ = PrevSlice[i].re * rx->SyncPhCorr[i].im + PrevSlice[i].im * rx->SyncPhCorr[i].re;
        Correl.re = dQ * pQ + dI * pI;
        Correl.im = dQ * pI - dI * pQ;
        mt63_lowpass2_cmpx(&Correl, &rx->CorrelMid[rx->SyncPtr][i], &rx->CorrelOut[rx->SyncPtr][i],
                           rx->W1, rx->W2, rx->W5);
        PrevSlice[i].re = dI;
        PrevSlice[i].im = dQ;
    }

    if (rx->SyncPtr != (rx->SymbPtr ^ 2))
        return;

    for (s = 0; s < SymbolDiv; s++) {
        for (i = 0; i < SCAN_LEN; i++) {
            if (rx->PowerOut[i] > 0.0) {
                rx->CorrelNorm[s][i].re = rx->CorrelOut[s][i].re / rx->PowerOut[i];
                rx->CorrelNorm[s][i].im = rx->CorrelOut[s][i].im / rx->PowerOut[i];
            } else
                rx->CorrelNorm[s][i].im = rx->CorrelNorm[s][i].re = 0.0;
        }
    }
    for (s = 0; s < SymbolDiv; s++) {
        s2 = (s + SymbolDiv / 2) & (SymbolDiv - 1);
        for (k = 0; k < 2 * DataCarrSepar; k++)
            do_correl_sum(rx->CorrelNorm[s] + k, rx->CorrelNorm[s2] + k + DataCarrSepar,
                          rx->CorrelAver[s] + k);
    }
    for (i = 0; i < FIT_LEN; i++) {
        rx->SymbFit[i].re = ampl(rx->CorrelAver[0][i]) - ampl(rx->CorrelAver[2][i]);
        rx->SymbFit[i].im = ampl(rx->CorrelAver[1][i]) - ampl(rx->CorrelAver[3][i]);
    }
    P = mt63_find_max_power(rx->SymbFit + 2, FIT_LEN - 4, &j);
    j += 2;
    k = (j - rx->SymbFitPos) / DataCarrSepar;
    if (k > 1)
        j -= (k - 1) * DataCarrSepar;
    else if (k < (-1))
        j -= (k + 1) * DataCarrSepar;
    rx->SymbFitPos = j;
    if (P > 0.0) {
        const mt63_cmpx *SF = rx->SymbFit;
        SymbConf = ampl(SF[j]) + 0.5 * (ampl(SF[j + 1]) + ampl(SF[j - 1]));
        SymbConf *= 0.5;
        I = SF[j].re + 0.5 * (SF[j - 1].re + SF[j + 1].re);
        Q = SF[j].im + 0.5 * (SF[j - 1].im + SF[j + 1].im);
        SymbTime.re = I;
        SymbTime.im = Q;
        SymbShift = (phase_of(SymbTime.re, SymbTime.im) / (2 * M_PI)) * SymbolDiv;
        if (SymbShift < 0)
            SymbShift += SymbolDiv;
        pI = sprod(I, Q, SF[j]) + 0.7 * sprod(I, Q, SF[j - 1]) + 0.7 * sprod(I, Q, SF[j + 1]);
        pQ = 0.7 * sprod(I, Q, SF[j + 1]) - 0.7 * sprod(I, Q, SF[j - 1]) +
             0.5 * sprod(I, Q, SF[j + 2]) - 0.5 * sprod(I, Q, SF[j - 2]);
        FreqOfs = j + phase_of(pI, pQ) / (2.0 * M_PI / 8);
        i = (int)floor(FreqOfs + 0.5);
        s = (int)floor(SymbShift);
        /* fldigi reads CorrelAver[s][i] without a range check: i leaves 0 .. FitLen-1 by up
         * to two bins at the scan edges and SymbShift can round up to SymbolDiv; fldigi
         * reads heap memory there, the port clamps (the frozen Python receiver does the
         * same for i). */
        if (i < 0)
            i = 0;
        if (i > FIT_LEN - 1)
            i = FIT_LEN - 1;
        if (s > SymbolDiv - 1)
            s = SymbolDiv - 1;
        s2 = (s + 1) & (SymbolDiv - 1);
        w0 = (s + 1 - SymbShift);
        w1 = (SymbShift - s);
        A = (0.5 * WindowLen) / SymbolSepar;
        I = w0 * rx->CorrelAver[s][i].re + w1 * rx->CorrelAver[s2][i].re;
        Q = w0 * rx->CorrelAver[s][i].im + w1 * rx->CorrelAver[s2][i].im;
        F0 = i + phase_of(I, Q) / (2.0 * M_PI) * A - FreqOfs;
        Fl = F0 - A;
        Fu = F0 + A;
        if (fabs(Fl) < fabs(F0))
            FreqOfs += (fabs(Fu) < fabs(Fl)) ? Fu : Fl;
        else
            FreqOfs += (fabs(Fu) < fabs(F0)) ? Fu : F0;
    } else {
        SymbTime.re = SymbTime.im = 0.0;
        SymbConf = 0.0;
        SymbShift = 0.0;
        FreqOfs = 0.0;
    }
    (void)SymbConf;
    (void)SymbShift;

    if (rx->SyncLocked) {
        if (mt63_scal_prod(SymbTime, rx->AverSymb) < 0.0) {
            SymbTime.re = (-SymbTime.re);
            SymbTime.im = (-SymbTime.im);
            FreqOfs -= DataCarrSepar;
        }
        A = 2 * DataCarrSepar;
        k = (int)floor((FreqOfs - rx->AverFreq) / A + 0.5);
        FreqOfs -= k * A;
        A = (0.5 * WindowLen) / SymbolSepar;
        F0 = FreqOfs - rx->AverFreq;
        Fl = F0 - A;
        Fu = F0 + A;
        if (fabs(Fl) < fabs(F0))
            FreqOfs += (fabs(Fu) < fabs(Fl)) ? A : -A;
        else
            FreqOfs += (fabs(Fu) < fabs(F0)) ? A : 0.0;
    } else {
        if (mt63_scal_prod(SymbTime, rx->SymbPipe[rx->TrackPipePtr]) < 0.0) {
            SymbTime.re = (-SymbTime.re);
            SymbTime.im = (-SymbTime.im);
            FreqOfs -= DataCarrSepar;
        }
        A = 2 * DataCarrSepar;
        k = (int)floor(FreqOfs / A + 0.5);
        FreqOfs -= k * A;
        F0 = FreqOfs - rx->FreqPipe[rx->TrackPipePtr];
        Fl = F0 - A;
        Fu = F0 + A;
        if (fabs(Fl) < fabs(F0))
            FreqOfs += (fabs(Fu) < fabs(Fl)) ? A : -A;
        else
            FreqOfs += (fabs(Fu) < fabs(F0)) ? A : 0.0;
    }

    rx->TrackPipePtr += 1;
    if (rx->TrackPipePtr >= rx->TrackPipeLen)
        rx->TrackPipePtr -= rx->TrackPipeLen;
    rx->SymbPipe[rx->TrackPipePtr] = SymbTime;
    rx->FreqPipe[rx->TrackPipePtr] = FreqOfs;

    mt63_sel_fit_aver_cmpx(rx->SymbPipe, rx->TrackPipeLen, 3.0, 4, &rx->AverSymb, &Rms);
    mt63_sel_fit_aver(rx->FreqPipe, rx->TrackPipeLen, 2.5, 4, &rx->AverFreq, &Rms);
    rx->SyncFreqDev = Rms;

    SymbConf = ampl(rx->AverSymb);
    rx->SyncSymbConf = SymbConf;
    rx->SyncFreqOfs = rx->AverFreq;
    if (SymbConf > 0.0) {
        SymbShift = phase_of(rx->AverSymb.re, rx->AverSymb.im) / (2 * M_PI) * SymbolSepar;
        if (SymbShift < 0.0)
            SymbShift += SymbolSepar;
        rx->SymbPtr =
            (int)floor((phase_of(rx->AverSymb.re, rx->AverSymb.im) / (2 * M_PI)) * SymbolDiv);
        if (rx->SymbPtr < 0)
            rx->SymbPtr += SymbolDiv;
        rx->SyncSymbShift = SymbShift;
    }

    if (rx->SyncLocked) {
        if ((rx->SyncSymbConf < rx->SyncHoldThres) || (rx->SyncFreqDev > 0.250))
            rx->SyncLocked = 0;
    } else {
        if ((rx->SyncSymbConf > rx->SyncLockThres) && (rx->SyncFreqDev < 0.125))
            rx->SyncLocked = 1;
    }
    rx->SyncSymbConf *= 0.5;
}

/* ---------------------------------------------------------------- DataProcess */

/* MT63rx::DataProcess (mt63base.cxx:1298-1447): even/odd carrier slices, differential
 * phase against the previous symbol, frequency correction, the IntegLen / 2 symbol pipe,
 * normalised real parts into the decoder. */
static void data_process(fmt63_rx *rx, const mt63_cmpx *EvenSlice, const mt63_cmpx *OddSlice,
                         double FreqOfs, int TimeDist)
{
    int i, c, r, incr, p;
    mt63_cmpx Freq, Phas, Dtmp, Ftmp;
    double I, Q, P;
    const int WindowLen = rx->WindowLen, mask = rx->WindowLenMask;
    const int DataCarrSepar = MT63_CARR_SEPAR;

    P = (-2 * M_PI * FreqOfs) / WindowLen;
    Freq.re = cos(P);
    Freq.im = sin(P);
    Phas.re = 1.0;
    Phas.im = 0.0;
    for (i = 0; i < WindowLen; i++) {
        r = rx->FFT.BitRevIdx[i];
        I = EvenSlice[i].re * Phas.re - EvenSlice[i].im * Phas.im;
        Q = EvenSlice[i].re * Phas.im + EvenSlice[i].im * Phas.re;
        rx->FFTbuff[r].re = I * mt63_symbol_shape[i];
        rx->FFTbuff[r].im = Q * mt63_symbol_shape[i];
        I = OddSlice[i].re * Phas.re - OddSlice[i].im * Phas.im;
        Q = OddSlice[i].re * Phas.im + OddSlice[i].im * Phas.re;
        rx->FFTbuff2[r].re = I * mt63_symbol_shape[i];
        rx->FFTbuff2[r].im = Q * mt63_symbol_shape[i];
        Dtmp = mt63_mult(Phas, Freq);
        Phas = Dtmp;
    }
    mt63_fft_core(&rx->FFT, rx->FFTbuff);
    mt63_fft_core(&rx->FFT, rx->FFTbuff2);

    incr = (TimeDist * DataCarrSepar) & mask;
    p = (TimeDist * rx->DataScanFirst) & mask;
    /* fldigi starts at c = DataScanFirst unmasked, which is negative for a carrier below
     * 32 bins (MT63-2000 under 1250 Hz); the FFT is periodic, the port masks. */
    for (c = rx->DataScanFirst & mask, i = 0; i < DATA_SCAN_LEN;) {
        Phas = rx->FFT.Twiddle[p];
        Dtmp = mt63_mult(rx->RefDataSlice[i], Phas);
        rx->DataVect[i] = mt63_mult_s(rx->FFTbuff[c], Dtmp);
        mt63_lowpass2(mt63_power(rx->FFTbuff[c]), &rx->DataPwrMid[i], &rx->DataPwrOut[i],
                      rx->dW1, rx->dW2, rx->dW5);
        rx->RefDataSlice[i++] = rx->FFTbuff[c];
        c = (c + DataCarrSepar) & mask;
        p = (p + incr) & mask;

        Phas = rx->FFT.Twiddle[p];
        Dtmp = mt63_mult(rx->RefDataSlice[i], Phas);
        rx->DataVect[i] = mt63_mult_s(rx->FFTbuff2[c], Dtmp);
        mt63_lowpass2(mt63_power(rx->FFTbuff2[c]), &rx->DataPwrMid[i], &rx->DataPwrOut[i],
                      rx->dW1, rx->dW2, rx->dW5);
        rx->RefDataSlice[i++] = rx->FFTbuff2[c];
        c = (c + DataCarrSepar) & mask;
        p = (p + incr) & mask;
    }

    P = (-TimeDist * 2 * M_PI * FreqOfs) / WindowLen;
    Freq.re = cos(P);
    Freq.im = sin(P);
    for (i = 0; i < DATA_SCAN_LEN; i++) {
        Ftmp = mt63_mult(rx->DataVect[i], Freq);
        rx->DataVect[i] = rx->DataPipe[rx->DataPipePtr][i];
        rx->DataPipe[rx->DataPipePtr][i] = Ftmp;
    }
    rx->DataPipePtr += 1;
    if (rx->DataPipePtr >= rx->DataPipeLen)
        rx->DataPipePtr = 0;

    for (i = 0; i < DATA_SCAN_LEN; i++) {
        if (rx->DataPwrOut[i] > 0.0) {
            P = rx->DataVect[i].re / rx->DataPwrOut[i];
            if (P > 1.0)
                P = 1.0;
            else if (P < (-1.0))
                P = (-1.0);
            rx->DataPhase[i] = P;
        } else
            rx->DataPhase[i] = 0.0;
    }
    decoder_process(&rx->Decoder, rx->DataPhase);
    if (rx->block_len < (int)sizeof rx->block_out)
        rx->block_out[rx->block_len++] = rx->Decoder.Output;
    rx->codes[rx->ncodes % CODE_TAP] = rx->Decoder.Output;
    rx->ncodes++;
}

/* MT63rx::Process (mt63base.cxx:944-986) for one block of at most MT63_BLOCK samples */
static void rx_block(fmt63_rx *rx, const double *x, int n)
{
    int s1, s2, m;
    rx->block_len = 0;
    m = mt63_split_process(&rx->InpSplit, x, n, rx->SplitOut);
    mt63_delay_process(&rx->ProcLine, rx->SplitOut, m);
    while ((rx->SyncProcPtr + rx->WindowLen) < rx->ProcLine.InpLen) {
        const mt63_cmpx *InpPtr = rx->ProcLine.Line + rx->ProcLine.InpPos;
        sync_process(rx, InpPtr + rx->SyncProcPtr);
        if (rx->SyncPtr == rx->SymbPtr) {
            s1 = rx->SyncProcPtr - rx->ProcDelay +
                 ((int)rx->SyncSymbShift - rx->SymbPtr * rx->SyncStep);
            s2 = s1 + MT63_SYMBOL_SEPAR / 2;
            data_process(rx, InpPtr + s1, InpPtr + s2, rx->SyncFreqOfs, s1 - rx->DataProcPtr);
            rx->DataProcPtr = s1;
        }
        rx->SyncProcPtr += rx->SyncStep;
    }
    rx->SyncProcPtr -= rx->ProcLine.InpLen;
    rx->DataProcPtr -= rx->ProcLine.InpLen;
}

/* The 8-bit escape of mt63::rx_process / rx_flush (mt63.cxx:205-222, 266-284) */
static size_t emit(fmt63_rx *rx, unsigned char *text, size_t max_text, size_t n)
{
    int i;
    for (i = 0; i < rx->block_len; i++) {
        unsigned int c = rx->block_out[i];
        if (!rx->cfg.no_8bit) {
            if ((c < 8) && (rx->escape == 0))
                continue;
            if (c == 127) {
                rx->escape = 1;
                continue;
            }
            if (rx->escape) {
                c += 128;
                rx->escape = 0;
            }
        }
        if (n < max_text)
            text[n++] = (unsigned char)c;
        else
            rx->dropped++;
    }
    return n;
}

/* ---------------------------------------------------------------- public API */

static void rx_free(fmt63_rx *rx)
{
    mt63_split_free(&rx->InpSplit);
    mt63_delay_free(&rx->ProcLine);
    mt63_fft_free(&rx->FFT);
    free(rx->SplitOut);
    free(rx->Decoder.IntlvPipe);
    free(rx);
}

/* mt63::mt63 + mt63::restart + MT63rx::Preset (mt63.cxx:297-320, 347-389,
 * mt63base.cxx:743-942): all memory is allocated here. */
fmt63_rx *fmt63_rx_create(const fmt63_rx_config *cfg)
{
    fmt63_rx *rx;
    const fmt63_submode *sub;
    int integ;
    if (!cfg || !(sub = mt63__submode(cfg->submode)) ||
        !mt63__carrier_ok(sub->bandwidth, cfg->carrier_hz))
        return NULL;
    rx = (fmt63_rx *)calloc(1, sizeof *rx);
    if (!rx)
        return NULL;
    rx->sub = sub;
    rx->cfg = *cfg;
    rx->cfg.submode = sub->name;
    rx->carrier = cfg->carrier_hz;
    integ = cfg->long_integration ? 32 : 16;
    rx->SplitOut = (mt63_cmpx *)malloc((MT63_BLOCK / 2 + 2) * sizeof(mt63_cmpx));
    rx->Decoder.IntlvPipe = (double *)malloc(
        (size_t)(64 + 1) * (MT63_CARRIERS + 2 * MT63_SCAN_MARGIN) * sizeof(double));
    if (!rx->SplitOut || !rx->Decoder.IntlvPipe ||
        mt63_fft_preset(&rx->FFT, MT63_SYMBOL_LEN) ||
        mt63_split_preset(&rx->InpSplit, sub->alias_len, sub->decimate, MT63_BLOCK) ||
        mt63_delay_preset(&rx->ProcLine,
                          integ * MT63_SYMBOL_SEPAR + MT63_SYMBOL_LEN + MT63_SYMBOL_SEPAR)) {
        rx_free(rx);
        return NULL;
    }
    rx_preset(rx);
    return rx;
}

void fmt63_rx_destroy(fmt63_rx *rx)
{
    if (rx)
        rx_free(rx);
}

void fmt63_rx_reset(fmt63_rx *rx)
{
    if (!rx)
        return;
    rx_preset(rx);
    rx->ncodes = 0;
    rx->dropped = 0;
}

/* mt63::rx_process (mt63.cxx:167-242) in fldigi's 512-sample blocks */
size_t fmt63_rx_process(fmt63_rx *rx, const double *x, size_t n, unsigned char *text,
                        size_t max_text)
{
    size_t out = 0, pos = 0;
    if (!rx || !x)
        return 0;
    if (!text)
        max_text = 0;
    while (pos < n) {
        int m = (n - pos > MT63_BLOCK) ? MT63_BLOCK : (int)(n - pos);
        rx_block(rx, x + pos, m);
        pos += (size_t)m;
        rx->snr = rx->Decoder.SignalToNoise;
        if (rx->cfg.squelch > 0.0 && rx->snr < rx->cfg.squelch)
            continue;                    /* mt63.cxx:198-203: the block prints nothing */
        out = emit(rx, text, max_text, out);
    }
    return out;
}

/* mt63::rx_flush (mt63.cxx:244-295) */
size_t fmt63_rx_flush(fmt63_rx *rx, size_t max_blocks, unsigned char *text, size_t max_text)
{
    static const double zeros[MT63_BLOCK];
    size_t out = 0, blocks = 0;
    if (!rx)
        return 0;
    if (!text)
        max_text = 0;
    rx_block(rx, zeros, MT63_BLOCK);
    while (rx->SyncLocked && blocks < max_blocks) {
        out = emit(rx, text, max_text, out);
        rx_block(rx, zeros, MT63_BLOCK);
        blocks++;
    }
    rx->snr = rx->Decoder.SignalToNoise;
    return out;
}

double fmt63_rx_metric(const fmt63_rx *rx)
{
    double snr = rx ? rx->snr : 0.0;
    return snr > 99.9 ? 99.9 : snr;              /* mt63.cxx:228 */
}

/* mt63.cxx:232 takes 10 log10(snr), 0.001 for 0; the integrator can undershoot to a tiny
 * negative s/n in silence, where fldigi's display shows NaN: here it is 0.001 as well. */
double fmt63_rx_snr_db(const fmt63_rx *rx)
{
    double snr = fmt63_rx_metric(rx);
    return 10.0 * log10(snr <= 0 ? 0.001 : snr);
}

/* MT63rx::TotalFreqOffset (mt63base.cxx:1477-1480) */
double fmt63_rx_offset_hz(const fmt63_rx *rx)
{
    if (!rx)
        return 0.0;
    return (rx->SyncFreqOfs + MT63_CARR_SEPAR * rx->Decoder.CarrOfs) *
           (8000.0 / rx->DecimateRatio) / rx->WindowLen;
}

/* MT63rx::SYNC_LockStatus (mt63base.cxx:1449-1451) */
int fmt63_rx_locked(const fmt63_rx *rx)
{
    return rx ? rx->SyncLocked : 0;
}

/* mt63::set_freq -> rx_init (mt63.cxx:48-58, 405-416) */
void fmt63_rx_set_carrier(fmt63_rx *rx, double carrier_hz)
{
    if (!rx || !mt63__carrier_ok(rx->sub->bandwidth, carrier_hz))
        return;
    rx->carrier = carrier_hz;
    rx->cfg.carrier_hz = carrier_hz;
    rx_preset(rx);
}

void fmt63_rx_set_squelch(fmt63_rx *rx, double level)
{
    if (rx)
        rx->cfg.squelch = level > 0.0 ? level : 0.0;
}

size_t fmt63_rx_dropped(const fmt63_rx *rx)
{
    return rx ? rx->dropped : 0;
}

size_t fmt63_rx_footprint(const fmt63_rx *rx)
{
    if (!rx)
        return 0;
    return sizeof *rx + (MT63_BLOCK / 2 + 2) * sizeof(mt63_cmpx) +
           (size_t)(64 + 1) * (MT63_CARRIERS + 2 * MT63_SCAN_MARGIN) * sizeof(double) +
           (size_t)MT63_SYMBOL_LEN * (sizeof(int) + sizeof(mt63_cmpx)) +
           (size_t)(2 * rx->InpSplit.Len + rx->InpSplit.TapCap) * sizeof(double) +
           (size_t)rx->ProcLine.LineSize * sizeof(mt63_cmpx);
}

size_t fmt63_rx_take_codes(fmt63_rx *rx, unsigned char *out, size_t max)
{
    size_t n, first, i;
    if (!rx || !out)
        return 0;
    n = rx->ncodes < CODE_TAP ? rx->ncodes : CODE_TAP;
    if (n > max)
        n = max;
    first = rx->ncodes - n;
    for (i = 0; i < n; i++)
        out[i] = rx->codes[(first + i) % CODE_TAP];
    rx->ncodes = 0;
    return n;
}
