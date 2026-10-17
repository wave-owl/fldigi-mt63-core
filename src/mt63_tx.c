/*
 * fldigi-mt63-core -- the MT63 transmitter: MT63tx and MT63encoder of fldigi 4.2.13
 * src/mt63/mt63base.cxx and mt63::tx_process (src/mt63/mt63.cxx), one call per step.
 * Every function names the fldigi lines it ports.
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

#define TONE_AMP 0.8           /* mt63.h:34 */
#define TONE_BLOCK 512         /* mt63.cxx:80 */
/* The state of the C library rand() when fldigi's tx_init presets the transmitter in the
 * bench (macOS libc, state 1 at process start; the constructor-time preset of
 * mt63::restart drew 64 x interleave numbers before). Derived in tests/test_mt63_fldigi.c
 * from fldigi's own TX output. */
#define FLDIGI_DRAWS_BEFORE_TX_INIT(intlv) (64 * (intlv))

enum { TX_NEW, TX_DATA, TX_DONE };

struct fmt63_tx {
    const fmt63_submode *sub;
    fmt63_tx_config cfg;
    int state;
    unsigned long rng;
    /* MT63tx (mt63base.h:147-226) */
    int FirstDataCarr, DecimateRatio, DataInterleave;
    double TxAmpl;
    int TxVect[MT63_CARRIERS], PhaseCorr[MT63_CARRIERS];
    mt63_cmpx WindowBuff[2 * MT63_SYMBOL_LEN];
    mt63_fft FFT;
    mt63_overlap Window;
    mt63_cmpx WinOut[MT63_SYMBOL_SEPAR];
    mt63_comb Comb;
    double *CombOut;          /* Comb.Output: SymbolSepar x DecimateRatio samples */
    int CombLen;
    /* MT63encoder (mt63base.h:60-77) */
    int IntlvLen, IntlvSize, IntlvPtr;
    int IntlvPatt[MT63_CARRIERS];
    unsigned char *IntlvPipe;
    double WalshBuff[MT63_CARRIERS];
    unsigned char EncOut[MT63_CARRIERS];
    /* mt63 (mt63.h:36-51) */
    double maxval;
};

/* ---------------------------------------------------------------- MT63encoder */

/* MT63encoder::Preset with PreFill (mt63base.cxx:369-401) */
static void encoder_preset(fmt63_tx *tx, const int *Pattern)
{
    int i, p;
    tx->IntlvLen = tx->DataInterleave;
    tx->IntlvSize = tx->IntlvLen * MT63_CARRIERS;
    for (i = 0; i < tx->IntlvSize; i++)
        tx->IntlvPipe[i] = (unsigned char)(mt63_rand(&tx->rng) & 1);
    tx->IntlvPtr = 0;
    for (p = 0, i = 0; i < MT63_CARRIERS; i++) {
        tx->IntlvPatt[i] = p * MT63_CARRIERS;
        p += Pattern[i];
        if (p >= tx->IntlvLen)
            p -= tx->IntlvLen;
    }
}

/* MT63encoder::Process (mt63base.cxx:403-432): Walsh code of the 7-bit character, through
 * the interleaver; Output[i] = 0 means a phase flip on carrier i. */
static void encoder_process(fmt63_tx *tx, int code)
{
    int i, k;
    code &= 2 * MT63_CARRIERS - 1;                 /* CodeMask */
    for (i = 0; i < MT63_CARRIERS; i++)
        tx->WalshBuff[i] = 0;
    if (code < MT63_CARRIERS)
        tx->WalshBuff[code] = 1.0;
    else
        tx->WalshBuff[code - MT63_CARRIERS] = (-1.0);
    mt63_walsh_inv(tx->WalshBuff, MT63_CARRIERS);
    for (i = 0; i < MT63_CARRIERS; i++)
        tx->IntlvPipe[tx->IntlvPtr + i] = (tx->WalshBuff[i] < 0.0);
    for (i = 0; i < MT63_CARRIERS; i++) {
        k = tx->IntlvPtr + tx->IntlvPatt[i];
        if (k >= tx->IntlvSize)
            k -= tx->IntlvSize;
        tx->EncOut[i] = tx->IntlvPipe[k + i];
    }
    tx->IntlvPtr += MT63_CARRIERS;
    if (tx->IntlvPtr >= tx->IntlvSize)
        tx->IntlvPtr -= tx->IntlvSize;
}

/* ---------------------------------------------------------------- MT63tx */

/* MT63tx::Preset (mt63base.cxx:87-186): carriers, interleaver, FFT, window, combiner with
 * the Blackman3 band pass around the signal, initial phases and phase corrections. */
static void tx_preset(fmt63_tx *tx)
{
    int i, p, step, incr, mask;
    int BandWidth = tx->sub->bandwidth;
    double freq = tx->cfg.carrier_hz;
    double hbw = 1.5 * BandWidth / 2;
    double omega_low = (freq - hbw);
    double omega_high = (freq + hbw);
    if (omega_low < 100)
        omega_low = 100;
    if (omega_high > 4000)
        omega_high = 4000;
    omega_low *= (M_PI / 4000);
    omega_high *= (M_PI / 4000);

    mask = tx->FFT.Size - 1;
    tx->FirstDataCarr = mt63__first_carrier(BandWidth, freq);
    tx->DecimateRatio = tx->sub->decimate;
    tx->TxAmpl = 4.0 / MT63_CARRIERS;
    tx->DataInterleave = tx->sub->interleave;
    memset(tx->WindowBuff, 0, sizeof tx->WindowBuff);
    encoder_preset(tx, tx->DataInterleave == 64 ? mt63_long_intlv_patt
                                                : mt63_short_intlv_patt);
    memset(tx->Window.Buff, 0, (size_t)tx->Window.Len * sizeof(mt63_cmpx));
    memset(tx->Comb.Tap, 0, (size_t)tx->Comb.Len * sizeof(double));
    tx->Comb.TapPtr = 0;
    mt63_comb_compute_shape(&tx->Comb, omega_low, omega_high);

    for (step = 0, incr = 1, p = 0, i = 0; i < MT63_CARRIERS; i++) {
        tx->TxVect[i] = p;
        step += incr;
        p = (p + step) & mask;
    }
    incr = (MT63_SYMBOL_SEPAR * MT63_CARR_SEPAR) & mask;
    for (p = (MT63_SYMBOL_SEPAR * tx->FirstDataCarr) & mask, i = 0; i < MT63_CARRIERS; i++) {
        tx->PhaseCorr[i] = p;
        p = (p + incr) & mask;
    }
}

/* MT63tx::ProcessTxVect (mt63base.cxx:301-329): carriers into the two FFT halves (even
 * carriers in the first, odd in the second, half a symbol apart), inverse FFT, overlapping
 * window, combiner to real 8 kHz audio in Comb.Output. */
static void process_tx_vect(fmt63_tx *tx)
{
    int i, c, r, mask;
    const int WindowLen = MT63_SYMBOL_LEN;
    mask = tx->FFT.Size - 1;
    for (i = 0; i < 2 * WindowLen; i++)
        tx->WindowBuff[i].im = tx->WindowBuff[i].re = 0.0;
    for (i = 0, c = tx->FirstDataCarr; i < MT63_CARRIERS; i++, c += MT63_CARR_SEPAR) {
        r = tx->FFT.BitRevIdx[c & mask] + WindowLen * (i & 1);
        tx->WindowBuff[r].re = tx->TxAmpl * tx->FFT.Twiddle[tx->TxVect[i]].re;
        tx->WindowBuff[r].im = (-tx->TxAmpl) * tx->FFT.Twiddle[tx->TxVect[i]].im;
    }
    mt63_fft_core(&tx->FFT, tx->WindowBuff);
    mt63_fft_core(&tx->FFT, tx->WindowBuff + WindowLen);
    for (i = 0; i < 2 * WindowLen; i++)
        tx->WindowBuff[i].im *= (-1.0);
    mt63_overlap_process(&tx->Window, tx->WindowBuff, 2 * WindowLen, tx->WinOut);
    tx->CombLen = mt63_comb_process(&tx->Comb, tx->WinOut, MT63_SYMBOL_SEPAR, tx->CombOut);
}

/* MT63tx::SendChar (mt63base.cxx:249-275): data bit 1 = phase correction only, 0 = flip. */
static void send_char(fmt63_tx *tx, int ch)
{
    int i, mask, flip;
    encoder_process(tx, ch);
    mask = tx->FFT.Size - 1;
    flip = tx->FFT.Size / 2;
    for (i = 0; i < MT63_CARRIERS; i++) {
        if (tx->EncOut[i])
            tx->TxVect[i] = (tx->TxVect[i] + tx->PhaseCorr[i]) & mask;
        else
            tx->TxVect[i] = (tx->TxVect[i] + tx->PhaseCorr[i] + flip) & mask;
    }
    process_tx_vect(tx);
}

/* MT63tx::SendJam (mt63base.cxx:277-295): every carrier turns +-90 degrees at random. */
static void send_jam(fmt63_tx *tx)
{
    int i, mask, left, right, j;
    mask = tx->FFT.Size - 1;
    left = tx->FFT.Size / 4;
    right = 3 * (tx->FFT.Size / 4);
    for (i = 0; i < MT63_CARRIERS; i++) {
        j = i & mask;
        if (mt63_rand(&tx->rng) & 0x100)
            tx->TxVect[j] = (tx->TxVect[j] + tx->PhaseCorr[j] + left) & mask;
        else
            tx->TxVect[j] = (tx->TxVect[j] + tx->PhaseCorr[j] + right) & mask;
    }
    process_tx_vect(tx);
}

/* ---------------------------------------------------------------- mt63::tx_process */

static void tx_free(fmt63_tx *tx)
{
    mt63_fft_free(&tx->FFT);
    mt63_overlap_free(&tx->Window);
    mt63_comb_free(&tx->Comb);
    free(tx->CombOut);
    free(tx->IntlvPipe);
    free(tx);
}

/* mt63::mt63 + mt63::tx_init (mt63.cxx:38-46, 347-389); all memory is allocated here. */
fmt63_tx *fmt63_tx_create(const fmt63_tx_config *cfg)
{
    fmt63_tx *tx;
    const fmt63_submode *sub;
    if (!cfg || !(sub = mt63__submode(cfg->submode)) ||
        !mt63__carrier_ok(sub->bandwidth, cfg->carrier_hz) || cfg->tune_s < 0 ||
        cfg->tune_s > 60)
        return NULL;
    tx = (fmt63_tx *)calloc(1, sizeof *tx);
    if (!tx)
        return NULL;
    tx->sub = sub;
    tx->cfg = *cfg;
    tx->cfg.submode = sub->name;
    if (!tx->cfg.tune_s)
        tx->cfg.tune_s = 4;                       /* mt63_tone_duration (configuration.h:900) */
    if (cfg->seed) {
        tx->rng = cfg->seed;
    } else {
        int k;
        tx->rng = 1;
        for (k = 0; k < FLDIGI_DRAWS_BEFORE_TX_INIT(sub->interleave); k++)
            mt63_rand(&tx->rng);
    }
    tx->CombOut = (double *)malloc((size_t)MT63_SYMBOL_SEPAR * (size_t)sub->decimate *
                                   sizeof(double));
    tx->IntlvPipe = (unsigned char *)malloc((size_t)sub->interleave * MT63_CARRIERS);
    if (!tx->CombOut || !tx->IntlvPipe || mt63_fft_preset(&tx->FFT, MT63_SYMBOL_LEN) ||
        mt63_overlap_preset(&tx->Window, MT63_SYMBOL_LEN, MT63_SYMBOL_SEPAR / 2,
                            mt63_symbol_shape) ||
        mt63_comb_preset(&tx->Comb, sub->alias_len, sub->decimate)) {
        tx_free(tx);
        return NULL;
    }
    tx_preset(tx);
    tx->state = TX_NEW;
    return tx;
}

void fmt63_tx_destroy(fmt63_tx *tx)
{
    if (tx)
        tx_free(tx);
}

size_t fmt63_tx_bound(const fmt63_tx *tx)
{
    if (!tx)
        return 0;
    /* end: interleave - 1 flush symbols and the jam symbol */
    return (size_t)tx->sub->interleave * (size_t)tx->sub->symbol_samples;
}

/* numsmpls of mt63::tx_process (mt63.cxx:80): integer arithmetic as fldigi */
static int tone_blocks(const fmt63_tx *tx)
{
    return FMT63_RATE * tx->cfg.tune_s / TONE_BLOCK;
}

size_t fmt63_tx_begin_len(const fmt63_tx *tx)
{
    if (!tx || tx->cfg.no_tune)
        return 0;
    return (size_t)tone_blocks(tx) * TONE_BLOCK;
}

/* The startflag part of mt63::tx_process (mt63.cxx:71-99): start tone(s) at the lowest
 * and (twotones) the highest carrier + 1 spacing, with fldigi's ramp on the first block
 * and its ramp test on block "duration - 1"; then DataInterleave silent symbols. */
size_t fmt63_tx_begin(fmt63_tx *tx, double *out, size_t out_max)
{
    size_t n = 0;
    int i, j;
    if (!tx || tx->state != TX_NEW || out_max < fmt63_tx_begin_len(tx) ||
        (fmt63_tx_begin_len(tx) && !out))
        return 0;
    tx->maxval = 0.0;
    if (!tx->cfg.no_tune) {
        const int samplerate = FMT63_RATE;
        const double bandwidth = tx->sub->bandwidth;
        const double f = tx->cfg.carrier_hz;
        const int twotones = !tx->cfg.one_tone;
        double w1 = 2.0 * M_PI * (f - bandwidth / 2.0) / samplerate;
        double w2 = 2.0 * M_PI * (f + 31.0 * bandwidth / 64.0) / samplerate;
        double phi1 = 0.0;
        double phi2 = 0.0;
        int numsmpls = tone_blocks(tx);
        for (i = 0; i < numsmpls; i++) {
            double *buff = out + n;
            for (j = 0; j < TONE_BLOCK; j++) {
                buff[j] = TONE_AMP * (twotones ? 0.5 : 1.0) * cos(phi1) +
                          TONE_AMP * (twotones ? 0.5 : 0.0) * cos(phi2);
                phi1 += w1;
                phi2 += w2;
                if (i == 0)
                    buff[j] *= (1.0 - exp(-1.0 * j / 40.0));
                if (i == tx->cfg.tune_s - 1)
                    buff[j] *= (1.0 - exp(-1.0 * (samplerate - j) / 40.0));
            }
            n += TONE_BLOCK;
        }
    }
    for (i = 0; i < tx->DataInterleave; i++)
        send_char(tx, 0);
    tx->state = TX_DATA;
    return n;
}

/* Track the running peak and normalise as "buff[i] = Data[i] * 1.0 / maxval"
 * (mt63.cxx:122-129, 141-147). */
static size_t out_div(fmt63_tx *tx, double *out)
{
    int i;
    for (i = 0; i < tx->CombLen; i++)
        if (fabs(tx->CombOut[i]) > tx->maxval)
            tx->maxval = fabs(tx->CombOut[i]);
    for (i = 0; i < tx->CombLen; i++)
        out[i] = tx->CombOut[i] * 1.0 / tx->maxval;
    return (size_t)tx->CombLen;
}

/* One data symbol (mt63.cxx:150-160): "buff[i] *= 1.0 / maxval". */
static size_t out_data(fmt63_tx *tx, double *buff)
{
    int i, len = tx->CombLen;
    for (i = 0; i < len; i++) {
        buff[i] = tx->CombOut[i];
        if (tx->maxval < fabs(buff[i]))
            tx->maxval = fabs(buff[i]);
    }
    for (i = 0; i < len; i++)
        buff[i] *= 1.0 / tx->maxval;
    return (size_t)len;
}

/* The character part of mt63::tx_process (mt63.cxx:133-162) */
size_t fmt63_tx_put(fmt63_tx *tx, unsigned char ch, double *out, size_t out_max)
{
    size_t n = 0;
    int c = ch, sendc;
    if (!tx || !out || tx->state != TX_DATA ||
        out_max < 2 * (size_t)tx->sub->symbol_samples)
        return 0;
    if (c > 255 || (tx->cfg.no_8bit && c > 127))
        c = '.';
    sendc = c;
    if (sendc > 127) {
        sendc &= 127;
        send_char(tx, 127);
        n += out_div(tx, out);
    }
    send_char(tx, sendc);
    n += out_data(tx, out + n);
    return n;
}

/* GET_TX_CHAR_NODATA: c = 0 (mt63.cxx:107) */
size_t fmt63_tx_idle(fmt63_tx *tx, double *out, size_t out_max)
{
    return fmt63_tx_put(tx, 0, out, out_max);
}

/* GET_TX_CHAR_ETX (mt63.cxx:101-131): "while (--flush)" sends DataInterleave - 1 NUL
 * symbols normalised by "Data[i] /= maxval", then the jam symbol normalised by its own
 * peak. */
size_t fmt63_tx_end(fmt63_tx *tx, double *out, size_t out_max)
{
    size_t n = 0;
    int i, flush;
    if (!tx || !out || tx->state != TX_DATA || out_max < fmt63_tx_bound(tx))
        return 0;
    flush = tx->DataInterleave;
    while (--flush) {
        send_char(tx, 0);
        for (i = 0; i < tx->CombLen; i++)
            if (fabs(tx->CombOut[i]) > tx->maxval)
                tx->maxval = fabs(tx->CombOut[i]);
        for (i = 0; i < tx->CombLen; i++)
            out[n + (size_t)i] = tx->CombOut[i] /= tx->maxval;
        n += (size_t)tx->CombLen;
    }
    send_jam(tx);
    tx->maxval = 0.0;
    n += out_div(tx, out + n);
    tx->state = TX_DONE;
    return n;
}
