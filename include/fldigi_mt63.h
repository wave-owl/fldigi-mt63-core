/*
 * fldigi-mt63-core -- the MT63 modem of fldigi 4.2.13 (MT63-500S/L, MT63-1000S/L,
 * MT63-2000S/L), receiver and transmitter, ported to C99.
 *
 * Copyright (C) 2026 Gerolf Ziegenhain
 * Ported from fldigi 4.2.13 (https://github.com/wave-owl/fldigi, tag v4.2.13, commit
 * e7c3ab3709ad62a7d91829364eb48a05fa325c0d): src/mt63/mt63base.cxx, src/mt63/dsp.cxx,
 * src/include/dsp.h, src/include/mt63base.h, src/mt63/mt63.cxx, src/mt63/symbol.dat and
 * src/mt63/mt63intl.dat; MT63 (c) 1999-2004 Pawel Jalocha SP9VRC, fldigi adaptation
 * (c) 2007-2011 Dave Freese W1HKJ.
 *
 * This program is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this
 * program. If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef FLDIGI_MT63_H
#define FLDIGI_MT63_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FMT63_VERSION_STRING "0.1.0"
#define FMT63_ABI 1

/* "fldigi-mt63-core 0.1.0 contract=mt63-c1 abi=1 tx=1". Static string. The contract tag
 * changes only with an incompatible API change; tx=1 announces the transmitter. */
const char *fmt63_version(void);
int fmt63_abi(void);                    /* FMT63_ABI */

/* All MT63 audio runs at 8000 Hz (mt63.cxx:386: samplerate = 8000). */
#define FMT63_RATE 8000

/* ------------------------------------------------------------------------------------
 * Submodes: the six MT63 modes of fldigi 4.2.13 (globals/globals.cxx:120-125, parameters
 * mt63/mt63.cxx:347-389, mt63base.cxx:87-186).
 * ------------------------------------------------------------------------------------ */
typedef struct fmt63_submode {
    const char *name;        /* "MT63-500S" ... "MT63-2000L" (fldigi's long name) */
    int fldigi_id;           /* trx_mode id of fldigi 4.2.13 (index into mode_info[]) */
    int bandwidth;           /* nominal 500, 1000 or 2000 Hz */
    int interleave;          /* 32 (S) or 64 (L) symbols */
    int decimate;            /* 8, 4, 2: 8 kHz / decimate is the baseband rate */
    int alias_len;           /* taps of the I/Q splitter/combiner (128, 64, 64) */
    double baud;             /* symbols per second: 5, 10 or 20 */
    double carrier_spacing;  /* Hz between carriers: bandwidth / 64 */
    int symbol_samples;      /* 8 kHz samples per symbol = 1600, 800, 400 */
} fmt63_submode;

int fmt63_submode_count(void);                      /* 6 */
const fmt63_submode *fmt63_submode_at(int index);   /* NULL outside 0 .. count-1 */
/* Case-insensitive lookup by name; also fldigi's short names ("MT63-1KL"). NULL when
 * unknown. */
const fmt63_submode *fmt63_submode_find(const char *name);

/* ------------------------------------------------------------------------------------
 * Receiver: mt63::rx_process (mt63.cxx:167-242) over MT63rx (mt63base.cxx:584-1481) with
 * fldigi's defaults (mt63_rx_integration off = 16 symbols, mt63_8bit on,
 * configuration.h:888-893).
 * ------------------------------------------------------------------------------------ */
typedef struct fmt63_rx_config {
    const char *submode;     /* see fmt63_submode_find */
    double carrier_hz;       /* audio centre of the 64 carriers (fldigi "frequency"), 1500;
                                bandwidth/2 .. 4000 - bandwidth/2 */
    int long_integration;    /* 1: 32 symbols (fldigi mt63_rx_integration); 0: 16 */
    int no_8bit;             /* 1: hand out the raw 7-bit codes (mt63_8bit off); 0: fldigi's
                                default 8-bit escape (codes < 8 dropped, 127 escapes) */
    double squelch;          /* 0: off (fldigi bench); else FEC s/n threshold as fldigi's
                                metric (progStatus.sldrSquelchValue, 0 .. 100) */
} fmt63_rx_config;

typedef struct fmt63_rx fmt63_rx;

/* NULL for an unknown submode, a carrier out of range or out of memory. */
fmt63_rx *fmt63_rx_create(const fmt63_rx_config *cfg);
void fmt63_rx_destroy(fmt63_rx *rx);
/* Back to the state after create (mt63::restart for the receiver). */
void fmt63_rx_reset(fmt63_rx *rx);
/* Feed n samples at 8 kHz (fldigi's sound card scale: int16 / 32768). fldigi's
 * 512-sample sound-card blocks are reproduced inside; the bytes fldigi hands to
 * put_rx_char() go to text and their number is returned. Bytes beyond max_text are
 * dropped and counted (fmt63_rx_dropped). text may be NULL with max_text 0. */
size_t fmt63_rx_process(fmt63_rx *rx, const double *x, size_t n, unsigned char *text,
                        size_t max_text);
/* mt63::rx_flush (mt63.cxx:244-295): feed 512-sample blocks of silence while the
 * synchroniser stays locked (at most max_blocks; fldigi has no bound) and return the bytes
 * that come out; fldigi does this when it switches to transmit. */
size_t fmt63_rx_flush(fmt63_rx *rx, size_t max_blocks, unsigned char *text, size_t max_text);
/* MT63rx::FEC_SNR (fldigi's metric, s/n at the Walsh decoder, capped at 99.9 as the
 * display); fmt63_rx_snr_db is fldigi's status "s/n" (10 log10, mt63.cxx:228-233). */
double fmt63_rx_metric(const fmt63_rx *rx);
double fmt63_rx_snr_db(const fmt63_rx *rx);
/* MT63rx::TotalFreqOffset in Hz (fldigi's "f/o"). */
double fmt63_rx_offset_hz(const fmt63_rx *rx);
int fmt63_rx_locked(const fmt63_rx *rx);           /* SYNC_LockStatus */
void fmt63_rx_set_carrier(fmt63_rx *rx, double carrier_hz);   /* restarts, like fldigi */
void fmt63_rx_set_squelch(fmt63_rx *rx, double level);       /* 0 = off */
size_t fmt63_rx_dropped(const fmt63_rx *rx);
/* Bytes the receiver allocated in fmt63_rx_create (it allocates nothing later). */
size_t fmt63_rx_footprint(const fmt63_rx *rx);

/* Test tap (R-840.8): the decoder's code of every symbol (MT63decoder::Output, before the
 * 8-bit escape) since the last call, oldest first, at most the last 4096. Returns the
 * number copied. */
size_t fmt63_rx_take_codes(fmt63_rx *rx, unsigned char *out, size_t max);

/* ------------------------------------------------------------------------------------
 * Transmitter: mt63::tx_process (mt63.cxx:60-165) over MT63tx (mt63base.cxx:60-336), one
 * call per step. Output: the samples fldigi hands to modem::ModulateXmtr (normalised by
 * the running peak, as mt63::tx_process does), before the TX level (multiply by
 * 10^(txlevel/20), fldigi's -12 dB, and clip at +-0.95 to reproduce fldigi's WAV).
 * ------------------------------------------------------------------------------------ */
typedef struct fmt63_tx_config {
    const char *submode;
    double carrier_hz;       /* audio centre of the 64 carriers (fldigi tx frequency) */
    int no_8bit;             /* 1: bytes > 127 become '.' (mt63_8bit off); 0: escape 127 */
    int no_tune;             /* 1: no start tone (mt63_usetones off); 0: fldigi default on */
    int one_tone;            /* 1: lower tone only (mt63_twotones off); 0: both, default */
    int tune_s;              /* tone duration in seconds; 0: fldigi's default 4 */
    unsigned long seed;      /* state of the C library rand() that fills the interleaver
                                and the jamming phases (MT63encoder::Preset, SendJam);
                                0: fldigi's state on the reference platform (see README) */
} fmt63_tx_config;

typedef struct fmt63_tx fmt63_tx;

fmt63_tx *fmt63_tx_create(const fmt63_tx_config *cfg);
void fmt63_tx_destroy(fmt63_tx *tx);
/* Largest number of samples one put/idle/end call can produce (the escape of an 8-bit
 * byte sends two symbols; end sends interleave symbols). */
size_t fmt63_tx_bound(const fmt63_tx *tx);
/* Samples of fmt63_tx_begin: the start tone (62 blocks of 512 samples for 4 s) or 0. */
size_t fmt63_tx_begin_len(const fmt63_tx *tx);
/* Each call returns the samples written to out, or 0 (nothing done) when out_max is
 * smaller than needed or the call is out of order (begin once, then put/idle, then end). */
/* tx_init + the start of tx_process: the start tone(s), then interleave silent symbols
 * that fill the interleaver (their samples are not sent, as in fldigi). */
size_t fmt63_tx_begin(fmt63_tx *tx, double *out, size_t out_max);
/* One keyboard byte (one symbol; two for a byte > 127 with the 8-bit escape). */
size_t fmt63_tx_put(fmt63_tx *tx, unsigned char c, double *out, size_t out_max);
/* No keyboard data: one NUL symbol (GET_TX_CHAR_NODATA). */
size_t fmt63_tx_idle(fmt63_tx *tx, double *out, size_t out_max);
/* GET_TX_CHAR_ETX: interleave - 1 NUL symbols that flush the interleaver, then one jamming
 * symbol. */
size_t fmt63_tx_end(fmt63_tx *tx, double *out, size_t out_max);

#ifdef __cplusplus
}
#endif

#endif /* FLDIGI_MT63_H */
