# fldigi-mt63-core

[![CI](https://github.com/wave-owl/fldigi-mt63-core/actions/workflows/ci.yml/badge.svg)](https://github.com/wave-owl/fldigi-mt63-core/actions/workflows/ci.yml)

A C99 library that receives and transmits **MT63** exactly the way fldigi 4.2.13 does: audio
in, characters out, and characters in, audio out, for all six fldigi MT63 submodes
(MT63-500S/L, MT63-1000S/L, MT63-2000S/L). It is a line-by-line port of Pawel Jalocha's MT63
modem as fldigi carries it (`src/mt63/mt63base.cxx`, `src/mt63/dsp.cxx`, `src/include/dsp.h`)
and of fldigi's glue around it (`src/mt63/mt63.cxx`: 8-bit escape, start tones, flush,
jamming symbol). On the reference build the transmitter reproduces fldigi's TX audio sample
for sample and the receiver prints fldigi's characters byte for byte (see
[Verification](#verification)). It is the only MT63 implementation of
[wave-owl](https://github.com/wave-owl/wave-owl).

| | |
|---|---|
| **Is** | A single-channel **receiver** for every MT63 submode of fldigi 4.2.13: the time/frequency synchroniser that tracks the signal by itself (no AFC switch, +-8 carriers of offset search in the Walsh decoder), short and long receive integration, the 8-bit escape (or raw 7-bit codes), fldigi's squelch, s/n, frequency offset and `rx_flush`. |
| **Is** | The matching **transmitter** (capability tag `tx=1`): start tone(s), the silent interleaver fill, one OFDM symbol per character (two for an escaped 8-bit character), NUL idles, interleaver flush and the jamming symbol, normalised as `mt63::tx_process` does. |
| **Is not** | A skimmer: each instance decodes one submode at one carrier. No RSID, no ARQ/KISS, no CW ID. |
| | An I/O layer, threaded or allocating at run time: no sound card or file access, no threads, all memory allocated in `*_create()`, no global mutable state. |

MT63 uses Jalocha's own DSP (radix-2 FFT, I/Q splitter and combiner, overlapping window,
selective averaging, Walsh transform), not fldigi's shared filters, so this core vendors it
(`src/mt63_dsp.c`) instead of depending on
[fldigi-dsp-core](https://github.com/wave-owl/fldigi-dsp-core) (wave-owl R-920.4).

## Contents

- [Signal chain: receiver](#signal-chain-receiver)
- [Signal chain: transmitter](#signal-chain-transmitter)
- [Submodes](#submodes)
- [Differences from fldigi](#differences-from-fldigi)
- [API reference](#api-reference)
- [Usage example](#usage-example)
- [Contract and versioning](#contract-and-versioning)
- [Verification](#verification)
- [Performance and memory](#performance-and-memory)
- [Building](#building)
- [Maintenance and fldigi updates](#maintenance-and-fldigi-updates)
- [License and credits](#license-and-credits)

## Signal chain: receiver

All citations refer to fldigi tag `v4.2.13` (mirror `wave-owl/fldigi`, commit
`e7c3ab3709ad62a7d91829364eb48a05fa325c0d`); every C function names the lines it ports.
Settings are fldigi's defaults (`configuration.h:887-908`): integration 16 symbols, 8-bit
on, squelch off.

```
 audio x[n] at 8000 Hz (int16 / 32768 scale), in fldigi's 512-sample blocks
      v
 dspQuadrSplit: complex band pass carrier +- 0.75 bandwidth (Blackman3-windowed
   FIR, 128/64/64 taps), decimation by 8/4/2                          dsp.cxx:817-884
      v
 delay line (integration x 200 + 512 + 200 samples)                   dsp.h:862-923
      v  four times per symbol (every 50 samples)
 SyncProcess: 512-point FFT of the windowed slice, per-bin squared-phase
   correlation with the slice one symbol earlier, two-pole integrators;
   once per symbol carrier-pattern sums, symbol-time and frequency fit,
   selective averaging (dspSelFitAver), lock hysteresis      mt63base.cxx:988-1296
      v  once per symbol, at the synchroniser's time and frequency
 DataProcess: even/odd carrier slices, frequency correction, FFT,
   differential phase per carrier, integration/2 symbol pipe,
   normalised real parts                                    mt63base.cxx:1298-1447
      v
 MT63decoder: de-interleaver (32 or 64 symbols), Walsh transform over 64
   carriers for 17 carrier offsets, best s/n wins              mt63base.cxx:475-579
      v
 8-bit escape: codes < 8 dropped, 127 escapes the next code    mt63.cxx:205-222
      v
 bytes as fldigi's put_rx_char() receives them
```

## Signal chain: transmitter

`fmt63_tx_begin` / `_put` / `_idle` / `_end` are the steps of fldigi's `tx_process`
(`mt63.cxx:60-165`) over `MT63tx` (`mt63base.cxx:60-336`):

| Call | fldigi | What is sent |
|---|---|---|
| `fmt63_tx_begin` | `startflag` block | 62 blocks of 512 samples of start tone (4 s; lowest carrier and, by default, highest carrier + 1 spacing, 0.8 amplitude, ramp on the first block), then `interleave` NUL symbols whose samples are dropped (they fill the interleaver, which `Preset` filled with `rand() & 1`) |
| `fmt63_tx_put(c)` | one character | Walsh code of the 7-bit character through the interleaver, one phase flip per 0 bit on 64 carriers, inverse FFT, overlapping window, I/Q combiner; a byte > 127 is sent as 127 then `c & 127` (or as `.` with `no_8bit`) |
| `fmt63_tx_idle` | `GET_TX_CHAR_NODATA` | one NUL symbol |
| `fmt63_tx_end` | `GET_TX_CHAR_ETX` | `interleave - 1` NUL symbols, then the jamming symbol (every carrier turns +-90 degrees at random) |

Every symbol is divided by the running peak of the over (`maxval`), as fldigi does, so the
output stays within +-1. The output is the signal before fldigi's TX level; multiply by
`10^(-12/20)` and clip at +-0.95 to get fldigi's sound-card samples. The interleaver fill and
the jamming symbol use the C library `rand()` of fldigi's reference build (macOS: Park-Miller
minimal standard); `seed` 0 starts it where fldigi's bench is when `tx_init` runs, so the
output equals fldigi's.

## Submodes

| Name | fldigi id | Bandwidth | Interleave | Decimation | Symbol | Baud | Carrier spacing |
|---|---|---|---|---|---|---|---|
| MT63-500S | 53 | 500 Hz | 32 | 8 | 1600 samples | 5 | 7.8125 Hz |
| MT63-500L | 54 | 500 Hz | 64 | 8 | 1600 samples | 5 | 7.8125 Hz |
| MT63-1000S | 55 | 1000 Hz | 32 | 4 | 800 samples | 10 | 15.625 Hz |
| MT63-1000L | 56 | 1000 Hz | 64 | 4 | 800 samples | 10 | 15.625 Hz |
| MT63-2000S | 57 | 2000 Hz | 32 | 2 | 400 samples | 20 | 31.25 Hz |
| MT63-2000L | 58 | 2000 Hz | 64 | 2 | 400 samples | 20 | 31.25 Hz |

64 carriers, the lowest at `carrier - bandwidth/2` (rounded to the FFT grid,
`mt63base.cxx:101-121`), the highest one spacing below `carrier + bandwidth/2`. Sources:
`mt63.cxx:347-389`, `globals/globals.cxx:120-125`, `symbol.dat:24-26`. The carrier must lie
in `bandwidth/2 .. 4000 - bandwidth/2` Hz.

## Differences from fldigi

fldigi's MT63 receiver has two memory faults that make its output depend on the heap. The port
gives both the defined meaning; with the same two changes applied to fldigi
(`tests/golden/fldigi-defined-ub.patch`) fldigi decodes deterministically and the port
prints exactly what it prints.

- **`FFTbuff[DataScanFirst]`.** `DataProcess` starts at `c = DataScanFirst` without the
  index mask it applies afterwards (`mt63base.cxx:1356`). For MT63-500 at usual carriers
  (`DataScanFirst` = 608 at 1500 Hz) this reads beyond the 512-entry FFT buffer. The port
  masks the index (the FFT is periodic).
- **`dspSeq::EnsureSpace`** (`dsp.h:179-190`) allocates a new buffer without copying the old
  contents. The receiver's character buffer grows the first time one 512-sample block yields
  two characters (MT63-2000, or a timing jump); fldigi then prints heap garbage instead of the
  first of them. The port keeps the character.

Smaller points, each named in the code:

- fldigi reads `CorrelAver[s][i]` without a range check where `i` can leave the 64 bins at the
  scan edge and `s` can round up to 4; the port clamps (never seen on the goldens).
- fldigi divides by zero in `dspSelFitAver` when no sample is inside the threshold; the port
  keeps the IEEE result, as fldigi.
- fldigi's status line takes `10 log10` of a slightly negative s/n in silence (NaN); the
  port reports it as 0.001 (-30 dB).
- fldigi's bench sends the bytes of its macro text with `char`'s sign (`get_tx_char` returns
  a negative value for a byte >= 0x80, which skips the escape); its keyboard path sends code
  points. `fmt63_tx_put` takes the code point (0..255) and escapes it, as fldigi's keyboard
  does.
- Left out: the waterfall display callback, `InpLevel` (display only), the unused test mixer,
  `SendTune`/`SendSilence` (not called by fldigi), `mt63_at500` / `mt63_centered` (the caller
  chooses the carrier).

## API reference

All declarations are in `include/fldigi_mt63.h`.

| Function | Description |
|---|---|
| `fmt63_version()`, `fmt63_abi()` | `"fldigi-mt63-core 0.1.0 contract=mt63-c1 abi=1 tx=1"`, 1 |
| `fmt63_submode_count()`, `fmt63_submode_at(i)`, `fmt63_submode_find(name)` | The 6 submodes (table above); case-insensitive, also fldigi's short names (`MT63-1KL`). |
| `fmt63_rx_create(const fmt63_rx_config *)` | `submode`, `carrier_hz`, `long_integration` (32 instead of 16 symbols), `no_8bit` (raw codes), `squelch` (0 off, else FEC s/n threshold). NULL for an unknown submode or a carrier out of range. |
| `fmt63_rx_process(rx, x, n, text, max)` | Feed 8 kHz samples; returns the bytes fldigi prints (overflow dropped and counted, `fmt63_rx_dropped`). |
| `fmt63_rx_flush(rx, max_blocks, text, max)` | `mt63::rx_flush`: silence blocks while the synchroniser is locked. |
| `fmt63_rx_metric`, `_snr_db`, `_offset_hz`, `_locked` | FEC s/n (fldigi's metric, capped at 99.9), its 10 log10, `TotalFreqOffset` in Hz, lock state. |
| `fmt63_rx_set_carrier`, `_set_squelch`, `_reset`, `_destroy`, `_footprint` | Retune (restarts the receiver, as fldigi), squelch, back to the created state, free, bytes allocated. |
| `fmt63_rx_take_codes` | Test tap: the decoder's code of every symbol since the last call (before the escape). |
| `fmt63_tx_create(const fmt63_tx_config *)` | `submode`, `carrier_hz`, `no_8bit`, `no_tune`, `one_tone`, `tune_s` (0: 4 s), `seed` (0: fldigi's). |
| `fmt63_tx_bound(tx)`, `fmt63_tx_begin_len(tx)` | The most samples one put/idle/end call produces; the samples of begin. |
| `fmt63_tx_begin`, `_put(c)`, `_idle`, `_end` | The steps above; each returns the samples written (0: buffer too small or out of order). |

## Usage example

```c
#include <stdio.h>
#include <stdlib.h>
#include "fldigi_mt63.h"

int main(void)
{
    fmt63_tx_config tc = {"MT63-1000L", 1500.0, 0, 0, 0, 0, 0};
    fmt63_rx_config rc = {"MT63-1000L", 1500.0, 0, 0, 0.0};
    fmt63_tx *tx = fmt63_tx_create(&tc);
    fmt63_rx *rx = fmt63_rx_create(&rc);
    size_t cap = fmt63_tx_bound(tx) > fmt63_tx_begin_len(tx) ? fmt63_tx_bound(tx)
                                                             : fmt63_tx_begin_len(tx);
    double *buf = malloc(cap * sizeof *buf);
    unsigned char out[4096];
    const char *text = "CQ DE DG6FL K";
    size_t n, i;

    n = fmt63_tx_begin(tx, buf, cap);
    fmt63_rx_process(rx, buf, n, out, sizeof out);
    for (i = 0; text[i]; i++) {
        n = fmt63_tx_put(tx, (unsigned char)text[i], buf, cap);
        fwrite(out, 1, fmt63_rx_process(rx, buf, n, out, sizeof out), stdout);
    }
    n = fmt63_tx_end(tx, buf, cap);            /* the flush pushes the text out */
    fwrite(out, 1, fmt63_rx_process(rx, buf, n, out, sizeof out), stdout);
    fwrite(out, 1, fmt63_rx_flush(rx, 100, out, sizeof out), stdout);
    free(buf);
    fmt63_tx_destroy(tx);
    fmt63_rx_destroy(rx);
    return 0;
}
```

wave-owl binds the library with ctypes in `src/mt63_native.py`.

## Contract and versioning

`fmt63_version()` reports `contract=mt63-c1 abi=1` and the capability tag `tx=1`. The
contract changes only with an incompatible change of the structs or functions; additions keep
it and raise the minor version. wave-owl refuses a library with another contract or ABI.

## Verification

`make test` runs two programs:

- `tests/test_mt63.c` (independent of fldigi): submode table and argument checks; TX -> RX
  loopback for all six submodes with short and long integration (lock, metric, offset); the
  8-bit escape round trip, 7-bit receive and 7-bit transmit; chunk invariance of text and
  codes (random chunks of 1 to 4000 samples); two receivers interleaved; reset; a 12 Hz
  carrier offset tracked and reported; flush; retuning; squelch; output overflow; the
  transmitter's call order and bounds.
- `tests/test_mt63_fldigi.c`: goldens from the fldigi 4.2.13 bench build
  (`tests/golden/make_fldigi_golden.py`; headless fldigi with the TX extension of wave-owl's
  archived change `fix-psk-rtty-vs-fldigi`): 24 overs (the six submodes with
  `CQ CQ DE DG6FL DG6FL K` and a pangram; long integration on three submodes, 7-bit
  receive, one start tone, no start tone, TX carrier 7, -23 and 3 Hz off the receiver,
  carriers at 1000, 1250 and 2200 Hz), each received clean and at -5 / -10 dB (2500 Hz;
  deterministic noise rebuilt bit for bit in C).
  - TX: the FNV-1a hash of the int16 output equals fldigi's; stored vectors match.
  - RX: the bytes equal those of fldigi with its two memory faults defined (`rx/`); against
    the unmodified, non-deterministic fldigi (`rx_stock/`, 7 runs each) the port gives one of
    fldigi's outputs or the same number of text errors as fldigi's most frequent output.

| | Result (macOS arm64, Apple clang, `-O3` and the `-O1` sanitizer build) |
|---|---|
| TX overs identical to fldigi | 24 of 24 |
| RX decodes identical to fldigi (memory faults defined) | 72 of 72 |
| RX decodes equal to one of the unmodified fldigi's outputs | 65 of 72 (the other 7: same text errors) |

On other platforms (another libm, x86 without FMA) the results are not bit-identical; there
the clean decodes must stay within 1 + 5 % edit distance of fldigi's and the noisy ones in
total within 20 % + 10 errors. The reference build is reproduced bit for bit because the
library is compiled with `-ffp-contract=on` (fusing the same scalar expressions as fldigi's
Apple clang -O2 build) and keeps fldigi's inline helpers as functions, so nothing is fused
across them. CI runs both programs on Ubuntu and macOS and again under AddressSanitizer and
UndefinedBehaviorSanitizer.

## Performance and memory

`make bench` (Apple M-series, one core), 60 s overs in 512-sample blocks:

| | MT63-500S | MT63-500L | MT63-1000S | MT63-1000L | MT63-2000S | MT63-2000L |
|---|---|---|---|---|---|---|
| Receiver, x real time | 5900 | 6300 | 4300 | 4300 | 2100 | 2100 |
| Receiver, ms CPU per s of audio | 0.17 | 0.16 | 0.23 | 0.23 | 0.47 | 0.47 |
| Transmitter, x real time | 14 000 | 20 000 | 16 000 | 15 000 | 8 700 | 8 500 |

Memory per receiver: 424-426 KiB with long integration (the delay line of 2 x 7112 complex
samples, the synchroniser's four 320-bin correlators and the de-interleaver), allocated in
`fmt63_rx_create`; nothing is allocated while processing.

## Building

```
make                             # build/libfldigi_mt63.{a,dylib|so}
make test                        # unit tests and fldigi goldens
make sanitize                    # the same under ASan + UBSan
make bench                       # CPU benchmark
make BUILD=/abs/dir shared       # out-of-tree build (wave-owl: make build-mt63-core)
```

Plain make, C99, `-Wall -Wextra -Werror -pedantic`, no dependencies besides libm.

## Maintenance and fldigi updates

wave-owl's `scripts/fldigi_upstream_check.py` (skill `fldigi-upstream-sync`) checks every
`file:line` citation of this repository against a new fldigi tag. After a bump: rebuild the
bench, build a second binary with `tests/golden/fldigi-defined-ub.patch`, rerun
`tests/golden/make_fldigi_golden.py` and `make test`.

## License and credits

GPL-3.0-or-later, as fldigi. MT63 by Pawel Jalocha SP9VRC (1999-2004, MT63ASC and the C++
modem), adapted to fldigi by Dave Freese W1HKJ (2007-2011). The C port and the tests:
Gerolf Ziegenhain DG6FL, 2026. See `CHANGELOG.md`.
