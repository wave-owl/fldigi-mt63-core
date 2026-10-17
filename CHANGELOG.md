# Changelog

All notable changes to fldigi-mt63-core. Versions follow semantic versioning; the contract
tag `mt63-c1` changes only with an incompatible API change.

## 0.1.1 - 2026-10-17

- The real-valued two-pole integrator cites the lines of `dsp.h` it ports (722-734) within
  the range wave-owl's frozen Python receiver cited (wave-owl citation gate, AC-840.3). No
  functional change.

## 0.1.0 - 2026-10-17

First release, for wave-owl change `add-fldigi-mt63-core` (umbrella
`add-fldigi-native-modes`).

- Receiver and transmitter for all six MT63 submodes of fldigi 4.2.13 (MT63-500S/L,
  MT63-1000S/L, MT63-2000S/L), ported from Pawel Jalocha's `mt63base.cxx` / `dsp.cxx` and
  fldigi's `mt63.cxx`: I/Q splitter, time/frequency synchroniser, data demodulator, Walsh
  decoder with carrier-offset search, short/long interleave and integration, 8-bit escape,
  squelch, s/n, frequency offset, `rx_flush`; start tones, interleaver fill, characters,
  idles, flush and jamming symbol with fldigi's normalisation and its `rand()` sequence.
  Capability tag `tx=1`.
- Jalocha's MT63 DSP vendored (`src/mt63_dsp.c`), no fldigi-dsp-core dependency.
- fldigi's two memory faults in the MT63 receive path (unmasked `FFTbuff[DataScanFirst]`,
  `dspSeq::EnsureSpace` without copy) given their defined meaning.
- Goldens from the fldigi 4.2.13 bench: 24 TX overs identical; 72 RX decodes identical to
  fldigi with the two faults defined (`tests/golden/fldigi-defined-ub.patch`), also in the
  sanitizer build; unit tests; CPU benchmark (`fmt63_rx_footprint` for the memory).
