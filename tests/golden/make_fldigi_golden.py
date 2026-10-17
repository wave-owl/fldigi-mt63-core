#!/usr/bin/env python3
"""Golden data from fldigi 4.2.13 itself for fldigi-mt63-core (stdlib only).

Drives the headless fldigi bench build (configure --enable-benchmark plus the TX
extension of wave-owl's archived change fix-psk-rtty-vs-fldigi, bench/BUILD.md) for the six
MT63 submodes of fldigi 4.2.13 (src/globals/globals.cxx:120-125):

* TX: fldigi renders the text (bench-tx, txlevel -12 dB, PCM16, 8000 Hz) with its defaults
  (4 s start tones, both tones) or the case's options. The FNV-1a 64 hash of every vector
  goes to cases.txt; a subset is kept as raw little-endian int16 in tx/<stem>.s16.
* RX: fldigi decodes 1 s silence + the TX vector + 12 s silence, clean and buried in
  deterministic noise at -5 and -10 dB (2500 Hz reference bandwidth, see noisy()); the raw
  bytes fldigi hands to put_rx_char() are kept.
  - rx/<stem>_<snr>.txt: fldigi 4.2.13 with its two memory faults in the MT63 receive path
    given defined behaviour (fldigi-defined-ub.patch, $FLDIGI_DEFINED_BIN): DataProcess reads
    FFTbuff[DataScanFirst] without the index mask (outside the 512-entry array for
    MT63-500 at usual carriers, mt63base.cxx:1356) and dspSeq::EnsureSpace (dsp.h:179-190)
    hands out a new buffer without the old contents, which replaces the first character of
    the first receive block that yields two characters with heap garbage. With both defined
    fldigi decodes deterministically; the port must reproduce these bytes exactly.
  - rx_stock/<stem>_<snr>.txt (+ .alt<k>.txt): the unmodified fldigi bench, decoded
    RX_RUNS times (default 7) because the faults make it non-deterministic; the most
    frequent output first, every other output fldigi gave as .alt<k>. A run that writes no
    output at all (it happens now and then under load) is repeated.

Case options (fldigi configuration, configuration.h:887-908): long integration
(MT63INTEGRATION), 7-bit receive (MT638BIT 0), one start tone (MT63TWOTONES 0), no start
tone (MT63USETONES 0); the TX and RX carriers can differ (offset decoding).

    FLDIGI_BENCH_BIN=/path/to/src-bench/src/fldigi \
    FLDIGI_DEFINED_BIN=/path/to/fldigi-with-fldigi-defined-ub.patch \
        python3 tests/golden/make_fldigi_golden.py

The C tests (tests/test_mt63_fldigi.c) rebuild the same noisy inputs bit for bit.
SPDX-License-Identifier: GPL-3.0-or-later
"""

from __future__ import annotations

import array
import math
import os
import shutil
import subprocess
import sys
import tempfile
import wave
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(os.environ.get("FLDIGI_GOLDEN_DIR", Path(__file__).resolve().parent))
FLDIGI = os.environ.get("FLDIGI_BENCH_BIN", "fldigi")
FLDIGI_DEFINED = os.environ.get("FLDIGI_DEFINED_BIN", "")
TEXTS = {"cq": "CQ CQ DE DG6FL DG6FL K",
         "pangram": "the quick brown fox jumps over the lazy dog 0123456789 <test?> 73"}
TAIL_S = 12
SNRS = ("clean", "m05", "m10")
RX_RUNS = int(os.environ.get("RX_RUNS", "7"))
#: fldigi trx_mode ids (index into mode_info[])
IDS = {"MT63-500S": 53, "MT63-500L": 54, "MT63-1000S": 55, "MT63-1000L": 56,
       "MT63-2000S": 57, "MT63-2000L": 58}
OPTION_XML = {"long": "<MT63INTEGRATION>1</MT63INTEGRATION>",
              "7bit": "<MT638BIT>0</MT638BIT>",
              "onetone": "<MT63TWOTONES>0</MT63TWOTONES>",
              "notone": "<MT63USETONES>0</MT63USETONES>"}
STORED = {"mt63_500s_cq", "mt63_1000l_cq", "mt63_2000s_pangram"}


def cases():
    """(stem, submode, text key, tx Hz, rx Hz, options)"""
    out = []
    for sub in IDS:
        stem = sub.lower().replace("-", "_")
        out.append((f"{stem}_cq", sub, "cq", 1500, 1500, ()))
        out.append((f"{stem}_pangram", sub, "pangram", 1500, 1500, ()))
    out += [
        ("mt63_1000l_long", "MT63-1000L", "cq", 1500, 1500, ("long",)),
        ("mt63_2000s_long", "MT63-2000S", "pangram", 1500, 1500, ("long",)),
        ("mt63_500l_long", "MT63-500L", "cq", 1500, 1500, ("long",)),
        ("mt63_1000s_7bit", "MT63-1000S", "pangram", 1500, 1500, ("7bit",)),
        ("mt63_1000l_onetone", "MT63-1000L", "cq", 1500, 1500, ("onetone",)),
        ("mt63_2000l_notone", "MT63-2000L", "cq", 1500, 1500, ("notone",)),
        ("mt63_1000s_off7", "MT63-1000S", "cq", 1507, 1500, ()),
        ("mt63_2000l_offm23", "MT63-2000L", "cq", 1477, 1500, ()),
        ("mt63_500s_off3", "MT63-500S", "cq", 1503, 1500, ()),
        ("mt63_1000l_at1000", "MT63-1000L", "pangram", 1000, 1000, ()),
        ("mt63_500l_at2200", "MT63-500L", "cq", 2200, 2200, ()),
        ("mt63_2000s_at1250", "MT63-2000S", "cq", 1250, 1250, ()),
    ]
    return out


def run_fldigi(sub, freq, args, options=(), env_extra=None, binary=None):
    cfg = tempfile.mkdtemp(prefix="fldigi-mt63-golden-")
    xml = "".join(OPTION_XML[o] for o in options)
    Path(cfg, "fldigi_def.xml").write_text(f"<FLDIGI_DEFS>\n{xml}\n</FLDIGI_DEFS>\n")
    env = {k: v for k, v in os.environ.items() if not k.startswith("FLDIGI_BENCH_TX")}
    env.update(env_extra or {})
    try:
        p = subprocess.run([binary or FLDIGI, "--config-dir", cfg, "--benchmark-modem", str(IDS[sub]),
                            "--benchmark-frequency", str(freq), *args],
                           env=env, capture_output=True, text=True, timeout=3600)
    finally:
        shutil.rmtree(cfg, ignore_errors=True)
    if "mode=MT63" not in p.stderr:
        raise RuntimeError(f"fldigi did not run {sub}: {p.stderr[-400:]}")
    return p.stderr


def read_wav(path):
    with wave.open(str(path)) as w:
        assert w.getnchannels() == 1 and w.getsampwidth() == 2
        sr = w.getframerate()
        a = array.array("h", w.readframes(w.getnframes()))
    if sys.byteorder == "big":
        a.byteswap()
    return sr, a


def write_wav(path, sr, pcm):
    a = array.array("h", pcm)
    if sys.byteorder == "big":
        a.byteswap()
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(a.tobytes())


def fnv1a64(pcm):
    h = 0xCBF29CE484222325
    for v in pcm:
        u = v & 0xFFFF
        for byte in (u & 0xFF, u >> 8):
            h ^= byte
            h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


class Lcg:
    """32-bit LCG (Numerical Recipes); the same in tests/test_mt63_fldigi.c."""

    def __init__(self, seed):
        self.s = seed & 0xFFFFFFFF

    def uniform(self):
        self.s = (self.s * 1664525 + 1013904223) & 0xFFFFFFFF
        return self.s / 4294967296.0


def noisy(pcm, sr, snr, seed, tail):
    """1 s + pcm + tail s, plus Irwin-Hall Gaussian noise at -5 or -10 dB SNR in 2500 Hz.

    Only +, -, *, / and sqrt (all correctly rounded), so C reproduces it bit for bit.
    SNR = mean square of the burst / (noise variance * 2500 / (sr / 2)).
    """
    lead = sr
    padded = [0.0] * lead + [float(v) for v in pcm] + [0.0] * (tail * sr)
    if snr == "clean":
        return [int(v) for v in padded]
    ps = 0.0
    for v in pcm:
        ps += float(v) * float(v)
    ps = ps / len(pcm)
    inv = 1.0 / math.sqrt(10.0) if snr == "m05" else 1.0 / 10.0
    s2 = ps / inv * (sr / 2.0) / 2500.0
    sd = math.sqrt(s2)
    gain = 3276.7 / math.sqrt(ps * len(pcm) / len(padded) + s2)
    rng = Lcg(seed)
    out = []
    for v in padded:
        z = 0.0
        for _ in range(12):
            z += rng.uniform()
        z -= 6.0
        y = (v + sd * z) * gain
        q = math.floor(y + 0.5)
        out.append(max(-32768, min(32767, int(q))))
    return out


def decode_once(td, inp, sub, rxf, options, tag, binary):
    """One bench decode; a run without output is repeated (at most 20 times)."""
    for attempt in range(20):
        out = Path(td, f"rx_{attempt}_{os.getpid()}.txt")
        run_fldigi(sub, rxf, ["--benchmark-input", str(inp), "--benchmark-output", str(out)],
                   options, binary=binary)
        text = out.read_bytes() if out.exists() else b""
        out.unlink(missing_ok=True)
        if text:
            return text
    raise RuntimeError(f"{tag}: fldigi wrote no output")


def decode_defined(td, inp, sub, rxf, options, tag):
    a = decode_once(td, inp, sub, rxf, options, tag, FLDIGI_DEFINED)
    b = decode_once(td, inp, sub, rxf, options, tag, FLDIGI_DEFINED)
    if a != b:
        raise RuntimeError(f"{tag}: the defined-UB fldigi is not deterministic")
    return a


def decode_stock(td, inp, sub, rxf, options, tag):
    seen = {}
    for _ in range(RX_RUNS):
        text = decode_once(td, inp, sub, rxf, options, tag, None)
        seen[text] = seen.get(text, 0) + 1
    return sorted(seen, key=lambda t: -seen[t])


def one(idx, case):
    stem, sub, key, txf, rxf, options = case
    with tempfile.TemporaryDirectory() as td:
        wav = Path(td, stem + ".wav")
        run_fldigi(sub, txf, ["--benchmark-input", "1"], options,
                   {"FLDIGI_BENCH_TX_TEXT": TEXTS[key], "FLDIGI_BENCH_TX_WAV": str(wav)})
        sr, pcm = read_wav(wav)
        opts = ",".join(options) or "-"
        line = (f"{stem} {sub} {key} {idx} {txf} {rxf} {opts} {len(pcm)} {TAIL_S} "
                f"{fnv1a64(pcm):016x}")
        if stem in STORED:
            raw = array.array("h", pcm)
            if sys.byteorder == "big":
                raw.byteswap()
            Path(HERE, "tx", stem + ".s16").write_bytes(raw.tobytes())
        for i, snr in enumerate(SNRS):
            inp = Path(td, f"{stem}_{snr}.wav")
            write_wav(inp, sr, noisy(pcm, sr, snr, 1000 * idx + i, TAIL_S))
            Path(HERE, "rx", f"{stem}_{snr}.txt").write_bytes(
                decode_defined(td, inp, sub, rxf, options, f"{stem} {snr}"))
            ranked = decode_stock(td, inp, sub, rxf, options, f"{stem} {snr}")
            for old in Path(HERE, "rx_stock").glob(f"{stem}_{snr}*.txt"):
                old.unlink()
            Path(HERE, "rx_stock", f"{stem}_{snr}.txt").write_bytes(ranked[0])
            for k, text in enumerate(ranked[1:], 1):
                Path(HERE, "rx_stock", f"{stem}_{snr}.alt{k}.txt").write_bytes(text)
    print(line, flush=True)
    return line


def main():
    (HERE / "tx").mkdir(exist_ok=True)
    (HERE / "rx").mkdir(exist_ok=True)
    (HERE / "rx_stock").mkdir(exist_ok=True)
    if not FLDIGI_DEFINED:
        sys.exit("set FLDIGI_DEFINED_BIN (fldigi built with tests/golden/fldigi-defined-ub.patch)")
    only = set(sys.argv[1:])
    todo = [(i, c) for i, c in enumerate(cases()) if not only or c[0] in only]
    with ThreadPoolExecutor(max_workers=int(os.environ.get("JOBS", "8"))) as ex:
        lines = list(ex.map(lambda a: one(*a), todo))
    if only:
        return 0
    header = ("# stem submode text index tx-Hz rx-Hz options samples tail-s fnv1a64(int16 LE)"
              " -- fldigi 4.2.13 bench TX, txlevel -12 dB; noise seed = 1000 * index + snr"
              " step\n")
    (HERE / "cases.txt").write_text(header + "\n".join(lines) + "\n")
    (HERE / "texts.txt").write_text("".join(f"{k} {v}\n" for k, v in TEXTS.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
