/*
 * fldigi-mt63-core against fldigi 4.2.13 itself (goldens from
 * tests/golden/make_fldigi_golden.py, the headless fldigi bench build).
 *
 * TX, all six MT63 submodes (two texts each) and the option cases (one start tone, no start
 * tone, other carriers): the port renders fldigi's TX vector (FNV-1a hash of the int16
 * output identical on the reference platform, at most 1 LSB from the stored vectors
 * everywhere).
 * RX, the same overs clean and at -5 / -10 dB (2500 Hz), plus long integration, 7-bit
 * receive and carrier offsets: the decoded bytes equal fldigi's own decode on the reference
 * platform; elsewhere (different libm / FMA use) the sent text must be recovered with at
 * most 1 + 5 % more errors than fldigi's own decode on clean overs, and the noisy decodes
 * in total within 20 % + 10 errors of fldigi's.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#define _POSIX_C_SOURCE 200809L

#include "test_util.h"

static const char *golden = "tests/golden";

static uint64_t fnv1a64(const int16_t *pcm, size_t n)
{
    uint64_t h = 0xCBF29CE484222325ULL;
    size_t i;
    for (i = 0; i < n; i++) {
        uint16_t u = (uint16_t)pcm[i];
        h ^= (uint8_t)(u & 0xFF);
        h *= 0x100000001B3ULL;
        h ^= (uint8_t)(u >> 8);
        h *= 0x100000001B3ULL;
    }
    return h;
}

/* Same as noisy() in make_fldigi_golden.py; this file is compiled without FP
 * contraction, so every step rounds like Python. */
static uint32_t lcg;
static double lcg_uniform(void)
{
    lcg = lcg * 1664525u + 1013904223u;
    return lcg / 4294967296.0;
}

static double *noisy(const int16_t *pcm, size_t n, int sr, int snr_step, uint32_t seed,
                     int tail_s, size_t *out_n)
{
    size_t lead = (size_t)sr, tail = (size_t)tail_s * (size_t)sr, total = lead + n + tail, i;
    double *x = (double *)calloc(total, sizeof(double));
    double ps = 0, inv, s2, sd, gain;
    int k;
    for (i = 0; i < n; i++)
        x[lead + i] = (double)pcm[i];
    *out_n = total;
    if (snr_step == 0) {
        for (i = 0; i < total; i++)
            x[i] /= 32768.0;
        return x;
    }
    for (i = 0; i < n; i++)
        ps += (double)pcm[i] * (double)pcm[i];
    ps = ps / (double)n;
    inv = snr_step == 1 ? 1.0 / sqrt(10.0) : 1.0 / 10.0;
    s2 = ps / inv * (sr / 2.0) / 2500.0;
    sd = sqrt(s2);
    gain = 3276.7 / sqrt(ps * (double)n / (double)total + s2);
    lcg = seed;
    for (i = 0; i < total; i++) {
        double z = 0.0, y, q;
        for (k = 0; k < 12; k++)
            z += lcg_uniform();
        z -= 6.0;
        y = (x[i] + sd * z) * gain;
        q = floor(y + 0.5);
        if (q > 32767)
            q = 32767;
        if (q < -32768)
            q = -32768;
        x[i] = q / 32768.0;  /* libsndfile sf_readf_double: int16 / 32768 */
    }
    return x;
}

static char texts[2][128];

static const char *text_for(const char *key)
{
    return strcmp(key, "cq") == 0 ? texts[0] : texts[1];
}

static void load_texts(void)
{
    char path[512], *buf, *line;
    size_t len;
    snprintf(path, sizeof path, "%s/texts.txt", golden);
    buf = (char *)read_file(path, &len);
    if (!buf) {
        fprintf(stderr, "cannot read %s\n", path);
        exit(2);
    }
    for (line = strtok(buf, "\n"); line; line = strtok(NULL, "\n")) {
        if (strncmp(line, "cq ", 3) == 0)
            snprintf(texts[0], sizeof texts[0], "%s", line + 3);
        else if (strncmp(line, "pangram ", 8) == 0)
            snprintf(texts[1], sizeof texts[1], "%s", line + 8);
    }
    free(buf);
}

static int has_option(const char *opts, const char *o)
{
    return strstr(opts, o) != NULL;
}

static int vectors_done, tx_exact, rx_exact, rx_total, stock_match;

/* The unmodified fldigi bench (rx_stock/): non-deterministic through its two memory faults
 * (see tests/golden/make_fldigi_golden.py). The port must give one of the outputs fldigi
 * gave, or recover the sent text within one character of fldigi's most frequent output (the
 * character fldigi loses to dspSeq::EnsureSpace). */
static void check_stock(const char *stem, const char *snr, const char *text,
                        const unsigned char *out, size_t got)
{
    char path[512];
    size_t glen, ns = strlen(text), dp, df = 0;
    int k, any = 0;
    for (k = 0; k < 8; k++) {
        unsigned char *want;
        if (k == 0)
            snprintf(path, sizeof path, "%s/rx_stock/%s_%s.txt", golden, stem, snr);
        else
            snprintf(path, sizeof path, "%s/rx_stock/%s_%s.alt%d.txt", golden, stem, snr, k);
        want = (unsigned char *)read_file(path, &glen);
        if (!want) {
            CHECK(k > 0, "%s %s: missing %s", stem, snr, path);
            break;
        }
        if (k == 0)
            df = window_distance((const unsigned char *)text, ns, want, glen);
        any |= glen == got && memcmp(want, out, got) == 0;
        free(want);
    }
    dp = window_distance((const unsigned char *)text, ns, out, got);
    stock_match += any;
    if (!any)
        printf("  %s %s: none of the unmodified fldigi's outputs; text errors port %zu, "
               "fldigi %zu\n", stem, snr, dp, df);
    CHECK(any || dp <= df + 1, "%s %s: %zu text errors, unmodified fldigi %zu", stem, snr, dp,
          df);
}
static size_t noisy_port, noisy_fldigi;

static void check_case(const char *stem, const char *sname, const char *key, int idx,
                       int txf, int rxf, const char *opts, size_t samples, int tail,
                       uint64_t hash)
{
    static const char *snr_names[3] = {"clean", "m05", "m10"};
    const fmt63_submode *m = fmt63_submode_find(sname);
    const char *text = text_for(key);
    fmt63_tx_config tc;
    double *audio;
    int16_t *pcm, *ref = NULL;
    size_t n, i, reflen = 0, diff1 = 0, diffbig = 0, ns = strlen(text);
    char path[512];
    int s;

    CHECK(m != NULL, "%s: submode %s", stem, sname);
    if (!m)
        return;
    memset(&tc, 0, sizeof tc);
    tc.submode = m->name;
    tc.carrier_hz = txf;
    tc.one_tone = has_option(opts, "onetone");
    tc.no_tune = has_option(opts, "notone");
    audio = render_over_cfg(&tc, text, 0, &n);
    pcm = (int16_t *)malloc((n + 1) * sizeof(int16_t));
    quantize(audio, n, pcm);
    CHECK(n == samples, "%s: %zu samples, fldigi %zu", stem, n, samples);
    tx_exact += fnv1a64(pcm, n) == hash;
    if (REFERENCE_PLATFORM)
        CHECK(fnv1a64(pcm, n) == hash, "%s: TX hash %016llx, fldigi %016llx", stem,
              (unsigned long long)fnv1a64(pcm, n), (unsigned long long)hash);
    snprintf(path, sizeof path, "%s/tx/%s.s16", golden, stem);
    ref = (int16_t *)read_file(path, &reflen);
    if (ref) {
        reflen /= 2;
        CHECK(reflen == n, "%s: stored vector %zu samples, port %zu", stem, reflen, n);
        for (i = 0; i < n && i < reflen; i++) {
            int d = abs((int)pcm[i] - (int)ref[i]);
            diff1 += d == 1;
            diffbig += d > 1;
        }
        CHECK(diffbig == 0, "%s: %zu samples differ by more than 1 LSB from fldigi", stem,
              diffbig);
        if (diff1)
            printf("  %s: %zu samples differ by 1 LSB from fldigi\n", stem, diff1);
    }

    for (s = 0; s < 3; s++) {
        fmt63_rx *rx;
        size_t nx, got, glen;
        /* the noisy inputs are built from fldigi's vector where it is stored, else from
         * the port's (identical on the reference platform) */
        double *x = noisy(ref ? ref : pcm, ref ? reflen : n, FMT63_RATE, s,
                          (uint32_t)(1000 * idx + s), tail, &nx);
        unsigned char *out, *want;
        rx = make_rx(m->name, rxf, has_option(opts, "long"), has_option(opts, "7bit"));
        CHECK(rx != NULL, "%s: rx_create", stem);
        if (!rx) {
            free(x);
            continue;
        }
        out = decode(rx, x, nx, 512, &got);
        snprintf(path, sizeof path, "%s/rx/%s_%s.txt", golden, stem, snr_names[s]);
        want = (unsigned char *)read_file(path, &glen);
        CHECK(want != NULL, "%s: missing %s", stem, path);
        check_stock(stem, snr_names[s], text, out, got);
        if (want) {
            int exact = got == glen && memcmp(out, want, glen) == 0;
            size_t dp = window_distance((const unsigned char *)text, ns, out, got);
            size_t df = window_distance((const unsigned char *)text, ns, want, glen);
            rx_total++;
            rx_exact += exact;
            if (REFERENCE_PLATFORM) {
                CHECK(exact, "%s %s: RX differs from fldigi (text errors port %zu, fldigi %zu)",
                      stem, snr_names[s], dp, df);
            } else {
                if (!exact)
                    printf("  %s %s: not identical; text errors port %zu, fldigi %zu\n", stem,
                           snr_names[s], dp, df);
                if (s == 0) {
                    CHECK(dp <= df + 1 + ns / 20, "%s clean: %zu text errors, fldigi %zu", stem,
                          dp, df);
                } else {
                    noisy_port += dp;
                    noisy_fldigi += df;
                }
            }
        }
        free(want);
        free(out);
        free(x);
        fmt63_rx_destroy(rx);
    }
    vectors_done++;
    free(ref);
    free(pcm);
    free(audio);
}

int main(int argc, char **argv)
{
    char path[512], *buf, *line, *save = NULL;
    size_t len;
    if (argc > 1)
        golden = argv[1];
    load_texts();
    snprintf(path, sizeof path, "%s/cases.txt", golden);
    buf = (char *)read_file(path, &len);
    if (!buf) {
        fprintf(stderr, "cannot read %s\n", path);
        return 2;
    }
    for (line = strtok_r(buf, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
        char stem[64], sname[32], key[16], opts[64];
        int idx, txf, rxf, tail;
        size_t samples;
        unsigned long long hash;
        if (line[0] == '#')
            continue;
        if (sscanf(line, "%63s %31s %15s %d %d %d %63s %zu %d %llx", stem, sname, key, &idx,
                   &txf, &rxf, opts, &samples, &tail, &hash) != 10) {
            CHECK(0, "bad line: %s", line);
            continue;
        }
        check_case(stem, sname, key, idx, txf, rxf, opts, samples, tail, (uint64_t)hash);
    }
    free(buf);
    if (!REFERENCE_PLATFORM)
        CHECK(noisy_port <= noisy_fldigi + noisy_fldigi / 5 + 10,
              "noisy decodes: %zu text errors, fldigi %zu", noisy_port, noisy_fldigi);
    CHECK(vectors_done == 24, "%d cases checked", vectors_done);
    printf("test_mt63_fldigi: %d cases, TX identical %d/%d, RX identical %d/%d%s, one of the "
           "unmodified fldigi's outputs %d/%d; %d failures\n",
           vectors_done, tx_exact, vectors_done, rx_exact, rx_total,
           REFERENCE_PLATFORM ? " (reference platform)" : "", stock_match, rx_total, failures);
    return failures ? 1 : 0;
}
