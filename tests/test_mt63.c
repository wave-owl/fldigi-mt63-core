/*
 * fldigi-mt63-core unit tests, independent of fldigi: submode table and argument checks,
 * TX -> RX loopback for all six submodes (short and long integration, 8-bit escape, 7-bit
 * receive), chunk invariance, two receivers interleaved, flush, reset and retuning, carrier
 * offsets, squelch and the call order of the transmitter.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#define _POSIX_C_SOURCE 200809L

#include "test_util.h"

static const char *TEXT = "CQ CQ DE DG6FL DG6FL K";
static const char *LONG_TEXT = "CQ CQ DE DG6FL DG6FL K the quick brown fox jumps over the lazy "
                                "dog 0123456789 <test?> THE QUICK BROWN FOX 73 DE DG6FL SK";

/* 1 s silence + over + tail_s silence at fldigi's -12 dB level, as the bench decodes it */
static double *padded_over_tail(const fmt63_tx_config *c, const char *text, int tail_s,
                                size_t *n)
{
    size_t m, i;
    double *a = render_over_cfg(c, text, 0, &m), *x;
    double mult = pow(10, -12.0 / 20.0);
    *n = m + (size_t)(1 + tail_s) * FMT63_RATE;
    x = (double *)calloc(*n, sizeof(double));
    for (i = 0; i < m; i++)
        x[FMT63_RATE + i] = a[i] * mult;
    free(a);
    return x;
}

static double *padded_over(const fmt63_tx_config *c, const char *text, size_t *n)
{
    return padded_over_tail(c, text, 12, n);
}

static void test_table(void)
{
    int i;
    const fmt63_submode *m;
    CHECK(fmt63_submode_count() == 6, "6 submodes");
    CHECK(fmt63_abi() == FMT63_ABI, "abi");
    CHECK(strstr(fmt63_version(), "contract=mt63-c1 abi=1 tx=1") != NULL, "version %s",
          fmt63_version());
    for (i = 0; i < 6; i++) {
        m = fmt63_submode_at(i);
        CHECK(m && fmt63_submode_find(m->name) == m, "find %d", i);
        CHECK(fabs(m->carrier_spacing * 64 - m->bandwidth) < 1e-9, "%s spacing", m->name);
        CHECK(m->symbol_samples == 200 * m->decimate, "%s symbol", m->name);
        CHECK(fabs(m->baud - 8000.0 / m->symbol_samples) < 1e-9, "%s baud", m->name);
    }
    CHECK(fmt63_submode_at(6) == NULL && fmt63_submode_at(-1) == NULL, "range");
    CHECK(fmt63_submode_find("mt63-1kl") == fmt63_submode_find("MT63-1000L"), "short name");
    CHECK(fmt63_submode_find("MT63-4000L") == NULL && fmt63_submode_find(NULL) == NULL, "unknown");
    CHECK(make_rx("MT63-2000L", 900.0, 0, 0) == NULL, "carrier below the band edge");
    CHECK(make_rx("MT63-2000L", 3100.0, 0, 0) == NULL, "carrier above 4 kHz - bandwidth/2");
    CHECK(make_rx("MT63-XL", 1500.0, 0, 0) == NULL, "unknown submode");
    CHECK(fmt63_rx_create(NULL) == NULL && fmt63_tx_create(NULL) == NULL, "NULL config");
}

static void test_loopback(void)
{
    int i, integ;
    for (i = 0; i < 6; i++) {
        const fmt63_submode *m = fmt63_submode_at(i);
        for (integ = 0; integ < 2; integ++) {
            fmt63_tx_config tc;
            size_t n, got;
            double *x;
            unsigned char *out;
            fmt63_rx *rx = make_rx(m->name, 1500.0, integ, 0);
            memset(&tc, 0, sizeof tc);
            tc.submode = m->name;
            tc.carrier_hz = 1500.0;
            /* MT63-500 with long integration needs about 25 s to bring a character out */
            size_t tail = (size_t)(m->bandwidth == 500 ? 30 : 12) * FMT63_RATE, g2;
            unsigned char *rest;
            x = padded_over_tail(&tc, LONG_TEXT, (int)(tail / FMT63_RATE), &n);
            /* the metric in the middle of a long text: the decoder sees received data
             * symbols about 1.5 x integration symbols after they arrive (ProcDelay and the
             * data pipe); an all-NUL flush leaves the carrier offset ambiguous */
            size_t mid = FMT63_RATE + 62 * 512 +
                         (strlen(LONG_TEXT) / 2 + (size_t)(integ ? 48 : 24)) *
                             (size_t)m->symbol_samples;
            tail = n - mid;
            out = decode(rx, x, mid, 512, &got);
            CHECK(fmt63_rx_locked(rx), "%s integ %d: not locked", m->name, integ);
            CHECK(fmt63_rx_metric(rx) > 3.0 && fmt63_rx_metric(rx) <= 99.9 &&
                      fmt63_rx_snr_db(rx) <= 20.0 && fabs(fmt63_rx_offset_hz(rx)) < 0.5,
                  "%s metric %g, s/n %g dB, offset %g Hz", m->name, fmt63_rx_metric(rx),
                  fmt63_rx_snr_db(rx), fmt63_rx_offset_hz(rx));
            rest = decode(rx, x + n - tail, tail, 512, &g2);
            out = (unsigned char *)realloc(out, got + g2 + 1);
            memcpy(out + got, rest, g2);
            got += g2;
            free(rest);
            CHECK(contains(out, got, LONG_TEXT), "%s integ %d: loopback lost the text", m->name,
                  integ);
            CHECK(!isnan(fmt63_rx_snr_db(rx)), "%s: s/n in silence", m->name);
            free(out);
            free(x);
            fmt63_rx_destroy(rx);
        }
    }
}

static void test_8bit(void)
{
    /* Latin-1 bytes go out as 127 + (c & 127) and come back as c (mt63.cxx:140-153,
     * 200-220); with 7-bit TX they become '.', a 7-bit receiver prints the raw codes. */
    const char *text = "Gr\xfc\xdf" "e aus K\xf6ln 73";
    fmt63_tx_config tc;
    size_t n, got;
    double *x;
    unsigned char *out;
    fmt63_rx *rx;
    memset(&tc, 0, sizeof tc);
    tc.submode = "MT63-1000S";
    tc.carrier_hz = 1500.0;
    x = padded_over(&tc, text, &n);
    rx = make_rx("MT63-1000S", 1500.0, 0, 0);
    out = decode(rx, x, n, 700, &got);
    CHECK(contains(out, got, text), "8-bit round trip");
    free(out);
    fmt63_rx_destroy(rx);
    rx = make_rx("MT63-1000S", 1500.0, 0, 1);
    out = decode(rx, x, n, 700, &got);
    CHECK(contains(out, got, "Gr\x7f|\x7f_e aus K\x7fvln 73"), "7-bit receive shows the escapes");
    free(out);
    fmt63_rx_destroy(rx);
    free(x);
    tc.no_8bit = 1;
    x = padded_over(&tc, text, &n);
    rx = make_rx("MT63-1000S", 1500.0, 0, 0);
    out = decode(rx, x, n, 512, &got);
    CHECK(contains(out, got, "Gr..e aus K.ln 73"), "7-bit TX sends '.'");
    free(out);
    fmt63_rx_destroy(rx);
    free(x);
}

static void test_chunks_and_instances(void)
{
    fmt63_tx_config tc;
    size_t n, g1, g2 = 0, g3 = 0, pos = 0;
    double *x;
    unsigned char *whole, out2[8192], out3[8192];
    unsigned char codes1[4096], codes2[4096];
    size_t nc1, nc2;
    fmt63_rx *a, *b, *c;
    uint32_t seed = 12345;
    memset(&tc, 0, sizeof tc);
    tc.submode = "MT63-2000L";
    tc.carrier_hz = 1700.0;
    x = padded_over(&tc, TEXT, &n);
    a = make_rx("MT63-2000L", 1700.0, 0, 0);
    whole = decode(a, x, n, n, &g1);
    nc1 = fmt63_rx_take_codes(a, codes1, sizeof codes1);
    /* random chunks of 1 .. 4000 samples, a second receiver on the same signal in between */
    b = make_rx("MT63-2000L", 1700.0, 0, 0);
    c = make_rx("MT63-2000L", 1700.0, 0, 0);
    while (pos < n) {
        size_t k;
        seed = seed * 1664525u + 1013904223u;
        k = 1 + (seed >> 8) % 4000;
        if (k > n - pos)
            k = n - pos;
        g2 += fmt63_rx_process(b, x + pos, k, out2 + g2, sizeof out2 - g2);
        g3 += fmt63_rx_process(c, x + pos, k, out3 + g3, sizeof out3 - g3);
        pos += k;
    }
    nc2 = fmt63_rx_take_codes(b, codes2, sizeof codes2);
    CHECK(g1 == g2 && memcmp(whole, out2, g1) == 0, "chunking changed the text");
    CHECK(nc1 == nc2 && memcmp(codes1, codes2, nc1) == 0, "chunking changed the codes");
    CHECK(g3 == g2 && memcmp(out3, out2, g2) == 0, "two receivers interfere");
    CHECK(contains(whole, g1, TEXT), "chunk test decodes");
    /* reset gives the state after create */
    fmt63_rx_reset(a);
    free(whole);
    whole = decode(a, x, n, 512, &g2);
    CHECK(g1 == g2 && memcmp(whole, out2, g1) == 0, "reset");
    free(whole);
    fmt63_rx_destroy(a);
    fmt63_rx_destroy(b);
    fmt63_rx_destroy(c);
    free(x);
}

static void test_offset_retune_flush(void)
{
    fmt63_tx_config tc;
    size_t n, got, m, i;
    double *x, *a;
    unsigned char *out, buf[4096];
    fmt63_rx *rx;
    memset(&tc, 0, sizeof tc);
    tc.submode = "MT63-1000L";
    tc.carrier_hz = 1512.0;   /* 12 Hz off the receiver: the synchroniser tracks it */
    x = padded_over(&tc, TEXT, &n);
    rx = make_rx("MT63-1000L", 1500.0, 0, 0);
    out = decode(rx, x, n - 12 * FMT63_RATE, 512, &got);   /* stop in the burst tail */
    CHECK(fmt63_rx_locked(rx), "locked on the offset signal");
    CHECK(fabs(fmt63_rx_offset_hz(rx) - 12.0) < 1.0, "offset %.2f Hz", fmt63_rx_offset_hz(rx));
    got += fmt63_rx_flush(rx, 200, out + got, 2048);
    CHECK(contains(out, got, TEXT), "flush brings out the text");
    CHECK(!fmt63_rx_locked(rx), "unlocked after the flush");
    free(out);
    /* retuning restarts the receiver at the new carrier */
    fmt63_rx_set_carrier(rx, 2000.0);
    free(x);
    tc.carrier_hz = 2000.0;
    x = padded_over(&tc, TEXT, &n);
    out = decode(rx, x, n, 512, &got);
    CHECK(contains(out, got, TEXT), "retuned receiver decodes");
    free(out);
    /* squelch: noise alone prints garbage, with the squelch nothing */
    {
        double *noise = (double *)malloc(10 * FMT63_RATE * sizeof(double));
        uint32_t r = 7;
        for (i = 0; i < 10 * FMT63_RATE; i++) {
            r = r * 1664525u + 1013904223u;
            noise[i] = ((double)r / 4294967296.0 - 0.5) * 0.2;
        }
        fmt63_rx_reset(rx);
        free(decode(rx, noise, 10 * FMT63_RATE, 512, &got));
        CHECK(got > 10, "noise printed %zu bytes", got);
        fmt63_rx_reset(rx);
        fmt63_rx_set_squelch(rx, 10.0);
        free(decode(rx, noise, 10 * FMT63_RATE, 512, &got));
        CHECK(got == 0, "squelch printed %zu bytes", got);
        free(noise);
    }
    /* output overflow is counted */
    fmt63_rx_set_squelch(rx, 0.0);
    fmt63_rx_reset(rx);
    got = fmt63_rx_process(rx, x, n, buf, 3);
    CHECK(got == 3 && fmt63_rx_dropped(rx) > 0, "dropped bytes counted");
    fmt63_rx_destroy(rx);
    free(x);

    /* transmitter call order and bounds */
    memset(&tc, 0, sizeof tc);
    tc.submode = "MT63-500L";
    tc.carrier_hz = 1500.0;
    {
        fmt63_tx *tx = fmt63_tx_create(&tc);
        size_t bound = fmt63_tx_bound(tx), begin = fmt63_tx_begin_len(tx);
        a = (double *)malloc((bound > begin ? bound : begin) * sizeof(double));
        CHECK(begin == 62 * 512, "begin length %zu", begin);
        CHECK(bound == 64 * 1600, "bound %zu", bound);
        CHECK(fmt63_tx_put(tx, 'A', a, bound) == 0, "put before begin refused");
        CHECK(fmt63_tx_begin(tx, a, begin - 1) == 0, "begin with a short buffer refused");
        CHECK(fmt63_tx_begin(tx, a, begin) == begin, "begin");
        CHECK(fmt63_tx_begin(tx, a, begin) == 0, "second begin refused");
        CHECK(fmt63_tx_put(tx, 'A', a, bound) == 1600, "one symbol");
        CHECK(fmt63_tx_put(tx, 0xE4, a, bound) == 3200, "escape + symbol");
        CHECK(fmt63_tx_idle(tx, a, bound) == 1600, "idle symbol");
        for (i = 0, m = 0; i < 3200; i++)
            m += fabs(a[i]) > 1.0 + 1e-12;
        CHECK(m == 0, "normalised output above 1");
        CHECK(fmt63_tx_end(tx, a, bound) == bound, "end");
        CHECK(fmt63_tx_put(tx, 'A', a, bound) == 0 && fmt63_tx_end(tx, a, bound) == 0,
              "after end refused");
        fmt63_tx_destroy(tx);
        free(a);
    }
    tc.no_tune = 1;
    {
        fmt63_tx *tx = fmt63_tx_create(&tc);
        double d[1];
        CHECK(fmt63_tx_begin_len(tx) == 0 && fmt63_tx_begin(tx, d, 0) == 0, "no tune");
        a = (double *)malloc(fmt63_tx_bound(tx) * sizeof(double));
        CHECK(fmt63_tx_put(tx, 'A', a, fmt63_tx_bound(tx)) == 1600, "put after a silent begin");
        free(a);
        fmt63_tx_destroy(tx);
    }
    tc.no_tune = 0;
    tc.tune_s = 61;
    CHECK(fmt63_tx_create(&tc) == NULL, "tone of more than 60 s refused");
}

int main(void)
{
    test_table();
    test_loopback();
    test_8bit();
    test_chunks_and_instances();
    test_offset_retune_flush();
    printf("test_mt63: %d failures\n", failures);
    return failures ? 1 : 0;
}
