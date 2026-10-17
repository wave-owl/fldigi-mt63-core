/*
 * fldigi-mt63-core CPU and memory benchmark: per submode the receiver on a 60 s over (CQ
 * text, idles, flush) at 8 kHz in 512-sample blocks, the transmitter, and the resident
 * memory of one receiver (fmt63_rx_footprint: everything it allocates, at create; long
 * integration, the largest).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#define _POSIX_C_SOURCE 200809L
#include <time.h>

#include "test_util.h"

static double cpu_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
    return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}


int main(void)
{
    int i, k;
    printf("%-11s %12s %14s %12s %14s\n", "submode", "RX x realtime", "RX ms per s", "TX x realtime",
           "RX KiB/instance");
    for (i = 0; i < fmt63_submode_count(); i++) {
        const fmt63_submode *m = fmt63_submode_at(i);
        fmt63_tx_config tc;
        size_t n, got, s;
        double *x, t0, rx_cpu = 1e9, tx_cpu = 1e9, audio_s;
        unsigned char out[8192];
        fmt63_rx *rxs[1];
        memset(&tc, 0, sizeof tc);
        tc.submode = m->name;
        tc.carrier_hz = 1500.0;
        for (k = 0; k < 3; k++) {
            sbuf b = {NULL, 0, 0};
            fmt63_tx *tx = fmt63_tx_create(&tc);
            size_t bound = fmt63_tx_bound(tx), q;
            double *tmp = (double *)malloc((bound > 62 * 512 ? bound : 62 * 512) * sizeof(double));
            t0 = cpu_s();
            sbuf_add(&b, tmp, fmt63_tx_begin(tx, tmp, 62 * 512));
            while (b.n < (size_t)60 * FMT63_RATE - bound) {
                for (q = 0; q < 22 && b.n < (size_t)55 * FMT63_RATE; q++)
                    sbuf_add(&b, tmp, fmt63_tx_put(tx, (unsigned char)"CQ CQ DE DG6FL DG6FL K"[q], tmp, bound));
                sbuf_add(&b, tmp, fmt63_tx_idle(tx, tmp, bound));
            }
            sbuf_add(&b, tmp, fmt63_tx_end(tx, tmp, bound));
            t0 = cpu_s() - t0;
            if (t0 < tx_cpu)
                tx_cpu = t0;
            free(tmp);
            fmt63_tx_destroy(tx);
            if (k < 2) {
                free(b.x);
                continue;
            }
            x = b.x;
            n = b.n;
        }
        audio_s = (double)n / FMT63_RATE;
        for (k = 0; k < 3; k++) {
            fmt63_rx *rx = make_rx(m->name, 1500.0, 0, 0);
            t0 = cpu_s();
            for (s = 0; s < n; s += 512)
                got = fmt63_rx_process(rx, x + s, n - s < 512 ? n - s : 512, out, sizeof out);
            t0 = cpu_s() - t0;
            (void)got;
            if (t0 < rx_cpu)
                rx_cpu = t0;
            fmt63_rx_destroy(rx);
        }
        /* the largest receiver: long integration (32 symbols) */
        rxs[0] = make_rx(m->name, 1500.0, 1, 0);
        for (s = 0; s < FMT63_RATE; s += 512)
            fmt63_rx_process(rxs[0], x + s, 512, out, sizeof out);
        printf("%-11s %12.0f %14.2f %12.0f %14.0f\n", m->name, audio_s / rx_cpu,
               1000.0 * rx_cpu / audio_s, audio_s / tx_cpu,
               (double)fmt63_rx_footprint(rxs[0]) / 1024.0);
        fmt63_rx_destroy(rxs[0]);
        free(x);
    }
    return 0;
}
