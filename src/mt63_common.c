/*
 * fldigi-mt63-core -- version, submode table, carrier arithmetic.
 *
 * Copyright (C) 2026 Gerolf Ziegenhain
 * Ported from fldigi 4.2.13 (tag v4.2.13, commit e7c3ab37); (c) 1999-2004 Pawel Jalocha
 * SP9VRC, 2007-2011 Dave Freese W1HKJ.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "mt63_internal.h"

#include <ctype.h>
#include <math.h>
#include <string.h>

const char *fmt63_version(void)
{
    return "fldigi-mt63-core " FMT63_VERSION_STRING " contract=mt63-c1 abi=1 tx=1";
}

int fmt63_abi(void)
{
    return FMT63_ABI;
}

/* mt63::mt63 (mt63.cxx:347-389): bandwidth and interleave per trx_mode; MT63rx::Preset /
 * MT63tx::Preset (mt63base.cxx:101-121, 761-777): decimation and anti-alias length;
 * mode_info[] ids and short names (globals/globals.cxx:120-125). Symbol rate =
 * 8000 / decimate / SymbolSepar, carrier spacing = DataCarrSepar bins of a 512-point FFT at
 * 8000 / decimate. */
static const fmt63_submode SUBMODES[] = {
    {"MT63-500S", 53, 500, 32, 8, 128, 5.0, 7.8125, 1600},
    {"MT63-500L", 54, 500, 64, 8, 128, 5.0, 7.8125, 1600},
    {"MT63-1000S", 55, 1000, 32, 4, 64, 10.0, 15.625, 800},
    {"MT63-1000L", 56, 1000, 64, 4, 64, 10.0, 15.625, 800},
    {"MT63-2000S", 57, 2000, 32, 2, 64, 20.0, 31.25, 400},
    {"MT63-2000L", 58, 2000, 64, 2, 64, 20.0, 31.25, 400},
};
/* fldigi's short names (globals/globals.cxx:120-125) */
static const char *const SHORT_NAMES[] = {"MT63-500S", "MT63-500L", "MT63-1KS",
                                          "MT63-1KL", "MT63-2KS", "MT63-2KL"};
#define NSUB ((int)(sizeof SUBMODES / sizeof SUBMODES[0]))

int fmt63_submode_count(void)
{
    return NSUB;
}

const fmt63_submode *fmt63_submode_at(int index)
{
    return (index >= 0 && index < NSUB) ? &SUBMODES[index] : NULL;
}

static int same(const char *a, const char *b)
{
    for (; *a && *b; a++, b++)
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b))
            return 0;
    return *a == *b;
}

const fmt63_submode *fmt63_submode_find(const char *name)
{
    int i;
    if (!name)
        return NULL;
    for (i = 0; i < NSUB; i++)
        if (same(name, SUBMODES[i].name) || same(name, SHORT_NAMES[i]))
            return &SUBMODES[i];
    return NULL;
}

const fmt63_submode *mt63__submode(const char *name)
{
    return fmt63_submode_find(name);
}

/* FirstDataCarr of MT63rx::Preset / MT63tx::Preset (mt63base.cxx:101-121, 761-777) */
int mt63__first_carrier(int bandwidth, double freq)
{
    switch (bandwidth) {
    case 500:
        return (int)floor((freq - bandwidth / 2.0) * 256 / 500 + 0.5);
    case 1000:
        return (int)floor((freq - bandwidth / 2.0) * 128 / 500 + 0.5);
    default:
        return (int)floor((freq - bandwidth / 2.0) * 64 / 500 + 0.5);
    }
}

int mt63__carrier_ok(int bandwidth, double freq)
{
    return isfinite(freq) && freq - bandwidth / 2.0 >= 0.0 &&
           freq + bandwidth / 2.0 <= FMT63_RATE / 2.0;
}
