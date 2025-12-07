/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <stdlib.h>
#include <stdio.h>
#include "csma.h"
#include "sx126x.h"
#include "util.h"
#include "rnode.h"

#define CSMA_CW_PER_BAND_WINDOWS    15
#define CSMA_SLOT_MAX_MS            100
#define CSMA_SLOT_MIN_MS            24
#define CSMA_BAND_1_MAX_AIRTIME     7
#define CSMA_BAND_N_MIN_AIRTIME     85
#define CSMA_CW_PER_BAND_WINDOWS    15
#define CSMA_CW_BANDS               4

#define AIRTIME_LONGTERM            3600
#define AIRTIME_LONGTERM_MS         (AIRTIME_LONGTERM * 1000)

#define STATUS_INTERVAL_MS          3
#define DCD_SAMPLES                 2500
#define AIRTIME_BINLEN_MS           (STATUS_INTERVAL_MS * DCD_SAMPLES)
#define AIRTIME_BINS                ((AIRTIME_LONGTERM * 1000) / AIRTIME_BINLEN_MS)

static csma_channel_t   channel = {
    .airtime = 0.0,
    .longterm_airtime = 0.0,
    .longterm_channel_util = 0.0
};

static csma_cw_t        cw = {
    .band =  1,
    .min = 0,
    .max = CSMA_CW_PER_BAND_WINDOWS
};

static uint16_t         airtime_bins[AIRTIME_BINS];
static float            longterm_bins[AIRTIME_BINS];

static int32_t          csma_slot_ms = CSMA_SLOT_MIN_MS;

static long map(long x, long in_min, long in_max, long out_min, long out_max) {
    const long run = in_max - in_min;
    const long rise = out_max - out_min;
    const long delta = x - in_min;

    return (delta * rise) / run + out_min;
}

static void update_csma_parameters() {
    int32_t airtime_pct = channel.airtime * 100;
    int32_t new_cw_band = cw.band;

    if (airtime_pct <= CSMA_BAND_1_MAX_AIRTIME) {
        new_cw_band = 1;
    } else {
        int32_t at = airtime_pct + CSMA_BAND_1_MAX_AIRTIME;

        new_cw_band = map(at, CSMA_BAND_1_MAX_AIRTIME, CSMA_BAND_N_MIN_AIRTIME, 2, CSMA_CW_BANDS);
    }

    if (new_cw_band > CSMA_CW_BANDS) {
        new_cw_band = CSMA_CW_BANDS;
    }

    if (new_cw_band != cw.band) {
        cw.band = (uint8_t)(new_cw_band);
        cw.min  = (cw.band - 1) * CSMA_CW_PER_BAND_WINDOWS;
        cw.max  = (cw.band) * CSMA_CW_PER_BAND_WINDOWS - 1;

        rnode_send_stat_csma(&cw);
    }
}

static uint16_t current_airtime_bin() {
    return (get_time() % AIRTIME_LONGTERM_MS) / AIRTIME_BINLEN_MS;
}

void csma_update_airtime() {
    uint16_t cb = current_airtime_bin();
    uint16_t pb = cb - 1;

    if (cb - 1 < 0) {
        pb = AIRTIME_BINS - 1;
    }

    uint16_t nb = cb+1;

    if (nb == AIRTIME_BINS) {
        nb = 0;
    }

    airtime_bins[nb] = 0; 
    channel.airtime = (float) (airtime_bins[cb] + airtime_bins[pb]) / (2.0*AIRTIME_BINLEN_MS);

    uint32_t longterm_airtime_sum = 0;

    for (uint16_t bin = 0; bin < AIRTIME_BINS; bin++) {
        longterm_airtime_sum += airtime_bins[bin];
    }

    channel.longterm_airtime = (float)longterm_airtime_sum / (float)AIRTIME_LONGTERM_MS;

    float longterm_channel_util_sum = 0.0;

    for (uint16_t bin = 0; bin < AIRTIME_BINS; bin++) {
        longterm_channel_util_sum += longterm_bins[bin];
    }

    channel.longterm_channel_util = (float)longterm_channel_util_sum / (float)AIRTIME_BINS;

    update_csma_parameters();
    rnode_send_stat_channel(&channel);
}

void csma_add_airtime(uint32_t ms) {
    uint16_t    cb = current_airtime_bin();
    uint16_t    nb = cb + 1;

    if (nb == AIRTIME_BINS) {
        nb = 0;
    }

    airtime_bins[cb] += ms;
    airtime_bins[nb] = 0;
}

uint32_t csma_get_cw() {
    return random() % (cw.max - cw.min) + cw.min * csma_slot_ms;
}

void csma_update_current_rssi() {
    channel.current_rssi = sx126x_current_rssi();
}
