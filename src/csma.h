/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t band;
    uint8_t min;
    uint8_t max;
} csma_cw_t;

typedef struct {
    float   airtime;
    float   longterm_airtime;
    float   total_channel_util;
    float   longterm_channel_util;
    int32_t current_rssi;
    int32_t noise_floor;
} csma_channel_t;

void csma_update_airtime();
void csma_update_current_rssi();
void csma_add_airtime(uint32_t ms);
uint32_t csma_get_cw();
