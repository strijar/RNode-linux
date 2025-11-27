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

#define MTU 1024

void rnode_from_channel(const uint8_t *buf, size_t len);

void rnode_signal_stat(uint8_t rssi, int8_t snr, uint8_t signal_rssi);
void rnode_from_air(const uint8_t *buf, size_t len);
void rnode_to_air(const uint8_t *buf, size_t len);

void rnode_tx_done();
void rnode_rx_done(uint16_t len);
