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
#include <stdbool.h>
#include "sx126x.h"

void queue_init();
void queue_set_busy_timeout(uint32_t header_ms, uint32_t data_ms);
void queue_push(const uint8_t *buf, size_t len);

void queue_medium_state(cause_medium_t cause);
