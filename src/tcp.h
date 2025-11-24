/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#pragma once

#include <stdint.h>

void tcp_init(uint32_t port);
void tcp_read();
void tcp_send(char *buf, size_t len);
