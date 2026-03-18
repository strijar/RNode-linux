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

typedef struct {
    uint8_t     port;
    uint8_t     pin;
} config_gpio_t;

typedef struct {
    char            *spi;
    config_gpio_t   cs;
    config_gpio_t   rst;
    config_gpio_t   busy;
    config_gpio_t   dio1;
    config_gpio_t   rx_en;
    config_gpio_t   tx_en;
    uint32_t        tcp_port;
} config_t;

bool config_load(const char *filename);

extern config_t *config;
