/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  SX126x
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    DIO3_OUTPUT_1_6 = 0x00,
    DIO3_OUTPUT_1_7 = 0x01,
    DIO3_OUTPUT_1_8 = 0x02,
    DIO3_OUTPUT_2_2 = 0x03
} dio3_voltage_t;

typedef enum {
    TXCO_DELAY_2_5  = 0x0140,
    TXCO_DELAY_5    = 0x0280,
    TXCO_DELAY_10   = 0x0560
} dio3_delay_t;

typedef enum {
    TX_POWER_SX1261 = 0x01,
    TX_POWER_SX1262 = 0x02,
    TX_POWER_SX1268 = 0x08
} dev_sel_t;

typedef enum {
    HEADER_EXPLICIT = 0x00,
    HEADER_IMPLICIT = 0x01
} header_type_t;

typedef enum {
    CRC_OFF = 0x00,
    CRC_ON  = 0x01
} crc_t;

typedef enum {
    BW_7800                 = 0x00,
    BW_10400                = 0x08,
    BW_15600                = 0x01,
    BW_20800                = 0x09,
    BW_31250                = 0x02,
    BW_41700                = 0x0A,
    BW_62500                = 0x03,
    BW_125000               = 0x04,
    BW_250000               = 0x05,
    BW_500000               = 0x06,
} bw_t;

typedef enum {
    CR_4_4                  = 0x00,
    CR_4_5                  = 0x01,
    CR_4_6                  = 0x02,
    CR_4_7                  = 0x03,
    CR_4_8                  = 0x04,
} cr_t;

typedef enum {
    LDRO_OFF                = 0x00,
    LDRO_ON                 = 0x01
} ldro_t;

typedef enum {
    RX_SINGLE               = 0x000000,
    RX_CONTINUOUS           = 0xFFFFFF
} rx_timeout_t;

typedef void (*sx126x_rx_done_callback_t)(uint16_t len);
typedef void (*sx126x_tx_done_callback_t)(void);
typedef void (*sx126x_medium_callback_t)(bool free);

bool sx126x_init_spi(const char *spidev, uint8_t cs_port, uint8_t cs_pin);
bool sx126x_init_rst(uint8_t port, uint8_t pin);
bool sx126x_init_busy(uint8_t port, uint8_t pin);
bool sx126x_init_dio1(uint8_t port, uint8_t pin);

void sx126x_set_rx_done_callback(sx126x_rx_done_callback_t callback);
void sx126x_set_tx_done_callback(sx126x_tx_done_callback_t callback);
void sx126x_set_medium_callback(sx126x_medium_callback_t callback);

bool sx126x_begin();

void sx126x_set_dio3_txco_ctrl(uint8_t voltage, uint16_t delay);
void sx126x_set_freq(uint64_t x);
void sx126x_set_tx_power(uint8_t db, dev_sel_t dev);
void sx126x_set_lora_modulation(uint8_t sf, bw_t bw, cr_t cr, ldro_t ldro);
void sx126x_set_lora_packet(header_type_t header_type, uint8_t preamble_len, uint8_t payload_len, crc_t crc);
void sx126x_set_sync_word(uint16_t x);

void sx126x_begin_packet();
void sx126x_write(const uint8_t *buf, uint8_t len);
void sx126x_end_packet();

void sx126x_request(uint32_t timeout);
uint8_t sx126x_available();
void sx126x_read(uint8_t *buf, uint16_t len);
void sx126x_packet_signal(float *rssi, float *snr, float *signal_rssi);
void sx126x_packet_signal_raw(uint8_t *rssi, int8_t *snr, uint8_t *signal_rssi);
