/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <unistd.h>
#include <stdio.h>

#include "sx126x.h"
#include "tcp.h"
#include "rnode.h"
#include "queue.h"

/* For Lora board RNS-Gate */

#define SPI_DEV     "/dev/spidev1.0"
#define SPI_PIN_CS  13

#define PIN_PORT    0
#define PIN_RST     6
#define PIN_BUSY    11
#define PIN_DIO1    12
#define PIN_RX_EN   2
#define PIN_TX_EN   18

int main() {
    if (!sx126x_init_spi(SPI_DEV, PIN_PORT, SPI_PIN_CS)) {
        printf("Err: SPI init\n");
        return 1;
    }

    if (!sx126x_init_rst(PIN_PORT, PIN_RST)) {
        printf("Err: RST pin\n");
        return 1;
    }

    if (!sx126x_init_busy(PIN_PORT, PIN_BUSY)) {
        printf("Err: Busy pin\n");
        return 1;
    }

    if (!sx126x_init_dio1(PIN_PORT, PIN_DIO1)) {
        printf("Err: DIO1 pin\n");
        return 1;
    }

    if (!sx126x_begin()) {
        printf("Err: Begin\n");
        return 1;
    }

    queue_init();
    tcp_init(7633);

    sx126x_set_dio3_txco_ctrl(DIO3_OUTPUT_1_8, TXCO_DELAY_10);
    sx126x_set_freq(915000000);
    sx126x_set_tx_power(17, TX_POWER_SX1262);

    sx126x_set_lora_modulation(7, BW_125000, CR_4_5, LDRO_OFF);
    sx126x_set_lora_packet(HEADER_EXPLICIT, 18, 15, CRC_ON);
    sx126x_set_sync_word(0x1424);

    sx126x_set_rx_done_callback(rnode_rx_done);
    sx126x_set_tx_done_callback(rnode_tx_done);
    sx126x_set_medium_callback(queue_medium_state);

    queue_medium_state(true);

    printf("Ready\n");

    sx126x_request(RX_CONTINUOUS);

    while (true) {
        tcp_read();
    }

    return 0;
}
