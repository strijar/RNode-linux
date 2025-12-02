/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <unistd.h>
#include <stdio.h>
#include <syslog.h>

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
    openlog("rnode", LOG_PID, LOG_USER);

    if (!sx126x_init_spi(SPI_DEV, PIN_PORT, SPI_PIN_CS)) {
        syslog(LOG_ERR, "SPI init");
        return 1;
    }

    if (!sx126x_init_rst(PIN_PORT, PIN_RST)) {
        syslog(LOG_ERR, "RST pin");
        return 1;
    }

    if (!sx126x_init_busy(PIN_PORT, PIN_BUSY)) {
        syslog(LOG_ERR, "Busy pin");
        return 1;
    }

    if (!sx126x_init_dio1(PIN_PORT, PIN_DIO1)) {
        syslog(LOG_ERR, "DIO1 pin");
        return 1;
    }

    if (!sx126x_init_tx_en(PIN_PORT, PIN_TX_EN)) {
        syslog(LOG_ERR, "TX EN pin");
        return 1;
    }

    if (!sx126x_init_rx_en(PIN_PORT, PIN_RX_EN)) {
        syslog(LOG_ERR, "RX EN pin");
        return 1;
    }

    queue_init();
    tcp_init(7633);

    sx126x_set_rx_done_callback(rnode_rx_done);
    sx126x_set_tx_done_callback(rnode_tx_done);
    sx126x_set_medium_callback(queue_medium_state);

    queue_medium_state(CAUSE_INIT);

    syslog(LOG_INFO, "Start");

    while (true) {
        tcp_read();
    }

    return 0;
}
