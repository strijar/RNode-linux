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

#include "config.h"
#include "sx126x.h"
#include "tcp.h"
#include "rnode.h"
#include "queue.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("RNode for SPI modules\nUsing: rnode <config_file.yaml>\n");
        return 1;
    }

    openlog("rnode", LOG_PID, LOG_USER);

    if (!config_load(argv[1])) {
        return 1;
    }

    if (!sx126x_init_spi(config->spi, config->cs.port, config->cs.pin)) {
        syslog(LOG_ERR, "SPI init");
        return 1;
    }

    if (!sx126x_init_rst(config->rst.port, config->rst.pin)) {
        syslog(LOG_ERR, "RST pin");
        return 1;
    }

    if (!sx126x_init_busy(config->busy.port, config->busy.pin)) {
        syslog(LOG_ERR, "Busy pin");
        return 1;
    }

    if (!sx126x_init_dio1(config->tx_en.port, config->tx_en.pin)) {
        syslog(LOG_ERR, "DIO1 pin");
        return 1;
    }

    if (!sx126x_init_tx_en(config->tx_en.port, config->tx_en.pin)) {
        syslog(LOG_ERR, "TX EN pin");
        return 1;
    }

    if (!sx126x_init_rx_en(config->rx_en.port, config->rx_en.pin)) {
        syslog(LOG_ERR, "RX EN pin");
        return 1;
    }

    queue_init();
    tcp_init(config->tcp_port);

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
