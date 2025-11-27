/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include "rnode.h"
#include "kiss.h"
#include "util.h"
#include "queue.h"
#include "sx126x.h"

#define SINGLE_MTU          255
#define HEADER_L            1

#define CMD_UNKNOWN         0xFE
#define CMD_DATA            0x00
#define CMD_FREQUENCY       0x01
#define CMD_BANDWIDTH       0x02
#define CMD_TXPOWER         0x03
#define CMD_SF              0x04
#define CMD_CR              0x05
#define CMD_RADIO_STATE     0x06
#define CMD_RADIO_LOCK      0x07
#define CMD_DETECT          0x08
#define CMD_IMPLICIT        0x09
#define CMD_LEAVE           0x0A
#define CMD_ST_ALOCK        0x0B
#define CMD_LT_ALOCK        0x0C
#define CMD_PROMISC         0x0E
#define CMD_READY           0x0F

#define CMD_STAT_RX         0x21
#define CMD_STAT_TX         0x22
#define CMD_STAT_RSSI       0x23
#define CMD_STAT_SNR        0x24
#define CMD_STAT_CHTM       0x25
#define CMD_STAT_PHYPRM     0x26
#define CMD_STAT_BAT        0x27
#define CMD_STAT_CSMA       0x28
#define CMD_STAT_TEMP       0x29
#define CMD_BLINK           0x30
#define CMD_RANDOM          0x40

#define CMD_FB_EXT          0x41
#define CMD_FB_READ         0x42
#define CMD_FB_WRITE        0x43
#define CMD_FB_READL        0x44
#define CMD_DISP_READ       0x66
#define CMD_DISP_INT        0x45
#define CMD_DISP_ADDR       0x63
#define CMD_DISP_BLNK       0x64
#define CMD_DISP_ROT        0x67
#define CMD_DISP_RCND       0x68
#define CMD_NP_INT          0x65
#define CMD_BT_CTRL         0x46
#define CMD_BT_UNPAIR       0x70
#define CMD_BT_PIN          0x62
#define CMD_DIS_IA          0x69
#define CMD_WIFI_MODE       0x6A
#define CMD_WIFI_SSID       0x6B
#define CMD_WIFI_PSK        0x6C
#define CMD_WIFI_CHN        0x6E
#define CMD_WIFI_IP         0x84
#define CMD_WIFI_NM         0x85

#define CMD_BOARD           0x47
#define CMD_PLATFORM        0x48
#define CMD_MCU             0x49
#define CMD_FW_VERSION      0x50
#define CMD_CFG_READ        0x6D
#define CMD_ROM_READ        0x51
#define CMD_ROM_WRITE       0x52
#define CMD_CONF_SAVE       0x53
#define CMD_CONF_DELETE     0x54
#define CMD_DEV_HASH        0x56
#define CMD_DEV_SIG         0x57
#define CMD_FW_HASH         0x58
#define CMD_HASHES          0x60
#define CMD_FW_UPD          0x61
#define CMD_UNLOCK_ROM      0x59
#define ROM_UNLOCK_BYTE     0xF8
#define CMD_RESET           0x55
#define CMD_RESET_BYTE      0xF8

#define CMD_LOG             0x80
#define CMD_TIME            0x81
#define CMD_MUX_CHAIN       0x82
#define CMD_MUX_DSCVR       0x83

#define DETECT_REQ          0x73
#define DETECT_RESP         0x46

#define RADIO_STATE_OFF     0x00
#define RADIO_STATE_ON      0x01

#define NIBBLE_SEQ          0xF0
#define NIBBLE_FLAGS        0x0F
#define FLAG_SPLIT          0x01
#define SEQ_UNSET           0xFF

#define CMD_ERROR           0x90
#define ERROR_INITRADIO     0x01
#define ERROR_TXFAILED      0x02
#define ERROR_EEPROM_LOCKED 0x03
#define ERROR_QUEUE_FULL    0x04
#define ERROR_MEMORY_LOW    0x05
#define ERROR_MODEM_TIMEOUT 0x06

static uint8_t  seq = SEQ_UNSET;
static uint8_t  buf_in[MTU];
static size_t   len_in = 0;

static uint8_t  seq_tx = SEQ_UNSET;
static uint8_t  buf_tx[SINGLE_MTU];
static size_t   len_tx = 0;


/* * */

static void ans_detect(const uint8_t *param) {
    uint8_t ans[] = { CMD_DETECT, DETECT_RESP };

    kiss_encode(ans, sizeof(ans));
}

static void ans_fw_version(const uint8_t *param) {
    uint8_t ans[] = { CMD_FW_VERSION, 1, 52 };

    kiss_encode(ans, sizeof(ans));
}

static void ans_platform(const uint8_t *param) {
    uint8_t ans[] = { CMD_PLATFORM, 0x60 };

    kiss_encode(ans, sizeof(ans));
}

static void ans_mcu(const uint8_t *param) {
    uint8_t ans[] = { CMD_MCU, 1, 1 };

    kiss_encode(ans, sizeof(ans));
}

static void ans_frequency(const uint8_t *param) {
    uint8_t ans[] = { CMD_FREQUENCY, param[0], param[1], param[2], param[3] };

    kiss_encode(ans, sizeof(ans));
}

static void ans_bandwidth(const uint8_t *param) {
    uint8_t ans[] = { CMD_BANDWIDTH, param[0], param[1], param[2], param[3] };

    kiss_encode(ans, sizeof(ans));
}

static void ans_txpower(const uint8_t *param) {
    uint8_t ans[] = { CMD_TXPOWER, param[0] };

    kiss_encode(ans, sizeof(ans));
}

static void ans_sf(const uint8_t *param) {
    uint8_t ans[] = { CMD_SF, param[0] };

    kiss_encode(ans, sizeof(ans));
}

static void ans_cr(const uint8_t *param) {
    uint8_t ans[] = { CMD_CR, param[0] };

    kiss_encode(ans, sizeof(ans));
}

static void ans_radio_state(const uint8_t *param) {
    uint8_t ans[] = { CMD_RADIO_STATE, param[0] };

    kiss_encode(ans, sizeof(ans));
}

/* * */

void rnode_from_channel(const uint8_t *buf, size_t len) {
    dump("RNode", buf, len);

    uint8_t cmd = buf[0];

    buf++;
    len--;

    switch (cmd) {
        case CMD_DETECT:
            ans_detect(buf);
            break;

        case CMD_FW_VERSION:
            ans_fw_version(buf);
            break;

        case CMD_PLATFORM:
            ans_platform(buf);
            break;

        case CMD_MCU:
            ans_mcu(buf);
            break;

        case CMD_FREQUENCY:
            ans_frequency(buf);
            break;

        case CMD_BANDWIDTH:
            ans_bandwidth(buf);
            break;

        case CMD_TXPOWER:
            ans_txpower(buf);
            break;

        case CMD_SF:
            ans_sf(buf);
            break;

        case CMD_CR:
            ans_cr(buf);
            break;

        case CMD_RADIO_STATE:
            ans_radio_state(buf);
            break;

        case CMD_DATA:
            queue_push(buf, len);
            break;

        case CMD_LEAVE:
            break;

        default:
            printf("RNode: Unknown %02X\n", cmd);
            break;
    }
}

void rnode_signal_stat(uint8_t rssi, int8_t snr, uint8_t signal_rssi) {
    uint8_t ans_rssi[] = { CMD_STAT_RSSI, rssi };
    uint8_t ans_snr[] = { CMD_STAT_SNR, snr };

    kiss_encode(ans_rssi, sizeof(ans_rssi));
    kiss_encode(ans_snr, sizeof(ans_snr));
}

static void append_buf(const uint8_t *buf, size_t len) {
    memcpy(&buf_in[len_in], buf, len);
    len_in += len;
}

void rnode_from_air(const uint8_t *buf, size_t len) {
    /* The standard operating mode allows large */
    /* packets with a payload up to 500 bytes,  */
    /* by combining two raw LoRa packets.       */
    /* We read the 1-byte header and extract    */
    /* packet sequence number and split flags   */

    dump("RNode from air", buf, len);

    uint8_t header = *buf;
    bool    split = header & FLAG_SPLIT;
    uint8_t sequence = header >> 4;
    bool    ready = false;

    buf++;
    len--;

    if (split) {
        if (seq == SEQ_UNSET) {
            /* This is the first part of a split    */
            /* packet, so we set the seq variable   */
            /* and add the data to the buffer       */

            len_in = 0;
            append_buf(buf, len);
            seq = sequence;
        } else if (seq == sequence) {
            /* This is the second part of a split   */
            /* packet, so we add it to the buffer   */
            /* and set ready flag                   */

            append_buf(buf, len);
            seq = SEQ_UNSET;
            ready = true;
        } else {
            /* This split packet does not carry the */
            /* same sequence id, so we must assume  */
            /* that we are seeing the first part of */
            /* a new split packet.                  */

            len_in = 0;
            append_buf(buf, len);
            seq = sequence;
        }
    } else {
        /* This is not a split packet, so we    */
        /* just read it and set the ready       */
        /* flag to true.                        */

        if (seq != SEQ_UNSET) {
            /* If we already had part of a split    */
            /* packet in the buffer, we clear it.   */

            len_in = 0;
            seq = SEQ_UNSET;
        }

        append_buf(buf, len);
        ready = true;
    }

    if (ready) {
        uint8_t buf_kiss[len_in + 1];

        buf_kiss[0] = CMD_DATA;
        memcpy(&buf_kiss[1], buf_in, len_in);
        kiss_encode(buf_kiss, len_in + 1);

        len_in = 0;
    }
}

static void tx_buf(const uint8_t *buf, size_t len, uint8_t flag) {
    uint8_t buf_air[len + HEADER_L];

    buf_air[0] = seq_tx | flag;
    memcpy(&buf_air[1], buf, len);

    sx126x_begin_packet();
    sx126x_write(buf_air, len + HEADER_L);
    sx126x_end_packet();
}

void rnode_to_air(const uint8_t *buf, size_t len) {
    seq_tx = random() & 0xF0;

    if (len <= SINGLE_MTU - HEADER_L) {
        /* Everything fit into one packet */

        tx_buf(buf, len, 0);
        len_tx = 0;
    } else {
        /* It didn't fit. Save tail... */

        len_tx = len - SINGLE_MTU - HEADER_L;
        memcpy(buf_tx, &buf[SINGLE_MTU - HEADER_L], len_tx);

        /*  ...and sending the first part */

        tx_buf(buf, SINGLE_MTU - HEADER_L, FLAG_SPLIT);
    }
}

void rnode_tx_done() {
    if (len_tx) {
        /* There is an unsent tail, sending it */

        tx_buf(buf_tx, len_tx, FLAG_SPLIT);
        len_tx = 0;
    } else {
        sx126x_request(RX_CONTINUOUS);
    }
}

void rnode_rx_done(uint16_t len) {
    uint8_t buf[len];

    if (len > 0) {
        uint8_t rssi, signal_rssi;
        int8_t snr;

        sx126x_packet_signal_raw(&rssi, &snr, &signal_rssi);
        sx126x_read(buf, len);

        rnode_signal_stat(rssi, snr, signal_rssi);
        rnode_from_air(buf, len);
    }
}
