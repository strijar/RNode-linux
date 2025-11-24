/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <stdio.h>
#include "rnode.h"
#include "kiss.h"
#include "util.h"

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

void rnode_send(const uint8_t *buf, size_t len) {
    dump("RNode", buf, len);

    uint8_t cmd = buf[0];

    buf++;

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

        case CMD_LEAVE:
            break;

        default:
            printf("RNode: Unknown %02X\n", cmd);
            break;
    }
}
