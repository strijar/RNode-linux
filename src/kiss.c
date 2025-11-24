/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <stdbool.h>
#include "kiss.h"
#include "rnode.h"
#include "tcp.h"

#define FEND                0xC0
#define FESC                0xDB
#define TFEND               0xDC
#define TFESC               0xDD

static uint8_t  buf_in[MTU];
static size_t   len_in = 0;
static bool     in_frame = false;
static bool     escape = false;

static uint8_t  buf_out[MTU];

void kiss_decode(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = buf[i];

        if (in_frame && byte == FEND) {
            if (len_in > 0) {
                rnode_send(buf_in, len_in);
            }
            in_frame = true;
            len_in = 0;
        } else if (byte == FEND) {
            in_frame = true;
            len_in = 0;
        } else if (in_frame) {
            if (byte == FESC) {
                escape = true;
            } else {
                if (escape) {
                    if (byte == TFEND) {
                        byte = FEND;
                    }
                    if (byte == TFESC) {
                        byte = FESC;
                    }
                    escape = false;
                }
                buf_in[len_in] = byte;
                len_in++;

                if (len_in >= sizeof(buf_in)) {
                    len_in = 0;
                    escape = false;
                    in_frame = false;

                    return;
                }
            }
        }
    }
}

void kiss_encode(const uint8_t *buf, size_t len) {
    size_t index = 0;

    buf_out[index] = FEND;
    index++;

    for (size_t i = 0; i < len; i++) {
        uint8_t x = buf[i];

        if (x == FEND) {
            buf_out[index] = FESC;      index++;
            buf_out[index] = TFEND;     index++;
        } else if (x == FESC) {
            buf_out[index] = FESC;      index++;
            buf_out[index] = TFESC;     index++;
        } else {
            buf_out[index] = x;         index++;
        }
    }

    buf_out[index] = FEND;
    index++;

    tcp_send(buf_out, index);
}
