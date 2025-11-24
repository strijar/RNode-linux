/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  SX126x
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <pthread.h>

#include "sx126x.h"

typedef enum {
    REG_FSK_WHITENING_INITIAL_MSB          = 0x06B8,
    REG_FSK_CRC_INITIAL_MSB                = 0x06BC,
    REG_FSK_SYNC_WORD_0                    = 0x06C0,
    REG_FSK_NODE_ADDRESS                   = 0x06CD,
    REG_IQ_POLARITY_SETUP                  = 0x0736,
    REG_LORA_SYNC_WORD_MSB                 = 0x0740,
    REG_TX_MODULATION                      = 0x0889,
    REG_RX_GAIN                            = 0x08AC,
    REG_TX_CLAMP_CONFIG                    = 0x08D8,
    REG_OCP_CONFIGURATION                  = 0x08E7,
    REG_RTC_CONTROL                        = 0x0902,
    REG_XTA_TRIM                           = 0x0911,
    REG_XTB_TRIM                           = 0x0912,
    REG_EVENT_MASK                         = 0x0944
} reg_t;

typedef enum {
    STANDBY_RC      = 0x00,
    STANDBY_XOSC    = 0x01
} standby_t;

typedef enum {
    STATUS_DATA_AVAILABLE   = 0x04,
    STATUS_CMD_TIMEOUT      = 0x06,
    STATUS_CMD_ERROR        = 0x08,
    STATUS_CMD_FAILED       = 0x0A,
    STATUS_CMD_TX_DONE      = 0x0C,
    STATUS_MODE_STDBY_RC    = 0x20,
    STATUS_MODE_STDBY_XOSC  = 0x30,
    STATUS_MODE_FS          = 0x40,
    STATUS_MODE_RX          = 0x50,
    STATUS_MODE_TX          = 0x60
} status_mode_t;

typedef enum {
    FSK_MODEM               = 0x00,
    LORA_MODEM              = 0x01
} modem_t;

typedef enum {
    CAL_IMG_430             = 0x6B,
    CAL_IMG_440             = 0x6F,
    CAL_IMG_470             = 0x75,
    CAL_IMG_510             = 0x81,
    CAL_IMG_779             = 0xC1,
    CAL_IMG_787             = 0xC5,
    CAL_IMG_863             = 0xD7,
    CAL_IMG_870             = 0xDB,
    CAL_IMG_902             = 0xE1,
    CAL_IMG_928             = 0xE9
} cal_img_t;

typedef enum {
    PA_RAMP_10U             = 0x00,
    PA_RAMP_20U             = 0x01,
    PA_RAMP_40U             = 0x02,
    PA_RAMP_80U             = 0x03,
    PA_RAMP_200U            = 0x04,
    PA_RAMP_800U            = 0x05,
    PA_RAMP_1700U           = 0x06,
    PA_RAMP_3400U           = 0x07
} pa_ramp_t;

typedef enum {
    IRQ_TX_DONE             = 0x0001,
    IRQ_RX_DONE             = 0x0002,
    IRQ_PREAMBLE_DETECTED   = 0x0004,
    IRQ_SYNC_WORD_VALID     = 0x0008,
    IRQ_HEADER_VALID        = 0x0010,
    IRQ_HEADER_ERR          = 0x0020,
    IRQ_CRC_ERR             = 0x0040,
    IRQ_CAD_DONE            = 0x0080,
    IRQ_CAD_DETECTED        = 0x0100,
    IRQ_TIMEOUT             = 0x0200,
    IRQ_ALL                 = 0x03FF,
    IRQ_NONE                = 0x0000
} irq_t;

typedef enum {
    STATE_IDLE = 0,
    STATE_RX_SINGLE,
    STATE_RX_CONTINUOUS,
    STATE_TX
} state_t;

static struct gpiod_line    *cs_line = NULL;
static struct gpiod_line    *rst_line = NULL;
static struct gpiod_line    *busy_line = NULL;
static struct gpiod_line    *dio1_line = NULL;
static int                  spi_fd;

static uint8_t              buffer_index = 0;
static uint8_t              payload_tx_rx = 32;

static header_type_t        save_header_type = HEADER_EXPLICIT;
static uint8_t              save_preamble_len = 12;
static uint8_t              save_payload_len = 32;
static crc_t                save_crc = CRC_ON;
static state_t              state = STATE_IDLE;

static sx126x_rx_done_callback_t    rx_done_callback = NULL;
static sx126x_tx_done_callback_t    tx_done_callback = NULL;

static void wait_on_busy() {
    while (gpiod_line_get_value(busy_line) == 1) {
        usleep(100);
    }
}

/* SPI helpers */

static bool write_bytes(const uint8_t *buf, size_t len) {
    struct spi_ioc_transfer k = {
        .tx_buf = (unsigned long) buf,
        .len = len,
        .delay_usecs = 0,
        .speed_hz = 1000000,
        .bits_per_word = 8,
        .cs_change = 0,
    };

    gpiod_line_set_value(cs_line, 0);
    int l = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &k);
    gpiod_line_set_value(cs_line, 1);

    return (l == k.len);
}

static bool write_read_bytes(const uint8_t *buf, size_t buf_len, uint8_t *res, size_t res_len) {
    struct spi_ioc_transfer k[] = {
        {
            .tx_buf = (unsigned long) buf,
            .len = buf_len,
            .delay_usecs = 0,
            .speed_hz = 1000000,
            .bits_per_word = 8,
            .cs_change = 0,
        },{
            .rx_buf = (unsigned long) res,
            .len = res_len,
            .delay_usecs = 0,
            .speed_hz = 1000000,
            .bits_per_word = 8,
            .cs_change = 0,
        }
    };

    gpiod_line_set_value(cs_line, 0);
    int l = ioctl(spi_fd, SPI_IOC_MESSAGE(2), &k);
    gpiod_line_set_value(cs_line, 1);

    return (l == (buf_len + res_len));
}

static bool read_bytes(const uint8_t *buf, uint8_t *res, size_t len) {
    struct spi_ioc_transfer k = {
        .tx_buf = (unsigned long) buf,
        .rx_buf = (unsigned long) res,
        .len = len,
        .delay_usecs = 0,
        .speed_hz = 1000000,
        .bits_per_word = 8,
        .cs_change = 0,
    };

    gpiod_line_set_value(cs_line, 0);
    int l = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &k);
    gpiod_line_set_value(cs_line, 1);

    return (l == k.len);
}

/* SX126x helpers */

static bool write_reg(uint16_t reg, const uint8_t *buf, size_t len) {
    uint8_t msg[len + 3];

    msg[0] = 0x0D;
    msg[1] = (reg >> 8) & 0xFF;
    msg[2] = reg & 0xFF;

    for (size_t i = 0; i < len; i++)
        msg[i + 3] = buf[i];

    return write_bytes(msg, len + 3);
}

static bool write_buffer(uint8_t offset, const uint8_t *buf, size_t len) {
    uint8_t msg[len + 2];

    msg[0] = 0x0E;
    msg[1] = offset;

    for (size_t i = 0; i < len; i++)
        msg[i + 2] = buf[i];

    return write_bytes(msg, len + 2);
}

static bool read_buffer(uint8_t offset, uint8_t *buf, size_t len) {
    uint8_t msg[2];
    uint8_t ans[len + 1];

    msg[0] = 0x1E;
    msg[1] = offset;

    write_read_bytes(msg, 2, ans, len + 1);
    memcpy(buf, &ans[1], len);

    return true;
}

/* * */

static bool set_standby(standby_t x) {
    uint8_t msg[] = { 0x80, x };

    return write_bytes(msg, sizeof(msg));
}

static status_mode_t get_mode() {
    uint8_t msg[] = { 0xC0, 0x00 };
    uint8_t res[] = { 0x00, 0x00 };

    read_bytes(msg, res, sizeof(msg));

    return res[1] & 0x70;
}

static bool set_packet_type(modem_t x) {
    uint8_t msg[] = { 0x8A, x };

    return write_bytes(msg, sizeof(msg));
}

static bool calibrate(uint8_t x) {
    uint8_t msg[] = { 0x89, x };

    return write_bytes(msg, sizeof(msg));
}

static bool calibrate_image(cal_img_t min, cal_img_t max) {
    uint8_t msg[] = { 0x98, min, max };

    return write_bytes(msg, sizeof(msg));
}

static bool set_packet_params_loRa(uint8_t preamble_len, header_type_t header_type, uint8_t payload_len, crc_t crc) {
    uint8_t msg[] = { 0x8C, (preamble_len >> 8) & 0xFF, preamble_len & 0xFF, header_type, payload_len, crc, 0, 0, 0, 0 };

    return write_bytes(msg, sizeof(msg));
}

static bool clear_irq_status(uint16_t irq_mask) {
    uint8_t msg[] = { 0x02, (irq_mask >> 8) & 0xFF, irq_mask & 0xFF };

    return write_bytes(msg, sizeof(msg));
}

static bool irq_setup(uint16_t irq_mask, uint16_t dio1, uint16_t dio2, uint16_t dio3) {
    clear_irq_status(0x03FF);

    uint8_t msg[] = {
        0x08,
        (irq_mask >> 8) & 0xFF, irq_mask & 0xFF,
        (dio1 >> 8) & 0xFF, dio1 & 0xFF,
        (dio2 >> 8) & 0xFF, dio2 & 0xFF,
        (dio3 >> 8) & 0xFF, dio3 & 0xFF
    };

    return write_bytes(msg, sizeof(msg));
}

static uint16_t get_irq_status() {
    uint8_t msg[] = { 0x12 };
    uint8_t ans[] = { 0, 0, 0 };

    write_read_bytes(msg, sizeof(msg), ans, sizeof(ans));

    return (ans[1] << 8) | ans[2];
}

static void get_rx_buffer_status(uint8_t *rx_len, uint8_t *buffer_ptr) {
    uint8_t msg[] = { 0x13 };
    uint8_t ans[] = { 0, 0, 0 };

    write_read_bytes(msg, sizeof(msg), ans, sizeof(ans));

    *rx_len = ans[1];
    *buffer_ptr * ans[2];
}

static void get_packet_status(uint8_t *rssi, int8_t *snr, uint8_t *signal_rssi) {
    uint8_t msg[] = { 0x14 };
    uint8_t ans[] = { 0, 0, 0, 0 };

    write_read_bytes(msg, sizeof(msg), ans, sizeof(ans));

    *rssi = ans[1];
    *snr = (int8_t) ans[2];
    *signal_rssi = ans[3];
}

static bool set_tx(uint32_t timeout) {
    uint8_t msg[] = { 0x83, (timeout >> 16) & 0xFF, (timeout >> 8) & 0xFF, timeout & 0xFF };

    return write_bytes(msg, sizeof(msg));
}

static bool set_rx(uint32_t timeout) {
    uint8_t msg[] = { 0x82, (timeout >> 16) & 0xFF, (timeout >> 8) & 0xFF, timeout & 0xFF };

    return write_bytes(msg, sizeof(msg));
}

/* * */

static void * irq_worker(void *p) {
    struct gpiod_line_event event;

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    while (true) {
        int res = gpiod_line_event_wait(dio1_line, NULL);

        if (res == 1) {
            if (gpiod_line_event_read(dio1_line, &event) != 0) {
                continue;
            }

            if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
                uint16_t status = get_irq_status();

                switch (status) {
                    case IRQ_RX_DONE:
                        get_rx_buffer_status(&payload_tx_rx, &buffer_index);

                        if (rx_done_callback) {
                            rx_done_callback(payload_tx_rx);
                        }

                        if (state == STATE_RX_CONTINUOUS) {
                            clear_irq_status(IRQ_ALL);
                        }
                        break;

                    case IRQ_TX_DONE:
                        if (tx_done_callback) {
                            tx_done_callback();
                        }
                        break;

                    default:
                        break;
                }
            }
        }
    }
}

/* * */

bool sx126x_init_spi(const char *spidev, uint8_t cs_port, uint8_t cs_pin) {
    spi_fd = open(spidev, O_RDWR);

    if (spi_fd < 0) {
        return false;
    }

    struct gpiod_chip *chip = gpiod_chip_open_by_number(cs_port);

    if (!chip) {
        return false;
    }

    cs_line = gpiod_chip_get_line(chip, cs_pin);

    if (!cs_line) {
        return false;
    }

    gpiod_line_request_output(cs_line, "sx126x_cs", 1);

    return true;
}

bool sx126x_init_rst(uint8_t port, uint8_t pin) {
    struct gpiod_chip *chip = gpiod_chip_open_by_number(port);

    if (!chip) {
        return false;
    }

    rst_line = gpiod_chip_get_line(chip, pin);

    if (!rst_line) {
        return false;
    }

    gpiod_line_request_output(rst_line, "sx126x_rst", 0);

    return true;
}

bool sx126x_init_busy(uint8_t port, uint8_t pin) {
    struct gpiod_chip *chip = gpiod_chip_open_by_number(port);

    if (!chip) {
        return false;
    }

    busy_line = gpiod_chip_get_line(chip, pin);

    if (!busy_line) {
        return false;
    }

    gpiod_line_request_input(busy_line, "sx126x_busy");

    return true;
}

bool sx126x_init_dio1(uint8_t port, uint8_t pin) {
    struct gpiod_chip *chip = gpiod_chip_open_by_number(port);

    if (!chip) {
        return false;
    }

    dio1_line = gpiod_chip_get_line(chip, pin);

    if (!dio1_line) {
        return false;
    }

    gpiod_line_request_both_edges_events(dio1_line, "sx126x_dio1");

    pthread_t thread;

    pthread_create(&thread, NULL, irq_worker, NULL);
    pthread_detach(thread);

    return true;
}

void sx126x_set_rx_done_callback(sx126x_rx_done_callback_t callback) {
    rx_done_callback = callback;
}

void sx126x_set_tx_done_callback(sx126x_tx_done_callback_t callback) {
    tx_done_callback = callback;
}

bool sx126x_begin() {
    gpiod_line_set_value(rst_line, 0);
    usleep(10000);
    gpiod_line_set_value(rst_line, 1);
    usleep(10000);

    if (!set_standby(STANDBY_RC)) {
        return false;
    }

    if (get_mode() != STATUS_MODE_STDBY_RC) {
        return false;
    }

    if (!set_packet_type(LORA_MODEM)) {
        return false;
    }

    return true;
}

void sx126x_set_dio3_txco_ctrl(uint8_t voltage, uint16_t delay) {
    uint8_t msg[] = { 0x97, voltage, (delay >> 16) & 0xFF, (delay >> 8) & 0xFF, delay &0xFF };

    write_bytes(msg, sizeof(msg));

    set_standby(STANDBY_RC);
    calibrate(0xFF);
}

void sx126x_set_freq(uint64_t x) {
    uint8_t min;
    uint8_t max;

    if (x < 446000000) {
        min = CAL_IMG_430;
        max = CAL_IMG_440;
    } else if (x < 734000000) {
        min = CAL_IMG_470;
        max = CAL_IMG_510;
    } else if (x < 828000000) {
        min = CAL_IMG_779;
        max = CAL_IMG_787;
    } else if (x < 877000000) {
        min = CAL_IMG_863;
        max = CAL_IMG_870;
    } else {
        min = CAL_IMG_902;
        max = CAL_IMG_928;
    }

    wait_on_busy();
    calibrate_image(min, max);

    uint64_t    freq = x * 33554432 / 32000000;
    uint8_t     msg[] = { 0x86, (freq >> 24) & 0xFF, (freq >> 16) & 0xFF, (freq >> 8) & 0xFF, freq & 0xFF };

    wait_on_busy();
    write_bytes(msg, sizeof(msg));
}

void sx126x_set_tx_power(uint8_t db, dev_sel_t dev) {
    if (db > 22) {
        db = 22;
    } else if (db > 15 && dev == TX_POWER_SX1261) {
        db = 15;
    }

    uint8_t duty_cycle = 0x00;
    uint8_t hp_max = 0x00;
    uint8_t dev_sel = 0x00;
    uint8_t power = 0x0E;

    if (dev == TX_POWER_SX1261) {
        dev_sel = 0x01;
    }

    if (db == 22) {
        duty_cycle = 0x04;
        hp_max = 0x07;
        power = 0x16;
    } else if (db >= 20) {
        duty_cycle = 0x03;
        hp_max = 0x05;
        power = 0x16;
    } else if (db >= 17) {
        duty_cycle = 0x02;
        hp_max = 0x03;
        power = 0x16;
    } else if (db >= 14 && dev == TX_POWER_SX1261) {
        duty_cycle = 0x04;
        hp_max = 0x00;
        power = 0x0E;
    } else if (db >= 14 && dev == TX_POWER_SX1262) {
        duty_cycle = 0x02;
        hp_max = 0x02;
        power = 0x16;
    } else if (db >= 14 && dev == TX_POWER_SX1268) {
        duty_cycle = 0x04;
        hp_max = 0x06;
        power = 0x0F;
    } else if (db >= 10 && dev == TX_POWER_SX1261) {
        duty_cycle = 0x01;
        hp_max = 0x00;
        power = 0x0D;
    } else if (db >= 10 && dev == TX_POWER_SX1268) {
        duty_cycle = 0x00;
        hp_max = 0x03;
        power = 0x0F;
    } else {
        return;
    }

    uint8_t pa_config[] = { 0x95, duty_cycle, hp_max, dev_sel, 0x01 };
    uint8_t tx_params[] = { 0x8E, power, PA_RAMP_800U };

    wait_on_busy();
    write_bytes(pa_config, sizeof(pa_config));

    wait_on_busy();
    write_bytes(tx_params, sizeof(tx_params));
}

void sx126x_set_lora_modulation(uint8_t sf, bw_t bw, cr_t cr, ldro_t ldro) {
    uint8_t msg[] = { 0x8B, sf, bw, cr, ldro, 0, 0, 0, 0 };

    wait_on_busy();
    write_bytes(msg, sizeof(msg));
}

void sx126x_set_lora_packet(header_type_t header_type, uint8_t preamble_len, uint8_t payload_len, crc_t crc) {
    wait_on_busy();
    set_packet_params_loRa(preamble_len, header_type, payload_len, crc);

    save_preamble_len = preamble_len;
    save_header_type = header_type;
    save_payload_len = payload_len;
    save_crc = crc;
}

void sx126x_set_sync_word(uint16_t x) {
    uint8_t msg[] = { (x >> 8) & 0xFF, x & 0xFF };

    if (x <= 0xFF) {
        msg[0] = (x & 0xF0) | 0x04;
        msg[1] = (x << 4) | 0x04;
    }

    wait_on_busy();
    write_reg(REG_LORA_SYNC_WORD_MSB, msg, sizeof(msg));
}

void sx126x_begin_packet() {
    payload_tx_rx = 0;

    uint8_t base_addr[] = { 0x8F, buffer_index, (buffer_index + 0xFF) % 0xFF };

    wait_on_busy();
    write_bytes(base_addr, sizeof(base_addr));

    /* TODO rx/tx pin control */
}

void sx126x_write(const uint8_t *buf, uint8_t len) {
    wait_on_busy();
    write_buffer(buffer_index, buf, len);

    buffer_index = (buffer_index + len) % 256;
    payload_tx_rx += len;
}

void sx126x_end_packet() {
    uint16_t mask = IRQ_TX_DONE | IRQ_TIMEOUT;

    wait_on_busy();
    irq_setup(mask, mask, 0, 0);

    wait_on_busy();
    set_packet_params_loRa(save_preamble_len, save_header_type, payload_tx_rx, save_crc);

    state = STATE_TX;
    wait_on_busy();
    set_tx(0);
}

void sx126x_request(uint32_t timeout) {
    uint16_t mask = IRQ_RX_DONE | IRQ_TIMEOUT | IRQ_HEADER_ERR | IRQ_CRC_ERR;

    wait_on_busy();
    irq_setup(mask, mask, 0, 0);

    if (timeout == RX_CONTINUOUS) {
        state = STATE_RX_CONTINUOUS;
    } else {
        state = STATE_RX_SINGLE;
        timeout <<= 6;

        if (timeout > 0xFFFFFF) {
            timeout = RX_SINGLE;
        }
    }

    wait_on_busy();
    set_rx(timeout);
}

uint8_t sx126x_available() {
    return payload_tx_rx;
}

void sx126x_read(uint8_t *buf, uint16_t len) {
    wait_on_busy();
    read_buffer(buffer_index, buf, len);

    buffer_index = (buffer_index + len) % 256;

    if (payload_tx_rx > len) {
        payload_tx_rx -= len;
    } else  {
        payload_tx_rx = 0;
    }
}

float sx126x_packet_signal(float *rssi, float *snr, float *signal_rssi) {
    uint8_t raw_rssi;
    int8_t  raw_snr;
    uint8_t raw_signal_rssi;

    wait_on_busy();
    get_packet_status(&raw_rssi, &raw_snr, &raw_signal_rssi);

    *rssi = -(float) (raw_rssi) / 2.0f;
    *snr = (float) (raw_snr) / 4.0f;
    *signal_rssi = -(float) (raw_signal_rssi) / 2.0f;
}
