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
#include <syslog.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <pthread.h>

#include "sx126x.h"

#define PHY_HEADER_LORA_SYMBOLS     20
#define PHY_CRC_LORA_BITS           16

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

static struct gpiod_line    *cs_line = NULL;
static struct gpiod_line    *rst_line = NULL;
static struct gpiod_line    *busy_line = NULL;
static struct gpiod_line    *dio1_line = NULL;
static struct gpiod_line    *tx_en_line = NULL;
static struct gpiod_line    *rx_en_line = NULL;
static int                  spi_fd;

static uint8_t              fifo_tx_addr_ptr = 0;
static uint8_t              fifo_rx_addr_ptr = 0;
static uint8_t              payload_tx_rx = 32;

static header_type_t        save_header_type = HEADER_EXPLICIT;
static uint8_t              save_preamble_len = 12;
static uint8_t              save_payload_len = 32;
static crc_t                save_crc = CRC_ON;
static uint8_t              save_sf;
static uint8_t              save_cr;
static uint32_t             save_bw;
static bool                 save_ldro;

static uint32_t             preamble_symbols;
static float                symbol_rate;
static float                symbol_time_ms;
static uint32_t             bitrate;

static state_t              state = SX126X_IDLE;

static sx126x_rx_done_callback_t    rx_done_callback = NULL;
static sx126x_tx_done_callback_t    tx_done_callback = NULL;
static sx126x_medium_callback_t     medium_callback = NULL;

static void wait_on_busy() {
    while (gpiod_line_get_value(busy_line) == 1) {
        usleep(100);
    }
}

static void switch_ant() {
    switch (state) {
        case SX126X_IDLE:
            if (rx_en_line) {
                gpiod_line_set_value(rx_en_line, 0);
            }
            if (tx_en_line) {
                gpiod_line_set_value(tx_en_line, 0);
            }
            break;

        case SX126X_RX_SINGLE:
        case SX126X_RX_CONTINUOUS:
            if (tx_en_line) {
                gpiod_line_set_value(tx_en_line, 0);
            }

            usleep(100);

            if (rx_en_line) {
                gpiod_line_set_value(rx_en_line, 1);
            }
            break;

        case SX126X_TX:
            if (rx_en_line) {
                gpiod_line_set_value(rx_en_line, 0);
            }

            usleep(100);

            if (tx_en_line) {
                gpiod_line_set_value(tx_en_line, 1);
            }
            break;
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

static bool write_buffer(const uint8_t *buf, size_t len) {
    uint8_t msg[len + 2];

    msg[0] = 0x0E;
    msg[1] = fifo_tx_addr_ptr;

    for (size_t i = 0; i < len; i++) {
        msg[i + 2] = buf[i];
        fifo_tx_addr_ptr++;
    }

    return write_bytes(msg, len + 2);
}

static bool read_buffer(uint8_t *buf, size_t len) {
    uint8_t msg[2];
    uint8_t ans[len + 1];

    msg[0] = 0x1E;
    msg[1] = fifo_rx_addr_ptr;

    write_read_bytes(msg, 2, ans, len + 1);
    memcpy(buf, &ans[1], len);

    return true;
}

/* * */

static bool set_standby(standby_t x) {
    uint8_t msg[] = { 0x80, x };

    return write_bytes(msg, sizeof(msg));
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
    uint8_t e = 1;
    uint8_t m = 1;

    while (e <= 15) {
        while (m <= 15) {
            preamble_symbols = m * (pow(2, e));

            if (preamble_symbols >= preamble_len - 4) {
                break;
            }
            m++;
        }

        if (preamble_symbols >= preamble_len - 4) {
            break;
        }

        m = 1; e++;
    }

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

static uint8_t get_rx_buffer_status() {
    uint8_t msg[] = { 0x13 };
    uint8_t ans[] = { 0, 0, 0 };

    write_read_bytes(msg, sizeof(msg), ans, sizeof(ans));

    fifo_rx_addr_ptr = ans[2];

    return ans[1];
}

static void get_packet_status(uint8_t *rssi, int8_t *snr, uint8_t *signal_rssi) {
    uint8_t msg[] = { 0x14 };
    uint8_t ans[] = { 0, 0, 0, 0 };

    write_read_bytes(msg, sizeof(msg), ans, sizeof(ans));

    *rssi = ans[1];
    *snr = (int8_t) ans[2];
    *signal_rssi = ans[3];
}

static int8_t get_current_rssi() {
    uint8_t msg[] = { 0x15 };
    uint8_t ans[] = { 0 };

    write_read_bytes(msg, sizeof(msg), ans, sizeof(ans));

    return ans[0];
}

static bool set_tx(uint32_t timeout) {
    uint8_t msg[] = { 0x83, (timeout >> 16) & 0xFF, (timeout >> 8) & 0xFF, timeout & 0xFF };

    return write_bytes(msg, sizeof(msg));
}

static bool set_rx(uint32_t timeout) {
    uint8_t msg[] = { 0x82, (timeout >> 16) & 0xFF, (timeout >> 8) & 0xFF, timeout & 0xFF };

    return write_bytes(msg, sizeof(msg));
}

static void set_irq_mask() {
    uint16_t mask = IRQ_HEADER_VALID | IRQ_TX_DONE | IRQ_RX_DONE | IRQ_HEADER_ERR | IRQ_CRC_ERR | IRQ_PREAMBLE_DETECTED;

    wait_on_busy();
    irq_setup(mask, mask, 0, 0);
}

/* * */

static void * irq_worker(void *p) {
    struct  gpiod_line_event event;
    bool    crc_ok = true;

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    while (state == SX126X_IDLE) {
        usleep(1000);
    }

    while (true) {
        int res = gpiod_line_event_wait(dio1_line, NULL);

        if (res == 1) {
            if (gpiod_line_event_read(dio1_line, &event) != 0) {
                continue;
            }

            if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
                uint16_t    status = get_irq_status();

                if (status & IRQ_CRC_ERR) {
                    clear_irq_status(IRQ_CRC_ERR);
                    syslog(LOG_INFO, "IRQ: CRC ERR");

                    crc_ok = false;
                }

                if (status & IRQ_PREAMBLE_DETECTED) {
                    clear_irq_status(IRQ_PREAMBLE_DETECTED);
                    syslog(LOG_INFO, "IRQ: PREAMBLE DETECTED");
                    crc_ok = true;

                    if (medium_callback) {
                        medium_callback(CAUSE_PREAMBLE_DETECTED);
                    }
                }

                if (status & IRQ_HEADER_VALID) {
                    clear_irq_status(IRQ_HEADER_VALID);
                    syslog(LOG_INFO, "IRQ: HEADER VALID");

                    if (medium_callback) {
                        medium_callback(CAUSE_HEADER_VALID);
                    }
                }

                if (status & IRQ_HEADER_ERR) {
                    clear_irq_status(IRQ_HEADER_ERR);
                    syslog(LOG_INFO, "IRQ: HEADER ERR");

                    if (medium_callback) {
                        medium_callback(CAUSE_HEADER_ERR);
                    }
                }

                if (status & IRQ_RX_DONE) {
                    if (state == SX126X_RX_CONTINUOUS) {
                        clear_irq_status(IRQ_RX_DONE);
                    }

                    syslog(LOG_INFO, "IRQ: RX DONE");

                    if (medium_callback) {
                        medium_callback(CAUSE_RX_DONE);
                    }

                    if (crc_ok) {
                        payload_tx_rx = get_rx_buffer_status();

                        if (rx_done_callback) {
                            rx_done_callback(payload_tx_rx);
                        }

                    }
                }

                if (status & IRQ_TX_DONE) {
                    clear_irq_status(IRQ_TX_DONE);
                    syslog(LOG_INFO, "IRQ: TX DONE");

                    if (medium_callback) {
                        medium_callback(CAUSE_TX_DONE);
                    }

                    if (tx_done_callback) {
                        tx_done_callback();
                    }
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

bool sx126x_init_tx_en(uint8_t port, uint8_t pin) {
    struct gpiod_chip *chip = gpiod_chip_open_by_number(port);

    if (!chip) {
        return false;
    }

    tx_en_line = gpiod_chip_get_line(chip, pin);

    if (!tx_en_line) {
        return false;
    }

    gpiod_line_request_output(tx_en_line, "sx126x_tx_en", 0);

    return true;
}

bool sx126x_init_rx_en(uint8_t port, uint8_t pin) {
    struct gpiod_chip *chip = gpiod_chip_open_by_number(port);

    if (!chip) {
        return false;
    }

    rx_en_line = gpiod_chip_get_line(chip, pin);

    if (!rx_en_line) {
        return false;
    }

    gpiod_line_request_output(rx_en_line, "sx126x_rx_en", 0);

    return true;
}

void sx126x_set_rx_done_callback(sx126x_rx_done_callback_t callback) {
    rx_done_callback = callback;
}

void sx126x_set_tx_done_callback(sx126x_tx_done_callback_t callback) {
    tx_done_callback = callback;
}

void sx126x_set_medium_callback(sx126x_medium_callback_t callback) {
    medium_callback = callback;
}

bool sx126x_begin() {
    gpiod_line_set_value(rst_line, 0);
    usleep(10000);
    gpiod_line_set_value(rst_line, 1);
    usleep(10000);

    state = SX126X_IDLE;
    switch_ant();

    if (!set_standby(STANDBY_RC)) {
        return false;
    }

    if ((sx126x_get_status_mode() & 0x70) != STATUS_MODE_STDBY_RC) {
        return false;
    }

    if (!set_packet_type(LORA_MODEM)) {
        return false;
    }

    uint8_t base_addr[] = { 0x8F, 0, 0 };

    wait_on_busy();
    write_bytes(base_addr, sizeof(base_addr));

    set_irq_mask();

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

void sx126x_set_tx_power(uint8_t db) {
    if (db > 22) {
        db = 22;
    }

    uint8_t pa_config[] = { 0x95, 0x04, 0x07, 0x00, 0x01 };

    wait_on_busy();
    write_bytes(pa_config, sizeof(pa_config));

    uint8_t ocp_param[] = { 0x18 };

    wait_on_busy();
    write_reg(REG_OCP_CONFIGURATION, ocp_param, sizeof(ocp_param));

    uint8_t tx_params[] = { 0x8E, db, PA_RAMP_40U };

    wait_on_busy();
    write_bytes(tx_params, sizeof(tx_params));
}

void sx126x_set_lora_modulation(uint8_t sf, bw_t bw, cr_t cr, ldro_t ldro) {
    uint8_t msg[] = { 0x8B, sf, bw, cr, ldro, 0, 0, 0, 0 };

    save_sf = sf;
    save_cr = cr + 4;
    save_ldro = ldro;

    switch (bw) {
        case BW_7800:   save_bw = 7800;     break;
        case BW_10400:  save_bw = 10400;    break;
        case BW_15600:  save_bw = 15600;    break;
        case BW_20800:  save_bw = 20800;    break;
        case BW_31250:  save_bw = 31250;    break;
        case BW_41700:  save_bw = 41700;    break;
        case BW_62500:  save_bw = 62500;    break;
        case BW_125000: save_bw = 125000;   break;
        case BW_250000: save_bw = 250000;   break;
        case BW_500000: save_bw = 500000;   break;
    }

    symbol_rate = (float) save_bw / (float)(pow(2, save_sf));
    symbol_time_ms = (1.0f / symbol_rate) * 1000.0f;
    bitrate = (uint32_t) (save_sf * ((4.0f / (float) save_cr) / ((float)(pow(2, save_sf)) / ((float)save_bw / 1000.0f))) * 1000.0f);

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
    fifo_tx_addr_ptr = 0;
}

void sx126x_write(const uint8_t *buf, uint8_t len) {
    wait_on_busy();
    write_buffer(buf, len);

    payload_tx_rx += len;
}

void sx126x_end_packet() {
    state = SX126X_TX;

    wait_on_busy();
    set_packet_params_loRa(save_preamble_len, save_header_type, payload_tx_rx, save_crc);

    set_irq_mask();
    switch_ant();

    wait_on_busy();
    set_tx(0);
}

void sx126x_request(uint32_t timeout) {
    if (timeout == RX_CONTINUOUS) {
        state = SX126X_RX_CONTINUOUS;
    } else {
        state = SX126X_RX_SINGLE;
        timeout <<= 6;

        if (timeout > 0xFFFFFF) {
            timeout = RX_SINGLE;
        }
    }

    wait_on_busy();
    set_packet_params_loRa(save_preamble_len, save_header_type, payload_tx_rx, save_crc);

    set_irq_mask();
    switch_ant();

    wait_on_busy();
    set_rx(timeout);
}

uint8_t sx126x_available() {
    return payload_tx_rx;
}

void sx126x_read(uint8_t *buf, uint16_t len) {
    wait_on_busy();
    read_buffer(buf, len);

    if (payload_tx_rx > len) {
        payload_tx_rx -= len;
    } else  {
        payload_tx_rx = 0;
    }
}

void sx126x_packet_signal(float *rssi, float *snr, float *signal_rssi) {
    uint8_t raw_rssi;
    int8_t  raw_snr;
    uint8_t raw_signal_rssi;

    wait_on_busy();
    get_packet_status(&raw_rssi, &raw_snr, &raw_signal_rssi);

    *rssi = -(float) (raw_rssi) / 2.0f;
    *snr = (float) (raw_snr) / 4.0f;
    *signal_rssi = -(float) (raw_signal_rssi) / 2.0f;
}

void sx126x_packet_signal_raw(uint8_t *rssi, int8_t *snr, uint8_t *signal_rssi) {
    wait_on_busy();
    get_packet_status(rssi, snr, signal_rssi);
}

int8_t sx126x_current_rssi() {
    int8_t rssi;

    wait_on_busy();

    return -get_current_rssi(&rssi) / 2;
}

uint32_t sx126x_air_time(uint16_t len, uint32_t *preamble_ms, uint32_t *data_ms) {
    float t_sym = (powf(2, save_sf) / save_bw ) * 1000.0f;
    float bits = 8.0f * len;

    bits -= 4.0f * save_sf;
    bits += 8.0f;
    bits += 16.0f;  /* CRC */
    bits += 20.0f;  /* Header */

    uint16_t    payload = ceil(bits / 4.0f / save_sf) * save_cr + 8;
    uint32_t    preamble = (save_preamble_len + 4.25f) * t_sym;
    uint32_t    data = payload * t_sym;

    if (preamble_ms) {
        *preamble_ms = preamble;
    }

    if (data_ms) {
        *data_ms = data;
    }

    return preamble + data;
}

state_t sx126x_get_state() {
    return state;
}

status_mode_t sx126x_get_status_mode() {
    uint8_t msg[] = { 0xC0, 0x00 };
    uint8_t res[] = { 0x00, 0x00 };

    read_bytes(msg, res, sizeof(msg));

    return res[1];
}
