/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  GPIO HAL (libgpiod v2 backend)
 */

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <gpiod.h>

#include "gpio_hal.h"

struct gpio_line {
    struct gpiod_line_request      *req;
    unsigned int                    pin;
    struct gpiod_edge_event_buffer *event_buf;
};

gpio_line_t *gpio_request(unsigned int  chip,
                          unsigned int  pin,
                          gpio_dir_t    dir,
                          gpio_value_t  initial,
                          const char   *consumer)
{
    char chip_path[32];
    snprintf(chip_path, sizeof(chip_path), "/dev/gpiochip%u", chip);

    struct gpiod_chip *chip_h = gpiod_chip_open(chip_path);
    if (!chip_h) return NULL;

    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) {
        gpiod_chip_close(chip_h);
        return NULL;
    }

    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip_h);
        return NULL;
    }

    struct gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip_h);
        return NULL;
    }

    switch (dir) {
        case GPIO_DIR_OUTPUT:
            gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
            gpiod_line_settings_set_output_value(settings,
                initial == GPIO_VALUE_ACTIVE
                    ? GPIOD_LINE_VALUE_ACTIVE
                    : GPIOD_LINE_VALUE_INACTIVE);
            break;
        case GPIO_DIR_INPUT:
            gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
            break;
        case GPIO_DIR_INPUT_EDGE_RISING:
            gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
            gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
            break;
    }

    unsigned int offsets[] = { pin };
    gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings);
    gpiod_request_config_set_consumer(req_cfg, consumer);

    gpio_line_t *gpio = malloc(sizeof(*gpio));
    if (gpio) {
        gpio->req = gpiod_chip_request_lines(chip_h, req_cfg, line_cfg);
        gpio->pin = pin;
        gpio->event_buf = NULL;

        if (!gpio->req) {
            free(gpio);
            gpio = NULL;
        } else if (dir == GPIO_DIR_INPUT_EDGE_RISING) {
            gpio->event_buf = gpiod_edge_event_buffer_new(1);
            if (!gpio->event_buf) {
                gpiod_line_request_release(gpio->req);
                free(gpio);
                gpio = NULL;
            }
        }
    }

    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);
    gpiod_chip_close(chip_h);

    return gpio;
}

void gpio_release(gpio_line_t *line) {
    if (!line) return;
    
    if (line->event_buf) {
        gpiod_edge_event_buffer_free(line->event_buf);
    }
    gpiod_line_request_release(line->req);
    free(line);
}

gpio_value_t gpio_get(gpio_line_t *line) {
    return gpiod_line_request_get_value(line->req, line->pin)
               == GPIOD_LINE_VALUE_ACTIVE
           ? GPIO_VALUE_ACTIVE
           : GPIO_VALUE_INACTIVE;
}

void gpio_set(gpio_line_t *line, gpio_value_t val) {
    gpiod_line_request_set_value(line->req, line->pin,
        val == GPIO_VALUE_ACTIVE
            ? GPIOD_LINE_VALUE_ACTIVE
            : GPIOD_LINE_VALUE_INACTIVE);
}

int gpio_read_edge_rising(gpio_line_t *line) {
    int res = gpiod_line_request_read_edge_events(line->req, line->event_buf, 1);

    if (res < 0) return -1;
    if (res == 0) return 0;

    struct gpiod_edge_event *event = gpiod_edge_event_buffer_get_event(line->event_buf, 0);
    return (gpiod_edge_event_get_event_type(event) == GPIOD_EDGE_EVENT_RISING_EDGE) ? 1 : 0;
}
