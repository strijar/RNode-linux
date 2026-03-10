/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  GPIO HAL (libgpiod v1 backend)
 */

#include <stdlib.h>
#include <errno.h>
#include <gpiod.h>

#include "gpio_hal.h"

struct gpio_line {
    struct gpiod_line *line;
};

gpio_line_t *gpio_request(const char   *chip_path,
                          unsigned int  pin,
                          gpio_dir_t    dir,
                          gpio_value_t  initial,
                          const char   *consumer)
{
    struct gpiod_chip *chip = gpiod_chip_open(chip);
    if (!chip) return NULL;

    struct gpiod_line *line = gpiod_chip_get_line(chip, pin);
    if (!line) {
        gpiod_chip_close(chip);
        return NULL;
    }

    int result;

    switch (dir) {
        case GPIO_DIR_OUTPUT:
            result = gpiod_line_request_output(line, consumer,
                     initial == GPIO_VALUE_ACTIVE ? 1 : 0);
            break;
        case GPIO_DIR_INPUT:
            result = gpiod_line_request_input(line, consumer);
            break;
        case GPIO_DIR_INPUT_EDGE_RISING: {
            struct gpiod_line_request_config cfg = {
                .consumer     = consumer,
                .request_type = GPIOD_LINE_REQUEST_EVENT_RISING_EDGE,
                .flags        = 0,
            };
            result = gpiod_line_request(line, &cfg, 0);
            break;
        }
        default:
            result = -1;
            break;
    }

    gpiod_chip_close(chip);

    if (result < 0) return NULL;

    gpio_line_t *gpio = malloc(sizeof(*gpio));
    if (!gpio) {
        gpiod_line_release(line);
        return NULL;
    }

    gpio->line = line;
    return gpio;
}

void gpio_release(gpio_line_t *line) {
    if (!line) return;
    gpiod_line_release(line->line);
    free(line);
}

gpio_value_t gpio_get(gpio_line_t *line) {
    return gpiod_line_get_value(line->line) == 1
           ? GPIO_VALUE_ACTIVE
           : GPIO_VALUE_INACTIVE;
}

void gpio_set(gpio_line_t *line, gpio_value_t val) {
    gpiod_line_set_value(line->line, val == GPIO_VALUE_ACTIVE ? 1 : 0);
}

int gpio_read_edge_rising(gpio_line_t *line) {
    struct timespec timeout = { .tv_sec = 1, .tv_nsec = 0 };
    int res = gpiod_line_event_wait(line->line, &timeout);

    if (res < 0) return -1;
    if (res == 0) return 0; // timeout, no event

    struct gpiod_line_event event;
    if (gpiod_line_event_read(line->line, &event) < 0) return -1;

    return (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) ? 1 : 0;
}
