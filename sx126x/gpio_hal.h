/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  GPIO HAL
 */

#pragma once

typedef struct gpio_line gpio_line_t;

typedef enum {
    GPIO_DIR_OUTPUT,
    GPIO_DIR_INPUT,
    GPIO_DIR_INPUT_EDGE_RISING
} gpio_dir_t;

typedef enum {
    GPIO_VALUE_INACTIVE = 0,
    GPIO_VALUE_ACTIVE   = 1
} gpio_value_t;

/* Allocate and configure a GPIO line. Returns NULL on failure. */
gpio_line_t *gpio_request(unsigned int  chip,
                          unsigned int  pin,
                          gpio_dir_t    dir,
                          gpio_value_t  initial,
                          const char   *consumer);

/* Release a line obtained with gpio_request(). */
void gpio_release(gpio_line_t *line);

/* Read the current level of an input line. */
gpio_value_t gpio_get(gpio_line_t *line);

/* Drive an output line to the given level. */
void gpio_set(gpio_line_t *line, gpio_value_t val);

/*
 * Block until one edge event is available on an edge-detection line.
 * Returns:  1 - event received; 0 - no event (timeout); <0 - error
 */
int gpio_read_edge_rising(gpio_line_t *line);