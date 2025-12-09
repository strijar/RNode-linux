/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <cyaml/cyaml.h>
#include <syslog.h>
#include "config.h"

const cyaml_schema_field_t gpio_fields_schema[] = {
    CYAML_FIELD_UINT("port",            CYAML_FLAG_DEFAULT, config_gpio_t, port),
    CYAML_FIELD_UINT("pin",             CYAML_FLAG_DEFAULT, config_gpio_t, pin),
    CYAML_FIELD_END
};

const cyaml_schema_field_t config_fields_schema[] = {
    CYAML_FIELD_STRING_PTR("spi",       CYAML_FLAG_POINTER,  config_t, spi, 0, CYAML_UNLIMITED),
    CYAML_FIELD_MAPPING("cs",           CYAML_FLAG_DEFAULT, config_t, cs, gpio_fields_schema),
    CYAML_FIELD_MAPPING("rst",          CYAML_FLAG_DEFAULT, config_t, rst, gpio_fields_schema),
    CYAML_FIELD_MAPPING("busy",         CYAML_FLAG_DEFAULT, config_t, busy, gpio_fields_schema),
    CYAML_FIELD_MAPPING("dio1",         CYAML_FLAG_DEFAULT, config_t, dio1, gpio_fields_schema),
    CYAML_FIELD_MAPPING("rx_en",        CYAML_FLAG_DEFAULT, config_t, rx_en, gpio_fields_schema),
    CYAML_FIELD_MAPPING("tx_en",        CYAML_FLAG_DEFAULT, config_t, tx_en, gpio_fields_schema),
    CYAML_FIELD_UINT("tcp_port",        CYAML_FLAG_DEFAULT, config_t, tcp_port),
    CYAML_FIELD_END
};

static const cyaml_schema_value_t config_schema = {
    CYAML_VALUE_MAPPING(CYAML_FLAG_POINTER, config_t, config_fields_schema)
};

static const cyaml_config_t cyaml_config = {
    .log_fn = cyaml_log,
    .mem_fn = cyaml_mem,
    .log_level = CYAML_LOG_WARNING,
};

config_t *config;

bool config_load(const char *filename) {
    cyaml_err_t err;

    err = cyaml_load_file(filename, &cyaml_config, &config_schema, (void **) &config, NULL);

    if (err != CYAML_OK) {
        syslog(LOG_ERR, "%s", cyaml_strerror(err));
        return false;
    }

    return true;
}
