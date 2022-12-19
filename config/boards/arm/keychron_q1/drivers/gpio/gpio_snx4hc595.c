/*
 * Copyright (c) 2022 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT ti_snx4hc595

#include <drivers/gpio.h>
#include <stdint.h>
#include <zephyr.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(snx4hc595);

#define OUTS_CNT 8
#define OUTS_MASK BIT_MASK(OUTS_CNT)

struct snx4hc595_config {
    struct gpio_driver_config common;
    struct gpio_dt_spec ser_gpio;
    struct gpio_dt_spec srclk_gpio;
    struct gpio_dt_spec rclk_gpio;
};

struct snx4hc595_data {
    struct gpio_driver_data common;
    uint32_t states;
};

static int snx4hc595_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags) {
    if ((flags & GPIO_OPEN_DRAIN) != 0U) {
        return -ENOTSUP;
    }

    if ((flags & GPIO_OUTPUT) == 0U) {
        return -ENOTSUP;
    }

    return 0;
}

static int snx4hc595_get_raw(const struct device *dev, gpio_port_value_t *value) {
    return -ENOTSUP;
}

static inline int _snx4hc595_pulse(const struct gpio_dt_spec *gpio) {
    gpio_pin_set_dt(gpio, 1);
    gpio_pin_set_dt(gpio, 0);
    return 0;
}

static int _snx4hc595_update_state(const struct device *dev) {
    const struct snx4hc595_config *config = dev->config;
    struct snx4hc595_data *data = dev->data;

    for (int i = OUTS_CNT - 1; i >= 0; i--) {
        gpio_pin_set_dt(&config->ser_gpio, (data->states >> i) & 0x1);
        _snx4hc595_pulse(&config->srclk_gpio);
    }

    _snx4hc595_pulse(&config->rclk_gpio);

    return 0;
}

static int snx4hc595_set_marked_raw(const struct device *dev, gpio_port_pins_t mask,
                                    gpio_port_value_t value) {
    struct snx4hc595_data *data = dev->data;

    value &= OUTS_MASK;
    data->states = (data->states & ~mask) | (value & mask);

    return _snx4hc595_update_state(dev);
}

static int snx4hc595_set_bits_raw(const struct device *dev, gpio_port_pins_t pins) {
    struct snx4hc595_data *data = dev->data;

    pins &= OUTS_MASK;
    data->states |= pins;

    return _snx4hc595_update_state(dev);
}

static int snx4hc595_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins) {
    struct snx4hc595_data *data = dev->data;

    pins &= OUTS_MASK;
    data->states &= ~pins;

    return _snx4hc595_update_state(dev);
}

static int snx4hc595_toggle_bits_raw(const struct device *dev, gpio_port_pins_t pins) {
    struct snx4hc595_data *data = dev->data;

    pins &= OUTS_MASK;
    uint32_t current = data->states & pins;
    data->states &= ~pins;
    data->states |= ~current;

    return _snx4hc595_update_state(dev);
}

static int snx4hc595_init(const struct device *dev) {
    const struct snx4hc595_config *config = dev->config;
    struct snx4hc595_data *data = dev->data;
    int ret;

    if (!device_is_ready(config->ser_gpio.port)) {
        LOG_ERR("Pin SER not ready");
        return -ENODEV;
    }

    if (!device_is_ready(config->srclk_gpio.port)) {
        LOG_ERR("Pin SRCLK not ready");
        return -ENODEV;
    }

    if (!device_is_ready(config->rclk_gpio.port)) {
        LOG_ERR("Pin RCLK not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->ser_gpio, GPIO_OUTPUT_HIGH);
    if (ret < 0) {
        LOG_ERR("Could not configure SER GPIO (%d)", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&config->srclk_gpio, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure SRCLK GPIO (%d)", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&config->rclk_gpio, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure RCLK GPIO (%d)", ret);
        return ret;
    }

    data->states = 0;

    return _snx4hc595_update_state(dev);
}

static const struct gpio_driver_api snx4hc595_gpio_api = {
    .pin_configure = snx4hc595_configure,
    .port_get_raw = snx4hc595_get_raw,
    .port_set_masked_raw = snx4hc595_set_marked_raw,
    .port_set_bits_raw = snx4hc595_set_bits_raw,
    .port_clear_bits_raw = snx4hc595_clear_bits_raw,
    .port_toggle_bits = snx4hc595_toggle_bits_raw,
};

#define SNX4HC595_DEVICE(n)                                                                        \
    static const struct snx4hc595_config snx4hc595_config_##n = {                                  \
        .common =                                                                                  \
            {                                                                                      \
                .port_pin_mask = OUTS_MASK,                                                        \
            },                                                                                     \
        .ser_gpio = GPIO_DT_SPEC_INST_GET(n, ser_gpios),                                           \
        .srclk_gpio = GPIO_DT_SPEC_INST_GET(n, srclk_gpios),                                       \
        .rclk_gpio = GPIO_DT_SPEC_INST_GET(n, rclk_gpios),                                         \
    };                                                                                             \
                                                                                                   \
    static struct snx4hc595_data snx4hc595_data_##n = {};                                          \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, &snx4hc595_init, NULL, &snx4hc595_data_##n, &snx4hc595_config_##n,    \
                          POST_KERNEL, CONFIG_GPIO_SNX4HC595_INIT_PRIORITY, &snx4hc595_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(SNX4HC595_DEVICE);
