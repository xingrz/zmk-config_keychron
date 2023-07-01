/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT ti_sn74hc595_bitbang

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sn74hc595_bitbang, CONFIG_GPIO_LOG_LEVEL);

#define OUTS_CNT  8
#define OUTS_MASK BIT_MASK(OUTS_CNT)

struct sn74hc595_bitbang_config {
	struct gpio_driver_config common;
	struct gpio_dt_spec ser_gpio;
	struct gpio_dt_spec srclk_gpio;
	struct gpio_dt_spec rclk_gpio;
};

struct sn74hc595_bitbang_data {
	struct gpio_driver_data common;
	uint32_t states;
};

static int sn74hc595_bitbang_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	if ((flags & GPIO_OPEN_DRAIN) != 0U) {
		return -ENOTSUP;
	}

	if ((flags & GPIO_OUTPUT) == 0U) {
		return -ENOTSUP;
	}

	return 0;
}

static int sn74hc595_bitbang_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	return -ENOTSUP;
}

static inline int sn74hc595_bitbang_pulse(const struct gpio_dt_spec *gpio)
{
	gpio_pin_set_dt(gpio, 1);
	gpio_pin_set_dt(gpio, 0);
	return 0;
}

static int sn74hc595_bitbang_update_state(const struct device *dev)
{
	const struct sn74hc595_bitbang_config *config = dev->config;
	struct sn74hc595_bitbang_data *data = dev->data;

	for (int i = OUTS_CNT - 1; i >= 0; i--) {
		gpio_pin_set_dt(&config->ser_gpio, (data->states >> i) & 0x1);
		sn74hc595_bitbang_pulse(&config->srclk_gpio);
	}

	sn74hc595_bitbang_pulse(&config->rclk_gpio);

	return 0;
}

static int sn74hc595_bitbang_set_marked_raw(const struct device *dev, gpio_port_pins_t mask,
					    gpio_port_value_t value)
{
	struct sn74hc595_bitbang_data *data = dev->data;

	value &= OUTS_MASK;
	data->states = (data->states & ~mask) | (value & mask);

	return sn74hc595_bitbang_update_state(dev);
}

static int sn74hc595_bitbang_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	struct sn74hc595_bitbang_data *data = dev->data;

	pins &= OUTS_MASK;
	data->states |= pins;

	return sn74hc595_bitbang_update_state(dev);
}

static int sn74hc595_bitbang_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	struct sn74hc595_bitbang_data *data = dev->data;

	pins &= OUTS_MASK;
	data->states &= ~pins;

	return sn74hc595_bitbang_update_state(dev);
}

static int sn74hc595_bitbang_toggle_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	struct sn74hc595_bitbang_data *data = dev->data;

	pins &= OUTS_MASK;
	uint32_t current = data->states & pins;
	data->states &= ~pins;
	data->states |= ~current;

	return sn74hc595_bitbang_update_state(dev);
}

static int sn74hc595_bitbang_init(const struct device *dev)
{
	const struct sn74hc595_bitbang_config *config = dev->config;
	struct sn74hc595_bitbang_data *data = dev->data;
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

	return sn74hc595_bitbang_update_state(dev);
}

static const struct gpio_driver_api sn74hc595_bitbang_gpio_api = {
	.pin_configure = sn74hc595_bitbang_configure,
	.port_get_raw = sn74hc595_bitbang_get_raw,
	.port_set_masked_raw = sn74hc595_bitbang_set_marked_raw,
	.port_set_bits_raw = sn74hc595_bitbang_set_bits_raw,
	.port_clear_bits_raw = sn74hc595_bitbang_clear_bits_raw,
	.port_toggle_bits = sn74hc595_bitbang_toggle_bits_raw,
};

#define SN74HC595_BITBANG_INIT(n)                                                                  \
	static struct sn74hc595_bitbang_data sn74hc595_bitbang_data_##n = {};                      \
                                                                                                   \
	static const struct sn74hc595_bitbang_config sn74hc595_bitbang_config_##n = {              \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = OUTS_MASK,                                        \
			},                                                                         \
		.ser_gpio = GPIO_DT_SPEC_INST_GET(n, ser_gpios),                                   \
		.srclk_gpio = GPIO_DT_SPEC_INST_GET(n, srclk_gpios),                               \
		.rclk_gpio = GPIO_DT_SPEC_INST_GET(n, rclk_gpios),                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &sn74hc595_bitbang_init, NULL, &sn74hc595_bitbang_data_##n,       \
			      &sn74hc595_bitbang_config_##n, POST_KERNEL,                          \
			      CONFIG_GPIO_SN74HC595_BITBANG_INIT_PRIORITY,                         \
			      &sn74hc595_bitbang_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(SN74HC595_BITBANG_INIT);
