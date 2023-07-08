/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_ckled2001

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led_strip.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ckled2001, CONFIG_LED_STRIP_LOG_LEVEL);

#define REG_SET_CMD_PAGE  0xFD
#define LED_CONTROL_PAGE  0x00
#define LED_PWM_PAGE      0x01
#define FUNCTION_PAGE     0x03
#define CURRENT_TUNE_PAGE 0x04

#define REG_CONFIGRATION    0x00
#define MSKSW_SHUTDOWN_MODE 0x0
#define MSKSW_NORMAL_MODE   0x1

#define REG_PDU              0x13
#define MSKSET_CA_CB_CHANNEL 0xAA
#define MSKCLR_CA_CB_CHANNEL 0x00

#define REG_SCAN_PHASE         0x14
#define MSKPHASE_CHANNELS(cnt) (12 - cnt)

#define REG_SLEW_RATE_CONTROL_MODE1 0x15
#define MSKPWM_DELAY_PHASE_ENABLE   0x04
#define MSKPWM_DELAY_PHASE_DISABLE  0x00

#define REG_SLEW_RATE_CONTROL_MODE2                  0x16
#define MSKDRIVING_SINKING_CHHANNEL_SLEWRATE_ENABLE  0xC0
#define MSKDRIVING_SINKING_CHHANNEL_SLEWRATE_DISABLE 0x00

#define REG_SOFTWARE_SLEEP 0x1A
#define MSKSLEEP_ENABLE    0x02
#define MSKSLEEP_DISABLE   0x00

#define COUNT_BETWEEN(a, b) ((b - a) + 1)

#define LED_CONTROL_CNT  24
#define LED_PWM_CNT      192
#define CURRENT_TUNE_CNT 12

struct ckled2001_channel_map {
	uint8_t ch_r;
	uint8_t ch_g;
	uint8_t ch_b;
};

struct ckled2001_config {
	struct i2c_dt_spec bus;
	uint32_t init_delay_us;
	uint8_t scan_phase_channels;
	const struct ckled2001_channel_map *map;
	uint32_t map_cnt;
};

struct ckled2001_data {
	uint8_t pwm_buffer[LED_PWM_CNT];
};

static inline int ckled2001_write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct ckled2001_config *config = dev->config;

	return i2c_burst_write_dt(&config->bus, reg, &value, 1);
}

static inline int ckled2001_set_control(const struct device *dev, uint8_t value)
{
	ckled2001_write_reg(dev, REG_SET_CMD_PAGE, LED_CONTROL_PAGE);

	for (int i = 0; i < LED_CONTROL_CNT; i++) {
		ckled2001_write_reg(dev, i, value);
	}

	return 0;
}

static inline int ckled2001_flush_pwm_buffer(const struct device *dev)
{
	const struct ckled2001_config *config = dev->config;
	struct ckled2001_data *data = dev->data;

	ckled2001_write_reg(dev, REG_SET_CMD_PAGE, LED_PWM_PAGE);

	return i2c_burst_write_dt(&config->bus, 0, data->pwm_buffer, LED_PWM_CNT);
}

static int ckled2001_update_rgb(const struct device *dev, struct led_rgb *pixels, size_t num_pixels)
{
	const struct ckled2001_config *config = dev->config;
	struct ckled2001_data *data = dev->data;

	if (num_pixels > config->map_cnt) {
		num_pixels = config->map_cnt;
	}

	for (size_t i = 0; i < num_pixels; i++) {
		data->pwm_buffer[config->map[i].ch_r] = pixels[i].r;
		data->pwm_buffer[config->map[i].ch_g] = pixels[i].g;
		data->pwm_buffer[config->map[i].ch_b] = pixels[i].b;
	}

	return ckled2001_flush_pwm_buffer(dev);
}

static int ckled2001_update_channels(const struct device *dev, uint8_t *channels,
				     size_t num_channels)
{
	return 0;
}

static int ckled2001_init(const struct device *dev)
{
	const struct ckled2001_config *config = dev->config;

	LOG_INF("Loaded %d channel mappings", config->map_cnt);

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C bus not ready: %s", config->bus.bus->name);
		return -ENODEV;
	}

	k_usleep(config->init_delay_us);

	/* Set functions */
	ckled2001_write_reg(dev, REG_SET_CMD_PAGE, FUNCTION_PAGE);
	ckled2001_write_reg(dev, REG_CONFIGRATION, MSKSW_SHUTDOWN_MODE);
	ckled2001_write_reg(dev, REG_PDU, MSKSET_CA_CB_CHANNEL);
	ckled2001_write_reg(dev, REG_SCAN_PHASE, MSKPHASE_CHANNELS(config->scan_phase_channels));
	ckled2001_write_reg(dev, REG_SLEW_RATE_CONTROL_MODE1, MSKPWM_DELAY_PHASE_ENABLE);
	ckled2001_write_reg(dev, REG_SLEW_RATE_CONTROL_MODE2,
			    MSKDRIVING_SINKING_CHHANNEL_SLEWRATE_ENABLE);
	ckled2001_write_reg(dev, REG_SOFTWARE_SLEEP, MSKSLEEP_DISABLE);

	/* Turn off all LEDs */
	ckled2001_set_control(dev, 0x00);

	/* Init PWM page */
	ckled2001_flush_pwm_buffer(dev);

	/* Init current page */
	ckled2001_write_reg(dev, REG_SET_CMD_PAGE, CURRENT_TUNE_PAGE);
	for (int i = 0; i < CURRENT_TUNE_CNT; i++) {
		switch (i) {
		case 2:
		case 5:
		case 8:
		case 11:
			ckled2001_write_reg(dev, i, 0xA0);
			break;
		default:
			ckled2001_write_reg(dev, i, 0xFF);
			break;
		}
	}

	/* Turn on all LEDs */
	ckled2001_set_control(dev, 0xFF);

	/* Set to normal mode */
	ckled2001_write_reg(dev, REG_SET_CMD_PAGE, FUNCTION_PAGE);
	ckled2001_write_reg(dev, REG_CONFIGRATION, MSKSW_NORMAL_MODE);

	return 0;
}

static const struct led_strip_driver_api ckled2001_api = {
	.update_rgb = ckled2001_update_rgb,
	.update_channels = ckled2001_update_channels,
};

#define CKLED2001_INIT(n)                                                                          \
	static struct ckled2001_data ckled2001_data_##n;                                           \
                                                                                                   \
	static const uint8_t ckled2001_channel_map##n[] = DT_INST_PROP(n, map);                    \
                                                                                                   \
	static const struct ckled2001_config ckled2001_config_##n = {                              \
		.bus = I2C_DT_SPEC_INST_GET(n),                                                    \
		.init_delay_us = DT_INST_PROP(n, init_delay_us),                                   \
		.scan_phase_channels = DT_INST_PROP_OR(n, scan_phase_channels, 12),                \
		.map = (const struct ckled2001_channel_map *)ckled2001_channel_map##n,             \
		.map_cnt = DT_INST_PROP_LEN(n, map) / 3,                                           \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &ckled2001_init, NULL, &ckled2001_data_##n,                       \
			      &ckled2001_config_##n, POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY,  \
			      &ckled2001_api);

DT_INST_FOREACH_STATUS_OKAY(CKLED2001_INIT);
