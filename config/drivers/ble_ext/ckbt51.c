/*
 * Copyright (c) 2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT keychron_ckbt51

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ckbt51, CONFIG_ZMK_LOG_LEVEL);

#include <drivers/ble_ext.h>

#define CKBT51_MAX_PROFILE_COUNT   3
#define CKBT51_PAIRING_TIMEOUT_SEC 0
#define CKBT51_CONNECT_TIMEOUT_SEC 3

#define CKBT51_MAGIC   0xAA
#define CKBT51_CMD     0x55
#define CKBT51_CMD_ACK 0x56
#define CKBT51_EVT     0x57

enum ckbt51_command {
	/* HID Report  */
	CKBT51_CMD_SEND_KB = 0x11,
	CKBT51_CMD_SEND_KB_NKRO = 0x12,
	CKBT51_CMD_SEND_CONSUMER = 0x13,
	CKBT51_CMD_SEND_SYSTEM = 0x14,
	CKBT51_CMD_SEND_BOOT_KB = 0x17,

	/* Bluetooth connections */
	CKBT51_CMD_PAIRING = 0x21,
	CKBT51_CMD_CONNECT = 0x22,
	CKBT51_CMD_DISCONNECT = 0x23,
	CKBT51_CMD_SWITCH_HOST = 0x24,
	CKBT51_CMD_READ_STATE_REG = 0x25,

	/* Battery */
	CKBT51_CMD_BATTERY_MANAGE = 0x31,
	CKBT51_CMD_UPDATE_BAT_LVL = 0x32,

	/* Set/get parameters */
	CKBT51_CMD_GET_MODULE_INFO = 0x40,
	CKBT51_CMD_SET_CONFIG = 0x41,
	CKBT51_CMD_GET_CONFIG = 0x42,
	CKBT51_CMD_SET_BDA = 0x43,
	CKBT51_CMD_GET_BDA = 0x44,
	CKBT51_CMD_SET_NAME = 0x45,
	CKBT51_CMD_GET_NAME = 0x46,

	/* DFU */
	CKBT51_CMD_GET_DFU_VER = 0x60,
	CKBT51_CMD_HAND_SHAKE_TOKEN = 0x61,
	CKBT51_CMD_START_DFU = 0x62,
	CKBT51_CMD_SEND_FW_DATA = 0x63,
	CKBT51_CMD_VERIFY_CRC32 = 0x64,
	CKBT51_CMD_SWITCH_FW = 0x65,

	/* Factory test */
	CKBT51_CMD_FACTORY_RESET = 0x71,
	CKBT51_CMD_INT_PIN_TEST = 0x72,
	CKBT51_CMD_RADIO_TEST = 0x73,

	/* Event */
	CKBT51_EVT_CKBT51_CMD_RECEIVED = 0xA1,
	CKBT51_EVT_OTA_RSP = 0xA3,
	CKBT51_CONNECTION_EVT_ACK = 0xA4,
};

enum ckbt51_event {
	CKBT51_EVT_ACK = 0xA1,
	CKBT51_EVT_QUERY_RSP = 0xA2,
	CKBT51_EVT_RESET = 0xB0,
	CKBT51_EVT_LE_CONNECTION = 0xB1,
	CKBT51_EVT_HOST_TYPE = 0xB2,
	CKBT51_EVT_CONNECTION = 0xB3,
	CKBT51_EVT_HID_EVENT = 0xB4,
	CKBT51_EVT_BATTERY = 0xB5,
};

enum ckbt51_connection_state {
	CKBT51_CONNECTED = 0x20,
	CKBT51_DISCOVERABLE = 0x21,
	CKBT51_RECONNECTING = 0x22,
	CKBT51_DISCONNECTED = 0x23,
	CKBT51_PINCODE_ENTRY = 0x24,
	CKBT51_EXIT_PINCODE_ENTRY = 0x25
};

enum ckbt51_pairing_mode {
	PAIRING_MODE_DEFAULT = 0x00,
	PAIRING_MODE_JUST_WORK,
	PAIRING_MODE_PASSKEY_ENTRY,
	PAIRING_MODE_LESC_OR_SSP,
	PAIRING_MODE_INVALID
};

enum ckbt51_bt_mode {
	BT_MODE_DEFAUL,
	BT_MODE_CLASSIC,
	BT_MODE_LE,
	BT_MODE_INVALID,
};

struct ckbt51_profile {
	enum ble_ext_state state;
};

struct ckbt51_config {
	const struct device *uart_dev;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec wakeup_gpio;
	uint32_t wakeup_delay_us;
};

struct ckbt51_data {
	uint8_t selected_profile;
	ble_ext_state_callback_t state_callback;

	uint8_t trans_id;

	K_THREAD_STACK_MEMBER(rx_thread_stk, CONFIG_BLE_EXT_CKBT51_THREAD_STACK_SIZE);
	struct k_thread rx_thread;
	uint8_t rx_buf[64];
	struct ring_buf rx_rb;

	struct ckbt51_profile profiles[CKBT51_MAX_PROFILE_COUNT];
};

static inline uint16_t ckbt51_checksum(uint8_t cmd, uint8_t *payload, size_t len)
{
	uint16_t checksum = cmd;
	int i;

	for (i = 0; i < len; i++) {
		checksum += payload[i];
	}

	return checksum;
}

static int ckbt51_send_cmd(const struct device *dev, enum ckbt51_command cmd, uint8_t *payload,
			   size_t len, bool expect_ack)
{
	const struct ckbt51_config *config = dev->config;
	struct ckbt51_data *data = dev->data;

	uint16_t checksum;
	int i;

	data->trans_id = ((uint16_t)data->trans_id + 1) & 0xFF;
	if (data->trans_id == 0) {
		data->trans_id = 1;
	}

	/* Wake up module */
	for (i = 0; i < 3; i++) {
		gpio_pin_toggle_dt(&config->wakeup_gpio);
		k_usleep(config->wakeup_delay_us / 3);
	}
	gpio_pin_set_dt(&config->wakeup_gpio, 0);

	/* Calculate checksum */
	checksum = ckbt51_checksum(cmd, payload, len);

	/* Send command */
	uart_poll_out(config->uart_dev, CKBT51_MAGIC);
	uart_poll_out(config->uart_dev, expect_ack ? CKBT51_CMD_ACK : CKBT51_CMD);
	uart_poll_out(config->uart_dev, (len + 3) & 0xFF);
	uart_poll_out(config->uart_dev, ~(len + 3) & 0xFF);
	uart_poll_out(config->uart_dev, data->trans_id);
	uart_poll_out(config->uart_dev, cmd);
	for (i = 0; i < len; i++) {
		uart_poll_out(config->uart_dev, payload[i]);
	}
	uart_poll_out(config->uart_dev, checksum & 0xFF);
	uart_poll_out(config->uart_dev, (checksum >> 8) & 0xFF);

	LOG_DBG("Sent command 0x%02x, payload size: %d, trans_id: %d, checksum: %04x", cmd, len,
		data->trans_id, checksum);

	return 0;
}

static int ckbt51_select_profile(const struct device *dev, uint8_t profile)
{
	struct ckbt51_data *data = dev->data;

	if (profile >= CKBT51_MAX_PROFILE_COUNT) {
		return -EINVAL;
	}

	data->selected_profile = profile;

	return 0;
}

static int ckbt51_pair(const struct device *dev)
{
	struct ckbt51_data *data = dev->data;

	LOG_DBG("Making profile %d discoverable", data->selected_profile);

	uint8_t payload[] = {
		data->selected_profile,                   /* host index */
		(CKBT51_PAIRING_TIMEOUT_SEC >> 0) & 0xFF, /* timeout (LSB) */
		(CKBT51_PAIRING_TIMEOUT_SEC >> 8) & 0xFF, /* timeout (MSB) */
		PAIRING_MODE_LESC_OR_SSP,                 /* pairing mode */
		BT_MODE_CLASSIC,                          /* BT mode  */
		0,                                        /* TX power (LE only) */
	};

	return ckbt51_send_cmd(dev, CKBT51_CMD_PAIRING, payload, sizeof(payload), true);
}

static int ckbt51_connect(const struct device *dev)
{
	struct ckbt51_data *data = dev->data;

	LOG_DBG("Connection to host with profile %d", data->selected_profile);

	uint8_t payload[] = {
		data->selected_profile,                   /* host index */
		(CKBT51_CONNECT_TIMEOUT_SEC >> 0) & 0xFF, /* timeout (LSB) */
		(CKBT51_CONNECT_TIMEOUT_SEC >> 8) & 0xFF, /* timeout (MSB) */
	};

	return ckbt51_send_cmd(dev, CKBT51_CMD_CONNECT, payload, sizeof(payload), true);
}

static int ckbt51_get_state(const struct device *dev, enum ble_ext_state *state)
{
	struct ckbt51_data *data = dev->data;

	*state = data->profiles[data->selected_profile].state;

	return 0;
}

static int ckbt51_set_state_callback(const struct device *dev, ble_ext_state_callback_t callback)
{
	struct ckbt51_data *data = dev->data;

	data->state_callback = callback;

	return 0;
}

static int ckbt51_send_keyboard_report(const struct device *dev,
				       struct zmk_hid_keyboard_report_body *body)
{
	struct ckbt51_data *data = dev->data;
	uint8_t buf[1 + sizeof(body->keys)];
	size_t len = MIN(sizeof(buf), 20);

	if (data->profiles[data->selected_profile].state != BLE_EXT_STATE_CONNECTED) {
		return -ENODEV;
	}

	buf[0] = body->modifiers;
	memcpy(buf + 1, body->keys, sizeof(body->keys));

	LOG_HEXDUMP_DBG(body, sizeof(buf), "Sending keyboard report");

#if defined(CONFIG_ZMK_HID_REPORT_TYPE_NKRO)
	return ckbt51_send_cmd(dev, CKBT51_CMD_SEND_KB_NKRO, buf, len, true);
#else
	return ckbt51_send_cmd(dev, CKBT51_CMD_SEND_KB, buf, len, true);
#endif
}

static int ckbt51_send_consumer_report(const struct device *dev,
				       struct zmk_hid_consumer_report_body *body)
{
	struct ckbt51_data *data = dev->data;

	if (data->profiles[data->selected_profile].state != BLE_EXT_STATE_CONNECTED) {
		return -ENODEV;
	}

	LOG_HEXDUMP_DBG(body, sizeof(struct zmk_hid_consumer_report_body),
			"Sending consumer report");

	return ckbt51_send_cmd(dev, CKBT51_CMD_SEND_CONSUMER, (uint8_t *)body,
			       sizeof(struct zmk_hid_consumer_report_body), true);
}

static void ckbt51_apply_state(const struct device *dev, enum ble_ext_state state)
{
	struct ckbt51_data *data = dev->data;

	if (data->profiles[data->selected_profile].state == state) {
		return;
	}

	data->profiles[data->selected_profile].state = state;

	if (data->state_callback) {
		data->state_callback(dev, data->selected_profile, state);
	}
}

static void ckbt51_handle_connection_evt(const struct device *dev, uint8_t *payload, size_t len,
					 uint8_t trans_id)
{
	LOG_DBG("Connection state: %02x", payload[0]);

	ckbt51_send_cmd(dev, CKBT51_CONNECTION_EVT_ACK, NULL, 0, false);

	switch ((enum ckbt51_connection_state)payload[0]) {
	case CKBT51_CONNECTED:
		LOG_DBG("Connected");
		ckbt51_apply_state(dev, BLE_EXT_STATE_CONNECTED);
		break;
	case CKBT51_DISCOVERABLE:
		LOG_DBG("Pairing");
		ckbt51_apply_state(dev, BLE_EXT_STATE_PAIRING);
		break;
	case CKBT51_RECONNECTING:
		LOG_DBG("Reconnecting");
		break;
	case CKBT51_DISCONNECTED:
		LOG_DBG("Idle");
		ckbt51_apply_state(dev, BLE_EXT_STATE_IDLE);
		break;
	default:
		/* Not handled */
		break;
	}
}

static int ckbt51_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = (const struct device *)p1;
	struct ckbt51_data *data = dev->data;
	uint8_t buf[32];
	uint8_t len;
	uint8_t trans_id;
	enum ckbt51_event evt;
	uint16_t checksum;

	while (1) {
		/* Read magic and check */
		if (!ring_buf_get(&data->rx_rb, buf, 1)) {
			k_usleep(10);
			continue;
		}

		if (buf[0] != CKBT51_MAGIC) {
			continue;
		}

		/* Read head and check */
		if (!ring_buf_get(&data->rx_rb, buf + 1, 5)) {
			continue;
		}

		if (buf[1] != CKBT51_EVT) {
			continue;
		}

		if ((~buf[2] & 0xFF) != buf[3]) {
			continue;
		}

		len = buf[2] - 3;
		trans_id = buf[4];
		evt = buf[5];

		/* Read payload + checksum  */
		if (!ring_buf_get(&data->rx_rb, buf, len + 3)) {
			continue;
		}

		checksum = ckbt51_checksum(evt, buf, len);
		if (checksum != (buf[len] | (buf[len + 1] << 8))) {
			continue;
		}

		LOG_DBG("Received event 0x%02x, payload size: %d, trans_id: %d, checksum: %04x",
			evt, len, trans_id, checksum);
		LOG_HEXDUMP_DBG(buf, len, "Received payload data:");

		switch (evt) {
		case CKBT51_EVT_CONNECTION:
			ckbt51_handle_connection_evt(dev, buf, len, trans_id);
			break;
		default: {
			/* Nothing to do */
		} break;
		}
	}

	return 0;
}

static void ckbt51_uart_rx_isr(const struct device *uart, void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct ckbt51_data *data = dev->data;

	int ret;
	uint8_t *buf;
	uint32_t bytes_allocated = 0;
	uint32_t bytes_read = 0;

	while (uart_irq_update(uart) && uart_irq_rx_ready(uart)) {
		if (!bytes_allocated) {
			bytes_allocated = ring_buf_put_claim(&data->rx_rb, &buf, UINT32_MAX);
		}

		if (!bytes_allocated) {
			LOG_ERR("Ring buffer full");
			break;
		}

		ret = uart_fifo_read(uart, buf, bytes_allocated);
		if (ret <= 0) {
			continue;
		}

		buf += ret;
		bytes_read += ret;
		bytes_allocated -= ret;
	}

	ring_buf_put_finish(&data->rx_rb, bytes_read);
}

static int ckbt51_init(const struct device *dev)
{
	const struct ckbt51_config *config = dev->config;
	struct ckbt51_data *data = dev->data;

	if (!device_is_ready(config->uart_dev)) {
		LOG_ERR("UART device %s is not ready", config->uart_dev->name);
		return -ENODEV;
	}

	gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT | GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&config->wakeup_gpio, GPIO_OUTPUT | GPIO_OUTPUT_INACTIVE);

	ring_buf_init(&data->rx_rb, sizeof(data->rx_buf), data->rx_buf);

	uart_irq_rx_disable(config->uart_dev);
	uart_irq_tx_disable(config->uart_dev);

	/* Drain the fifo */
	uint8_t c;
	while (uart_fifo_read(config->uart_dev, &c, 1)) {
		continue;
	}

	uart_irq_callback_user_data_set(config->uart_dev, ckbt51_uart_rx_isr, (void *)dev);
	uart_irq_rx_enable(config->uart_dev);

	k_thread_create(&data->rx_thread, data->rx_thread_stk,
			CONFIG_BLE_EXT_CKBT51_THREAD_STACK_SIZE, (k_thread_entry_t)ckbt51_rx_thread,
			(void *)dev, 0, NULL, K_PRIO_COOP(CONFIG_BLE_EXT_CKBT51_THREAD_PRIORITY), 0,
			K_NO_WAIT);

	gpio_pin_set_dt(&config->reset_gpio, 1);
	k_usleep(100);
	gpio_pin_set_dt(&config->reset_gpio, 0);

	return 0;
}

static const struct ble_ext_driver_api ckbt51_api = {
	.select_profile = ckbt51_select_profile,
	.pair = ckbt51_pair,
	.connect = ckbt51_connect,
	.get_state = ckbt51_get_state,
	.set_state_callback = ckbt51_set_state_callback,
	.send_keyboard_report = ckbt51_send_keyboard_report,
	.send_consumer_report = ckbt51_send_consumer_report,
};

#define CKBT51_INIT(n)                                                                             \
	static struct ckbt51_data ckbt51_data_##n;                                                 \
                                                                                                   \
	static const struct ckbt51_config ckbt51_config_##n = {                                    \
		.uart_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, uart)),                               \
		.reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios),                               \
		.wakeup_gpio = GPIO_DT_SPEC_INST_GET(n, wakeup_gpios),                             \
		.wakeup_delay_us = DT_INST_PROP(n, wakeup_delay_us),                               \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ckbt51_init, NULL, &ckbt51_data_##n, &ckbt51_config_##n,          \
			      POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, &ckbt51_api);

DT_INST_FOREACH_STATUS_OKAY(CKBT51_INIT);
