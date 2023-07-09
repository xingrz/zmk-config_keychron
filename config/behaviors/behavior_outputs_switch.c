/*
 * Copyright (c) 2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_outputs_switch

#include <zephyr/device.h>

#include <drivers/behavior.h>

#include <dt-bindings/zmk/outputs.h>

#include <zmk/behavior.h>
#include <zmk/endpoints.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int osw_init(const struct device *dev)
{
	return 0;
}

static int osw_switch(uint32_t output)
{
	LOG_DBG("Selecting output %d", output);

	switch (output) {
	case OUT_USB:
		return zmk_endpoints_select(ZMK_ENDPOINT_USB);
	case OUT_BLE:
		return zmk_endpoints_select(ZMK_ENDPOINT_BLE);
	default:
		LOG_ERR("Unknown output command: %d", output);
	}

	return -ENOTSUP;
}

static int osw_keymap_binding_pressed(struct zmk_behavior_binding *binding,
				      struct zmk_behavior_binding_event event)
{
	return osw_switch(binding->param2);
}

static int osw_keymap_binding_released(struct zmk_behavior_binding *binding,
				       struct zmk_behavior_binding_event event)
{
	return osw_switch(binding->param1);
}

static const struct behavior_driver_api behavior_mo_driver_api = {
	.binding_pressed = osw_keymap_binding_pressed,
	.binding_released = osw_keymap_binding_released,
};

DEVICE_DT_INST_DEFINE(0, osw_init, NULL, NULL, NULL, APPLICATION,
		      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_mo_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
