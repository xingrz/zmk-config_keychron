# Copyright (c) 2022-2023 XiNGRZ
# SPDX-License-Identifier: MIT

zephyr_sources_ifdef(CONFIG_LED_STRIP_COMPOSITE led_strip_composite.c)
zephyr_sources_ifdef(CONFIG_CKLED2001 ckled2001.c)
