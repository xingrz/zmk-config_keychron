# Copyright (c) 2023 XiNGRZ
# SPDX-License-Identifier: MIT

zephyr_sources_ifdef(CONFIG_BLE_EXT_CKBT51 ckbt51.c)

zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
