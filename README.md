zmk-config for Keychron Q1
========

![Keychron Q1](https://cdn.shopify.com/s/files/1/0059/0630/1017/t/5/assets/keychronq1qmkviacustommechanicalkeyboardrotartyencoderknobversionwithdoublegasketdesignscrewinpcbstabilizerandhotswappablesouthfacingrgbbareboneuslayout-1639996031362.png)

[Q1](https://www.keychron.com/products/keychron-q1) is a hotswap 75% mechanical keyboard made by Keychron.

This repository is the configuration used to build [ZMK firmware](https://github.com/zmkfirmware/zmk) for Keychron Q1 (Version 2, as well as the "sspai customized version").

## Flash

1. Download the latest build of ZMK firmware from [Releases](https://github.com/xingrz/zmk-config_keychron_q1/releases)
2. Enter DFU mode by holding the "RESET" button hidden under the space bar keycap and re-plug your USB cable
2. Open [WebDFU](https://devanlai.github.io/webdfu/dfu-util/), select the firmware, and then click "Download"

## Customize

You can fork this repository to your own GitHub account and do any modification.

### Keymap

This firmware tries to match the default keymap of the stock firmware as closely as possible, but you can customize it using the [ZMK Keymap Editor](https://nickcoutsos.github.io/keymap-editor/) or by manually editing the [`keychron_q1.keymap`](config/keychron_q1.keymap).

### Lights

* `Fn` + `Tab`: Toggle
* `Fn` + `Q`/`A`: Change effects
* `Fn` + `W`/`S`: Change brightness
* `Fn` + `E`/`D`: Change hue
* `Fn` + `R`/`F`: Change saturation
* `Fn` + `T`/`G`: Change animation speed

## Development

Follows [ZMK documentation](https://zmk.dev/docs/development/setup#prerequisites)。

```sh
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
export GNUARMEMB_TOOLCHAIN_PATH=/path/to/gcc-arm-none-eabi-10.3-2021.10

west init -l config
west update
west zephyr-export
west build -s zmk/app -b keychron_q1 -- -DZMK_CONFIG=$PWD/config
west flash
```

## Links

* [ZMK Firmware](https://zmk.dev/)

## License

[Apache License 2.0](LICENSE)
