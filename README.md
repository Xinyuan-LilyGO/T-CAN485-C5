<h1 align="center">T-CAN485-C5</h1>

## **English** | [Chinese](./README_CN.md)

[![License](https://img.shields.io/github/license/Xinyuan-LilyGO/T-CAN485-C5?style=flat-square)](./LICENSE)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.x%2B-ff6f00?style=flat-square)](https://github.com/espressif/esp-idf)
[![C++](https://img.shields.io/badge/C%2B%2B-17%2B-00599c?style=flat-square)](https://isocpp.org/)

## Overview

T-CAN485-C5 is an ESP32-C5 development board for industrial and embedded communication. It integrates CAN and RS485 interfaces, W5500-based Ethernet, a microSD card slot, USB-C, an ADC input, an optocoupler-isolated digital input, and an addressable RGB LED.

## Directory

- [Overview](#overview)
- [Device Versions](#device-versions)
- [Notes](#notes)
- [Preview](#preview)
- [Supported Frameworks](#supported-frameworks)
- [Quick Start](#quick-start)
- [Hardware Modules](#hardware-modules)
- [Pin Overview](#pin-overview)
- [Project Materials](#project-materials)
- [FAQ](#faq)

## Device Versions

| Version | Date | Description |
| :---: | :---: | --- |
| T-CAN485-C5 V1.0 | 2026-06-12 | Initial hardware version |

## Notes

- Connect CAN and RS485 wiring to the matching labeled terminal and observe the bus polarity.
- The optocoupler input is active low in the example firmware. Refer to the schematic for its supported input range and wiring.
- The W5500 Ethernet interface is for data communication; this board does not support Power over Ethernet (PoE). Do not inject power through the Ethernet cable.

## Preview

See the [T-CAN485-C5 V1.0 schematic](./project/T-CAN485-C5-V1.0.pdf) for the board layout and connectors.

## Supported Frameworks

| Framework | Status | Version |
| --- | --- | --- |
| ESP-IDF | Recommended | ESP-IDF 5.x |

## Quick Start

### Build With ESP-IDF

Install ESP-IDF first. For environment setup, refer to the official [ESP32-C5 Get Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c5/get-started/index.html).

Clone the repository together with its submodules:

```bash
git clone --recursive https://github.com/Xinyuan-LilyGO/T-CAN485-C5.git
cd T-CAN485-C5
idf.py set-target esp32c5
idf.py menuconfig
idf.py build
idf.py flash monitor
```

Select an example in `menuconfig`:

```text
Example Configuration
`-- Select the example to build
```

### Examples

| Example Link | Description |
| --- | --- |
| [`general_test`](./main/examples/general_test) | General board test for networking, storage, inputs, LED, CAN, and RS485 |
| [`rs485_can`](./main/examples/rs485_can) | CAN and RS485 send/receive test |
| [`ethernet_iperf`](./main/examples/ethernet_iperf) | W5500 Ethernet throughput test |
| [`sd`](./main/examples/sd) | microSD card monitor |
| [`adc_digital_input`](./main/examples/adc_digital_input) | ADC and optocoupler input test |
| [`ws2812`](./main/examples/ws2812) | RGB LED test |

### Prebuilt Firmware

The following general-test firmware is provided:

| Firmware | Flash Address | Description |
| --- | --- | --- |
| [`general_test`](<./firmware/[t-can485-c5][general_test]/[t-can485-c5][general_test]_firmware_202608271614.bin>) | `0x0` | Prebuilt general board test firmware |

## Hardware Modules

### T-CAN485-C5 V1.0

| Function | Device | Interface |
| --- | --- | --- |
| Main MCU | ESP32-C5-WROOM-1U-N16R8 | — |
| CAN transceiver | SN65HVD251DR | TWAI |
| RS485 transceiver | MAX13487E | UART |
| Digital isolators | ADuM121N | UART signal isolation |
| Ethernet controller | W5500 | SPI |
| Storage | microSD card slot | SPI |
| Digital input | Optocoupler-isolated input | GPIO |
| Analog input | ADC input | ADC |
| RGB LED | WS2812B-MINI-V6 | GPIO / RMT |

- Related documents:
  - [ESP32-C5-WROOM-1/1U product page and datasheets](https://www.espressif.com/en/products/modules/esp32-c5-wroom-1)
  - [SN65HVD251 datasheet](https://www.ti.com/lit/ds/symlink/sn65hvd251.pdf)
  - [MAX13487E datasheet](./docs/MAX13487E_Datasheet.pdf)
  - [W5500 datasheet](./docs/W5500_Datasheet.pdf)
  - [T-CAN485-C5 V1.0 schematic](./project/T-CAN485-C5-V1.0.pdf)

## Pin Overview

Board pin definitions are available in [`config.h`](https://github.com/Xinyuan-LilyGO/lilygo_device_driver/blob/main/src/device/t_can485_c5/config.h).

## Project Materials

| Document | Description |
| --- | --- |
| [`T-CAN485-C5 V1.0`](./project/T-CAN485-C5-V1.0.pdf) | Board schematic |
| [`docs`](./docs) | Component datasheets used by the board |
| [`firmware`](./firmware) | Prebuilt firmware and version notes |

## FAQ

<details>
<summary>Q. Why does the board fail to flash?</summary>

A. Hold down the `BOOT` button while starting the download, then release it after flashing begins. Select `ESP32-C5` as the target.

</details>

<details>
<summary>Q. How do I select a different example?</summary>

A. Run `idf.py menuconfig`, open `Example Configuration`, select the desired example, save, and rebuild the project.

</details>

<details>
<summary>Q. Why are submodule headers or components missing?</summary>

A. Initialize all repository submodules:

```bash
git submodule update --init --recursive
```

</details>
