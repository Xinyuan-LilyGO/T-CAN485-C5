<h1 align="center">T-CAN485-C5</h1>

## [英语](./README.md) | **中文**

[![License](https://img.shields.io/github/license/Xinyuan-LilyGO/T-CAN485-C5?style=flat-square)](./LICENSE)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.x%2B-ff6f00?style=flat-square)](https://github.com/espressif/esp-idf)
[![C++](https://img.shields.io/badge/C%2B%2B-17%2B-00599c?style=flat-square)](https://isocpp.org/)

## 概述

T-CAN485-C5 是一款基于 ESP32-C5 的工业与嵌入式通信开发板，集成 CAN 和 RS485 通信接口、基于 W5500 的以太网、microSD 卡槽、USB-C、ADC 输入、光耦隔离数字输入及可编程 RGB LED。

## 目录

- [概述](#概述)
- [设备版本](#设备版本)
- [注意事项](#注意事项)
- [预览](#预览)
- [支持框架](#支持框架)
- [快速开始](#快速开始)
- [硬件模块](#硬件模块)
- [引脚总览](#引脚总览)
- [项目资料](#项目资料)
- [常见问题](#常见问题)

## 设备版本

| 版本 | 日期 | 说明 |
| :---: | :---: | --- |
| T-CAN485-C5 V1.0 | 2026-06-12 | 初始硬件版本 |

## 注意事项

- 请将 CAN 和 RS485 总线接至对应标识的端子，并注意信号极性。
- 示例固件中的光耦输入为低电平有效。输入范围及接线方式请参照原理图。
- W5500 以太网接口仅用于数据通信，本板不支持 PoE。请勿通过网线向设备注入电源。

## 预览

板卡布局和连接器请参见 [T-CAN485-C5 V1.0 原理图](./project/T-CAN485-C5-V1.0.pdf)。

## 支持框架

| 框架 | 状态 | 版本 |
| --- | --- | --- |
| ESP-IDF | 推荐 | ESP-IDF 5.x |

## 快速开始

### 使用 ESP-IDF 构建

请先安装 ESP-IDF。环境安装与配置请参考乐鑫官方 [ESP32-C5 入门指南](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c5/get-started/index.html)。

克隆仓库时请同时拉取子模块：

```bash
git clone --recursive https://github.com/Xinyuan-LilyGO/T-CAN485-C5.git
cd T-CAN485-C5
idf.py set-target esp32c5
idf.py menuconfig
idf.py build
idf.py flash monitor
```

在 `menuconfig` 中选择示例：

```text
Example Configuration
`-- Select the example to build
```

### 示例

| 示例链接 | 说明 |
| --- | --- |
| [`general_test`](./main/examples/general_test) | 板卡综合测试，包含网络、存储、输入、LED、CAN 和 RS485 |
| [`rs485_can`](./main/examples/rs485_can) | CAN 与 RS485 收发测试 |
| [`ethernet_iperf`](./main/examples/ethernet_iperf) | W5500 以太网吞吐测试 |
| [`sd`](./main/examples/sd) | microSD 卡监测 |
| [`adc_digital_input`](./main/examples/adc_digital_input) | ADC 与光耦输入测试 |
| [`ws2812`](./main/examples/ws2812) | RGB LED 测试 |

### 预编译固件

目前提供以下综合测试固件：

| 固件 | 烧录地址 | 说明 |
| --- | --- | --- |
| [`general_test`](<./firmware/[t-can485-c5][general_test]/[t-can485-c5][general_test]_firmware_202608271614.bin>) | 参见固件说明 | 板卡综合测试预编译固件；烧录前请确认固件格式和对应的 Flash 参数 |

## 硬件模块

### T-CAN485-C5 V1.0

| 功能 | 器件 | 接口 |
| --- | --- | --- |
| 主控 MCU | ESP32-C5-WROOM-1U-N16R8 | — |
| CAN 收发器 | SN65HVD251DR | TWAI |
| RS485 收发器 | MAX13487E | UART |
| 数字隔离器 | ADuM121N | UART 信号隔离 |
| 以太网控制器 | W5500 | SPI |
| 存储 | microSD 卡槽 | SPI |
| 数字输入 | 光耦隔离输入 | GPIO |
| 模拟输入 | ADC 输入 | ADC |
| RGB LED | WS2812B-MINI-V6 | GPIO / RMT |

- 相关资料：
  - [ESP32-C5-WROOM-1/1U 产品页面及数据手册](https://www.espressif.com/en/products/modules/esp32-c5-wroom-1)
  - [SN65HVD251 数据手册](https://www.ti.com/lit/ds/symlink/sn65hvd251.pdf)
  - [MAX13487E 数据手册](./docs/MAX13487E_Datasheet.pdf)
  - [W5500 数据手册](./docs/W5500_Datasheet.pdf)
  - [T-CAN485-C5 V1.0 原理图](./project/T-CAN485-C5-V1.0.pdf)

## 引脚总览

板级引脚定义请参见 [`config.h`](https://github.com/Xinyuan-LilyGO/lilygo_device_driver/blob/main/src/device/t_can485_c5/config.h)。

## 项目资料

| 文档 | 说明 |
| --- | --- |
| [`T-CAN485-C5 V1.0`](./project/T-CAN485-C5-V1.0.pdf) | 板卡原理图 |
| [`docs`](./docs) | 板上器件数据手册 |
| [`firmware`](./firmware) | 预编译固件及版本记录 |

## 常见问题

<details>
<summary>Q. 为什么开发板烧录失败？</summary>

A. 开始下载时按住 `BOOT` 按键，烧录开始后再松开。目标芯片请选择 `ESP32-C5`。

</details>

<details>
<summary>Q. 如何切换不同的示例？</summary>

A. 执行 `idf.py menuconfig`，打开 `Example Configuration`，选择需要的示例，保存后重新构建工程。

</details>

<details>
<summary>Q. 为什么缺少子模块头文件或组件？</summary>

A. 请初始化仓库的全部子模块：

```bash
git submodule update --init --recursive
```

</details>
