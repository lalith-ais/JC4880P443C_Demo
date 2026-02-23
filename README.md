# ESP32-P4 + JC4880P443C (ST7701 480x800 MIPI-DSI Display)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A clean, properly-architected driver for the AliExpress JC4880P443C display module.

## Hardware
- **Display**: JC4880P443C with ST7701 controller
- **Resolution**: 480×800
- **Interface**: MIPI-DSI (2 data lanes)
- **MCU**: ESP32-P4

## Quick Start

```bash
git clone https://github.com/lalith-ais/JC4880P443C_Demo.git
cd JC4880P443C_Demo
idf.py -p /dev/ttyACM0 build flash monitor
