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
git clone https://github.com/yourusername/ESP32P4-JC4880P443C.git
cd ESP32P4-JC4880P443C
idf.py set-target esp32p4
idf.py build flash monitor