# Firmware Build for ESP32-C6 Coprocessor

The ESP32-C6 on this board acts as a Wi-Fi coprocessor connected to the ESP32-P4 via a 4-bit SDIO interface. If the C6 firmware is outdated or missing, you need to build and flash it manually using the `esp-hosted-mcu` repository.

> **Important:** Use [`esp-hosted-mcu`](https://github.com/espressif/esp-hosted-mcu) — not `esp-hosted`, which is the Linux host variant. They are different projects.

---

## Prerequisites

- ESP-IDF v5.5.2 or later installed and sourced
- A USB-to-UART adapter (CP2102, CH340, FT232, etc.) at **3.3V logic**
- `esptool.py` installed (`pip install esptool`)

---

## Step 1 — Clone the Repository

```bash
git clone https://github.com/espressif/esp-hosted-mcu.git
cd esp-hosted-mcu/slave
```

---

## Step 2 — Build the Firmware

```bash
idf.py set-target esp32c6
idf.py build
```

The build output will be in the `build/` directory.

---

## Step 3 — Wire Up the UART Adapter

To flash the C6 coprocessor, connect a USB-to-UART adapter to the C6 test pads on the board:

| C6 Test Pad | USB-UART Adapter |
|-------------|-----------------|
| C6_U0RXD    | TX              |
| C6_U0TXD    | RX              |
| GND         | GND             |
| C6_IO9      | GND (hold LOW to enter download mode) |

> **Do not** connect the adapter's 3.3V — the board is powered via its own USB connection. Only share GND.

---

## Step 4 — Enter Download Mode

Pull **C6_IO9 LOW** (tie to GND) **before** powering on or resetting the board. The C6 will boot into its UART bootloader instead of running firmware.

---

## Step 5 — Erase the C6 Flash

```bash
esptool.py --chip esp32c6 --port /dev/ttyUSB0 erase_flash
```

Replace `/dev/ttyUSB0` with your actual port (`COMx` on Windows, `/dev/cu.usbserialXXXX` on macOS).

---

## Step 6 — Flash the Firmware

```bash
esptool.py --chip esp32c6 --port /dev/ttyUSB0 --baud 460800 write_flash \
  0x0     build/bootloader/bootloader.bin \
  0x8000  build/partition_table/partition-table.bin \
  0x10000 build/network_adapter.bin
```

Use `--baud 115200` if you encounter errors at higher speed.

---

## Step 7 — Release and Reboot

Once flashing completes:

1. Disconnect **C6_IO9 from GND**
2. Reset or power-cycle the board
3. The C6 will boot into the new firmware and the P4 should connect to it over SDIO

---

## Troubleshooting

**`sdio_card_fn_init failed` on the P4**
The P4 host firmware and C6 slave firmware must be built from compatible versions of `esp-hosted-mcu`. Check the P4 serial output at boot for the expected slave version and ensure your clone matches.

**Flash fails at high baud rate**
Lower the baud rate: replace `--baud 460800` with `--baud 115200`.

**C6 not detected by esptool**
Make sure C6_IO9 is held LOW before power-on, not after. Also verify TX/RX are not swapped on your adapter.
