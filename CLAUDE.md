# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Sentinel** — drone-based RF direction-finding (radio bearing / пеленгація) system.

Two AD8302 RF phase/magnitude detector ICs each compare the signals from a pair of antennas. The ESP32-C6 reads the two AD8302 VPHS outputs via ADC, determines the strongest-signal bearing, and sends flight commands to the drone flight controller over MAVLink. By sweeping the drone left/right and up/down, the system triangulates the direction to the RF target and navigates toward it.

Hardware chain: antennas → AD8302 (phase output 0–1.8 V) → ESP32-C6 ADC → MAVLink → drone FC.

Built with ESP-IDF >= 5.3.0.

## Build & Flash Commands

Requires ESP-IDF environment to be sourced first. IDF is at `/Users/oleksiikolyvaiko/esp/v5.5.2/esp-idf`.

```bash
# Source the environment (if not already done)
source /Users/oleksiikolyvaiko/esp/v5.5.2/esp-idf/export.sh

idf.py set-target esp32c6    # First time only
idf.py build                 # Compile
idf.py flash                 # Flash via UART (/dev/tty.usbmodem14201)
idf.py monitor               # Serial monitor (115200 baud)
idf.py flash monitor         # Flash then monitor
idf.py fullclean             # Wipe build directory
```

No test framework is configured in this project.

## Architecture

Two FreeRTOS tasks communicate via a single-slot queue (`ADC_QUEUE_LENGTH=1`):

- **`adc_task`** — initializes ADC continuous mode on ADC1 channels 1 & 2 (GPIO1, GPIO2). Samples at 20 kHz, reads every 10 ms. Every `DISPLAY_UPDATE_INTERVAL` (50) samples, sends an `adc_sample_t` to the queue.
- **`display_task`** — blocks on the queue; on receipt, renders both ADC values to the SSD1306 display.

The queue uses a drop-oldest strategy: if full when a new sample arrives, the old value is discarded.

### Key Hardware Config (hardcoded in source)

| Parameter | Value |
|-----------|-------|
| Target | ESP32-C6 |
| ADC channels | ADC1_CH1 (GPIO1), ADC1_CH2 (GPIO2) |
| I2C SDA / SCL | GPIO22 / GPIO23 |
| I2C speed | 400 kHz |
| SSD1306 address | 0x3C |
| Display size | 128x32 |

### Component Dependency

The SSD1306 driver (`components/ssd1306 ^1.1.2`) is managed via `idf_component.yml` and locked in `dependencies.lock`. Run `idf.py update-dependencies` to update it.

### Configuration Constants

All tunable parameters live in `main/app_config.h` — task stack sizes, priorities, queue length, and display update interval.
