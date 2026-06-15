# Wheelchair ESP32-S3 Firmware

ESP-IDF firmware for an ESP32-S3-based wheelchair control system.

The current release is **v0.1.0**. This release establishes the repository,
ESP-IDF project structure, environment checks, and a minimal firmware used to
verify that the board boots and the development toolchain works.

## v0.1 behavior

The firmware:

- logs its name, version, hardware target, and `boot_ok` status at startup;
- logs an incrementing heartbeat once per second;
- uses only the default `app_main` execution flow.

This release intentionally does **not** implement joystick input, ADC setup,
PWM or MCPWM, encoders or PCNT, motor control, JSON communication, Raspberry
Pi communication, or a multi-task application architecture.

## Build and run

Install ESP-IDF first by following the official Espressif documentation and
the project notes in [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md). In a terminal:

```bash
source "$HOME/esp/esp-idf/export.sh"
./scripts/check_env.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Depending on the ESP32-S3 board and USB cable, the serial port may instead be
`/dev/ttyUSB0`. Exit the serial monitor with `Ctrl+]`.

## Project layout

```text
.
├── CMakeLists.txt
├── README.md
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── app_main.c
│   └── version.h
├── docs/
└── scripts/
```

See [docs/ROADMAP.md](docs/ROADMAP.md) for planned releases. No functionality
from v0.2 or later is included in this version.
