# Wheelchair ESP32-S3 Firmware

ESP-IDF firmware for an ESP32-S3-based wheelchair control system.

The current release is **v0.2.0**. This release validates two-axis joystick
input through the ESP32-S3 ADC while preserving the v0.1 boot message and
heartbeat.

## v0.2 behavior

The firmware:

- logs its name, version, hardware target, and `boot_ok` status at startup;
- logs an incrementing heartbeat once per second;
- reads joystick X on GPIO1 / ADC1 channel 0;
- reads joystick Y on GPIO2 / ADC1 channel 1;
- samples both axes at approximately 20 Hz using the modern ADC oneshot
  driver;
- logs raw ADC readings and normalized values from approximately `-1.0` to
  `+1.0`;
- applies a deadzone of `0.08` around the default raw center of `2048`.

The initial normalization assumes a raw range of 0 to 4095. Actual joystick
centers and endpoint ranges can vary and will be calibrated in a later
version.

This release intentionally does **not** implement PWM or MCPWM, encoders or
PCNT, motor control, JSON communication, Raspberry Pi communication, PI
control, or safety logic.

## Joystick wiring

| Joystick connection | ESP32-S3 connection |
| --- | --- |
| X axis output | GPIO1 / ADC1_CH0 |
| Y axis output | GPIO2 / ADC1_CH1 |
| VCC | 3V3 |
| GND | GND |

Power the joystick from **3.3 V, not 5 V**. ESP32-S3 GPIO inputs are not
5 V tolerant.

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
│   ├── drivers/
│   │   ├── joystick_adc.c
│   │   └── joystick_adc.h
│   └── version.h
├── docs/
└── scripts/
```

See [docs/TEST_PLAN_V0_2.md](docs/TEST_PLAN_V0_2.md) for joystick validation
steps and [docs/ROADMAP.md](docs/ROADMAP.md) for future releases. No
functionality from v0.3 or later is included in this version.
