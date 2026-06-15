# Wheelchair ESP32-S3 Firmware

ESP-IDF firmware for an ESP32-S3-based wheelchair control system.

The current project release is **v0.3.1**. It adds a host-side graphical
joystick monitor for the JSON telemetry introduced by firmware v0.3.0.
Firmware behavior and its reported version remain `0.3.0` because v0.3.1
changes only Linux host tooling.

## Firmware telemetry

The firmware:

- logs its name, version, hardware target, and `boot_ok` status at startup;
- logs an incrementing heartbeat once per second;
- reads joystick X on GPIO1 / ADC1 channel 0;
- reads joystick Y on GPIO2 / ADC1 channel 1;
- samples both axes at approximately 20 Hz using the modern ADC oneshot
  driver;
- sends raw ADC readings and normalized values from approximately `-1.0` to
  `+1.0` as one JSON object per line;
- applies a deadzone of `0.08` around the default raw center of `2048`.

The joystick Y axis is inverted in software so that upward movement maps to
positive Y and downward movement maps to negative Y. This does not alter the
raw ADC readings or X-axis behavior.

The initial normalization assumes a raw range of 0 to 4095. Actual joystick
centers and endpoint ranges can vary and will be calibrated in a later
version.

This release intentionally does **not** implement host-to-ESP32 command
reception, Raspberry Pi-specific behavior, ROS 2, PWM or MCPWM, encoders or
PCNT, motor control, PI control, or safety logic.

## JSON telemetry

Each successful joystick sample produces one UTF-8 JSON line:

```json
{"type":"joystick","version":"0.3.0","seq":1,"raw_x":2030,"raw_y":2052,"x":0,"y":0}
```

Packet fields:

| Field | Meaning |
| --- | --- |
| `type` | Packet type, currently `joystick` |
| `version` | Firmware version |
| `seq` | Incrementing telemetry sequence number |
| `raw_x`, `raw_y` | Unmodified ADC readings |
| `x`, `y` | Deadzone-adjusted normalized axes |

ESP-IDF boot and heartbeat logs share the serial console and are not JSON.
The host test script reports those lines as invalid and continues reading.

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

## Linux host reader

Install `pyserial` in the Python environment used by the host:

```bash
python3 -m pip install -r requirements-dev.txt
```

Flash the board, close any ESP-IDF monitor using the same port, and run:

```bash
python3 scripts/read_json_serial.py /dev/ttyACM0
```

The default baud rate is 115200. Override it when needed:

```bash
python3 scripts/read_json_serial.py /dev/ttyACM0 --baud-rate 115200
```

The script is generic Linux host tooling. It can run on a notebook or a
Raspberry Pi and does not contain host-specific paths or ROS 2 integration.

## Joystick GUI

Version 0.3.1 adds a Tkinter visualization tool that reads the same one-way
JSON telemetry:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements-dev.txt
python3 scripts/joystick_gui.py /dev/ttyACM0
```

Tkinter is provided by the Linux system Python package and is intentionally
not listed in `requirements-dev.txt`.

The default baud rate is 115200. It can be set explicitly:

```bash
python3 scripts/joystick_gui.py /dev/ttyACM0 --baud 115200
```

The GUI displays raw and normalized axes, sequence number, packet status,
packet age, and valid/invalid counters. Current v0.3.0 packets do not include
`t_ms` or an explicit `status`, so the GUI displays `t_ms` as `n/a` and
derives status as `ok` for parsed JSON objects.

The serial reader runs in a background thread and sends parsed events through
a queue. Tkinter widgets are updated only from the main GUI thread.

Version 0.3.1 remains telemetry-only. Version 0.4 is reserved for
host-to-ESP32 command reception; it is not implemented here.

## Project layout

```text
.
├── CMakeLists.txt
├── README.md
├── requirements-dev.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── app_main.c
│   ├── drivers/
│   │   ├── joystick_adc.c
│   │   └── joystick_adc.h
│   ├── telemetry/
│   │   ├── json_telemetry.c
│   │   └── json_telemetry.h
│   └── version.h
├── docs/
└── scripts/
    ├── joystick_gui.py
    └── read_json_serial.py
```

See [docs/TEST_PLAN_V0_3_1.md](docs/TEST_PLAN_V0_3_1.md) for GUI validation,
[docs/TEST_PLAN_V0_3.md](docs/TEST_PLAN_V0_3.md) for serial telemetry
validation, and [docs/ROADMAP.md](docs/ROADMAP.md) for future releases. No
functionality from v0.4 or later is included in this version.
