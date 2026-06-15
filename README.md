# Wheelchair ESP32-S3 Firmware

ESP-IDF firmware and Linux host tools for an ESP32-S3-based wheelchair
control system.

The current project release is **v0.5.0**. This version adds MCPWM PWM
generation for two IBT-2 H-bridge modules. A host-side `pwm_test` command
drives the PWM outputs directly for oscilloscope validation. No motor is
connected during this stage.

## Firmware behavior

The firmware:

- uses USB Serial/JTAG for newline-delimited JSON input and output;
- emits JSON startup status and an incrementing heartbeat;
- samples joystick X on GPIO1 / ADC1_CH0 and Y on GPIO2 / ADC1_CH1 at
  approximately 20 Hz using the ADC oneshot driver;
- reports raw and normalized joystick values;
- receives `cmd`, `stop`, and `pwm_test` JSON objects in the `comm_rx_task`;
- stores the latest valid host command and its receive timestamp;
- sends an ACK for valid commands and an error packet for invalid input;
- includes the latest command state in joystick telemetry;
- generates MCPWM PWM signals on GPIO10 (left RPWM), GPIO11 (left LPWM),
  GPIO12 (right RPWM), GPIO13 (right LPWM) at 20 kHz;
- never drives RPWM and LPWM simultaneously for the same motor channel.

The joystick Y axis is inverted in software so that upward movement maps to
positive Y and downward movement maps to negative Y. Raw ADC readings are not
modified. A deadzone of `0.08` is applied around the default raw center of
`2048`.

Version 0.5 does **not** implement encoders, PI control, closed-loop motor
control, safety logic, ROS 2, or command timeout behavior.

## JSON protocol

Every packet is one UTF-8 JSON object followed by `\n`.

Movement command:

```json
{"type":"cmd","seq":1,"v":0.2,"w":0.0}
```

Stop command:

```json
{"type":"stop","seq":2}
```

`v` is the requested linear velocity in m/s and `w` is the requested angular
velocity in rad/s. They are stored but not acted upon in v0.5.

PWM test command:

```json
{"type":"pwm_test","seq":3,"left":0.5,"right":0.5}
```

`left` and `right` are duty-cycle fractions in [-1.0, 1.0]. Positive drives
RPWM; negative drives LPWM. The complementary output is held at zero.

Successful commands receive an ACK:

```json
{"type":"ack","seq":1,"cmd_seq":1,"status":"ok"}
```

Malformed JSON, unknown packet types, and invalid fields receive an error:

```json
{"type":"err","seq":2,"code":"invalid_json","status":"error"}
```

Joystick telemetry includes the latest command and motor test state:

```json
{"type":"joy","seq":10,"t_ms":12345,"fw":"0.5.0","raw_x":2030,"raw_y":2050,"x":0.02,"y":0.0,"cmd_v":0.2,"cmd_w":0.0,"cmd_seq":1,"cmd_valid":true,"last_cmd_age_ms":50,"motor_left":0.5,"motor_right":0.5,"motor_test_active":true,"status":"ok"}
```

Before the first valid command, `cmd_valid` is `false`, command values are
zero, and `last_cmd_age_ms` is `null`. Before the first `pwm_test` command,
`motor_test_active` is `false` and motor values are zero.

## Joystick wiring

| Joystick connection | ESP32-S3 connection |
| --- | --- |
| X axis output | GPIO1 / ADC1_CH0 |
| Y axis output | GPIO2 / ADC1_CH1 |
| VCC | 3V3 |
| GND | GND |

Power the joystick from **3.3 V, not 5 V**. ESP32-S3 GPIO inputs are not
5 V tolerant.

## Build and flash

Install ESP-IDF and follow [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md). Then:

```bash
source "$HOME/esp/esp-idf/export.sh"
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash
```

The port may instead be `/dev/ttyUSB0`, depending on the board and USB
connection.

## Host command test

Install the Python dependency in a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements-dev.txt
```

Send one movement command:

```bash
python3 scripts/send_json_command.py /dev/ttyACM0 --v 0.20 --w 0.00
```

Send stop:

```bash
python3 scripts/send_json_command.py /dev/ttyACM0 --stop
```

Send commands at 10 Hz for 5 seconds:

```bash
python3 scripts/send_json_command.py /dev/ttyACM0 \
    --v 0.20 --w 0.00 --rate 10 --duration 5
```

Send a literal malformed line for error-path testing:

```bash
python3 scripts/send_json_command.py /dev/ttyACM0 \
    --raw-line '{"type":"cmd"'
```

The script writes commands and reads ACK, error, heartbeat, status, and
joystick packets through the same serial connection. It decodes UTF-8 safely
and reports invalid response lines without crashing.

Only one process can normally open `/dev/ttyACM0` at a time. Close
`idf.py monitor`, `read_json_serial.py`, or `joystick_gui.py` before running
`send_json_command.py`.

## Other host tools

Read and validate the JSON stream without sending commands:

```bash
python3 scripts/read_json_serial.py /dev/ttyACM0
```

Display joystick telemetry in the Tkinter GUI:

```bash
python3 scripts/joystick_gui.py /dev/ttyACM0
```

The GUI keeps numeric telemetry unchanged while filtering and interpolating
only the visual dot. The dot remains clamped inside its circular boundary.
Tkinter is supplied by the system Python package and is not listed in
`requirements-dev.txt`.

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
│   │   ├── joystick_adc.h
│   │   ├── motor_pwm.c
│   │   └── motor_pwm.h
│   ├── protocol/
│   │   ├── json_command.c
│   │   └── json_command.h
│   ├── serial/
│   │   ├── serial_io.c
│   │   └── serial_io.h
│   ├── telemetry/
│   │   ├── json_telemetry.c
│   │   └── json_telemetry.h
│   └── version.h
├── docs/
└── scripts/
    ├── joystick_gui.py
    ├── read_json_serial.py
    ├── send_json_command.py
    └── send_motor_test.py
```

See [docs/ROADMAP.md](docs/ROADMAP.md) for future releases. Do not begin v0.6
until v0.5 has been validated independently with an oscilloscope.
