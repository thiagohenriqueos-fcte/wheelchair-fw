# Wheelchair ESP32-S3 Firmware

ESP-IDF firmware and Linux host tools for an ESP32-S3-based wheelchair
control system.

The current project release is **v0.6.2**. This version tests MCPWM output
with a real IBT-2 H-bridge and motor physically connected. The motor must be
suspended (wheel off the ground, no load) for all v0.6 tests. A 500 ms
command watchdog stops all PWM automatically if no fresh `pwm_test` command
arrives. The GUI PWM limit defaults to 0.30; the firmware accepts the full
±1.0 range but operator safety relies on the GUI limit and the STOP button.

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
- includes the latest command and motor test state in joystick telemetry;
- generates MCPWM PWM signals on GPIO10 (left RPWM), GPIO11 (left LPWM),
  GPIO12 (right RPWM), GPIO13 (right LPWM) at 25 kHz;
- never drives RPWM and LPWM simultaneously for the same motor channel;
- stops all PWM if no `pwm_test` command is received within 500 ms (watchdog);
- accepts duty-cycle commands up to ±1.0; the GUI PWM limit (default 0.30)
  is the operator-facing safety gate — do not raise it without a suspended motor.

The joystick Y axis is inverted in software so that upward movement maps to
positive Y and downward movement maps to negative Y. Raw ADC readings are not
modified. A deadzone of `0.08` is applied around the default raw center of
`2048`.

Version 0.6 does **not** implement encoders, PI control, closed-loop motor
control, or ROS 2.

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
{"type":"pwm_test","seq":3,"left":0.15,"right":0.00}
```

`left` and `right` are duty-cycle fractions. Positive drives RPWM; negative
drives LPWM. The complementary output is held at zero. The firmware clamps
values to ±0.30. If no fresh `pwm_test` command arrives within 500 ms, all
PWM outputs are forced low automatically.

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
{"type":"joy","seq":10,"t_ms":12345,"fw":"0.6.0","raw_x":2030,"raw_y":2050,"x":0.02,"y":0.0,"cmd_v":0.0,"cmd_w":0.0,"cmd_seq":1,"cmd_valid":true,"last_cmd_age_ms":50,"motor_left":0.15,"motor_right":0.0,"motor_test_active":true,"status":"ok"}
```

Before the first valid command, `cmd_valid` is `false`, command values are
zero, and `last_cmd_age_ms` is `null`. `motor_test_active` is `false` when no
`pwm_test` command has been received, when a `stop` command clears the state,
or when the 500 ms watchdog fires.

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

Integrated control GUI (v0.6.2 — monitor, send commands, stream PWM, tune smoothing, fullscreen, configurable PWM limit):

```bash
python3 scripts/wheelchair_control_gui.py /dev/ttyACM0
```

Reads telemetry and sends `pwm_test` / `stop` commands over the same serial
connection, solving the one-process-per-port limitation. The GUI has four
panels: connection status, joystick monitor, motor PWM monitor, and a motor
control panel with sliders and buttons.

**Send Once** sends a single `pwm_test` command with the current slider values.
**Start PWM Stream** begins continuous sending at `PWM_STREAM_HZ` (10 Hz) using
Tkinter's `after()` scheduler — moving a slider during an active stream takes
effect on the next tick automatically. **Stop PWM Stream** cancels the stream.
`STOP` is always active, immediately cancels streaming, sends a `stop` command,
and zeros both sliders. Closing the window stops streaming and sends a `stop`
command before releasing the serial port.

All stream-related buttons are disabled until the safety checkbox is ticked.

Press **F11** (or the Fullscreen button) to enter fullscreen. Press **Esc** or
"Exit Fullscreen" to return. STOP remains always visible in fullscreen.

A **PWM limit** slider in the motor control panel sets the maximum duty cycle
the GUI will send (0.00–1.00, default 0.30). Click **Apply PWM Limit** to
update the motor sliders and clamp current values. A warning appears when the
limit exceeds 0.30. **Reset to 0.30** returns to the safe default. The firmware
accepts up to ±1.0 but the GUI limit is the operator-facing safety gate.

A **Joystick smoothing settings** panel lets you tune three visualization
parameters at runtime without restarting the GUI:

- **Filter alpha** (0.01–1.00, default 0.25): exponential moving average applied
  to incoming normalized x/y telemetry. 1.0 = raw, lower = smoother but slower.
- **Interpolation alpha** (0.01–1.00, default 0.20): per-frame interpolation of
  the joystick dot toward the filtered value. 1.0 = instant, lower = smooth lag.
- **Update interval** (10–100 ms, default 33 ms): Tkinter frame period.

The joystick panel also shows `filt_x/y` (filtered) and `vis_x/y` (visual) so
you can observe each stage of the smoothing pipeline. These controls affect
visualization only and have no effect on motor commands.

Default slider range: ±0.30 (conservative, for suspended-motor testing).
Full ±1.0 range requires editing `SLIDER_MIN` / `SLIDER_MAX` in the script.

Read-only monitor GUI (v0.5.1 — no send capability, one process needed):

```bash
python3 scripts/wheelchair_gui.py /dev/ttyACM0
```

Joystick-only GUI (the original v0.3.x monitor, preserved for reference):

```bash
python3 scripts/joystick_gui.py /dev/ttyACM0
```

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
    ├── send_motor_test.py
    ├── wheelchair_control_gui.py
    └── wheelchair_gui.py
```

See [docs/ROADMAP.md](docs/ROADMAP.md) for future releases. Do not begin v0.7
until v0.6 has been validated independently with a suspended motor.
