# Wheelchair ESP32-S3 Firmware

ESP-IDF firmware and Linux host tools for an ESP32-S3-based wheelchair
control system, including a host-side ROS 2 layer for LIDAR-assisted obstacle
avoidance on a Raspberry Pi.

The current firmware release is **v0.7.0**. This version adds quadrature encoder
reading via the ESP32-S3 PCNT peripheral. Encoder counts and per-cycle deltas
appear in joystick telemetry. The motor watchdog, STOP command, and GUI PWM
limit remain unchanged from v0.6.2. Do not connect motors during encoder
validation unless they are suspended off the ground.

A **host-side ROS 2 semi-assist layer** (Raspberry Pi 5 + RPLIDAR C1) has been
added on top of this firmware. It does **not** modify the firmware: it speaks
the same newline-delimited JSON protocol over USB serial. See
[ROS 2 host-side layer](#ros-2-host-side-layer-semi-assisted-lidar-control)
below, [docs/ROS2_INTEGRATION.md](docs/ROS2_INTEGRATION.md), and
[docs/TEST_PLAN_ROS2_ASSIST.md](docs/TEST_PLAN_ROS2_ASSIST.md). This layer is
experimental and pending manual validation with a suspended motor.

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
- reads two quadrature encoders via PCNT: left A/B on GPIO4/GPIO5, right A/B
  on GPIO6/GPIO7 (4× decoding, 1 µs glitch filter);
- never drives RPWM and LPWM simultaneously for the same motor channel;
- stops all PWM if no `pwm_test` command is received within 500 ms (watchdog);
- accepts duty-cycle commands up to ±1.0; the operator-facing safety gate
  (default 0.30, set by the GUI or the ROS 2 bridge) must not be raised
  without a suspended motor.

The joystick Y axis is inverted in software so that upward movement maps to
positive Y and downward movement maps to negative Y. Raw ADC readings are not
modified. A deadzone of `0.08` is applied around the default raw center of
`2048`.

The firmware itself does **not** implement PI control, closed-loop motor
control, odometry, or ROS 2. ROS 2 integration is provided host-side on the
Raspberry Pi (see below) and does not change the firmware.

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
velocity in rad/s. They are stored but not acted upon by the firmware (the
v0.9 open-loop mapping is performed host-side by the ROS 2 bridge).

PWM test command:

```json
{"type":"pwm_test","seq":3,"left":0.15,"right":0.00}
```

`left` and `right` are duty-cycle fractions. Positive drives RPWM; negative
drives LPWM. The complementary output is held at zero. The firmware hard clamp
is ±1.0; the operator-facing safety gate (default 0.30) lives in the GUI and in
the ROS 2 bridge (`max_duty`). If no fresh `pwm_test` command arrives within
500 ms, all PWM outputs are forced low automatically.

Successful commands receive an ACK:

```json
{"type":"ack","seq":1,"cmd_seq":1,"status":"ok"}
```

Malformed JSON, unknown packet types, and invalid fields receive an error:

```json
{"type":"err","seq":2,"code":"invalid_json","status":"error"}
```

Joystick telemetry includes the latest command, motor test state, and encoder counts:

```json
{"type":"joy","seq":10,"t_ms":12345,"fw":"0.7.0","raw_x":2030,"raw_y":2050,"x":0.02,"y":0.0,"cmd_v":0.0,"cmd_w":0.0,"cmd_seq":1,"cmd_valid":true,"last_cmd_age_ms":50,"motor_left":0.15,"motor_right":0.0,"motor_test_active":true,"status":"ok","enc_left_count":1234,"enc_right_count":1230,"enc_left_delta":42,"enc_right_delta":41,"enc_status":"ok"}
```

Before the first valid command, `cmd_valid` is `false`, command values are
zero, and `last_cmd_age_ms` is `null`. `motor_test_active` is `false` when no
`pwm_test` command has been received, when a `stop` command clears the state,
or when the 500 ms watchdog fires.

`enc_left_count` / `enc_right_count` are running 32-bit totals since boot.
`enc_left_delta` / `enc_right_delta` are counts accumulated in the current
50 ms sample period. If encoder init failed, only `enc_status: "error"` is
present.

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

## ROS 2 host-side layer (semi-assisted LIDAR control)

A Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy) runs a semi-assisted obstacle
avoidance layer on top of the firmware. **The joystick remains the primary
command**; the LIDAR only modulates it — passing the command through, slowing
and steering within a bounded window, or stopping. The system never adds motion
the user did not request. This is assistive, **not autonomous**, control.

Architecture:

```
                          /joystick_cmd_vel
   [ ESP32-S3 ] --serial--> [ esp_bridge ] -----------------+
       ^  | telemetry            |  ^                        |
       |  +----------------------+  | /cmd_vel               v
       |     pwm_test / stop        |               [ shared_control ]
       +----------------------------+                        ^ /scan
                                                    [ sllidar_ros2 (C1) ]
```

Components (host-side only; firmware unchanged):

- **`sllidar_ros2`** — RPLIDAR C1 driver, publishes `/scan` (baud 460800).
- **`esp_bridge`** — serial ↔ ROS 2 bridge. Decodes the joystick into
  `/joystick_cmd_vel`, converts `/cmd_vel` into per-wheel `pwm_test` (open-loop,
  the v0.9 role), feeds the 500 ms watchdog at 20 Hz, and republishes telemetry.
- **`shared_control`** — fuses joystick intent and `/scan` via a **minimum-cost**
  decision: pass through, deviate, or stop the forward motion.

Run the full pipeline (RPLIDAR C1 = baud 460800), with the motor suspended:

```bash
ros2 launch wheelchair_ros wheelchair_assist.launch.py \
    esp_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0 lidar_baud:=460800
```

For first contact, run with `assist_gain:=0.0` (stop-only, no steering) before
enabling deviation. Safety layers are independent: firmware watchdog (500 ms),
bridge command timeout, sensor timeout, stop-on-exit, and the `max_duty` gate.

Limitations: the front LIDAR does not see behind (reverse is unprotected), the
scan is a single 2D plane (low obstacles and drop-offs are not seen), and the
velocity-to-PWM mapping is uncalibrated until RPM calibration (v0.8). Full
detail and parameters are in [docs/ROS2_INTEGRATION.md](docs/ROS2_INTEGRATION.md);
validation steps are in [docs/TEST_PLAN_ROS2_ASSIST.md](docs/TEST_PLAN_ROS2_ASSIST.md).

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
│   │   ├── encoder_pcnt.c
│   │   ├── encoder_pcnt.h
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
├── ros2/
│   └── wheelchair_ros/
│       ├── package.xml
│       ├── setup.py
│       ├── launch/
│       │   └── wheelchair_assist.launch.py
│       └── wheelchair_ros/
│           ├── esp_bridge_node.py
│           └── shared_control_node.py
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
