# Wheelchair ESP32-S3 Firmware

ESP-IDF firmware and Linux host tools for an ESP32-S3-based wheelchair
control system.

## LIDAR semi-assist layer

The repository now includes a ROS 2 package in
[`ros2/wheelchair_ros`](ros2/wheelchair_ros) for semi-assisted movement with an
RPLIDAR. It reads the joystick intent from ESP telemetry, evaluates obstacle
clearance from `/scan`, and sends assisted `drive_cmd` wheel commands back to
the firmware while `drive_cfg` remains the safety gate.

Start with wheels suspended and `assist_gain:=0.0` to validate braking/stop
before enabling steering correction. See
[`docs/ROS2_SEMI_ASSIST.md`](docs/ROS2_SEMI_ASSIST.md) and
[`docs/TEST_PLAN_SEMI_ASSIST.md`](docs/TEST_PLAN_SEMI_ASSIST.md).

The current project release is **v0.7.0**. This version adds quadrature encoder
reading via the ESP32-S3 PCNT peripheral. Encoder counts and per-cycle deltas
appear in joystick telemetry. The motor watchdog, STOP command, and GUI PWM
limit remain unchanged from v0.6.2. Do not connect motors during encoder
validation unless they are suspended off the ground.

## Firmware behavior

The firmware:

- uses USB Serial/JTAG for newline-delimited JSON input and output;
- emits JSON startup status and an incrementing heartbeat;
- samples joystick X on GPIO1 / ADC1_CH0 and Y on GPIO2 / ADC1_CH1 at
  approximately 20 Hz using the ADC oneshot driver;
- reports raw and normalized joystick values;
- receives `drive_cfg`, `drive_cmd`, and `stop` JSON objects in the
  `comm_rx_task`;
- stores the latest valid drive config and assisted wheel command timestamps;
- sends an ACK for valid commands and an error packet for invalid input;
- includes drive mode, arm state, output duty, and assist state in telemetry;
- generates MCPWM PWM signals on GPIO13 (left RPWM), GPIO12 (left LPWM),
  GPIO14 (right RPWM), GPIO27 (right LPWM) at 25 kHz;
- reads two quadrature encoders via PCNT: left A/B on GPIO4/GPIO5, right A/B
  on GPIO6/GPIO7 (4× decoding, 1 µs glitch filter);
- never drives RPWM and LPWM simultaneously for the same motor channel;
- stops all PWM if the armed `drive_cfg` is stale, and stops assisted movement
  if `drive_cmd` is stale;
- applies the `max_duty` limit from `drive_cfg` (default host tools use 0.30);
  do not raise it without suspended wheels.

The joystick Y axis is inverted in software so that upward movement maps to
positive Y and downward movement maps to negative Y. Raw ADC readings are not
modified. A deadzone of `0.08` is applied around the default raw center of
`2048`.

Version 0.7 does **not** implement PI control or closed-loop motor control.

## JSON protocol

Every packet is one UTF-8 JSON object followed by `\n`.

Drive configuration / safety gate:

```json
{"type":"drive_cfg","seq":1,"accel":1.5,"decel":3.0,"max_duty":0.30,"armed":true}
```

Assisted wheel command:

```json
{"type":"drive_cmd","seq":2,"left":0.25,"right":0.10}
```

`left` and `right` are normalized wheel requests in `[-1.0, +1.0]`. The
firmware still applies `max_duty`, acceleration/deceleration ramping, and
freshness watchdogs.

Stop command:

```json
{"type":"stop","seq":3}
```

`stop` disarms immediately and clears any pending assisted command.

Successful commands receive an ACK:

```json
{"type":"ack","seq":1,"cmd_seq":1,"status":"ok"}
```

Malformed JSON, unknown packet types, and invalid fields receive an error:

```json
{"type":"err","seq":2,"code":"invalid_json","status":"error"}
```

Drive telemetry includes joystick input, firmware state, and motor output:

```json
{"type":"drive","seq":10,"t_ms":12345,"fw":"0.7.0","raw_x":2030,"raw_y":2050,"x":0.02,"y":0.0,"out_left":0.12,"out_right":0.08,"armed":true,"driving":true,"drive_mode":"assist","assist_active":true,"max_duty":0.30,"accel":1.5,"decel":3.0,"cfg_age_ms":45,"assist_age_ms":20,"assist_left":0.40,"assist_right":0.27,"status":"ok"}
```

`drive_mode` is `manual`, `assist`, `assist_timeout`, or `disarmed`. In manual
mode, the ESP mixes the physical joystick locally. In assist mode, the ROS 2
layer supplies `drive_cmd` values from the LIDAR cost function.

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

Send a conservative armed drive config:

```bash
python3 scripts/send_json_command.py /dev/ttyACM0 \
    --drive-cfg --armed --max-duty 0.20 --accel 1.5 --decel 3.0
```

Send one assisted wheel command:

```bash
python3 scripts/send_json_command.py /dev/ttyACM0 --left 0.20 --right 0.20
```

Send stop:

```bash
python3 scripts/send_json_command.py /dev/ttyACM0 --stop
```

Send assisted wheel commands at 10 Hz for 5 seconds:

```bash
python3 scripts/send_json_command.py /dev/ttyACM0 --left 0.20 --right 0.20 \
    --rate 10 --duration 5
```

Send a literal malformed line for error-path testing:

```bash
python3 scripts/send_json_command.py /dev/ttyACM0 \
    --raw-line '{"type":"drive_cmd"'
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

Integrated control GUI (monitor, tune drive config, LIDAR, IMU, fullscreen):

```bash
python3 scripts/wheelchair_control_gui.py /dev/ttyACM0
```

Reads telemetry and sends `drive_cfg` / `stop` commands over the same serial
connection, solving the one-process-per-port limitation. The GUI can arm the
firmware's local differential-drive loop, tune `max_duty`, `accel`, and `decel`,
and monitor LIDAR/IMU panels.

`STOP` is always active, disarms the firmware immediately, and sends a `stop`
command before releasing the serial port on close.

Press **F11** (or the Fullscreen button) to enter fullscreen. Press **Esc** or
"Exit Fullscreen" to return. STOP remains always visible in fullscreen.

The `Duty máx` slider sets the maximum duty cycle the firmware will apply
(0.00–1.00, default 0.30). A warning appears when the limit exceeds 0.30.

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
