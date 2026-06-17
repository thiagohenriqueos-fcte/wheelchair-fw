# Test Plan: v0.7.0

Version 0.7.0 adds quadrature encoder reading via the ESP32-S3 PCNT peripheral.

**New hardware:**
- Left encoder: A → GPIO4, B → GPIO5
- Right encoder: A → GPIO6, B → GPIO7
- Differential line receiver outputs connect to these GPIO pins.

**Changes in this version:**
- `main/drivers/encoder_pcnt.c/.h` — new PCNT driver, 4× quadrature, 1 µs glitch filter
- `main/telemetry/json_telemetry.c/.h` — five new fields in `joy` packet:
  `enc_left_count`, `enc_right_count`, `enc_left_delta`, `enc_right_delta`, `enc_status`
- `main/app_main.c` — encoder init and per-cycle `read_sample` call
- `main/version.h`, `CMakeLists.txt` — version 0.7.0

**No changes** to motor PWM, watchdog, GUI, or JSON command protocol.

## IMPORTANT — No motors during software tests

Do not connect motors or apply motor supply power during serial telemetry
validation (steps 1–13). If encoders are physically wired to wheels, those
wheels must be suspended off the ground during encoder spin tests (steps 14–20).

## Prerequisites

- ESP32-S3 with v0.7.0 firmware flashed.
- Python virtual environment active (`source .venv/bin/activate`).
- No other process holding `/dev/ttyACM0`.

## Build and flash

```bash
cd ~/wheelchair-fw
source ~/esp/esp-idf/export.sh && idf.py build
idf.py -p /dev/ttyACM0 flash
```

Build must complete with zero errors and zero warnings in `encoder_pcnt.c`.

## Procedure — Serial telemetry validation (no encoders connected)

### 1. Boot status packets

```bash
python3 scripts/read_json_serial.py /dev/ttyACM0
```

Immediately after flash, confirm four `status` packets appear:

```
{"type":"status", ..., "event":"boot",             "status":"ok"}
{"type":"status", ..., "event":"joystick_adc",     "status":"ok"}
{"type":"status", ..., "event":"motor_pwm",        "status":"ok"}
{"type":"status", ..., "event":"command_receiver", "status":"ok"}
{"type":"status", ..., "event":"encoder_pcnt",     "status":"ok"}
```

> **Note:** With no encoders connected, GPIO inputs float. The encoder init
> should still succeed (PCNT does not require signals to be present at init).
> If `encoder_pcnt` shows `"status":"error"`, note the `detail` field and
> report it before continuing.

### 2. Firmware version in telemetry

Confirm each `joy` packet contains `"fw":"0.7.0"`.

### 3. Encoder fields present

Confirm each `joy` packet contains all five encoder fields:

```json
"enc_left_count": 0,
"enc_right_count": 0,
"enc_left_delta": 0,
"enc_right_delta": 0,
"enc_status": "ok"
```

Counts should be zero or near-zero while GPIO inputs float. Some noise counts
are possible from floating inputs; this is normal.

### 4. Encoder fields absent when enc_status is error

If step 1 showed `encoder_pcnt: error`, confirm that `joy` packets contain
only `"enc_status":"error"` and do NOT contain `enc_left_count`,
`enc_right_count`, `enc_left_delta`, or `enc_right_delta`.

### 5. Existing fields unchanged

Confirm `joy` packets still contain all pre-v0.7 fields:
`type`, `seq`, `t_ms`, `fw`, `raw_x`, `raw_y`, `x`, `y`,
`cmd_v`, `cmd_w`, `cmd_seq`, `cmd_valid`, `last_cmd_age_ms`,
`motor_left`, `motor_right`, `motor_test_active`, `status`.

### 6. Heartbeat unchanged

Confirm `heartbeat` packets still appear every ~1 second and contain no
encoder fields.

### 7. GUI compatibility

Launch the integrated GUI:

```bash
python3 scripts/wheelchair_control_gui.py /dev/ttyACM0
```

Confirm:
- GUI opens without Python traceback.
- Joystick panel updates normally.
- Motor control panel operates as in v0.6.2.
- The unknown encoder fields in the JSON are silently ignored by the GUI
  (no crash, no error).
- Close GUI cleanly.

### 8. Motor commands unaffected

With `read_json_serial.py` running, send a PWM test command:

```bash
python3 scripts/send_motor_test.py /dev/ttyACM0 --left 0.10 --right 0.00
```

Confirm:
- ACK received.
- `joy` packets show `motor_left ≈ 0.10`, `motor_test_active: true`.
- Encoder fields remain present with unchanged format.

Send stop:

```bash
python3 scripts/send_motor_test.py /dev/ttyACM0 --stop
```

Confirm `motor_test_active: false` in telemetry.

## Procedure — Physical encoder validation (encoders wired, wheels suspended)

Perform these steps only with encoders physically connected and wheels
suspended off the ground with no load.

### 9. Left encoder counts forward

With GPIO4/GPIO5 connected to the left encoder, slowly rotate the left wheel
forward. Confirm `enc_left_count` increases monotonically in telemetry.
`enc_right_count` must remain near zero.

### 10. Left encoder counts backward

Slowly rotate the left wheel backward. Confirm `enc_left_count` decreases.

### 11. Right encoder counts forward

Slowly rotate the right wheel forward. Confirm `enc_right_count` increases.
`enc_left_count` must remain near zero.

### 12. Right encoder counts backward

Slowly rotate the right wheel backward. Confirm `enc_right_count` decreases.

### 13. 4× quadrature: counts per revolution

Rotate one wheel exactly one full revolution forward. Using the known encoder
PPR (pulses per revolution, from encoder datasheet), confirm:

```
enc_left_count ≈ 4 × PPR
```

(4× because the PCNT driver counts all four edges per electrical cycle.)

### 14. Delta resets each cycle

While rotating a wheel slowly, confirm `enc_left_delta` or `enc_right_delta`
shows a small positive number each cycle (~50 ms) that reflects only the
rotation in that cycle — not the total count.

### 15. Both encoders independent

Rotate both wheels simultaneously in opposite directions.
Confirm `enc_left_count` increases and `enc_right_count` decreases (or vice
versa), with no cross-interference.

### 16. Glitch filter

With the wheel stationary, confirm that encoder counts are stable (not
incrementing from electrical noise). If significant noise is observed on
stationary wheels, investigate signal integrity before proceeding to v0.8.

## Commit after validation

```bash
git add main/drivers/encoder_pcnt.c main/drivers/encoder_pcnt.h \
        main/CMakeLists.txt \
        main/telemetry/json_telemetry.c main/telemetry/json_telemetry.h \
        main/app_main.c main/version.h \
        CMakeLists.txt \
        README.md docs/ROADMAP.md docs/TEST_PLAN_V0_7.md
git commit -m "v0.7.0: add PCNT encoder reading"
```

## Tag after validation

```bash
git tag -a v0.7.0 -m "Version 0.7.0: PCNT quadrature encoder reading"
git push && git push origin v0.7.0
```

Do not tag v0.7.0 until at least steps 1–8 (serial telemetry) have been
verified. Steps 9–16 require physical encoders and may be deferred.

## Notes

- The firmware hard clamp (`MOTOR_TEST_MAX_DUTY = 1.0f`) and 500 ms watchdog
  are unchanged. The GUI PWM limit (default 0.30) remains the operator safety gate.
- PCNT hardware counter limits: ±30000. At 50 ms polling, the counter is read
  and cleared each cycle. For typical encoder speeds this will not overflow.
- GPIO4/5/6/7 have internal pull-ups enabled by the PCNT driver. With no
  encoder connected, inputs float and noise counts are possible — this is
  expected and harmless for v0.7 telemetry-only monitoring.
- v0.7 does not implement RPM calculation, PI control, closed-loop control,
  odometry, or ROS 2. These are planned for v0.8 and beyond.
