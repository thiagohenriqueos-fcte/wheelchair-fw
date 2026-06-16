# Test Plan: v0.5.3

Version 0.5.3 adds a continuous PWM stream mode to
`scripts/wheelchair_control_gui.py`. While the stream is active, the current
slider values are sent as `pwm_test` commands at 10 Hz automatically, without
requiring repeated button clicks.

This is a host-side Python script change only. No firmware changes are included.

## IMPORTANT — No motors during this test

Do not connect motors or apply motor supply power during GUI validation.
All stream buttons are gated by the safety checkbox.
v0.5.3 validates the serial communication path and streaming behaviour only.

## Prerequisites

- ESP32-S3 running firmware v0.5.0 or later, connected over USB.
- Joystick connected per `docs/PINOUT.md`.
- Python virtual environment active (`source .venv/bin/activate`).
- No other process holding `/dev/ttyACM0`
  (close `read_json_serial.py`, `wheelchair_gui.py`, `send_motor_test.py`,
  or `idf.py monitor` first).

## Procedure

- [ ] 1. Activate the virtual environment and compile-check:

  ```bash
  cd ~/wheelchair-fw
  source .venv/bin/activate
  python3 -m py_compile scripts/wheelchair_control_gui.py
  ```

  Confirm no output (zero errors).

- [ ] 2. Launch the GUI:

  ```bash
  python3 scripts/wheelchair_control_gui.py /dev/ttyACM0
  ```

  Confirm:
  - Window titled "Wheelchair Control  v0.5.3" opens.
  - Status bar shows `Port: connected: /dev/ttyACM0 @ 115200`.
  - `Valid` counter begins incrementing.
  - `FW` field shows the current firmware version.
  - All of `Send Once`, `Start PWM Stream`, and `Stop PWM Stream` are disabled.
  - `STOP` button is visible, large, and red.
  - `PWM stream: OFF` label is visible.
  - `Last TX: L +0.00  R +0.00` shown.

- [ ] 3. Tick the safety checkbox:

  ```
  [✓] I understand — motors must be disconnected or suspended
  ```

  Confirm:
  - `Send Once` becomes enabled.
  - `Start PWM Stream` becomes enabled.
  - `Stop PWM Stream` remains disabled.
  - `PWM stream: OFF` label still shown.

- [ ] 4. Test `Send Once`:

  Set left slider to +0.15 and right to 0.00. Press `Send Once`.

  Confirm:
  - `Latest Response → ACK` shows `cmd_seq=1  status=ok`.
  - `seq: 1` shown.
  - `Last TX: L +0.15  R +0.00` updated.
  - Motor monitor updates: `motor_test_active: true`, left bar ~15% forward.
  - Stream is NOT started — `PWM stream: OFF` label remains.
  - No repeated commands are sent (seq stays at 1).

- [ ] 5. Press `STOP` to clear state:

  Confirm:
  - Both sliders return to 0.00.
  - ACK received for stop command.
  - `motor_test_active: false`.

- [ ] 6. Start the PWM stream:

  Set left to +0.15, right to 0.00. Press `Start PWM Stream`.

  Confirm:
  - `PWM stream: ON` label appears in blue.
  - `Start PWM Stream` becomes disabled.
  - `Send Once` becomes disabled.
  - `Stop PWM Stream` becomes enabled.
  - Seq counter increments continuously (approximately every 100 ms).
  - `Last TX: L +0.15  R +0.00` updates continuously.
  - Motor monitor shows `motor_test_active: true` and left bar filled forward.
  - ACK packets arrive continuously (last_cmd_age_ms < 500 ms in telemetry).

- [ ] 7. Move the left slider while streaming:

  Drag the left slider from +0.15 to -0.15 while the stream is active.

  Confirm:
  - Motor monitor transitions: left bar moves from forward (green) to reverse (red).
  - No gap or manual send step required — the next tick picks up the new value.
  - Seq increments continuously throughout.

- [ ] 8. Press `Stop PWM Stream`:

  Confirm:
  - `PWM stream: OFF` label shown (grey).
  - `Stop PWM Stream` becomes disabled.
  - `Start PWM Stream` becomes enabled.
  - `Send Once` becomes enabled.
  - Seq counter stops incrementing.
  - After 500 ms, motor monitor shows `motor_test_active: false` (firmware watchdog fires).

- [ ] 9. Test `STOP` during an active stream:

  Start stream again (left +0.10, right +0.10). Wait for motor monitor to show active.
  Press `STOP`.

  Confirm:
  - Stream stops immediately: `PWM stream: OFF` label.
  - Both sliders return to 0.00.
  - A single `stop` command is sent and ACK received.
  - Motor monitor clears to `motor_test_active: false`.
  - No further stream ticks are sent after pressing `STOP`.

- [ ] 10. Test `STOP` without an active stream:

  With stream off, set left to +0.15, press `Send Once`. Then press `STOP`.

  Confirm same behaviour as v0.5.2: stop command sent, sliders zeroed, motor clears.

- [ ] 11. Test safety checkbox deactivation during stream:

  Start stream (any non-zero values). Uncheck the safety checkbox.

  Confirm:
  - Stream stops immediately: `PWM stream: OFF`.
  - `Start PWM Stream` and `Send Once` become disabled.
  - After 500 ms, motor monitor shows `motor_test_active: false`.

  Re-check the safety checkbox. Confirm `Start PWM Stream` and `Send Once` become
  enabled again.

- [ ] 12. Test close-during-stream behaviour:

  Set left to +0.15. Start stream. While streaming, close the GUI window.

  Confirm:
  - GUI closes cleanly (no Python traceback).
  - After close, open a second terminal and run:

    ```bash
    python3 scripts/read_json_serial.py /dev/ttyACM0
    ```

  - Telemetry shows `motor_test_active: false`, confirming the stop command was
    sent and the firmware watchdog did not need to fire.

- [ ] 13. Test close without stream:

  Open the GUI without starting the stream. Set left to +0.15, press `Send Once`,
  then close the window. Same stop-on-close behaviour as v0.5.2 must apply.

- [ ] 14. Test zero utilities during stream:

  Start stream (left +0.15, right +0.15). Press `Zero Left`.

  Confirm left slider moves to 0.00. The next stream tick sends `left: 0.00`.
  Motor monitor left bar clears to stopped while right stays at ~15%.

  Press `Zero Both`. Confirm both sliders return to 0.00.
  After 500 ms, motor monitor clears fully.

  Press `Stop PWM Stream`.

- [ ] 15. Test `Send Once` is disabled while streaming:

  Start stream. Confirm `Send Once` is greyed out and cannot be clicked.
  Press `Stop PWM Stream`. Confirm `Send Once` becomes enabled again.

- [ ] 16. Test 10 Hz rate:

  Start stream. Watch the seq counter for 10 seconds. Confirm seq increments by
  approximately 100 (±5) during that period, indicating ~10 Hz send rate.

- [ ] 17. Test invalid-JSON resilience during stream:

  With stream active, confirm the GUI stays open and streaming continues
  if non-JSON firmware output appears (e.g. a startup log line from a reset).
  The `Invalid` counter should increment without crashing or stopping the stream.

## Expected packet flow (streaming)

```
TX: {"type":"pwm_test","seq":1,"left":0.15,"right":0.0}
RX: {"type":"ack","seq":1,"cmd_seq":1,"status":"ok"}
TX: {"type":"pwm_test","seq":2,"left":0.15,"right":0.0}
RX: {"type":"ack","seq":2,"cmd_seq":2,"status":"ok"}
...  (repeated at 10 Hz)
[user presses STOP]
TX: {"type":"stop","seq":N}
RX: {"type":"ack","seq":N,"cmd_seq":N,"status":"ok"}
RX: {"type":"joy",...,"motor_test_active":false,...}
```

## Notes

- Firmware version is unchanged. `fw` in telemetry reflects the flashed version.
- The 500 ms watchdog fires automatically when the stream is paused or stopped,
  clearing `motor_test_active` in telemetry. This is expected and correct.
- `scripts/send_motor_test.py` and `scripts/wheelchair_gui.py` cannot run
  simultaneously with `wheelchair_control_gui.py`.
- Do not tag v0.5.3 until this test plan is manually validated.
