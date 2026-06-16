# Test Plan: v0.5.2

Version 0.5.2 adds an integrated control GUI (`scripts/wheelchair_control_gui.py`)
that reads ESP32 telemetry and sends `pwm_test` and `stop` commands over the
same serial connection. This removes the requirement to close the monitor
before sending a command.

This is a host-side Python script only. No firmware changes are included.
The firmware may report `fw: "0.5.0"` or later; all telemetry fields used
by the GUI are present from v0.5.0 onward.

## IMPORTANT — No motors during this test

Do not connect motors or apply motor supply power during GUI validation.
The `Send PWM` button is gated by a safety checkbox for this reason.
v0.5.2 validates the serial communication path and GUI behaviour only.

## Prerequisites

- ESP32-S3 running firmware v0.5.0 or later, connected over USB.
- Joystick connected per `docs/PINOUT.md`.
- Python virtual environment active (`source .venv/bin/activate`).
- No other process holding `/dev/ttyACM0`
  (close `read_json_serial.py`, `joystick_gui.py`, `wheelchair_gui.py`,
  `send_motor_test.py`, or `idf.py monitor` first).

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
  - Window titled "Wheelchair Control  v0.5.2" opens.
  - Status bar shows `Port: connected: /dev/ttyACM0 @ 115200`.
  - `Valid` counter begins incrementing.
  - `Invalid` counter stays at 0.
  - `FW` field shows `0.5.0` or the current firmware version.
  - `HB` field increments approximately every second.
  - `Send PWM` button is disabled (greyed out).
  - `STOP` button is visible, large, and red.

- [ ] 3. Confirm telemetry is received:

  Confirm `raw_x`, `raw_y`, `x`, and `y` fields update in the joystick panel.
  Move the physical joystick and confirm the dot moves smoothly inside the
  circular boundary.

- [ ] 4. Confirm motor monitor shows stopped:

  With no prior `pwm_test` command:
  - `motor_test_active: false` (grey label).
  - Both motor bars show no fill (only the centre line).
  - Both labels show `Stopped / 0%`.

- [ ] 5. Tick the safety checkbox:

  ```
  [✓] I understand — motors must be disconnected or suspended
  ```

  Confirm `Send PWM` becomes enabled.

- [ ] 6. Set left slider to approximately +0.15 and right slider to 0.00.

  Confirm the numeric display next to the slider shows `+0.15` and `+0.00`.

- [ ] 7. Press `Send PWM`.

  Confirm:
  - `Latest Response → ACK` updates to show `cmd_seq=1  status=ok`.
  - `Last RX` shows `ACK  cmd_seq=1`.
  - `seq: 1` shown in the control panel.
  - Within one 50 ms telemetry cycle, motor monitor updates:
    - `motor_test_active: true` (green label).
    - Left bar fills to the right (green zone) at approximately 15%.
    - Left label: `Forward / RPWM / 15%`.
    - Left GPIO: `Active: GPIO10 (RPWM)`.
    - Right bar: `Stopped / 0%`.

- [ ] 8. Press `STOP`.

  Confirm:
  - Both sliders return to 0.00.
  - `Latest Response → ACK` updates to show `cmd_seq=2  status=ok`.
  - `seq: 2` shown.
  - Motor monitor clears:
    - `motor_test_active: false`.
    - Both bars: `Stopped / 0%`.

- [ ] 9. Test left reverse:

  Set left slider to approximately -0.15. Press `Send PWM`.

  Confirm:
  - ACK received (`cmd_seq=3`).
  - Motor monitor:
    - Left bar fills to the left (red zone) at approximately 15%.
    - Left label: `Reverse / LPWM / 15%`.
    - Left GPIO: `Active: GPIO11 (LPWM)`.

  Press `STOP`. Confirm motors clear.

- [ ] 10. Test both motors simultaneously:

  Set left to +0.15 and right to +0.15. Press `Send PWM`.

  Confirm:
  - Both bars show green fill at ~15%.
  - Left GPIO: GPIO10 (RPWM). Right GPIO: GPIO12 (RPWM).

  Press `STOP`.

- [ ] 11. Test utility buttons:

  Set left to +0.20, right to -0.10.
  Press `Zero Left` — confirm left slider returns to 0.00.
  Press `Zero Right` — confirm right slider returns to 0.00.
  Set both to non-zero values.
  Press `Zero Both` — confirm both return to 0.00.

- [ ] 12. Test close-on-stop behaviour:

  Set left to +0.15 and press `Send PWM`.
  Close the GUI window.

  Confirm:
  - GUI closes cleanly (no Python traceback).
  - After close, reopen with a separate read tool:

    ```bash
    python3 scripts/read_json_serial.py /dev/ttyACM0
    ```

  - Telemetry shows `motor_test_active: false`, confirming the stop command
    was sent by the GUI before it closed.

- [ ] 13. Test invalid-JSON resilience:

  In a second terminal, while the GUI is running, use another tool to send
  a malformed packet. Because only one process can hold the port, this test
  may be deferred to a firmware-side log inspection. Alternatively, stop the
  GUI, inject a bad line with `send_json_command.py --raw-line`, then reopen
  the GUI. Confirm `Invalid` counter increments and GUI does not crash.

- [ ] 14. Confirm `Send PWM` is disabled after unchecking the safety box:

  Uncheck the safety checkbox mid-session. Confirm `Send PWM` becomes
  disabled immediately. Re-check to re-enable.

## Expected packet flow

```
TX: {"type":"pwm_test","seq":1,"left":0.15,"right":0.0}
RX: {"type":"ack","seq":1,"cmd_seq":1,"status":"ok"}
RX: {"type":"joy","seq":12,...,"motor_left":0.15,"motor_right":0.0,"motor_test_active":true,...}

TX: {"type":"stop","seq":2}
RX: {"type":"ack","seq":2,"cmd_seq":2,"status":"ok"}
RX: {"type":"joy","seq":13,...,"motor_left":0.0,"motor_right":0.0,"motor_test_active":false,...}
```

## Notes

- The firmware version does not change for v0.5.2. `fw` in telemetry may
  show `"0.5.0"` or a later version depending on what is flashed.
- `scripts/send_motor_test.py` and `scripts/wheelchair_gui.py` are still
  available but cannot run simultaneously with `wheelchair_control_gui.py`
  since all three need exclusive serial port access.
- Do not add automatic periodic PWM streaming in this version. Commands are
  sent only when the user presses `Send PWM`.
