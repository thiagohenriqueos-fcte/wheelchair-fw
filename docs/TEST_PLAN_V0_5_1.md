# Test Plan: v0.5.1

Version 0.5.1 adds a combined joystick and motor PWM monitor GUI
(`scripts/wheelchair_gui.py`). This is a host-side Python script only. No
firmware changes are included.

The GUI is a read-only monitor. Motor PWM commands must still be sent with
`scripts/send_motor_test.py`. Only one process can use the serial port at a
time — close the GUI before running `send_motor_test.py`, and vice versa.

## Prerequisites

- ESP32-S3 running firmware v0.5.0 or later, connected over USB.
- Joystick connected per `docs/PINOUT.md`.
- Python virtual environment active.
- No other process using `/dev/ttyACM0`.

## Procedure

- [ ] 1. Activate the virtual environment:

  ```bash
  cd ~/wheelchair-fw
  source .venv/bin/activate
  ```

- [ ] 2. Compile-check the script:

  ```bash
  python3 -m py_compile scripts/wheelchair_gui.py
  ```

  Confirm no output (zero errors).

- [ ] 3. Launch the GUI:

  ```bash
  python3 scripts/wheelchair_gui.py /dev/ttyACM0
  ```

  Confirm:
  - Window titled "Wheelchair Monitor" opens.
  - Status bar shows `Port: connected: /dev/ttyACM0 @ 115200`.
  - `Valid` counter begins incrementing.
  - `Invalid` counter stays at 0.
  - `FW` field shows `0.5.0` (or the current firmware version).
  - `HB seq` field increments approximately every second.

- [ ] 4. Confirm joystick dot moves:

  Move the physical joystick (or observe idle drift). Confirm:
  - The dot moves smoothly inside the circular boundary.
  - The dot stays clamped within the circle for large deflections.
  - `raw_x`, `raw_y`, `x`, and `y` numeric labels update continuously.
  - Y-axis convention: pushing the joystick forward (away from you) gives
    positive `y` and moves the dot upward.

- [ ] 5. Confirm motor panel shows stopped when `motor_test_active` is false:

  With no prior `pwm_test` command sent, confirm:
  - `motor_test_active: false` (label in grey).
  - Both motor bars show zero width (no coloured fill, only the centre line).
  - Both label lines show `Stopped / 0%`.
  - Both GPIO lines show `Active GPIO: —`.

- [ ] 6. Close the GUI window (press the window close button or `Ctrl+C`).
  Confirm it closes cleanly without errors.

- [ ] 7. Send a PWM test command:

  ```bash
  python3 scripts/send_motor_test.py /dev/ttyACM0 --left 0.5 --right 0.5
  ```

  Wait for the ACK response.

- [ ] 8. Reopen the GUI:

  ```bash
  python3 scripts/wheelchair_gui.py /dev/ttyACM0
  ```

  Confirm (if firmware v0.5.0 without watchdog, command state persists):
  - `motor_test_active: true` (label in green).
  - Left motor bar fills to the right (green zone) at approximately 50%.
  - Right motor bar fills to the right (green zone) at approximately 50%.
  - Left label shows `Forward / RPWM / 50%`.
  - Right label shows `Forward / RPWM / 50%`.
  - Left GPIO shows `Active GPIO: GPIO10 (RPWM)`.
  - Right GPIO shows `Active GPIO: GPIO12 (RPWM)`.

  If firmware v0.6.0 or later (with 500 ms watchdog), the state may have
  expired. In that case `motor_test_active` will be `false` and bars will
  show stopped. This is expected.

- [ ] 9. Close the GUI. Send stop:

  ```bash
  python3 scripts/send_motor_test.py /dev/ttyACM0 --stop
  ```

- [ ] 10. Reopen the GUI. Confirm:

  - `motor_test_active: false`.
  - Both motor bars show `Stopped / 0%`.
  - Both GPIO lines show `Active GPIO: —`.

- [ ] 11. Test reverse display:

  Close the GUI. Send a reverse command:

  ```bash
  python3 scripts/send_motor_test.py /dev/ttyACM0 --left -0.25 --right 0.0
  ```

  Reopen the GUI. Confirm:
  - Left motor bar fills to the left (red zone) at approximately 25%.
  - Left label shows `Reverse / LPWM / 25%`.
  - Left GPIO shows `Active GPIO: GPIO11 (LPWM)`.
  - Right motor shows `Stopped / 0%`.

- [ ] 12. Confirm the orange dashed tick marks inside the motor bars. They
  mark ±30% and should remain visible at all times. The bar fill should not
  extend past the orange ticks for commands at the ±30% firmware clamp limit.

- [ ] 13. Confirm invalid JSON lines do not crash the GUI:

  In a separate terminal, open the serial port and send a malformed line
  using `send_json_command.py --raw-line`:

  ```bash
  python3 scripts/send_json_command.py /dev/ttyACM0 --raw-line 'not json'
  ```

  The GUI must stay open and the `Invalid` counter must increment by one.

- [ ] 14. Confirm the GUI remains responsive while receiving telemetry.
  The joystick dot should continue to animate during all tests.

## Notes

- `wheelchair_gui.py` replaces `joystick_gui.py` as the primary monitor for
  v0.5.1 and later. `joystick_gui.py` is preserved for reference.
- The existing `send_motor_test.py` remains the tool used to send `pwm_test`
  commands. Do not add send buttons to the monitor GUI in this version.
- A future combined control GUI that multiplexes serial access may allow
  simultaneous monitoring and command sending.
