# Test Plan: v0.6.2

Version 0.6.2 includes three changes:

1. **Fullscreen support** in `scripts/wheelchair_control_gui.py`
   (F11 / Esc / Fullscreen and Exit Fullscreen buttons).
2. **User-configurable PWM limit** in the GUI motor control panel
   (Apply PWM Limit / Reset to 0.30 / warning above 0.30).
3. **25 kHz MCPWM frequency** in `main/drivers/motor_pwm.c`
   (changed from 20 kHz; period ticks reduced from 1000 to 800).

Firmware change: `MOTOR_TEST_MAX_DUTY` raised from 0.30 to 1.0 so the GUI
limit acts as the operator-facing safety gate. The 500 ms watchdog and STOP
command remain fully active.

## IMPORTANT — No motors during software tests

Do not connect motors or apply motor supply power during GUI validation
(steps 1–19). Motor power is only needed for physical PWM validation
(steps 20–26), and only with the motor physically suspended off the ground.

## Prerequisites

- ESP32-S3 with v0.6.2 firmware flashed.
- Joystick connected per `docs/PINOUT.md`.
- Python virtual environment active (`source .venv/bin/activate`).
- No other process holding `/dev/ttyACM0`.

## Build and flash

```bash
cd ~/wheelchair-fw
source ~/esp/esp-idf/export.sh && idf.py build
idf.py -p /dev/ttyACM0 flash
```

## Python syntax check

```bash
source .venv/bin/activate
python3 -m py_compile scripts/wheelchair_control_gui.py
```

Confirm no output (zero errors).

## Procedure — GUI validation

- [ ] 1. Launch the GUI:

  ```bash
  python3 scripts/wheelchair_control_gui.py /dev/ttyACM0
  ```

  Confirm:
  - Window titled "Wheelchair Control  v0.6.2" opens.
  - Status bar shows `Port: connected: /dev/ttyACM0 @ 115200`.
  - `Valid` counter begins incrementing.

- [ ] 2. Confirm telemetry is received:

  Confirm `raw_x`, `raw_y`, `x`, `y` update. Move joystick; confirm dot moves.

- [ ] 3. Confirm firmware version:

  Confirm `FW: 0.6.2` in the status bar.

- [ ] 4. Confirm view toolbar:

  Confirm "View:" label with "Fullscreen  (F11)" and "Exit Fullscreen  (Esc)"
  buttons are visible below the status bar.

- [ ] 5. Enter fullscreen with keyboard:

  Press `F11`. Confirm the window fills the screen.
  Confirm all panels are still visible: joystick, motor monitor, motor control,
  smoothing settings, safety, status bar.
  Confirm STOP button is visible and large.

- [ ] 6. Exit fullscreen with keyboard:

  Press `Esc`. Confirm the window returns to its normal windowed size.

- [ ] 7. Enter fullscreen with button:

  Click "Fullscreen  (F11)". Confirm fullscreen enters.

- [ ] 8. Exit fullscreen with button:

  Click "Exit Fullscreen  (Esc)". Confirm fullscreen exits.

- [ ] 9. Confirm STOP in fullscreen:

  Tick the safety checkbox. Set left slider to +0.15. Press `Send Once`.
  Enter fullscreen (F11). Confirm STOP button is visible and clickable.
  Press STOP. Confirm:
  - Both sliders zero.
  - Stop ACK received.
  - Motor monitor shows `motor_test_active: false`.
  Exit fullscreen.

- [ ] 10. Confirm default PWM limit:

  In the motor control panel, confirm:
  - "PWM limit:" slider shows 0.30.
  - No warning label is visible.
  - Left and right motor sliders are bounded to ±0.30 (cannot drag beyond).

- [ ] 11. Reduce PWM limit to 0.20:

  Drag the PWM limit slider to 0.20. Click "Apply PWM Limit".
  Confirm:
  - Motor slider range contracts to ±0.20.
  - No warning label (0.20 ≤ 0.30).

- [ ] 12. Send Once at 0.20 limit:

  Set left slider to +0.20. Press `Send Once`.
  Confirm ACK received and motor monitor shows `motor_left ≈ 0.20`.
  Press STOP.

- [ ] 13. Raise PWM limit to 0.50:

  Drag PWM limit slider to 0.50. Click "Apply PWM Limit".
  Confirm:
  - Warning label appears:
    "⚠  PWM limit above 0.30 — use only with suspended motor and controlled
    test conditions."
  - Motor slider range expands to ±0.50.

- [ ] 14. Confirm slider clamp on limit reduction:

  Set left slider to +0.45 (within ±0.50). Reduce PWM limit to 0.20.
  Click "Apply PWM Limit". Confirm:
  - Left slider clamped to +0.20.
  - No warning (0.20 ≤ 0.30).

- [ ] 15. Reset PWM Limit to 0.30:

  Click "Reset to 0.30". Confirm:
  - PWM limit slider returns to 0.30.
  - Motor slider range returns to ±0.30.
  - Warning label clears.

- [ ] 16. Start PWM stream:

  Set limit to 0.30 (default). Tick safety. Set left +0.15, right 0.00.
  Click "Start PWM Stream". Confirm stream is ON and seq increments at ~10 Hz.

- [ ] 17. Press STOP during stream:

  Press STOP. Confirm:
  - Stream stops immediately.
  - Both sliders zero.
  - ACK for stop received.
  - Motor monitor clears.

- [ ] 18. Confirm stream unaffected by limit changes:

  Start stream (left +0.10). While streaming, adjust PWM limit slider to 0.50
  without clicking Apply. Confirm stream continues (limit is not applied until
  Apply is pressed). Click "Stop PWM Stream".

- [ ] 19. Close GUI:

  Close the window. Confirm clean exit (no Python traceback). Verify with
  `scripts/read_json_serial.py` that `motor_test_active: false` in telemetry.

## Procedure — Physical PWM validation (oscilloscope / logic analyzer)

Perform these steps with the motor physically suspended off the ground and
motor supply power applied. Do not hold or touch the motor shaft during tests.

- [ ] 20. Confirm 25 kHz frequency on all active outputs:

  After flashing v0.6.2, connect oscilloscope probes to GPIO10 and GPIO11.
  Send `left: +0.20` via GUI `Send Once`. Confirm:
  - GPIO10 shows PWM at approximately **25 kHz** (period ≈ 40 µs).
  - GPIO11 stays LOW.

- [ ] 21. Confirm left forward duty cycle at 0.20:

  GPIO10 duty cycle should be approximately **20%** (8 µs HIGH per 40 µs period).
  Period ticks = 800; compare value for 0.20 duty = 160 ticks.

- [ ] 22. Confirm left reverse:

  Send `left: -0.20`. Confirm:
  - GPIO11 shows PWM at 25 kHz with approximately 20% duty.
  - GPIO10 stays LOW.

- [ ] 23. Confirm right forward:

  Send `right: +0.20` (left: 0). Confirm:
  - GPIO12 shows PWM at 25 kHz with approximately 20% duty.
  - GPIO13 stays LOW.

- [ ] 24. Confirm right reverse:

  Send `right: -0.20`. Confirm:
  - GPIO13 shows PWM at 25 kHz with approximately 20% duty.
  - GPIO12 stays LOW.

- [ ] 25. Confirm STOP forces all outputs LOW:

  Press STOP. Confirm GPIO10, GPIO11, GPIO12, GPIO13 all stay LOW.

- [ ] 26. Confirm watchdog fires at 500 ms:

  Start PWM stream (left +0.20). Kill the GUI process or disconnect serial
  unexpectedly. After 500 ms, confirm all GPIO outputs return to LOW.

## Commit after validation

```bash
git status
git add .
git commit -m "v0.6.2: add fullscreen GUI, PWM limit control, and 25 kHz PWM"
git push
```

## Tag after validation

```bash
git tag -a v0.6.2 -m "Version 0.6.2: fullscreen GUI, PWM limit control, and 25 kHz PWM"
git push origin v0.6.2
```

Do not tag v0.6.2 until both GUI and physical PWM frequency have been
manually validated.

## Notes

- The firmware hard clamp is now 1.0 (±100%). The GUI PWM limit (default 0.30)
  is the operator-facing safety gate. Commands above 0.30 will reach the motor
  if the GUI limit is raised — only raise it with a suspended motor.
- Period ticks at 25 kHz with 20 MHz resolution: 800 (was 1000 at 20 kHz).
  Duty granularity: 0.125% per tick (was 0.1%).
- The 500 ms watchdog and STOP behavior are unchanged from v0.6.
- `scripts/wheelchair_gui.py` and `scripts/send_motor_test.py` still work
  alongside v0.6.2 firmware (protocol unchanged).
