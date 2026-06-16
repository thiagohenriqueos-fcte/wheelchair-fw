# Test Plan: v0.6.1

Version 0.6.1 adds a **Joystick smoothing settings** panel to
`scripts/wheelchair_control_gui.py`. The panel contains three live-adjustable
controls — filter alpha, interpolation alpha, and GUI update interval — plus
a reset button. Values take effect immediately without restarting the GUI.

The joystick panel now also shows four internal state values:
`filt_x`, `filt_y` (filtered) and `vis_x`, `vis_y` (visual dot position).

This is a host-side Python script change only. No firmware changes are included.

## IMPORTANT — No motors during this test

Do not connect motors or apply motor supply power during this validation.
Joystick smoothing settings affect visualization only and have no effect on
motor commands.

## Prerequisites

- ESP32-S3 running firmware v0.5.0 or later, connected over USB.
- Joystick connected per `docs/PINOUT.md`.
- Python virtual environment active (`source .venv/bin/activate`).
- No other process holding `/dev/ttyACM0`.

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
  - Window titled "Wheelchair Control  v0.6.1" opens.
  - Status bar shows `Port: connected: /dev/ttyACM0 @ 115200`.
  - `Valid` counter begins incrementing.
  - **Joystick smoothing settings** panel is visible below the three main panels.
  - Filter alpha slider starts at 0.25, value label shows `0.25`.
  - Interp alpha slider starts at 0.20, value label shows `0.20`.
  - Update interval slider starts at 33, value label shows `33`.
  - `Reset smoothing defaults` button is visible.
  - Joystick panel shows 8 numeric rows: `raw_x`, `raw_y`, `x`, `y`,
    `filt_x`, `filt_y`, `vis_x`, `vis_y`.

- [ ] 3. Confirm telemetry is received:

  Confirm `raw_x`, `raw_y`, `x`, `y` update as joystick input arrives.
  Confirm `filt_x`, `filt_y`, `vis_x`, `vis_y` also update (in grey).
  Move the joystick and confirm the dot moves smoothly inside the circle.

- [ ] 4. Confirm filtering state labels trail the raw values:

  Move the joystick quickly to one extreme. Observe that `x`/`y` jump
  immediately while `filt_x`/`filt_y` lag behind, and `vis_x`/`vis_y` lag
  further. Releasing back to center, confirm all three converge slowly.

- [ ] 5. Set filter alpha to 1.00:

  Drag the Filter alpha slider to the far right (1.00).
  Confirm value label shows `1.00`.

  Move the joystick. Confirm `filt_x`/`filt_y` now track `x`/`y` with
  minimal delay (no smoothing on the filter stage).

- [ ] 6. Set filter alpha to 0.10:

  Drag the Filter alpha slider to approximately 0.10.
  Confirm value label shows `0.10`.

  Move the joystick. Confirm `filt_x`/`filt_y` respond more slowly than
  `x`/`y`. The dot should move noticeably more smoothly/slowly.

- [ ] 7. Set interpolation alpha to 1.00:

  Drag the Interp alpha slider to the far right (1.00).
  Confirm value label shows `1.00`.

  Move the joystick. Confirm `vis_x`/`vis_y` now jump directly to
  `filt_x`/`filt_y` on each frame with no additional delay.

- [ ] 8. Set interpolation alpha to 0.10:

  Drag the Interp alpha slider to approximately 0.10.
  Confirm value label shows `0.10`.

  Move the joystick. Confirm `vis_x`/`vis_y` trail `filt_x`/`filt_y`
  noticeably — the visual dot is very sluggish compared to the filtered values.

- [ ] 9. Set GUI update interval to 16 ms:

  Drag the Update interval slider to 16.
  Confirm value label shows `16`.
  Confirm the GUI remains responsive, the joystick dot continues animating.

- [ ] 10. Set GUI update interval to 50 ms:

  Drag the Update interval slider to 50.
  Confirm value label shows `50`.
  Confirm the GUI remains responsive. The dot animation will be visibly
  less smooth (fewer frames) but functional.

- [ ] 11. Press `Reset smoothing defaults`:

  Confirm:
  - Filter alpha returns to 0.25.
  - Interp alpha returns to 0.20.
  - Update interval returns to 33.
  - All three value labels update immediately.
  - Joystick dot resumes default smoothing behaviour.

- [ ] 12. Confirm raw display is not filtered:

  Set filter alpha to 0.01 (maximum smoothing on the filter stage).
  Move the joystick rapidly. Confirm:
  - `raw_x`, `raw_y` still show the latest ADC values from telemetry (not smoothed).
  - `x`, `y` show the latest normalized values (not smoothed).
  - Only `filt_x`/`filt_y` and `vis_x`/`vis_y` are sluggish.

- [ ] 13. Confirm STOP works normally:

  Tick the safety checkbox. Set left slider to +0.15. Press `Send Once`.
  Confirm ACK received and motor monitor shows active.
  Press `STOP`. Confirm:
  - Both sliders return to 0.00.
  - Stop ACK received.
  - Motor monitor clears.

- [ ] 14. Confirm PWM stream is unaffected by smoothing settings:

  Start the PWM stream (safety checkbox ticked, left +0.10, right +0.10).
  While streaming, adjust filter alpha, interp alpha, and update interval.
  Confirm:
  - Stream continues without interruption.
  - Seq counter keeps incrementing.
  - Motor monitor stays active.
  - Smoothing changes only affect the joystick dot visualization.
  
  Press `Stop PWM Stream`.

- [ ] 15. Confirm STOP during stream still works:

  Start stream again. Change filter alpha to 0.01. Press `STOP`.
  Confirm stream stops and motors clear, regardless of current smoothing settings.

- [ ] 16. Confirm safety checkbox interaction:

  Uncheck the safety checkbox while adjusting smoothing sliders.
  Confirm smoothing sliders remain adjustable regardless of the safety checkbox
  state (they are visualization-only, not gated by safety).

- [ ] 17. Confirm close-on-stop behaviour:

  Start the stream. While streaming, close the GUI window.
  Confirm:
  - GUI closes cleanly (no Python traceback).
  - After close, verify with `scripts/read_json_serial.py` that the firmware
    reports `motor_test_active: false`.

- [ ] 18. Confirm slider bounds:

  Set filter alpha slider to the leftmost position. Confirm value shows `0.01`
  (not 0.00 — the minimum is clamped to avoid freezing the display).
  Set interp alpha to leftmost. Confirm `0.01`.
  Set update interval to leftmost. Confirm `10` (ms).
  Set update interval to rightmost. Confirm `100` (ms).

- [ ] 19. Confirm command-line defaults are honoured:

  Close the GUI. Relaunch with explicit overrides:

  ```bash
  python3 scripts/wheelchair_control_gui.py /dev/ttyACM0 \
      --filter-alpha 0.5 --interp-alpha 0.8 --gui-update-ms 20
  ```

  Confirm:
  - Filter alpha slider starts at 0.50.
  - Interp alpha slider starts at 0.80.
  - Update interval slider starts at 20.

## Expected smoothing pipeline

```
telemetry arrival (variable rate, ~20 Hz):
  latest_x = pkt["x"]   # raw normalized value from firmware

per GUI frame (default 33 ms ≈ 30 Hz):
  filtered_x = filter_alpha * latest_x + (1 - filter_alpha) * filtered_x
  visual_x   = interp_alpha * filtered_x + (1 - interp_alpha) * visual_x
  dot drawn at clamp_unit_circle(visual_x, visual_y)
```

Motor slider values are read directly from the slider widgets and are
completely independent of this pipeline.

## Notes

- No firmware changes in v0.6.1. The `fw` field in telemetry shows the
  version of whatever firmware is flashed (v0.5.0 through v0.6.0).
- Smoothing settings affect only the joystick dot visualization.
  They have no influence on `pwm_test` commands sent to the firmware.
- The `Reset smoothing defaults` button resets to the module-level constants
  `FILTER_ALPHA`, `INTERP_ALPHA`, `GUI_UPDATE_MS` (0.25, 0.20, 33).
- Do not tag v0.6.1 until this test plan is manually validated.
