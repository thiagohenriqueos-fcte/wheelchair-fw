# Test Plan: v0.3.2

Version 0.3.2 validates low-pass filtering and frame-based interpolation in
the host-side joystick GUI. It does not change the v0.3.0 ESP32 firmware.

## Validation checklist

- [ ] 1. Activate the Python virtual environment:
      `source .venv/bin/activate`.
- [ ] 2. Install requirements:
      `python3 -m pip install -r requirements-dev.txt`.
- [ ] 3. Close `idf.py monitor` and run the GUI:
      `python3 scripts/joystick_gui.py /dev/ttyACM0`.
- [ ] 4. Confirm the window opens.
- [ ] 5. Move the joystick quickly and confirm the dot moves smoothly.
- [ ] 6. Hold the joystick still and confirm the dot is stable.
- [ ] 7. Confirm the dot remains confined inside the circular boundary.
- [ ] 8. Confirm `raw_x` and `raw_y` still update.
- [ ] 9. Confirm latest numeric `x` and `y` still update independently of the
      smoothed visual position.
- [ ] 10. Confirm the valid packet counter increases.
- [ ] 11. Confirm invalid packets increase the invalid counter and do not
      crash the GUI.
- [ ] 12. Close the window using the Close button or window control and
      confirm the process exits cleanly.

## Smoothing configuration

Default values:

```text
filter alpha: 0.25
interpolation alpha: 0.20
GUI update interval: 33 ms
```

Run with custom smoothing:

```bash
python3 scripts/joystick_gui.py /dev/ttyACM0 \
    --filter-alpha 0.20 \
    --interp-alpha 0.15
```

The `x` and `y` fields show the latest packet. The `visual x` and `visual y`
fields show the interpolated position used to draw the dot.
