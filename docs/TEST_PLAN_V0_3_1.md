# Test Plan: v0.3.1

Version 0.3.1 validates the host-side Tkinter joystick monitor. It does not
change the v0.3.0 ESP32 firmware or add host-to-ESP32 commands.

## Validation checklist

- [ ] 1. Activate the Python virtual environment:
      `source .venv/bin/activate`.
- [ ] 2. Install requirements:
      `python3 -m pip install -r requirements-dev.txt`.
- [ ] 3. Flash the ESP32-S3 with the v0.3.0 firmware if needed:
      `idf.py -p /dev/ttyACM0 flash`.
- [ ] 4. Close `idf.py monitor` and run the GUI:
      `python3 scripts/joystick_gui.py /dev/ttyACM0`.
- [ ] 5. Confirm the GUI window opens.
- [ ] 6. Confirm the dot stays near the center when the joystick is centered.
- [ ] 7. Move the joystick up and confirm the dot moves upward.
- [ ] 8. Move the joystick down and confirm the dot moves downward.
- [ ] 9. Move the joystick left and confirm the dot moves left.
- [ ] 10. Move the joystick right and confirm the dot moves right.
- [ ] 11. Confirm `raw_x` and `raw_y` update.
- [ ] 12. Confirm normalized `x` and `y` update.
- [ ] 13. Confirm the valid packet counter increases.
- [ ] 14. Confirm invalid packets increase the invalid counter and do not
      crash the GUI.
- [ ] 15. Close the window using the Close button or window control and
      confirm the process exits cleanly.

## Expected behavior

- Positive X moves the dot right and negative X moves it left.
- Positive Y moves the dot up and negative Y moves it down.
- The serial thread never updates Tkinter widgets directly.
- Current firmware packets show `t_ms` as `n/a` because v0.3.0 does not
  transmit that field.
- Current firmware packets show status as `ok` unless an explicit `status`
  field is present.

The GUI default baud rate is 115200. To provide it explicitly:

```bash
python3 scripts/joystick_gui.py /dev/ttyACM0 --baud 115200
```
