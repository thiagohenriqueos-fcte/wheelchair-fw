# Test Plan: v0.3

Version 0.3 validates one-way joystick JSON telemetry from the ESP32-S3 to a
generic Linux host. It does not implement command reception or ROS 2.

## Host preparation

Install `pyserial` in the Python environment used for the test:

```bash
python3 -m pip install pyserial
```

Verify the import:

```bash
python3 -c "import serial; print(serial.__version__)"
```

## Validation checklist

- [ ] 1. Source ESP-IDF with `source "$HOME/esp/esp-idf/export.sh"`.
- [ ] 2. Build the firmware with `idf.py build`.
- [ ] 3. Flash with `idf.py -p /dev/ttyACM0 flash`.
- [ ] 4. Close `idf.py monitor` or any other program using the serial port.
- [ ] 5. Start the host reader:
      `python3 scripts/read_json_serial.py /dev/ttyACM0`.
- [ ] 6. Confirm valid packets contain `type`, `version`, `seq`, `raw_x`,
      `raw_y`, `x`, and `y`.
- [ ] 7. Confirm `seq` increments for each joystick packet.
- [ ] 8. Keep the joystick centered and confirm normalized `x` and `y` are
      near zero.
- [ ] 9. Move the joystick through both axes and confirm raw and normalized
      values change.
- [ ] 10. Confirm upward movement produces positive `y` and downward movement
      produces negative `y`.
- [ ] 11. Confirm ESP-IDF boot and heartbeat lines are counted as invalid
      without stopping the reader.
- [ ] 12. Stop the reader with `Ctrl+C` and confirm it prints valid and
      invalid packet totals.

Example firmware telemetry line:

```json
{"type":"joystick","version":"0.3.0","seq":1,"raw_x":2030,"raw_y":2052,"x":0,"y":0}
```

Example host command for an alternate port:

```bash
python3 scripts/read_json_serial.py /dev/ttyUSB0
```
