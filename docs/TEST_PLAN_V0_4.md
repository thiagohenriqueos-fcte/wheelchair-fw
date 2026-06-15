# Test Plan: v0.4

Version 0.4 validates host-to-ESP32 JSON movement command reception over USB
serial. Commands are parsed, stored, acknowledged, and reported in telemetry.
They do not control motors.

## Prerequisites

- ESP32-S3 connected to the Linux host over USB.
- Joystick powered from 3.3 V and connected according to `docs/PINOUT.md`.
- ESP-IDF environment installed.
- Serial port identified, using `/dev/ttyACM0` below.

Only one process may use the serial port. Close `idf.py monitor`,
`read_json_serial.py`, and `joystick_gui.py` before running the command test
script.

## Procedure

- [ ] 1. Activate ESP-IDF and build the firmware:

  ```bash
  source "$HOME/esp/esp-idf/export.sh"
  idf.py build
  ```

- [ ] 2. Flash the ESP32-S3:

  ```bash
  idf.py -p /dev/ttyACM0 flash
  ```

- [ ] 3. Activate the Python virtual environment:

  ```bash
  source .venv/bin/activate
  ```

- [ ] 4. Install the host requirements:

  ```bash
  python3 -m pip install -r requirements-dev.txt
  ```

- [ ] 5. Send a movement command:

  ```bash
  python3 scripts/send_json_command.py /dev/ttyACM0 \
      --v 0.20 --w 0.00
  ```

- [ ] 6. Confirm the ESP32 responds with an ACK whose `cmd_seq` matches the
  transmitted command sequence.

- [ ] 7. Confirm joystick telemetry includes `cmd_v` near `0.20`, `cmd_w`
  near `0.00`, `cmd_valid` set to `true`, and an increasing
  `last_cmd_age_ms`.

- [ ] 8. Send stop:

  ```bash
  python3 scripts/send_json_command.py /dev/ttyACM0 --stop
  ```

- [ ] 9. Confirm the ESP32 responds with an ACK for the stop command.

- [ ] 10. Confirm telemetry includes `cmd_v` near `0.00` and `cmd_w` near
  `0.00`.

- [ ] 11. Send invalid JSON:

  ```bash
  python3 scripts/send_json_command.py /dev/ttyACM0 \
      --raw-line '{"type":"cmd"'
  ```

  Confirm an `err` packet with `code` set to `invalid_json` is received and
  the firmware continues running.

- [ ] 12. Confirm joystick `raw_x`, `raw_y`, `x`, and `y` telemetry continues
  updating while commands are received.

- [ ] 13. Confirm heartbeat JSON packets continue without interruption.

- [ ] 14. Optionally validate repeated command reception:

  ```bash
  python3 scripts/send_json_command.py /dev/ttyACM0 \
      --v 0.20 --w 0.00 --rate 10 --duration 5
  ```

- [ ] 15. Confirm no PWM, MCPWM, PCNT, encoder, motor-control, PI-control,
  safety, or ROS 2 functionality is present in this version.

## Expected packet examples

```json
{"type":"ack","seq":1,"cmd_seq":1,"status":"ok"}
{"type":"err","seq":2,"code":"invalid_json","status":"error"}
{"type":"joy","seq":10,"t_ms":12345,"fw":"0.4.0","raw_x":2030,"raw_y":2050,"x":0.02,"y":0.0,"cmd_v":0.2,"cmd_w":0.0,"cmd_seq":1,"cmd_valid":true,"last_cmd_age_ms":50,"status":"ok"}
```
