# Test Plan: v0.1

Use this checklist to validate the development environment and minimal
firmware:

- [ ] 1. Source the ESP-IDF environment:
      `source "$HOME/esp/esp-idf/export.sh"`.
- [ ] 2. Run `./scripts/check_env.sh`.
- [ ] 3. Run `idf.py set-target esp32s3`.
- [ ] 4. Run `idf.py build`.
- [ ] 5. Connect the ESP32-S3 to the Linux host by USB.
- [ ] 6. Identify the port with `ls /dev/ttyACM*` and
      `ls /dev/ttyUSB*`.
- [ ] 7. Run `idf.py -p <PORT> flash monitor`.
- [ ] 8. Confirm the firmware name, version, target, and `Status: boot_ok`
      messages appear.
- [ ] 9. Confirm `heartbeat=N` increments once per second.
- [ ] 10. Stop the monitor with `Ctrl+]`.

Expected startup log content:

```text
Wheelchair ESP32-S3 Firmware
Version: 0.1.0
Target: ESP32-S3
Status: boot_ok
heartbeat=1
heartbeat=2
```

Exact ESP-IDF log prefixes and timestamps vary by configuration.
