# Test Plan: v0.2

## Wiring

Connect the joystick before powering the board:

| Joystick connection | ESP32-S3 connection |
| --- | --- |
| X axis output | GPIO1 / ADC1_CH0 |
| Y axis output | GPIO2 / ADC1_CH1 |
| VCC | 3V3 |
| GND | GND |

Do not power the joystick from 5 V. ESP32-S3 GPIO inputs are not 5 V tolerant.

## Validation checklist

- [ ] 1. Source ESP-IDF and build the firmware with `idf.py build`.
- [ ] 2. Flash the ESP32-S3 with `idf.py -p <PORT> flash`.
- [ ] 3. Open the monitor with `idf.py -p <PORT> monitor`.
- [ ] 4. Keep the joystick centered and confirm normalized `x` and `y` are
      close to `0.0`.
- [ ] 5. Move joystick X positive and confirm `x` approaches `+1.0`.
- [ ] 6. Move joystick X negative and confirm `x` approaches `-1.0`.
- [ ] 7. Move joystick Y positive and confirm `y` approaches `+1.0`.
- [ ] 8. Move joystick Y negative and confirm `y` approaches `-1.0`.
- [ ] 9. Release the joystick and confirm values return to `0.0` inside the
      deadzone.
- [ ] 10. Confirm `heartbeat=N` continues to print once per second while
      joystick readings are active.

Example output:

```text
Wheelchair ESP32-S3 Firmware
Version: 0.2.0
Target: ESP32-S3
Status: boot_ok
Joystick ADC: ready
joy raw_x=2030 raw_y=2052 x=0.00 y=0.00
heartbeat=1
joy raw_x=2040 raw_y=3060 x=0.00 y=0.49
```

The center and endpoint readings vary between joystick modules. Version 0.2
uses fixed initial values: center `2048`, minimum `0`, maximum `4095`, and
deadzone `0.08`.
