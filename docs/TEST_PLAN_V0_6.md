# Test Plan: v0.6

Version 0.6 validates MCPWM PWM output with an IBT-2 H-bridge and a real
motor physically connected. The watchdog, duty-cycle clamping, and direction
control are all verified in this stage.

## DANGER — Motor must be suspended

**The motor wheel must be off the ground and free to spin without load for
every test in this plan.** Never run these commands with the wheel in contact
with the ground or with any load attached. An unexpected direction or runaway
caused by firmware configuration errors could cause injury or damage.

Secure the wheelchair so it cannot move before connecting power to the IBT-2.

## IBT-2 wiring

| IBT-2 pin | ESP32-S3 GPIO | Function |
| --- | --- | --- |
| RPWM (left H-bridge)  | GPIO10 | Left forward |
| LPWM (left H-bridge)  | GPIO11 | Left reverse |
| RPWM (right H-bridge) | GPIO12 | Right forward |
| LPWM (right H-bridge) | GPIO13 | Right reverse |
| R_EN, L_EN | 3V3 | Always-enabled (tie high) |
| VCC | 5 V | Logic supply |
| B+ | Motor supply | 12–24 V motor power |
| GND | GND | Common ground |

The IBT-2 R_EN and L_EN pins must be tied high (to 3.3 V or 5 V) to enable
the H-bridge outputs. If they float, the bridge will not drive the motor
regardless of PWM.

## Prerequisites

- ESP32-S3 connected to the Linux host over USB.
- Joystick connected per `docs/PINOUT.md`.
- IBT-2 H-bridge wired as above, motor leads connected to the bridge output.
- Motor mechanically suspended — wheel clear of the ground.
- Motor power supply connected but **not yet powered on** at test start.
- ESP-IDF environment installed.
- Serial port at `/dev/ttyACM0`.
- Python virtual environment active with `pyserial` installed.

Only one process may use the serial port at a time. Close `idf.py monitor`,
`read_json_serial.py`, and `joystick_gui.py` before running any test script.

## Procedure

- [ ] 1. Build the firmware:

  ```bash
  source "$HOME/esp/esp-idf/export.sh"
  idf.py build
  ```

  Confirm the build log shows:

  ```
  App "wheelchair_esp32s3_firmware" version: 0.6.0
  ```

- [ ] 2. Flash:

  ```bash
  idf.py -p /dev/ttyACM0 flash
  ```

- [ ] 3. Confirm telemetry version:

  ```bash
  source .venv/bin/activate
  python3 scripts/read_json_serial.py /dev/ttyACM0
  ```

  The first status packet must include `"fw":"0.6.0"`. The `motor_pwm` status
  event must show `"status":"ok"`. Stop reading (`Ctrl+C`) before the next
  step.

- [ ] 4. Confirm all PWM outputs are LOW on boot.

  Before applying motor power, use a multimeter or oscilloscope on GPIO10–13.
  All four pins must read 0 V immediately after reset. Apply motor power only
  after this is confirmed.

- [ ] 5. Send a low-duty forward command to the left motor only:

  ```bash
  python3 scripts/send_motor_test.py /dev/ttyACM0 --left 0.15 --right 0.00
  ```

  Expected:
  - ACK packet received: `{"type":"ack","cmd_seq":1,"status":"ok",...}`
  - Joystick telemetry shows `"motor_left":0.15`, `"motor_right":0.0`,
    `"motor_test_active":true`.
  - Left motor spins slowly in the forward direction.
  - Right motor does not move.
  - GPIO10 shows approximately 15% duty at 20 kHz.
  - GPIO11 stays LOW.
  - GPIO12 and GPIO13 stay LOW.

- [ ] 6. Send stop and confirm all outputs go LOW:

  ```bash
  python3 scripts/send_motor_test.py /dev/ttyACM0 --stop
  ```

  Expected:
  - ACK received.
  - Left motor stops immediately.
  - `"motor_test_active":false` in the next telemetry packet.
  - GPIO10–13 all LOW.

- [ ] 7. Test reverse at low duty on the left motor:

  ```bash
  python3 scripts/send_motor_test.py /dev/ttyACM0 --left -0.15 --right 0.00
  ```

  Expected:
  - Left motor spins in the reverse direction.
  - GPIO11 shows approximately 15% duty at 20 kHz.
  - GPIO10 stays LOW.
  - GPIO12 and GPIO13 stay LOW.
  - Telemetry shows `"motor_left":-0.15`.

  Send stop again to confirm motor stops.

- [ ] 8. Test right motor forward at low duty:

  ```bash
  python3 scripts/send_motor_test.py /dev/ttyACM0 --left 0.00 --right 0.15
  ```

  Expected:
  - Right motor spins forward.
  - GPIO12 shows approximately 15% duty.
  - GPIO13 stays LOW.
  - GPIO10 and GPIO11 stay LOW.

  Send stop before the next step.

- [ ] 9. Test watchdog automatic stop:

  Send a command that will run for two seconds, then stop sending:

  ```bash
  python3 scripts/send_motor_test.py /dev/ttyACM0 --left 0.15 --right 0.00 \
      --listen-seconds 3.0
  ```

  The script sends one command and then only reads for 3 seconds. Expected:
  - Motor starts after the command.
  - After approximately 500 ms of silence, the motor stops automatically.
  - `"motor_test_active":false` appears in telemetry within 550 ms of the
    last command (one 50 ms sample period of margin).
  - No stop command is needed from the host.

- [ ] 10. Confirm both motors together at low duty:

  ```bash
  python3 scripts/send_motor_test.py /dev/ttyACM0 --left 0.15 --right 0.15
  ```

  Expected:
  - Both motors spin in the forward direction.
  - GPIO10 and GPIO12 both show duty near 15%.
  - GPIO11 and GPIO13 stay LOW.

  Send stop.

- [ ] 11. Confirm the ±0.30 firmware clamp:

  The host script rejects values outside ±0.30 with a validation error.
  If you bypass the script and send a raw JSON command with `left: 0.80`,
  the firmware must clamp it to 0.30 — the motor must not spin faster than
  the 30% cap.

- [ ] 12. Power down motor supply. Confirm GPIO10–13 return to 0 V after a
  reset (`idf.py -p /dev/ttyACM0 flash` or hardware reset button).

- [ ] 13. Confirm no encoder, PI control, or kinematic logic is present in
  this version.

## Expected packet examples

```json
{"type":"status","event":"boot","status":"ok","fw":"0.6.0",...}
{"type":"status","event":"motor_pwm","status":"ok","fw":"0.6.0",...}
{"type":"ack","seq":1,"cmd_seq":1,"status":"ok"}
{"type":"joy","seq":10,"t_ms":12345,"fw":"0.6.0","motor_left":0.15,"motor_right":0.0,"motor_test_active":true,"status":"ok",...}
{"type":"joy","seq":20,"t_ms":12900,"fw":"0.6.0","motor_left":0.0,"motor_right":0.0,"motor_test_active":false,"status":"ok",...}
```

The second `joy` packet above shows the watchdog firing: `motor_test_active`
is `false` and motor values are zero, without any explicit `stop` command.
