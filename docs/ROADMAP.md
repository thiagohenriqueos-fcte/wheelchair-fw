# Firmware Roadmap

Only v0.1 is implemented in the current repository state.

| Version | Planned scope |
| --- | --- |
| v0.1 | Project base, Git, environment checks, and minimal boot firmware |
| v0.2 | ADC joystick reading |
| v0.3 | ESP32-to-Raspberry Pi JSON telemetry |
| v0.4 | Raspberry Pi-to-ESP32 JSON command reception |
| v0.5 | MCPWM generation without a motor |
| v0.6 | PWM test with a suspended motor |
| v0.7 | PCNT encoder reading |
| v0.8 | RPM and speed calculation |
| v0.9 | Open-loop joystick-to-PWM control |
| v1.0 | Closed-loop PI control for one wheel |
| v1.1 | Closed-loop PI control for both wheels |
| v1.2 | Watchdog and safety states |
| v1.3 | ROS 2 serial bridge integration |

Each version should be tested independently before work starts on the next
version.
