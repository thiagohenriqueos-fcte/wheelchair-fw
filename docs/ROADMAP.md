# Firmware Roadmap

Versions v0.1 and v0.2 are implemented in the current repository state.

| Version | Scope | Status |
| --- | --- | --- |
| v0.1 | Project base, Git, environment checks, and minimal boot firmware | Complete |
| v0.2 | ADC joystick reading | Complete |
| v0.3 | ESP32-to-Raspberry Pi JSON telemetry | Planned |
| v0.4 | Raspberry Pi-to-ESP32 JSON command reception | Planned |
| v0.5 | MCPWM generation without a motor | Planned |
| v0.6 | PWM test with a suspended motor | Planned |
| v0.7 | PCNT encoder reading | Planned |
| v0.8 | RPM and speed calculation | Planned |
| v0.9 | Open-loop joystick-to-PWM control | Planned |
| v1.0 | Closed-loop PI control for one wheel | Planned |
| v1.1 | Closed-loop PI control for both wheels | Planned |
| v1.2 | Watchdog and safety states | Planned |
| v1.3 | ROS 2 serial bridge integration | Planned |

Each version should be tested independently before work starts on the next
version.
