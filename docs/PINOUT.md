# Planned Pinout

**Planned, not used in v0.1.**

The assignments below document the current hardware plan only. Version 0.1
does not initialize, read, or drive any of these pins.

| GPIO | Planned signal | Planned peripheral | v0.1 status |
| --- | --- | --- | --- |
| GPIO1 | Joystick X | ADC1 | Future use |
| GPIO2 | Joystick Y | ADC1 | Future use |
| GPIO4 | Encoder left A | PCNT | Future use |
| GPIO5 | Encoder left B | PCNT | Future use |
| GPIO6 | Encoder right A | PCNT | Future use |
| GPIO7 | Encoder right B | PCNT | Future use |
| GPIO10 | IBT-2 left RPWM | MCPWM | Future use |
| GPIO11 | IBT-2 left LPWM | MCPWM | Future use |
| GPIO12 | IBT-2 right RPWM | MCPWM | Future use |
| GPIO13 | IBT-2 right LPWM | MCPWM | Future use |

Pin assignments must be checked against the exact ESP32-S3 board schematic
before a future version enables any peripheral.
