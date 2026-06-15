# Pinout

GPIO1 and GPIO2 are used for joystick ADC validation in v0.2. All other
assignments remain planned future use.

The joystick must be powered from **3.3 V, not 5 V**, with its ground connected
to ESP32-S3 ground.

| GPIO | Signal | Peripheral/channel | Status |
| --- | --- | --- | --- |
| GPIO1 | Joystick X | ADC1_CH0 | Used in v0.2 |
| GPIO2 | Joystick Y | ADC1_CH1 | Used in v0.2 |
| GPIO4 | Encoder left A | PCNT | Future use |
| GPIO5 | Encoder left B | PCNT | Future use |
| GPIO6 | Encoder right A | PCNT | Future use |
| GPIO7 | Encoder right B | PCNT | Future use |
| GPIO10 | IBT-2 left RPWM | MCPWM | Future use |
| GPIO11 | IBT-2 left LPWM | MCPWM | Future use |
| GPIO12 | IBT-2 right RPWM | MCPWM | Future use |
| GPIO13 | IBT-2 right LPWM | MCPWM | Future use |

Future pin assignments must be checked against the exact ESP32-S3 board
schematic before their peripherals are enabled.
