#!/usr/bin/env bash

cat <<'EOF'
Wheelchair firmware: Linux setup notes

This script only prints guidance. It does not modify the system.

1. Install Git, Python 3 with pip, CMake, and Ninja using the package manager
   for your Linux distribution.

2. Install ESP-IDF by following Espressif's official Linux instructions.
   A common installation directory is:

     $HOME/esp/esp-idf

3. Activate ESP-IDF in each new terminal:

     source "$HOME/esp/esp-idf/export.sh"

4. Check the project environment:

     ./scripts/check_env.sh

5. Configure and build:

     idf.py set-target esp32s3
     idf.py build

6. After connecting the board, look for its serial port:

     ls /dev/ttyACM*
     ls /dev/ttyUSB*

7. Flash and monitor, replacing the port when necessary:

     idf.py -p /dev/ttyACM0 flash monitor

If serial access is denied, review your distribution's serial-device group
and udev guidance. Inspect and run any command requiring root privileges
manually.
EOF
