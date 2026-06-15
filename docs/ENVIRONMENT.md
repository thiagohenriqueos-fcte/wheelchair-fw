# Development Environment

The expected host operating system is Linux. ESP-IDF provides and manages
additional Python packages and cross-compilation tools after installation.

## Required tools

Verify the host tools before configuring the project:

| Tool | Verification command |
| --- | --- |
| Git | `git --version` |
| Python 3 | `python3 --version` |
| pip | `pip --version` or `pip3 --version` |
| CMake | `cmake --version` |
| Ninja | `ninja --version` |
| ESP-IDF | `test -f "$HOME/esp/esp-idf/export.sh"` |
| idf.py | `idf.py --version` after sourcing ESP-IDF |

Run the project helper for a consolidated check:

```bash
./scripts/check_env.sh
```

The helper only reports status. It does not install packages or modify the
system.

## Install and activate ESP-IDF

Use the current official ESP-IDF Linux installation instructions from
Espressif. A common installation location is `$HOME/esp/esp-idf`.

Activate an installed environment in every new terminal:

```bash
source "$HOME/esp/esp-idf/export.sh"
```

Then verify it:

```bash
echo "$IDF_PATH"
idf.py --version
```

If ESP-IDF is installed elsewhere, source the `export.sh` from that location.
Do not add guessed paths to system configuration.

## Build configuration

From the project root:

```bash
idf.py set-target esp32s3
idf.py build
```

The generated `sdkconfig` and `build/` directory are intentionally excluded
from Git.

## Find the serial port

Connect the ESP32-S3 over USB, then inspect common serial device names:

```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

Either command may print "No such file or directory" when no matching device
exists. Typical ports are `/dev/ttyACM0` and `/dev/ttyUSB0`.

Flash and monitor with the detected port:

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

Exit the monitor with `Ctrl+]`.

## Linux USB permissions

If the serial device exists but cannot be opened, inspect its owner and group:

```bash
ls -l /dev/ttyACM0
groups
```

Many Linux distributions grant serial access through a group such as
`dialout` or `uucp`. Follow the policy for the host distribution before
changing group membership or udev rules. Commands requiring root privileges
must be reviewed and run manually; project scripts do not execute them.
