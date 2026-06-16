#!/usr/bin/env python3
"""Send pwm_test commands to the ESP32-S3 and print its JSON responses.

Usage examples:
  python3 scripts/send_motor_test.py /dev/ttyACM0 --left 0.5 --right 0.5
  python3 scripts/send_motor_test.py /dev/ttyACM0 --left 0.25 --right -0.25
  python3 scripts/send_motor_test.py /dev/ttyACM0 --stop
"""

import argparse
import json
import math
import sys
import time
from typing import Any, Optional, Tuple


DEFAULT_BAUD_RATE = 115200
DEFAULT_LISTEN_SECONDS = 2.0
DEFAULT_STARTUP_DELAY = 1.0
UINT32_MAX = (1 << 32) - 1


MOTOR_TEST_MAX_DUTY = 0.30


def clamped_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError("must be a finite number")
    if result < -MOTOR_TEST_MAX_DUTY or result > MOTOR_TEST_MAX_DUTY:
        raise argparse.ArgumentTypeError(
            f"must be between -{MOTOR_TEST_MAX_DUTY} and {MOTOR_TEST_MAX_DUTY} "
            "(v0.6 motor test limit — motor must be suspended)"
        )
    return result


def nonnegative_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise argparse.ArgumentTypeError("must be a finite value >= 0")
    return result


def uint32_value(value: str) -> int:
    result = int(value)
    if result < 0 or result > UINT32_MAX:
        raise argparse.ArgumentTypeError(f"must be between 0 and {UINT32_MAX}")
    return result


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Send pwm_test commands to the ESP32-S3 motor PWM driver and "
            "print JSON responses. left/right are duty-cycle fractions in [-1.0, 1.0]."
        )
    )
    parser.add_argument(
        "port",
        help="Serial port, for example /dev/ttyACM0.",
    )
    parser.add_argument(
        "--baud-rate",
        dest="baud_rate",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help=f"Serial baud rate (default: {DEFAULT_BAUD_RATE}).",
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="Send a stop command to clear the motor test state.",
    )
    parser.add_argument(
        "--left",
        type=clamped_float,
        default=0.0,
        help="Left motor command in [-0.30, 0.30]. Positive = forward (RPWM).",
    )
    parser.add_argument(
        "--right",
        type=clamped_float,
        default=0.0,
        help="Right motor command in [-0.30, 0.30]. Positive = forward (RPWM).",
    )
    parser.add_argument(
        "--seq",
        type=uint32_value,
        default=1,
        help="Command sequence number (default: 1).",
    )
    parser.add_argument(
        "--listen-seconds",
        type=nonnegative_float,
        default=DEFAULT_LISTEN_SECONDS,
        help=f"Seconds to keep reading after the command (default: {DEFAULT_LISTEN_SECONDS}).",
    )
    parser.add_argument(
        "--startup-delay",
        type=nonnegative_float,
        default=DEFAULT_STARTUP_DELAY,
        help=f"Seconds to wait after opening the port before sending (default: {DEFAULT_STARTUP_DELAY}).",
    )

    args = parser.parse_args()

    if args.stop and (args.left != 0.0 or args.right != 0.0):
        parser.error("--stop cannot be combined with --left or --right")

    return args


def parse_json_line(raw_line: bytes) -> Tuple[bool, Optional[Any], str]:
    line = raw_line.decode("utf-8", errors="replace").strip()
    if not line:
        return False, None, line

    try:
        return True, json.loads(line), line
    except json.JSONDecodeError:
        return False, None, line


def print_received_line(raw_line: bytes) -> Tuple[int, int]:
    is_valid, packet, decoded_line = parse_json_line(raw_line)
    if not decoded_line:
        return 0, 0

    if not is_valid:
        print(f"RX INVALID: {decoded_line}", file=sys.stderr)
        return 0, 1

    print(f"RX {json.dumps(packet, sort_keys=True)}")
    return 1, 0


def main() -> int:
    args = parse_arguments()

    try:
        import serial
    except ModuleNotFoundError:
        print(
            "Missing dependency: pyserial. Install it with "
            "'python3 -m pip install -r requirements-dev.txt'.",
            file=sys.stderr,
        )
        return 2

    valid_packets = 0
    invalid_packets = 0

    if args.stop:
        packet = {"type": "stop", "seq": args.seq}
    else:
        packet = {
            "type": "pwm_test",
            "seq": args.seq,
            "left": args.left,
            "right": args.right,
        }

    tx_line = json.dumps(packet, separators=(",", ":"))

    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baud_rate,
            timeout=0.05,
            write_timeout=1,
        ) as connection:
            print(
                f"Connected to {args.port} at {args.baud_rate} baud. "
                "Press Ctrl+C to stop."
            )

            time.sleep(args.startup_delay)

            connection.write((tx_line + "\n").encode("utf-8"))
            connection.flush()
            print(f"TX {tx_line}")

            final_read_until = time.monotonic() + args.listen_seconds

            while time.monotonic() < final_read_until:
                raw_line = connection.readline()
                if raw_line:
                    valid_delta, invalid_delta = print_received_line(raw_line)
                    valid_packets += valid_delta
                    invalid_packets += invalid_delta

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except serial.SerialException as error:
        print(f"Serial error: {error}", file=sys.stderr)
        return 1
    finally:
        print(
            f"Summary: valid={valid_packets} invalid={invalid_packets}",
            file=sys.stderr,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
