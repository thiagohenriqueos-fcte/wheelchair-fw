#!/usr/bin/env python3

import argparse
import json
import sys
from typing import Any, Tuple


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read newline-delimited JSON telemetry from a serial port."
    )
    parser.add_argument(
        "port",
        help="Serial port, for example /dev/ttyACM0 or /dev/ttyUSB0.",
    )
    parser.add_argument(
        "--baud-rate",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200).",
    )
    return parser.parse_args()


def parse_json_line(raw_line: bytes) -> Tuple[bool, Any, str]:
    line = raw_line.decode("utf-8", errors="replace").strip()
    if not line:
        return False, None, line

    try:
        return True, json.loads(line), line
    except json.JSONDecodeError:
        return False, None, line


def main() -> int:
    args = parse_arguments()

    try:
        import serial
    except ModuleNotFoundError:
        print(
            "Missing dependency: pyserial. Install it with "
            "'python3 -m pip install pyserial'.",
            file=sys.stderr,
        )
        return 2

    valid_packets = 0
    invalid_packets = 0

    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baud_rate,
            timeout=1,
        ) as connection:
            print(
                f"Reading JSON lines from {args.port} at "
                f"{args.baud_rate} baud. Press Ctrl+C to stop."
            )

            while True:
                raw_line = connection.readline()
                if not raw_line:
                    continue

                is_valid, packet, decoded_line = parse_json_line(raw_line)
                if not is_valid:
                    if decoded_line:
                        invalid_packets += 1
                        print(f"INVALID: {decoded_line}", file=sys.stderr)
                    continue

                valid_packets += 1
                print(json.dumps(packet, sort_keys=True))

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
