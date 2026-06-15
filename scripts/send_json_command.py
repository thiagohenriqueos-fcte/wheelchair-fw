#!/usr/bin/env python3

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


def positive_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite value greater than 0")
    return result


def nonnegative_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise argparse.ArgumentTypeError(
            "must be a finite value greater than or equal to 0"
        )
    return result


def uint32_value(value: str) -> int:
    result = int(value)
    if result < 0 or result > UINT32_MAX:
        raise argparse.ArgumentTypeError(
            f"must be between 0 and {UINT32_MAX}"
        )
    return result


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Send newline-delimited JSON movement commands to the ESP32-S3 "
            "and print its JSON responses."
        )
    )
    parser.add_argument(
        "port",
        help="Serial port, for example /dev/ttyACM0 or /dev/ttyUSB0.",
    )
    parser.add_argument(
        "--baud-rate",
        "--baud",
        dest="baud_rate",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help=f"Serial baud rate (default: {DEFAULT_BAUD_RATE}).",
    )

    command_group = parser.add_mutually_exclusive_group()
    command_group.add_argument(
        "--stop",
        action="store_true",
        help="Send a stop command.",
    )
    command_group.add_argument(
        "--raw-line",
        help="Send one literal line, useful for invalid-JSON testing.",
    )

    parser.add_argument("--v", type=float, help="Linear velocity in m/s.")
    parser.add_argument("--w", type=float, help="Angular velocity in rad/s.")
    parser.add_argument(
        "--seq",
        type=uint32_value,
        default=1,
        help="Initial host command sequence number (default: 1).",
    )
    parser.add_argument(
        "--rate",
        type=positive_float,
        help="Repeated command rate in Hz; requires --duration.",
    )
    parser.add_argument(
        "--duration",
        type=positive_float,
        help="Repeated command duration in seconds; requires --rate.",
    )
    parser.add_argument(
        "--listen-seconds",
        type=nonnegative_float,
        default=DEFAULT_LISTEN_SECONDS,
        help=(
            "Seconds to keep reading after the final command "
            f"(default: {DEFAULT_LISTEN_SECONDS})."
        ),
    )
    parser.add_argument(
        "--startup-delay",
        type=nonnegative_float,
        default=DEFAULT_STARTUP_DELAY,
        help=(
            "Seconds to wait after opening the port before sending "
            f"(default: {DEFAULT_STARTUP_DELAY})."
        ),
    )

    args = parser.parse_args()

    if args.stop or args.raw_line is not None:
        if args.v is not None or args.w is not None:
            parser.error("--v and --w cannot be combined with --stop or --raw-line")
    elif args.v is None or args.w is None:
        parser.error("provide both --v and --w, or use --stop or --raw-line")

    if args.v is not None and (
        not math.isfinite(args.v) or not math.isfinite(args.w)
    ):
        parser.error("--v and --w must be finite numbers")

    if (args.rate is None) != (args.duration is None):
        parser.error("--rate and --duration must be used together")

    if args.raw_line is not None and args.rate is not None:
        parser.error("--raw-line cannot be repeated")

    return args


def parse_json_line(raw_line: bytes) -> Tuple[bool, Optional[Any], str]:
    line = raw_line.decode("utf-8", errors="replace").strip()
    if not line:
        return False, None, line

    try:
        return True, json.loads(line), line
    except json.JSONDecodeError:
        return False, None, line


def build_command(args: argparse.Namespace, sequence: int) -> str:
    if args.raw_line is not None:
        return args.raw_line.rstrip("\r\n")

    if args.stop:
        packet = {"type": "stop", "seq": sequence}
    else:
        packet = {
            "type": "cmd",
            "seq": sequence,
            "v": args.v,
            "w": args.w,
        }

    return json.dumps(packet, separators=(",", ":"))


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
    sent_commands = 0

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

            opened_at = time.monotonic()
            first_send_at = opened_at + args.startup_delay
            next_send_at = first_send_at
            interval = 1.0 / args.rate if args.rate is not None else None
            repeat_until = (
                first_send_at + args.duration
                if args.duration is not None
                else first_send_at
            )
            final_read_until: Optional[float] = None
            sequence = args.seq

            while True:
                now = time.monotonic()

                should_send = False
                if sent_commands == 0 and now >= first_send_at:
                    should_send = True
                elif (
                    interval is not None
                    and now >= next_send_at
                    and next_send_at < repeat_until
                ):
                    should_send = True

                if should_send:
                    line = build_command(args, sequence)
                    connection.write((line + "\n").encode("utf-8"))
                    connection.flush()
                    sent_commands += 1
                    print(f"TX {line}")

                    if interval is None:
                        final_read_until = now + args.listen_seconds
                    else:
                        sequence = (sequence + 1) & UINT32_MAX
                        next_send_at += interval
                        if next_send_at >= repeat_until:
                            final_read_until = now + args.listen_seconds

                raw_line = connection.readline()
                if raw_line:
                    valid_delta, invalid_delta = print_received_line(raw_line)
                    valid_packets += valid_delta
                    invalid_packets += invalid_delta

                if (
                    final_read_until is not None
                    and time.monotonic() >= final_read_until
                ):
                    break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except serial.SerialException as error:
        print(f"Serial error: {error}", file=sys.stderr)
        return 1
    finally:
        print(
            f"Summary: sent={sent_commands} valid={valid_packets} "
            f"invalid={invalid_packets}",
            file=sys.stderr,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
