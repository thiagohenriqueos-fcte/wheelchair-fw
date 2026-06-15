#!/usr/bin/env python3

import argparse
import json
import math
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk
from typing import Any, Dict, Optional, Tuple


CANVAS_SIZE = 420
JOYSTICK_RADIUS = 160
DOT_RADIUS = 9
GUI_UPDATE_MS = 50
AGE_UPDATE_MS = 100


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display ESP32-S3 joystick JSON telemetry."
    )
    parser.add_argument(
        "port",
        help="Serial port, for example /dev/ttyACM0 or /dev/ttyUSB0.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200).",
    )
    return parser.parse_args()


def parse_json_line(raw_line: bytes) -> Tuple[Optional[Dict[str, Any]], str]:
    line = raw_line.decode("utf-8", errors="replace").strip()
    if not line:
        return None, line

    try:
        packet = json.loads(line)
    except json.JSONDecodeError:
        return None, line

    if not isinstance(packet, dict):
        return None, line

    return packet, line


def serial_reader(
    serial_module: Any,
    port: str,
    baud: int,
    event_queue: "queue.Queue[Tuple[str, Any]]",
    stop_event: threading.Event,
) -> None:
    try:
        with serial_module.Serial(
            port=port,
            baudrate=baud,
            timeout=0.2,
        ) as connection:
            event_queue.put(("connection", f"connected: {port} @ {baud}"))

            while not stop_event.is_set():
                raw_line = connection.readline()
                if not raw_line:
                    continue

                packet, decoded_line = parse_json_line(raw_line)
                if packet is None:
                    if decoded_line:
                        event_queue.put(("invalid", decoded_line))
                    continue

                event_queue.put(("packet", packet))

    except serial_module.SerialException as error:
        event_queue.put(("error", str(error)))
    finally:
        event_queue.put(("stopped", None))


def finite_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None

    if not math.isfinite(result):
        return None

    return result


class JoystickMonitor:
    def __init__(
        self,
        root: tk.Tk,
        event_queue: "queue.Queue[Tuple[str, Any]]",
        stop_event: threading.Event,
        reader_thread: threading.Thread,
    ) -> None:
        self.root = root
        self.event_queue = event_queue
        self.stop_event = stop_event
        self.reader_thread = reader_thread
        self.closing = False
        self.valid_packets = 0
        self.invalid_packets = 0
        self.last_packet_time: Optional[float] = None

        self.raw_x_var = tk.StringVar(value="n/a")
        self.raw_y_var = tk.StringVar(value="n/a")
        self.x_var = tk.StringVar(value="n/a")
        self.y_var = tk.StringVar(value="n/a")
        self.seq_var = tk.StringVar(value="n/a")
        self.t_ms_var = tk.StringVar(value="n/a")
        self.status_var = tk.StringVar(value="waiting")
        self.connection_var = tk.StringVar(value="opening serial port")
        self.valid_var = tk.StringVar(value="0")
        self.invalid_var = tk.StringVar(value="0")
        self.age_var = tk.StringVar(value="n/a")

        self.root.title("Wheelchair Joystick Monitor")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self._build_ui()
        self.root.after(GUI_UPDATE_MS, self._process_events)
        self.root.after(AGE_UPDATE_MS, self._update_packet_age)

    def _build_ui(self) -> None:
        main_frame = ttk.Frame(self.root, padding=12)
        main_frame.grid(row=0, column=0, sticky="nsew")

        self.canvas = tk.Canvas(
            main_frame,
            width=CANVAS_SIZE,
            height=CANVAS_SIZE,
            background="#f7f7f7",
            highlightthickness=1,
            highlightbackground="#a0a0a0",
        )
        self.canvas.grid(row=0, column=0, padx=(0, 16))

        center = CANVAS_SIZE / 2
        radius = JOYSTICK_RADIUS
        self.canvas.create_oval(
            center - radius,
            center - radius,
            center + radius,
            center + radius,
            outline="#404040",
            width=2,
        )
        self.canvas.create_line(
            center - radius,
            center,
            center + radius,
            center,
            fill="#a0a0a0",
        )
        self.canvas.create_line(
            center,
            center - radius,
            center,
            center + radius,
            fill="#a0a0a0",
        )
        self.canvas.create_text(center, center - radius - 18, text="+Y")
        self.canvas.create_text(center, center + radius + 18, text="-Y")
        self.canvas.create_text(center - radius - 20, center, text="-X")
        self.canvas.create_text(center + radius + 20, center, text="+X")

        self.dot = self.canvas.create_oval(
            center - DOT_RADIUS,
            center - DOT_RADIUS,
            center + DOT_RADIUS,
            center + DOT_RADIUS,
            fill="#1976d2",
            outline="#0d47a1",
            width=2,
        )

        values_frame = ttk.LabelFrame(
            main_frame,
            text="Telemetry",
            padding=10,
        )
        values_frame.grid(row=0, column=1, sticky="n")

        fields = (
            ("raw_x", self.raw_x_var),
            ("raw_y", self.raw_y_var),
            ("x", self.x_var),
            ("y", self.y_var),
            ("seq", self.seq_var),
            ("t_ms", self.t_ms_var),
            ("status", self.status_var),
            ("valid packets", self.valid_var),
            ("invalid packets", self.invalid_var),
            ("last packet age", self.age_var),
            ("connection", self.connection_var),
        )

        for row, (label, variable) in enumerate(fields):
            ttk.Label(values_frame, text=f"{label}:").grid(
                row=row,
                column=0,
                sticky="w",
                padx=(0, 12),
                pady=3,
            )
            ttk.Label(
                values_frame,
                textvariable=variable,
                width=28,
            ).grid(row=row, column=1, sticky="w", pady=3)

        ttk.Button(
            values_frame,
            text="Close",
            command=self.close,
        ).grid(
            row=len(fields),
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(14, 0),
        )

    def _process_events(self) -> None:
        while True:
            try:
                event_type, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break

            if event_type == "packet":
                self._update_packet(payload)
            elif event_type == "invalid":
                self.invalid_packets += 1
                self.invalid_var.set(str(self.invalid_packets))
            elif event_type == "connection":
                self.connection_var.set(str(payload))
            elif event_type == "error":
                self.connection_var.set(f"error: {payload}")
                self.status_var.set("serial error")
            elif (
                event_type == "stopped"
                and not self.closing
                and not self.connection_var.get().startswith("error:")
            ):
                self.connection_var.set("serial reader stopped")

        if not self.closing:
            self.root.after(GUI_UPDATE_MS, self._process_events)

    def _update_packet(self, packet: Dict[str, Any]) -> None:
        self.valid_packets += 1
        self.valid_var.set(str(self.valid_packets))
        self.last_packet_time = time.monotonic()

        self.raw_x_var.set(str(packet.get("raw_x", "n/a")))
        self.raw_y_var.set(str(packet.get("raw_y", "n/a")))
        self.seq_var.set(str(packet.get("seq", "n/a")))
        self.t_ms_var.set(str(packet.get("t_ms", "n/a")))
        self.status_var.set(str(packet.get("status", "ok")))

        x = finite_float(packet.get("x"))
        y = finite_float(packet.get("y"))
        self.x_var.set("n/a" if x is None else f"{x:.3f}")
        self.y_var.set("n/a" if y is None else f"{y:.3f}")

        if x is not None and y is not None:
            self._move_dot(x, y)

    def _move_dot(self, x: float, y: float) -> None:
        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))

        center = CANVAS_SIZE / 2
        canvas_x = center + x * JOYSTICK_RADIUS
        canvas_y = center - y * JOYSTICK_RADIUS
        self.canvas.coords(
            self.dot,
            canvas_x - DOT_RADIUS,
            canvas_y - DOT_RADIUS,
            canvas_x + DOT_RADIUS,
            canvas_y + DOT_RADIUS,
        )

    def _update_packet_age(self) -> None:
        if self.last_packet_time is None:
            self.age_var.set("n/a")
        else:
            age_ms = (time.monotonic() - self.last_packet_time) * 1000
            self.age_var.set(f"{age_ms:.0f} ms")

        if not self.closing:
            self.root.after(AGE_UPDATE_MS, self._update_packet_age)

    def close(self) -> None:
        if self.closing:
            return

        self.closing = True
        self.stop_event.set()
        self.connection_var.set("closing")
        self._wait_for_reader()

    def _wait_for_reader(self) -> None:
        if self.reader_thread.is_alive():
            self.root.after(50, self._wait_for_reader)
            return

        self.root.destroy()


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

    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(f"Unable to start Tkinter: {error}", file=sys.stderr)
        return 1

    event_queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=serial_reader,
        args=(serial, args.port, args.baud, event_queue, stop_event),
        name="serial-reader",
        daemon=True,
    )

    JoystickMonitor(root, event_queue, stop_event, reader_thread)
    reader_thread.start()
    root.mainloop()

    stop_event.set()
    reader_thread.join(timeout=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
