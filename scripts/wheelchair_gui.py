#!/usr/bin/env python3
"""Wheelchair monitor GUI: joystick position and motor PWM test state.

Reads newline-delimited JSON telemetry from the ESP32-S3 and displays:
  - joystick position (animated dot in a circular area)
  - motor PWM test state (horizontal bar meters for left and right)
  - connection status, packet counts, firmware version, heartbeat counter

Usage:
    python3 scripts/wheelchair_gui.py /dev/ttyACM0

NOTE: Only one process can use the serial port at a time.
      Close this monitor before running send_motor_test.py, and vice versa.
      A future combined control GUI may lift this restriction.
"""

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


# ── Layout constants ──────────────────────────────────────────────────────────

CANVAS_SIZE     = 360
JOYSTICK_RADIUS = 140
DOT_RADIUS      = 9

BAR_W    = 280          # motor bar canvas width in pixels
BAR_H    = 34           # motor bar canvas height in pixels
BAR_HALF = BAR_W // 2  # center x = 140 px

# ── Timing constants ──────────────────────────────────────────────────────────

FILTER_ALPHA   = 0.25   # low-pass filter applied to incoming telemetry x/y
INTERP_ALPHA   = 0.20   # visual interpolation applied each GUI frame
GUI_UPDATE_MS  = 33     # ~30 fps GUI frame interval
AGE_UPDATE_MS  = 100    # how often to refresh "last packet age" label

# ── Colours ───────────────────────────────────────────────────────────────────

COLOR_FWD     = "#2e7d32"  # dark green — RPWM active
COLOR_REV     = "#b71c1c"  # dark red   — LPWM active
COLOR_STOP_FG = "#546e7a"  # blue-grey  — stopped text
COLOR_BG_L    = "#fce4e4"  # light-red tint for LPWM half of bar
COLOR_BG_R    = "#e8f5e9"  # light-green tint for RPWM half of bar
COLOR_LIMIT   = "#ff9800"  # orange dashed lines at the ±0.30 firmware clamp


# ── Argument parsing ──────────────────────────────────────────────────────────

def _alpha(value: str) -> float:
    v = float(value)
    if not 0.0 < v <= 1.0:
        raise argparse.ArgumentTypeError("must be in (0, 1]")
    return v


def _pos_int(value: str) -> int:
    v = int(value)
    if v <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return v


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Wheelchair monitor: joystick position and motor PWM test state. "
            "NOTE: only one process can use the serial port at a time — "
            "close this monitor before running send_motor_test.py."
        )
    )
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--filter-alpha", type=_alpha, default=FILTER_ALPHA,
        help=f"Low-pass filter alpha for incoming x/y (default: {FILTER_ALPHA})",
    )
    parser.add_argument(
        "--interp-alpha", type=_alpha, default=INTERP_ALPHA,
        help=f"Visual interpolation alpha per GUI frame (default: {INTERP_ALPHA})",
    )
    parser.add_argument(
        "--gui-update-ms", type=_pos_int, default=GUI_UPDATE_MS,
        help=f"GUI frame interval in milliseconds (default: {GUI_UPDATE_MS})",
    )
    return parser.parse_args()


# ── Helpers ───────────────────────────────────────────────────────────────────

def parse_json_line(raw: bytes) -> Tuple[Optional[Dict[str, Any]], str]:
    line = raw.decode("utf-8", errors="replace").strip()
    if not line:
        return None, line
    try:
        pkt = json.loads(line)
    except json.JSONDecodeError:
        return None, line
    return (pkt if isinstance(pkt, dict) else None), line


def finite_float(value: Any) -> Optional[float]:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def exp_step(current: float, target: float, alpha: float) -> float:
    return alpha * target + (1.0 - alpha) * current


def clamp_unit_circle(x: float, y: float) -> Tuple[float, float]:
    m = math.hypot(x, y)
    return (x / m, y / m) if m > 1.0 else (x, y)


def motor_label(value: float, active: bool) -> str:
    """Human-readable description of one motor channel's current state."""
    if not active or value == 0.0:
        return "Stopped / 0%"
    pct = abs(value) * 100.0
    direction = "Forward / RPWM" if value > 0 else "Reverse / LPWM"
    return f"{direction} / {pct:.0f}%"


def active_gpio(side: str, value: float, active: bool) -> str:
    """Return the GPIO label for the currently driven pin, or '—'."""
    if not active or value == 0.0:
        return "—"
    table = {
        ("left",  True):  "GPIO10 (RPWM)",
        ("left",  False): "GPIO11 (LPWM)",
        ("right", True):  "GPIO12 (RPWM)",
        ("right", False): "GPIO13 (LPWM)",
    }
    return table[(side, value > 0)]


# ── Serial reader thread ──────────────────────────────────────────────────────

def serial_reader(
    serial_module: Any,
    port: str,
    baud: int,
    event_queue: "queue.Queue[Tuple[str, Any]]",
    stop_event: threading.Event,
) -> None:
    try:
        with serial_module.Serial(port=port, baudrate=baud, timeout=0.2) as conn:
            event_queue.put(("connection", f"connected: {port} @ {baud}"))
            while not stop_event.is_set():
                raw = conn.readline()
                if not raw:
                    continue
                pkt, line = parse_json_line(raw)
                if pkt is None:
                    if line:
                        event_queue.put(("invalid", line))
                else:
                    event_queue.put(("packet", pkt))
    except serial_module.SerialException as exc:
        event_queue.put(("error", str(exc)))
    finally:
        event_queue.put(("stopped", None))


# ── Main GUI class ────────────────────────────────────────────────────────────

class WheelchairMonitor:
    def __init__(
        self,
        root: tk.Tk,
        event_queue: "queue.Queue[Tuple[str, Any]]",
        stop_event: threading.Event,
        reader_thread: threading.Thread,
        *,
        filter_alpha: float,
        interp_alpha: float,
        gui_update_ms: int,
    ) -> None:
        self.root          = root
        self.event_queue   = event_queue
        self.stop_event    = stop_event
        self.reader_thread = reader_thread
        self.closing       = False

        self.filter_alpha   = filter_alpha
        self.interp_alpha   = interp_alpha
        self.gui_update_ms  = gui_update_ms

        # counters
        self.valid_packets    = 0
        self.invalid_packets  = 0
        self.last_packet_time: Optional[float] = None

        # joystick state — filtered values feed the visual interpolation
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.visual_x   = 0.0
        self.visual_y   = 0.0

        # motor state updated from telemetry
        self.motor_left   = 0.0
        self.motor_right  = 0.0
        self.motor_active = False

        self._build_ui()
        self.root.after(self.gui_update_ms, self._gui_frame)
        self.root.after(AGE_UPDATE_MS,      self._tick_age)

    # ── Widget construction ───────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.root.title("Wheelchair Monitor")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        outer = ttk.Frame(self.root, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")

        self._build_status_bar(outer)

        # left column: joystick
        joy_frame = ttk.LabelFrame(outer, text="Joystick", padding=8)
        joy_frame.grid(row=1, column=0, sticky="n", padx=(0, 10))
        self._build_joystick_panel(joy_frame)

        # right column: motor PWM
        motor_frame = ttk.LabelFrame(outer, text="Motor PWM Test", padding=10)
        motor_frame.grid(row=1, column=1, sticky="nsew")
        self._build_motor_panel(motor_frame)

    def _build_status_bar(self, parent: ttk.Frame) -> None:
        bar = ttk.LabelFrame(parent, text="Connection", padding=6)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        self._sv_conn    = tk.StringVar(value="opening serial port…")
        self._sv_fw      = tk.StringVar(value="—")
        self._sv_hb      = tk.StringVar(value="—")
        self._sv_valid   = tk.StringVar(value="0")
        self._sv_invalid = tk.StringVar(value="0")
        self._sv_age     = tk.StringVar(value="—")

        fields = [
            ("Port",    self._sv_conn,    18),
            ("FW",      self._sv_fw,       8),
            ("HB seq",  self._sv_hb,       6),
            ("Valid",   self._sv_valid,    6),
            ("Invalid", self._sv_invalid,  6),
            ("Last pkt",self._sv_age,      8),
        ]
        for col, (label, var, width) in enumerate(fields):
            ttk.Label(bar, text=f"{label}:").grid(
                row=0, column=col * 2, sticky="e", padx=(10, 2))
            ttk.Label(bar, textvariable=var, width=width).grid(
                row=0, column=col * 2 + 1, sticky="w")

    def _build_joystick_panel(self, parent: ttk.LabelFrame) -> None:
        self._joy_canvas = tk.Canvas(
            parent,
            width=CANVAS_SIZE, height=CANVAS_SIZE,
            background="#f7f7f7",
            highlightthickness=1,
            highlightbackground="#a0a0a0",
        )
        self._joy_canvas.pack()

        c      = self._joy_canvas
        center = CANVAS_SIZE / 2
        r      = JOYSTICK_RADIUS

        c.create_oval(
            center - r, center - r, center + r, center + r,
            outline="#404040", width=2,
        )
        c.create_line(center - r, center, center + r, center, fill="#c0c0c0")
        c.create_line(center, center - r, center, center + r, fill="#c0c0c0")
        c.create_text(center,       center - r - 16, text="+Y", fill="#555")
        c.create_text(center,       center + r + 16, text="-Y", fill="#555")
        c.create_text(center - r - 18, center,       text="-X", fill="#555")
        c.create_text(center + r + 18, center,       text="+X", fill="#555")

        self._dot = c.create_oval(
            center - DOT_RADIUS, center - DOT_RADIUS,
            center + DOT_RADIUS, center + DOT_RADIUS,
            fill="#1976d2", outline="#0d47a1", width=2,
        )

        num = ttk.Frame(parent, padding=(4, 8, 0, 0))
        num.pack(fill="x")
        self._sv_raw_x = tk.StringVar(value="—")
        self._sv_raw_y = tk.StringVar(value="—")
        self._sv_x     = tk.StringVar(value="—")
        self._sv_y     = tk.StringVar(value="—")
        for row, (lbl, var) in enumerate([
            ("raw_x", self._sv_raw_x),
            ("raw_y", self._sv_raw_y),
            ("x",     self._sv_x),
            ("y",     self._sv_y),
        ]):
            ttk.Label(num, text=f"{lbl}:", width=7, anchor="e").grid(
                row=row, column=0, sticky="e", padx=(0, 6), pady=2)
            ttk.Label(num, textvariable=var, width=10, anchor="w").grid(
                row=row, column=1, sticky="w", pady=2)

    def _build_motor_panel(self, parent: ttk.LabelFrame) -> None:
        # active indicator
        active_row = ttk.Frame(parent)
        active_row.pack(fill="x", pady=(0, 10))
        ttk.Label(active_row, text="motor_test_active:").pack(side="left")
        self._sv_active = tk.StringVar(value="false")
        self._lbl_active = tk.Label(
            active_row,
            textvariable=self._sv_active,
            width=8,
            font=("", 10, "bold"),
            foreground=COLOR_STOP_FG,
        )
        self._lbl_active.pack(side="left", padx=(6, 0))

        # left motor
        self._left_bar, self._sv_left_lbl, self._sv_left_gpio = \
            self._build_motor_section(parent, "Left Motor  (GPIO10 / GPIO11)")

        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=8)

        # right motor
        self._right_bar, self._sv_right_lbl, self._sv_right_gpio = \
            self._build_motor_section(parent, "Right Motor (GPIO12 / GPIO13)")

        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=8)

        # motion command readout
        cmd_frame = ttk.LabelFrame(parent, text="Last Motion Command", padding=6)
        cmd_frame.pack(fill="x", pady=(4, 0))
        self._sv_cmd_v     = tk.StringVar(value="—")
        self._sv_cmd_w     = tk.StringVar(value="—")
        self._sv_cmd_valid = tk.StringVar(value="—")
        for row, (lbl, var) in enumerate([
            ("cmd_v",     self._sv_cmd_v),
            ("cmd_w",     self._sv_cmd_w),
            ("cmd_valid", self._sv_cmd_valid),
        ]):
            ttk.Label(cmd_frame, text=f"{lbl}:", width=10, anchor="e").grid(
                row=row, column=0, sticky="e", padx=(0, 4), pady=2)
            ttk.Label(cmd_frame, textvariable=var, anchor="w").grid(
                row=row, column=1, sticky="w", pady=2)

    def _build_motor_section(
        self,
        parent: tk.Widget,
        title: str,
    ) -> Tuple[tk.Canvas, tk.StringVar, tk.StringVar]:
        """One motor bar section. Returns (bar canvas, label StringVar, gpio StringVar)."""
        section = ttk.Frame(parent)
        section.pack(fill="x", pady=2)

        ttk.Label(section, text=title, font=("", 9, "bold")).pack(anchor="w")

        bar_row = ttk.Frame(section)
        bar_row.pack(fill="x", pady=(4, 2))

        ttk.Label(bar_row, text="LPWM", foreground=COLOR_REV,
                  width=5, anchor="e").pack(side="left", padx=(0, 2))

        bar = tk.Canvas(
            bar_row,
            width=BAR_W, height=BAR_H,
            background=COLOR_BG_R,          # default: neutral
            highlightthickness=1,
            highlightbackground="#b0bec5",
        )
        bar.pack(side="left")
        self._init_bar(bar)

        ttk.Label(bar_row, text="RPWM", foreground=COLOR_FWD,
                  width=5, anchor="w").pack(side="left", padx=(2, 0))

        lbl_var  = tk.StringVar(value="Stopped / 0%")
        gpio_var = tk.StringVar(value="Active GPIO: —")

        ttk.Label(section, textvariable=lbl_var,  anchor="w").pack(anchor="w")
        ttk.Label(section, textvariable=gpio_var,
                  foreground=COLOR_STOP_FG, anchor="w").pack(anchor="w")

        return bar, lbl_var, gpio_var

    def _init_bar(self, canvas: tk.Canvas) -> None:
        """Draw the static bar background. The 'bar' tag is the dynamic rectangle."""
        w    = BAR_W
        h    = BAR_H
        half = BAR_HALF

        # LPWM zone (left half, light-red tint)
        canvas.create_rectangle(0,    0, half, h, fill=COLOR_BG_L, outline="")
        # RPWM zone (right half, light-green tint)
        canvas.create_rectangle(half, 0, w,    h, fill=COLOR_BG_R, outline="")

        # orange dashed tick marks at the ±0.30 firmware clamp limit
        limit_px = int(0.30 * half)
        for tick_x in (half - limit_px, half + limit_px):
            canvas.create_line(
                tick_x, 0, tick_x, h,
                fill=COLOR_LIMIT, width=1, dash=(3, 3),
            )

        # permanent centre divider
        canvas.create_line(half, 0, half, h, fill="#455a64", width=2)

        # the active bar — updated every GUI frame via tag "bar"
        canvas.create_rectangle(
            half, 4, half, h - 4,
            fill=COLOR_STOP_FG, outline="",
            tags="bar",
        )

    # ── Event processing ──────────────────────────────────────────────────────

    def _process_queue(self) -> None:
        while True:
            try:
                kind, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "packet":
                self._handle_packet(payload)
            elif kind == "invalid":
                self.invalid_packets += 1
                self._sv_invalid.set(str(self.invalid_packets))
            elif kind == "connection":
                self._sv_conn.set(str(payload))
            elif kind == "error":
                self._sv_conn.set(f"error: {payload}")
            elif kind == "stopped" and not self.closing:
                if not self._sv_conn.get().startswith("error:"):
                    self._sv_conn.set("reader stopped")

    def _handle_packet(self, pkt: Dict[str, Any]) -> None:
        self.valid_packets += 1
        self._sv_valid.set(str(self.valid_packets))
        self.last_packet_time = time.monotonic()

        fw = pkt.get("fw")
        if fw:
            self._sv_fw.set(str(fw))

        pkt_type = pkt.get("type", "")

        if pkt_type == "heartbeat":
            seq = pkt.get("seq")
            if seq is not None:
                self._sv_hb.set(str(seq))
            return  # heartbeat carries no joystick or motor data

        # joystick fields
        raw_x = pkt.get("raw_x")
        raw_y = pkt.get("raw_y")
        self._sv_raw_x.set("—" if raw_x is None else str(raw_x))
        self._sv_raw_y.set("—" if raw_y is None else str(raw_y))

        x = finite_float(pkt.get("x"))
        y = finite_float(pkt.get("y"))
        self._sv_x.set("—" if x is None else f"{x:+.3f}")
        self._sv_y.set("—" if y is None else f"{y:+.3f}")

        if x is not None:
            self.filtered_x = exp_step(self.filtered_x, x, self.filter_alpha)
        if y is not None:
            self.filtered_y = exp_step(self.filtered_y, y, self.filter_alpha)

        # motor fields
        self.motor_active = bool(pkt.get("motor_test_active", False))
        ml = finite_float(pkt.get("motor_left"))
        mr = finite_float(pkt.get("motor_right"))
        self.motor_left  = ml if ml is not None else 0.0
        self.motor_right = mr if mr is not None else 0.0

        # motion command fields
        cmd_v     = finite_float(pkt.get("cmd_v"))
        cmd_w     = finite_float(pkt.get("cmd_w"))
        cmd_valid = pkt.get("cmd_valid")
        self._sv_cmd_v.set("—" if cmd_v is None else f"{cmd_v:+.3f}")
        self._sv_cmd_w.set("—" if cmd_w is None else f"{cmd_w:+.3f}")
        if cmd_valid is None:
            self._sv_cmd_valid.set("—")
        else:
            self._sv_cmd_valid.set("true" if cmd_valid else "false")

    # ── GUI frame (called every gui_update_ms) ────────────────────────────────

    def _gui_frame(self) -> None:
        self._process_queue()

        # interpolate joystick dot position
        self.visual_x = exp_step(self.visual_x, self.filtered_x, self.interp_alpha)
        self.visual_y = exp_step(self.visual_y, self.filtered_y, self.interp_alpha)
        self._move_dot(self.visual_x, self.visual_y)

        # when motor_test_active is false, show both motors as stopped
        eff_left  = self.motor_left  if self.motor_active else 0.0
        eff_right = self.motor_right if self.motor_active else 0.0

        self._update_bar(self._left_bar,  eff_left)
        self._update_bar(self._right_bar, eff_right)

        self._sv_left_lbl.set(motor_label(eff_left,   self.motor_active))
        self._sv_right_lbl.set(motor_label(eff_right,  self.motor_active))
        self._sv_left_gpio.set(
            f"Active GPIO: {active_gpio('left',  eff_left,  self.motor_active)}")
        self._sv_right_gpio.set(
            f"Active GPIO: {active_gpio('right', eff_right, self.motor_active)}")

        if self.motor_active:
            self._sv_active.set("true")
            self._lbl_active.configure(foreground=COLOR_FWD)
        else:
            self._sv_active.set("false")
            self._lbl_active.configure(foreground=COLOR_STOP_FG)

        if not self.closing:
            self.root.after(self.gui_update_ms, self._gui_frame)

    def _move_dot(self, x: float, y: float) -> None:
        x_draw, y_draw = clamp_unit_circle(x, y)
        center  = CANVAS_SIZE / 2
        draw_r  = JOYSTICK_RADIUS - DOT_RADIUS
        canvas_x = center + x_draw * draw_r
        canvas_y = center - y_draw * draw_r
        self._joy_canvas.coords(
            self._dot,
            canvas_x - DOT_RADIUS, canvas_y - DOT_RADIUS,
            canvas_x + DOT_RADIUS, canvas_y + DOT_RADIUS,
        )

    def _update_bar(self, canvas: tk.Canvas, value: float) -> None:
        """Redraw the active bar for one motor channel."""
        v    = max(-1.0, min(1.0, value))
        half = BAR_HALF
        h    = BAR_H
        px   = int(v * half)

        if v > 0.0:
            x0, x1, color = half,      half + px, COLOR_FWD
        elif v < 0.0:
            x0, x1, color = half + px, half,      COLOR_REV
        else:
            # zero — collapse bar to invisible; centre line is always visible
            x0, x1, color = half, half, COLOR_STOP_FG

        canvas.itemconfigure("bar", fill=color)
        canvas.coords("bar", x0, 4, x1, h - 4)

    # ── Age ticker (every age_update_ms) ─────────────────────────────────────

    def _tick_age(self) -> None:
        if self.last_packet_time is None:
            self._sv_age.set("—")
        else:
            ms = (time.monotonic() - self.last_packet_time) * 1000
            self._sv_age.set(f"{ms:.0f} ms")
        if not self.closing:
            self.root.after(AGE_UPDATE_MS, self._tick_age)

    # ── Close ─────────────────────────────────────────────────────────────────

    def _on_close(self) -> None:
        if self.closing:
            return
        self.closing = True
        self.stop_event.set()
        self._sv_conn.set("closing…")
        self._wait_for_reader()

    def _wait_for_reader(self) -> None:
        if self.reader_thread.is_alive():
            self.root.after(50, self._wait_for_reader)
            return
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────

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
    except tk.TclError as exc:
        print(f"Unable to start Tkinter: {exc}", file=sys.stderr)
        return 1

    event_queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
    stop_event = threading.Event()
    reader = threading.Thread(
        target=serial_reader,
        args=(serial, args.port, args.baud, event_queue, stop_event),
        name="serial-reader",
        daemon=True,
    )

    WheelchairMonitor(
        root,
        event_queue,
        stop_event,
        reader,
        filter_alpha=args.filter_alpha,
        interp_alpha=args.interp_alpha,
        gui_update_ms=args.gui_update_ms,
    )
    reader.start()
    root.mainloop()

    stop_event.set()
    reader.join(timeout=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
