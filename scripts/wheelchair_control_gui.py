#!/usr/bin/env python3
"""Wheelchair integrated control GUI: telemetry monitor + PWM command sender.

Opens one serial connection shared for continuous telemetry reading and for
sending pwm_test / stop commands.

v0.5.3 adds a continuous PWM stream mode.
v0.6.1 adds live-adjustable joystick smoothing controls.
v0.6.2 adds fullscreen support, a user-configurable PWM limit, and documents
the 25 kHz firmware PWM frequency.

Usage:
    python3 scripts/wheelchair_control_gui.py /dev/ttyACM0

WARNING: v0.6.2 is for suspended / no-load PWM testing ONLY.
         Do not use with wheels on the ground or motors under any load.
         The GUI PWM limit defaults to 0.30 and must only be raised for
         controlled suspended-motor tests.
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


# ── Safety / Range ────────────────────────────────────────────────────────────

# Default PWM limit presented to the operator.  Raise only for suspended tests.
PWM_LIMIT_DEFAULT = 0.30
PWM_LIMIT_MIN     = 0.00
PWM_LIMIT_MAX     = 1.00
# Threshold above which a safety warning is displayed in the GUI.
PWM_LIMIT_WARN_THRESHOLD = 0.30


# ── Layout ────────────────────────────────────────────────────────────────────

CANVAS_SIZE     = 280
JOYSTICK_RADIUS = 108
DOT_RADIUS      = 8

BAR_W    = 220
BAR_H    = 28
BAR_HALF = BAR_W // 2


# ── Timing / smoothing defaults ────────────────────────────────────────────────

FILTER_ALPHA  = 0.25
INTERP_ALPHA  = 0.20
GUI_UPDATE_MS = 33
AGE_UPDATE_MS = 100

SMOOTHING_ALPHA_MIN = 0.01
SMOOTHING_ALPHA_MAX = 1.0
GUI_MS_MIN = 10
GUI_MS_MAX = 100

# Continuous PWM stream rate.
PWM_STREAM_HZ        = 10
PWM_STREAM_PERIOD_MS = 1000 // PWM_STREAM_HZ   # 100 ms at 10 Hz


# ── Colours ───────────────────────────────────────────────────────────────────

COLOR_FWD      = "#2e7d32"
COLOR_REV      = "#b71c1c"
COLOR_STOP_FG  = "#546e7a"
COLOR_BG_L     = "#fce4e4"
COLOR_BG_R     = "#e8f5e9"
COLOR_LIMIT    = "#ff9800"
COLOR_STOP_BTN = "#c62828"
COLOR_STOP_TXT = "white"
COLOR_WARN     = "#bf360c"
COLOR_STREAM   = "#1565c0"


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
            "Wheelchair integrated control GUI. "
            "Reads telemetry and sends pwm_test/stop commands over the same "
            "serial connection. "
            "WARNING: for suspended/no-load testing only."
        )
    )
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--filter-alpha", type=_alpha, default=FILTER_ALPHA,
        help=f"Low-pass filter alpha for telemetry x/y (default: {FILTER_ALPHA})",
    )
    parser.add_argument(
        "--interp-alpha", type=_alpha, default=INTERP_ALPHA,
        help=f"Visual interpolation alpha per frame (default: {INTERP_ALPHA})",
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
    r = math.sqrt(x * x + y * y)
    return (x / r, y / r) if r > 1.0 else (x, y)


def motor_label(value: float, active: bool) -> str:
    if not active or value == 0.0:
        return "Stopped / 0%"
    pct = abs(value) * 100.0
    direction = "Forward / RPWM" if value > 0 else "Reverse / LPWM"
    return f"{direction} / {pct:.0f}%"


def active_gpio(side: str, value: float, active: bool) -> str:
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
    """Read lines from serial and route them to the event queue.

    Also passes the live Serial object back to the main thread so it can
    write commands without opening a second connection.
    """
    try:
        with serial_module.Serial(port=port, baudrate=baud, timeout=0.2) as conn:
            event_queue.put(("conn_ready", conn))
            event_queue.put(("connection", f"connected: {port} @ {baud}"))

            while not stop_event.is_set():
                try:
                    raw = conn.readline()
                except serial_module.SerialException as exc:
                    event_queue.put(("error", str(exc)))
                    return
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
        event_queue.put(("conn_gone", None))
        event_queue.put(("stopped", None))


# ── Main GUI class ────────────────────────────────────────────────────────────

class WheelchairControlGUI:
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

        self.filter_alpha  = filter_alpha
        self.interp_alpha  = interp_alpha
        self.gui_update_ms = gui_update_ms

        # currently applied PWM limit (operator-facing safety gate)
        self._pwm_limit = PWM_LIMIT_DEFAULT

        # shared serial connection — written by main thread only, under _write_lock
        self._conn: Optional[Any] = None
        self._write_lock = threading.Lock()
        self._seq = 0

        # continuous PWM stream state
        self._streaming: bool = False
        self._stream_after_id: Optional[str] = None

        # counters / timing
        self.valid_packets   = 0
        self.invalid_packets = 0
        self.last_packet_time: Optional[float] = None

        # Three-state joystick pipeline:
        #   latest   — most recent normalized x/y from telemetry
        #   filtered — exponential moving average of latest
        #   visual   — per-frame interpolation toward filtered (drives the dot)
        self.latest_x   = 0.0
        self.latest_y   = 0.0
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.visual_x   = 0.0
        self.visual_y   = 0.0

        # motor state from telemetry
        self.motor_left   = 0.0
        self.motor_right  = 0.0
        self.motor_active = False

        # motor slider widget references (set in _build_motor_control_panel)
        self._slider_left:  Optional[tk.Scale] = None
        self._slider_right: Optional[tk.Scale] = None

        self._build_ui()
        self.root.after(self.gui_update_ms, self._gui_frame)
        self.root.after(AGE_UPDATE_MS,      self._tick_age)

    # ── Widget construction ───────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.root.title("Wheelchair Control  v0.6.2")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Allow the root to distribute any extra fullscreen space to the outer frame.
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        outer = ttk.Frame(self.root, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")

        self._build_status_bar(outer)        # row 0
        self._build_view_toolbar(outer)      # row 1

        joy_frame = ttk.LabelFrame(outer, text="Joystick", padding=8)
        joy_frame.grid(row=2, column=0, sticky="n", padx=(0, 8), pady=(0, 6))
        self._build_joystick_panel(joy_frame)

        mon_frame = ttk.LabelFrame(outer, text="Motor Monitor", padding=8)
        mon_frame.grid(row=2, column=1, sticky="n", padx=(0, 8), pady=(0, 6))
        self._build_motor_monitor_panel(mon_frame)

        ctrl_frame = ttk.LabelFrame(outer, text="Motor Control", padding=10)
        ctrl_frame.grid(row=2, column=2, sticky="nsew", pady=(0, 6))
        self._build_motor_control_panel(ctrl_frame)

        smooth_frame = ttk.LabelFrame(
            outer, text="Joystick smoothing settings", padding=8)
        smooth_frame.grid(
            row=3, column=0, columnspan=3, sticky="ew", pady=(0, 6))
        self._build_smoothing_panel(smooth_frame)

        self._build_safety_panel(outer)      # row 4

        # Keyboard shortcuts for fullscreen.
        self.root.bind("<F11>",    lambda _e: self._toggle_fullscreen())
        self.root.bind("<Escape>", lambda _e: self._exit_fullscreen())

    # ── View toolbar ─────────────────────────────────────────────────────────

    def _build_view_toolbar(self, parent: ttk.Frame) -> None:
        toolbar = ttk.Frame(parent)
        toolbar.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(0, 4))

        ttk.Label(
            toolbar,
            text="View:",
            foreground=COLOR_STOP_FG,
        ).pack(side="left", padx=(0, 6))

        ttk.Button(
            toolbar,
            text="Fullscreen  (F11)",
            command=self._toggle_fullscreen,
        ).pack(side="left", padx=(0, 4))

        ttk.Button(
            toolbar,
            text="Exit Fullscreen  (Esc)",
            command=self._exit_fullscreen,
        ).pack(side="left")

    # ── Status bar ────────────────────────────────────────────────────────────

    def _build_status_bar(self, parent: ttk.Frame) -> None:
        bar = ttk.LabelFrame(parent, text="Connection", padding=6)
        bar.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 4))

        self._sv_conn    = tk.StringVar(value="opening serial port…")
        self._sv_fw      = tk.StringVar(value="—")
        self._sv_hb      = tk.StringVar(value="—")
        self._sv_valid   = tk.StringVar(value="0")
        self._sv_invalid = tk.StringVar(value="0")
        self._sv_age     = tk.StringVar(value="—")
        self._sv_last_rx = tk.StringVar(value="—")

        fields = [
            ("Port",    self._sv_conn,    20),
            ("FW",      self._sv_fw,       8),
            ("HB",      self._sv_hb,       6),
            ("Valid",   self._sv_valid,    6),
            ("Invalid", self._sv_invalid,  6),
            ("Last pkt",self._sv_age,      8),
            ("Last RX", self._sv_last_rx, 28),
        ]
        for col, (label, var, width) in enumerate(fields):
            ttk.Label(bar, text=f"{label}:").grid(
                row=0, column=col * 2, sticky="e", padx=(8, 2))
            ttk.Label(bar, textvariable=var, width=width).grid(
                row=0, column=col * 2 + 1, sticky="w")

    # ── Joystick panel ────────────────────────────────────────────────────────

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
        c.create_text(center,          center - r - 14, text="+Y", fill="#555")
        c.create_text(center,          center + r + 14, text="-Y", fill="#555")
        c.create_text(center - r - 16, center,          text="-X", fill="#555")
        c.create_text(center + r + 16, center,          text="+X", fill="#555")

        self._dot = c.create_oval(
            center - DOT_RADIUS, center - DOT_RADIUS,
            center + DOT_RADIUS, center + DOT_RADIUS,
            fill="#1976d2", outline="#0d47a1", width=2,
        )

        num = ttk.Frame(parent, padding=(4, 8, 0, 0))
        num.pack(fill="x")

        self._sv_raw_x  = tk.StringVar(value="—")
        self._sv_raw_y  = tk.StringVar(value="—")
        self._sv_x      = tk.StringVar(value="—")
        self._sv_y      = tk.StringVar(value="—")
        self._sv_filt_x = tk.StringVar(value="—")
        self._sv_filt_y = tk.StringVar(value="—")
        self._sv_vis_x  = tk.StringVar(value="—")
        self._sv_vis_y  = tk.StringVar(value="—")

        rows = [
            ("raw_x",  self._sv_raw_x,  None),
            ("raw_y",  self._sv_raw_y,  None),
            ("x",      self._sv_x,      None),
            ("y",      self._sv_y,      None),
            ("filt_x", self._sv_filt_x, COLOR_STOP_FG),
            ("filt_y", self._sv_filt_y, COLOR_STOP_FG),
            ("vis_x",  self._sv_vis_x,  COLOR_STOP_FG),
            ("vis_y",  self._sv_vis_y,  COLOR_STOP_FG),
        ]
        for row_idx, (lbl, var, fg) in enumerate(rows):
            lbl_w = ttk.Label(num, text=f"{lbl}:", width=7, anchor="e")
            lbl_w.grid(row=row_idx, column=0, sticky="e", padx=(0, 6), pady=1)
            val_w = ttk.Label(num, textvariable=var, width=10, anchor="w")
            val_w.grid(row=row_idx, column=1, sticky="w", pady=1)
            if fg is not None:
                lbl_w.configure(foreground=fg)
                val_w.configure(foreground=fg)

    # ── Motor monitor panel ───────────────────────────────────────────────────

    def _build_motor_monitor_panel(self, parent: ttk.LabelFrame) -> None:
        active_row = ttk.Frame(parent)
        active_row.pack(fill="x", pady=(0, 8))
        ttk.Label(active_row, text="motor_test_active:").pack(side="left")
        self._sv_active = tk.StringVar(value="false")
        self._lbl_active = tk.Label(
            active_row,
            textvariable=self._sv_active,
            width=6,
            font=("", 10, "bold"),
            foreground=COLOR_STOP_FG,
        )
        self._lbl_active.pack(side="left", padx=(6, 0))

        self._left_bar,  self._sv_left_lbl,  self._sv_left_gpio = \
            self._build_motor_section(parent, "Left  (GPIO10 / GPIO11)")

        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=6)

        self._right_bar, self._sv_right_lbl, self._sv_right_gpio = \
            self._build_motor_section(parent, "Right (GPIO12 / GPIO13)")

    def _build_motor_section(
        self,
        parent: tk.Widget,
        title: str,
    ) -> Tuple[tk.Canvas, tk.StringVar, tk.StringVar]:
        section = ttk.Frame(parent)
        section.pack(fill="x", pady=2)

        ttk.Label(section, text=title, font=("", 9, "bold")).pack(anchor="w")

        bar_row = ttk.Frame(section)
        bar_row.pack(fill="x", pady=(3, 2))

        ttk.Label(bar_row, text="LPWM", foreground=COLOR_REV,
                  width=5, anchor="e").pack(side="left", padx=(0, 2))

        bar = tk.Canvas(
            bar_row,
            width=BAR_W, height=BAR_H,
            highlightthickness=1,
            highlightbackground="#b0bec5",
        )
        bar.pack(side="left")
        self._init_bar(bar)

        ttk.Label(bar_row, text="RPWM", foreground=COLOR_FWD,
                  width=5, anchor="w").pack(side="left", padx=(2, 0))

        lbl_var  = tk.StringVar(value="Stopped / 0%")
        gpio_var = tk.StringVar(value="Active: —")

        ttk.Label(section, textvariable=lbl_var,  anchor="w").pack(anchor="w")
        ttk.Label(section, textvariable=gpio_var,
                  foreground=COLOR_STOP_FG, anchor="w").pack(anchor="w")

        return bar, lbl_var, gpio_var

    def _init_bar(self, canvas: tk.Canvas) -> None:
        w, h, half = BAR_W, BAR_H, BAR_HALF
        canvas.create_rectangle(0,    0, half, h, fill=COLOR_BG_L, outline="")
        canvas.create_rectangle(half, 0, w,    h, fill=COLOR_BG_R, outline="")
        limit_px = int(0.30 * half)
        for tick_x in (half - limit_px, half + limit_px):
            canvas.create_line(
                tick_x, 0, tick_x, h,
                fill=COLOR_LIMIT, width=1, dash=(3, 3),
            )
        canvas.create_line(half, 0, half, h, fill="#455a64", width=2)
        canvas.create_rectangle(
            half, 3, half, h - 3,
            fill=COLOR_STOP_FG, outline="",
            tags="bar",
        )

    # ── Motor control panel ───────────────────────────────────────────────────

    def _build_motor_control_panel(self, parent: ttk.LabelFrame) -> None:

        # ── PWM limit ─────────────────────────────────────────────────────────
        self._var_pwm_limit     = tk.DoubleVar(value=PWM_LIMIT_DEFAULT)
        self._sv_pwm_limit_val  = tk.StringVar(value=f"{PWM_LIMIT_DEFAULT:.2f}")
        self._sv_pwm_limit_warn = tk.StringVar(value="")

        def _fmt_limit(*_: Any) -> None:
            v = round(self._var_pwm_limit.get(), 2)
            self._sv_pwm_limit_val.set(f"{v:.2f}")

        self._var_pwm_limit.trace_add("write", _fmt_limit)

        ttk.Label(parent, text="PWM limit:", width=12, anchor="e").grid(
            row=0, column=0, sticky="e", pady=(4, 0))
        tk.Scale(
            parent,
            variable=self._var_pwm_limit,
            from_=PWM_LIMIT_MIN,
            to=PWM_LIMIT_MAX,
            resolution=0.01,
            orient="horizontal",
            length=180,
            showvalue=False,
        ).grid(row=0, column=1, sticky="ew", padx=(4, 4), pady=(4, 0))
        ttk.Label(
            parent,
            textvariable=self._sv_pwm_limit_val,
            width=6, anchor="w",
            font=("Courier", 10),
        ).grid(row=0, column=2, sticky="w", pady=(4, 0))

        limit_btns = ttk.Frame(parent)
        limit_btns.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(4, 0))
        ttk.Button(
            limit_btns, text="Apply PWM Limit",
            command=self._on_apply_pwm_limit,
        ).pack(side="left", padx=(0, 4))
        ttk.Button(
            limit_btns, text="Reset to 0.30",
            command=self._on_reset_pwm_limit,
        ).pack(side="left")

        self._lbl_pwm_warn = ttk.Label(
            parent,
            textvariable=self._sv_pwm_limit_warn,
            foreground=COLOR_WARN,
            font=("", 8, "bold"),
            wraplength=280,
        )
        self._lbl_pwm_warn.grid(
            row=2, column=0, columnspan=3, sticky="ew", pady=(2, 0))

        ttk.Separator(parent, orient="horizontal").grid(
            row=3, column=0, columnspan=3, sticky="ew", pady=8)

        # ── Motor sliders ─────────────────────────────────────────────────────
        self._var_left  = tk.DoubleVar(value=0.0)
        self._var_right = tk.DoubleVar(value=0.0)
        self._sv_left_cmd  = tk.StringVar(value="+0.00")
        self._sv_right_cmd = tk.StringVar(value="+0.00")

        def _fmt_left(*_: Any) -> None:
            self._sv_left_cmd.set(f"{self._var_left.get():+.2f}")

        def _fmt_right(*_: Any) -> None:
            self._sv_right_cmd.set(f"{self._var_right.get():+.2f}")

        self._var_left.trace_add("write",  _fmt_left)
        self._var_right.trace_add("write", _fmt_right)

        lim = self._pwm_limit
        slider_opts = dict(
            from_=-lim,
            to=lim,
            resolution=0.01,
            orient="horizontal",
            length=180,
            showvalue=False,
        )

        for row_offset, (label, var, sv) in enumerate([
            ("Left motor:",  self._var_left,  self._sv_left_cmd),
            ("Right motor:", self._var_right, self._sv_right_cmd),
        ]):
            row = row_offset + 4
            ttk.Label(parent, text=label).grid(
                row=row, column=0, sticky="w", pady=(4, 0))
            slider = tk.Scale(parent, variable=var, **slider_opts)
            slider.grid(row=row, column=1, sticky="ew", padx=(4, 4), pady=(4, 0))
            ttk.Label(parent, textvariable=sv, width=6, anchor="w",
                      font=("Courier", 10)).grid(
                row=row, column=2, sticky="w", pady=(4, 0))
            if row_offset == 0:
                self._slider_left  = slider
            else:
                self._slider_right = slider

        ttk.Separator(parent, orient="horizontal").grid(
            row=6, column=0, columnspan=3, sticky="ew", pady=8)

        # ── Send Once ─────────────────────────────────────────────────────────
        self._btn_send_once = ttk.Button(
            parent, text="Send Once",
            command=self._on_send_once,
            state="disabled",
        )
        self._btn_send_once.grid(
            row=7, column=0, columnspan=3, sticky="ew", pady=(0, 4))

        # ── Stream buttons ────────────────────────────────────────────────────
        stream_btn_frame = ttk.Frame(parent)
        stream_btn_frame.grid(
            row=8, column=0, columnspan=3, sticky="ew", pady=(0, 4))
        stream_btn_frame.columnconfigure(0, weight=1)
        stream_btn_frame.columnconfigure(1, weight=1)

        self._btn_start_stream = ttk.Button(
            stream_btn_frame, text="Start PWM Stream",
            command=self._on_start_stream,
            state="disabled",
        )
        self._btn_start_stream.grid(row=0, column=0, sticky="ew", padx=(0, 2))

        self._btn_stop_stream = ttk.Button(
            stream_btn_frame, text="Stop PWM Stream",
            command=self._on_stop_stream,
            state="disabled",
        )
        self._btn_stop_stream.grid(row=0, column=1, sticky="ew", padx=(2, 0))

        # ── Stream status ─────────────────────────────────────────────────────
        stream_info = ttk.Frame(parent)
        stream_info.grid(
            row=9, column=0, columnspan=3, sticky="ew", pady=(0, 2))

        self._sv_stream_status = tk.StringVar(value="PWM stream: OFF")
        self._lbl_stream_status = tk.Label(
            stream_info,
            textvariable=self._sv_stream_status,
            font=("", 9, "bold"),
            foreground=COLOR_STOP_FG,
        )
        self._lbl_stream_status.pack(side="left", padx=(0, 10))
        ttk.Label(
            stream_info,
            text=f"{PWM_STREAM_HZ} Hz",
            foreground=COLOR_STOP_FG,
        ).pack(side="left")

        # ── Last transmitted values ───────────────────────────────────────────
        tx_frame = ttk.Frame(parent)
        tx_frame.grid(row=10, column=0, columnspan=3, sticky="ew", pady=(0, 6))

        ttk.Label(tx_frame, text="Last TX:", foreground=COLOR_STOP_FG).pack(
            side="left")
        ttk.Label(tx_frame, text="L", foreground=COLOR_STOP_FG).pack(
            side="left", padx=(8, 2))
        self._sv_stream_left = tk.StringVar(value="+0.00")
        ttk.Label(
            tx_frame, textvariable=self._sv_stream_left,
            width=6, font=("Courier", 9),
        ).pack(side="left")
        ttk.Label(tx_frame, text="R", foreground=COLOR_STOP_FG).pack(
            side="left", padx=(8, 2))
        self._sv_stream_right = tk.StringVar(value="+0.00")
        ttk.Label(
            tx_frame, textvariable=self._sv_stream_right,
            width=6, font=("Courier", 9),
        ).pack(side="left")

        ttk.Separator(parent, orient="horizontal").grid(
            row=11, column=0, columnspan=3, sticky="ew", pady=6)

        # ── STOP ─────────────────────────────────────────────────────────────
        self._btn_stop = tk.Button(
            parent,
            text="STOP",
            font=("", 14, "bold"),
            background=COLOR_STOP_BTN,
            foreground=COLOR_STOP_TXT,
            activebackground="#ef5350",
            activeforeground="white",
            relief="raised",
            borderwidth=3,
            padx=16, pady=8,
            command=self._on_stop,
        )
        self._btn_stop.grid(
            row=12, column=0, columnspan=3, sticky="ew", pady=(0, 8))

        # ── Zero utilities ────────────────────────────────────────────────────
        util = ttk.Frame(parent)
        util.grid(row=13, column=0, columnspan=3, sticky="ew")
        for col, (label, cmd) in enumerate([
            ("Zero Left",  self._on_zero_left),
            ("Zero Right", self._on_zero_right),
            ("Zero Both",  self._on_zero_both),
        ]):
            ttk.Button(util, text=label, command=cmd).grid(
                row=0, column=col, sticky="ew",
                padx=(0 if col == 0 else 2, 0))
        util.columnconfigure(0, weight=1)
        util.columnconfigure(1, weight=1)
        util.columnconfigure(2, weight=1)

        ttk.Separator(parent, orient="horizontal").grid(
            row=14, column=0, columnspan=3, sticky="ew", pady=8)

        # ── Latest response ───────────────────────────────────────────────────
        resp = ttk.LabelFrame(parent, text="Latest Response", padding=4)
        resp.grid(row=15, column=0, columnspan=3, sticky="ew")
        self._sv_last_ack = tk.StringVar(value="—")
        self._sv_last_err = tk.StringVar(value="—")
        ttk.Label(resp, text="ACK:", width=5, anchor="e").grid(
            row=0, column=0, sticky="e", padx=(0, 4), pady=1)
        ttk.Label(resp, textvariable=self._sv_last_ack,
                  width=28, anchor="w").grid(row=0, column=1, sticky="w", pady=1)
        ttk.Label(resp, text="ERR:", width=5, anchor="e").grid(
            row=1, column=0, sticky="e", padx=(0, 4), pady=1)
        ttk.Label(resp, textvariable=self._sv_last_err,
                  width=28, foreground="#b71c1c", anchor="w").grid(
            row=1, column=1, sticky="w", pady=1)

        self._sv_seq = tk.StringVar(value="seq: 0")
        ttk.Label(parent, textvariable=self._sv_seq,
                  foreground=COLOR_STOP_FG, font=("", 8)).grid(
            row=16, column=0, columnspan=3, sticky="w", pady=(4, 0))

    # ── Smoothing settings panel ──────────────────────────────────────────────

    def _build_smoothing_panel(self, parent: ttk.LabelFrame) -> None:
        parent.columnconfigure(1, weight=1)

        self._var_smooth_filter = tk.DoubleVar(value=self.filter_alpha)
        self._var_smooth_interp = tk.DoubleVar(value=self.interp_alpha)
        self._var_smooth_ms     = tk.IntVar(value=self.gui_update_ms)

        self._sv_smooth_filter = tk.StringVar(value=f"{self.filter_alpha:.2f}")
        self._sv_smooth_interp = tk.StringVar(value=f"{self.interp_alpha:.2f}")
        self._sv_smooth_ms     = tk.StringVar(value=str(self.gui_update_ms))

        def _on_filter(*_: Any) -> None:
            v = round(self._var_smooth_filter.get(), 2)
            self.filter_alpha = max(SMOOTHING_ALPHA_MIN, min(SMOOTHING_ALPHA_MAX, v))
            self._sv_smooth_filter.set(f"{self.filter_alpha:.2f}")

        def _on_interp(*_: Any) -> None:
            v = round(self._var_smooth_interp.get(), 2)
            self.interp_alpha = max(SMOOTHING_ALPHA_MIN, min(SMOOTHING_ALPHA_MAX, v))
            self._sv_smooth_interp.set(f"{self.interp_alpha:.2f}")

        def _on_ms(*_: Any) -> None:
            v = self._var_smooth_ms.get()
            self.gui_update_ms = max(GUI_MS_MIN, min(GUI_MS_MAX, v))
            self._sv_smooth_ms.set(str(self.gui_update_ms))

        self._var_smooth_filter.trace_add("write", _on_filter)
        self._var_smooth_interp.trace_add("write", _on_interp)
        self._var_smooth_ms.trace_add("write",     _on_ms)

        rows = [
            (
                "Filter alpha:",
                self._var_smooth_filter,
                SMOOTHING_ALPHA_MIN, SMOOTHING_ALPHA_MAX, 0.01,
                self._sv_smooth_filter,
                "1.00 = no filtering    0.01 = very slow response",
            ),
            (
                "Interp alpha:",
                self._var_smooth_interp,
                SMOOTHING_ALPHA_MIN, SMOOTHING_ALPHA_MAX, 0.01,
                self._sv_smooth_interp,
                "1.00 = dot jumps instantly    0.01 = very slow visual",
            ),
            (
                "Update interval:",
                self._var_smooth_ms,
                GUI_MS_MIN, GUI_MS_MAX, 1,
                self._sv_smooth_ms,
                f"ms per frame   ({GUI_MS_MIN} ms = fast   {GUI_MS_MAX} ms = less CPU)",
            ),
        ]

        for row_idx, (label, var, from_, to_, res, sv, hint) in enumerate(rows):
            ttk.Label(parent, text=label, width=16, anchor="e").grid(
                row=row_idx, column=0, sticky="e", padx=(0, 4), pady=3)
            tk.Scale(
                parent,
                variable=var,
                from_=from_,
                to=to_,
                resolution=res,
                orient="horizontal",
                length=220,
                showvalue=False,
            ).grid(row=row_idx, column=1, sticky="ew", padx=(0, 4), pady=3)
            ttk.Label(
                parent, textvariable=sv,
                width=6, anchor="w", font=("Courier", 9),
            ).grid(row=row_idx, column=2, sticky="w", padx=(0, 12), pady=3)
            ttk.Label(
                parent, text=hint,
                foreground=COLOR_STOP_FG, font=("", 8),
            ).grid(row=row_idx, column=3, sticky="w", pady=3)

        ttk.Button(
            parent,
            text="Reset smoothing defaults",
            command=self._on_reset_smoothing,
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))

    # ── Safety panel ──────────────────────────────────────────────────────────

    def _build_safety_panel(self, parent: ttk.Frame) -> None:
        safety = ttk.LabelFrame(parent, text="Safety", padding=6)
        safety.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(4, 0))

        warn = tk.Label(
            safety,
            text=(
                "⚠  Do not use with wheels on the ground. "
                "v0.6.2 is for suspended / no-load PWM testing only."
            ),
            foreground=COLOR_WARN,
            font=("", 9, "bold"),
            anchor="w",
            justify="left",
        )
        warn.pack(fill="x", pady=(0, 4))

        self._safety_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            safety,
            text="I understand — motors must be disconnected or suspended",
            variable=self._safety_var,
            command=self._on_safety_toggled,
        ).pack(anchor="w")

    # ── Event processing ──────────────────────────────────────────────────────

    def _process_queue(self) -> None:
        while True:
            try:
                kind, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "conn_ready":
                self._conn = payload
            elif kind == "conn_gone":
                self._conn = None
            elif kind == "packet":
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
            return

        if pkt_type == "ack":
            cmd_seq = pkt.get("cmd_seq", "?")
            self._sv_last_ack.set(
                f"cmd_seq={cmd_seq}  status={pkt.get('status','?')}")
            self._sv_last_rx.set(f"ACK  cmd_seq={cmd_seq}")
            return

        if pkt_type == "err":
            code = pkt.get("code", "?")
            self._sv_last_err.set(code)
            self._sv_last_rx.set(f"ERR  code={code}")
            return

        raw_x = pkt.get("raw_x")
        raw_y = pkt.get("raw_y")
        self._sv_raw_x.set("—" if raw_x is None else str(raw_x))
        self._sv_raw_y.set("—" if raw_y is None else str(raw_y))

        x = finite_float(pkt.get("x"))
        y = finite_float(pkt.get("y"))
        self._sv_x.set("—" if x is None else f"{x:+.3f}")
        self._sv_y.set("—" if y is None else f"{y:+.3f}")

        if x is not None:
            self.latest_x = x
        if y is not None:
            self.latest_y = y

        self.motor_active = bool(pkt.get("motor_test_active", False))
        ml = finite_float(pkt.get("motor_left"))
        mr = finite_float(pkt.get("motor_right"))
        self.motor_left  = ml if ml is not None else 0.0
        self.motor_right = mr if mr is not None else 0.0

    # ── GUI frame ─────────────────────────────────────────────────────────────

    def _gui_frame(self) -> None:
        self._process_queue()

        self.filtered_x = exp_step(self.filtered_x, self.latest_x, self.filter_alpha)
        self.filtered_y = exp_step(self.filtered_y, self.latest_y, self.filter_alpha)

        self.visual_x = exp_step(self.visual_x, self.filtered_x, self.interp_alpha)
        self.visual_y = exp_step(self.visual_y, self.filtered_y, self.interp_alpha)

        self._move_dot(self.visual_x, self.visual_y)

        self._sv_filt_x.set(f"{self.filtered_x:+.3f}")
        self._sv_filt_y.set(f"{self.filtered_y:+.3f}")
        self._sv_vis_x.set(f"{self.visual_x:+.3f}")
        self._sv_vis_y.set(f"{self.visual_y:+.3f}")

        eff_left  = self.motor_left  if self.motor_active else 0.0
        eff_right = self.motor_right if self.motor_active else 0.0
        self._update_bar(self._left_bar,  eff_left)
        self._update_bar(self._right_bar, eff_right)
        self._sv_left_lbl.set(motor_label(eff_left,   self.motor_active))
        self._sv_right_lbl.set(motor_label(eff_right,  self.motor_active))
        self._sv_left_gpio.set(
            f"Active: {active_gpio('left',  eff_left,  self.motor_active)}")
        self._sv_right_gpio.set(
            f"Active: {active_gpio('right', eff_right, self.motor_active)}")

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
        center   = CANVAS_SIZE / 2
        draw_r   = JOYSTICK_RADIUS - DOT_RADIUS
        canvas_x = center + x_draw * draw_r
        canvas_y = center - y_draw * draw_r
        self._joy_canvas.coords(
            self._dot,
            canvas_x - DOT_RADIUS, canvas_y - DOT_RADIUS,
            canvas_x + DOT_RADIUS, canvas_y + DOT_RADIUS,
        )

    def _update_bar(self, canvas: tk.Canvas, value: float) -> None:
        v    = max(-1.0, min(1.0, value))
        half = BAR_HALF
        h    = BAR_H
        px   = int(v * half)
        if v > 0.0:
            x0, x1, color = half,      half + px, COLOR_FWD
        elif v < 0.0:
            x0, x1, color = half + px, half,      COLOR_REV
        else:
            x0, x1, color = half,      half,      COLOR_STOP_FG
        canvas.itemconfigure("bar", fill=color)
        canvas.coords("bar", x0, 3, x1, h - 3)

    # ── Age ticker ────────────────────────────────────────────────────────────

    def _tick_age(self) -> None:
        if self.last_packet_time is None:
            self._sv_age.set("—")
        else:
            ms = (time.monotonic() - self.last_packet_time) * 1000
            self._sv_age.set(f"{ms:.0f} ms")
        if not self.closing:
            self.root.after(AGE_UPDATE_MS, self._tick_age)

    # ── Serial write ──────────────────────────────────────────────────────────

    def _send_raw(self, line: str) -> bool:
        """Write one JSON line to the serial port. Thread-safe via _write_lock."""
        with self._write_lock:
            if self._conn is None:
                self._sv_conn.set("not connected — cannot send")
                return False
            try:
                self._conn.write((line + "\n").encode("utf-8"))
                self._conn.flush()
                return True
            except Exception as exc:
                self._sv_conn.set(f"write error: {exc}")
                return False

    def _send_command(self, packet: Dict[str, Any]) -> bool:
        self._seq += 1
        packet["seq"] = self._seq
        self._sv_seq.set(f"seq: {self._seq}")
        return self._send_raw(json.dumps(packet, separators=(",", ":")))

    def _send_stop(self) -> bool:
        return self._send_command({"type": "stop"})

    # ── Continuous stream ─────────────────────────────────────────────────────

    def _stream_tick(self) -> None:
        """Send one pwm_test command and re-schedule the next tick."""
        if not self._streaming or self.closing:
            self._stream_after_id = None
            return

        left  = round(self._var_left.get(),  2)
        right = round(self._var_right.get(), 2)

        ok = self._send_command({"type": "pwm_test", "left": left, "right": right})
        if not ok:
            self._streaming = False
            self._stream_after_id = None
            self._update_stream_ui()
            return

        self._sv_stream_left.set(f"{left:+.2f}")
        self._sv_stream_right.set(f"{right:+.2f}")

        self._stream_after_id = self.root.after(
            PWM_STREAM_PERIOD_MS, self._stream_tick)

    def _update_stream_ui(self) -> None:
        safety    = self._safety_var.get()
        streaming = self._streaming

        idle_state = "normal" if (safety and not streaming) else "disabled"
        self._btn_send_once.configure(state=idle_state)
        self._btn_start_stream.configure(state=idle_state)

        self._btn_stop_stream.configure(
            state="normal" if streaming else "disabled")

        if streaming:
            self._sv_stream_status.set("PWM stream: ON")
            self._lbl_stream_status.configure(foreground=COLOR_STREAM)
        else:
            self._sv_stream_status.set("PWM stream: OFF")
            self._lbl_stream_status.configure(foreground=COLOR_STOP_FG)

    # ── Button / keyboard handlers ────────────────────────────────────────────

    def _toggle_fullscreen(self) -> None:
        is_fs = bool(self.root.attributes("-fullscreen"))
        self.root.attributes("-fullscreen", not is_fs)

    def _exit_fullscreen(self) -> None:
        self.root.attributes("-fullscreen", False)

    def _on_apply_pwm_limit(self) -> None:
        new_limit = round(self._var_pwm_limit.get(), 2)
        new_limit = max(PWM_LIMIT_MIN, min(PWM_LIMIT_MAX, new_limit))
        self._pwm_limit = new_limit

        assert self._slider_left  is not None
        assert self._slider_right is not None
        self._slider_left.configure(from_=-new_limit, to=new_limit)
        self._slider_right.configure(from_=-new_limit, to=new_limit)

        # Clamp current slider values into the new range.
        for var in (self._var_left, self._var_right):
            v = var.get()
            var.set(max(-new_limit, min(new_limit, v)))

        if new_limit > PWM_LIMIT_WARN_THRESHOLD:
            self._sv_pwm_limit_warn.set(
                f"⚠  PWM limit above {PWM_LIMIT_WARN_THRESHOLD:.2f} — "
                "use only with suspended motor and controlled test conditions.")
        else:
            self._sv_pwm_limit_warn.set("")

        self._sv_pwm_limit_val.set(f"{new_limit:.2f}")

    def _on_reset_pwm_limit(self) -> None:
        self._var_pwm_limit.set(PWM_LIMIT_DEFAULT)
        self._on_apply_pwm_limit()

    def _on_send_once(self) -> None:
        left  = round(self._var_left.get(),  2)
        right = round(self._var_right.get(), 2)
        self._send_command({"type": "pwm_test", "left": left, "right": right})
        self._sv_stream_left.set(f"{left:+.2f}")
        self._sv_stream_right.set(f"{right:+.2f}")

    def _on_start_stream(self) -> None:
        if not self._safety_var.get():
            return
        self._streaming = True
        self._update_stream_ui()
        self._stream_tick()

    def _on_stop_stream(self) -> None:
        self._streaming = False
        if self._stream_after_id is not None:
            self.root.after_cancel(self._stream_after_id)
            self._stream_after_id = None
        self._update_stream_ui()

    def _on_stop(self) -> None:
        self._on_stop_stream()
        self._send_stop()
        self._var_left.set(0.0)
        self._var_right.set(0.0)

    def _on_zero_left(self) -> None:
        self._var_left.set(0.0)

    def _on_zero_right(self) -> None:
        self._var_right.set(0.0)

    def _on_zero_both(self) -> None:
        self._var_left.set(0.0)
        self._var_right.set(0.0)

    def _on_safety_toggled(self) -> None:
        if not self._safety_var.get() and self._streaming:
            self._on_stop_stream()
        self._update_stream_ui()

    def _on_reset_smoothing(self) -> None:
        self._var_smooth_filter.set(FILTER_ALPHA)
        self._var_smooth_interp.set(INTERP_ALPHA)
        self._var_smooth_ms.set(GUI_UPDATE_MS)

    # ── Close ─────────────────────────────────────────────────────────────────

    def _on_close(self) -> None:
        if self.closing:
            return
        self.closing = True
        self._on_stop_stream()
        self._send_stop()
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

    WheelchairControlGUI(
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
