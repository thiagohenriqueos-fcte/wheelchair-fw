#!/usr/bin/env python3
"""Wheelchair differential-drive control GUI — v0.9.0

Usage:
    python3 scripts/wheelchair_control_gui.py /dev/ttyUSB1

The physical joystick is read by the ESP32, which runs the differential-drive
control loop (mixing + accel/decel ramp + max-duty clamp). This GUI only tunes
those parameters and holds the operator safety gate: while ARMED it streams a
`drive_cfg` keep-alive at 10 Hz, so if the GUI or USB link drops the firmware
disarms within ~400 ms.

WARNING: arming makes the wheels move. Use with the chair suspended / no load
until the behaviour is validated.
"""

import argparse
import json
import math
import queue
import sys
import threading
import time
import tkinter as tk
from typing import Any, Dict, Optional, Tuple

import ttkbootstrap as ttk  # drop-in replacement for tkinter.ttk
from ttkbootstrap.constants import *


# ── Theme ─────────────────────────────────────────────────────────────────────

THEME = "darkly"


# ── Drive limits / defaults ───────────────────────────────────────────────────

MAX_DUTY_DEFAULT = 0.30
MAX_DUTY_MIN     = 0.00
MAX_DUTY_MAX     = 1.00
MAX_DUTY_WARN    = 0.30      # above this → on-screen warning

ACCEL_DEFAULT = 1.5         # duty / second (0 → full in ~0.67 s)
DECEL_DEFAULT = 3.0         # duty / second (faster stop than start)
RAMP_MIN      = 0.2
RAMP_MAX      = 20.0

DRIVE_STREAM_HZ        = 10
DRIVE_STREAM_PERIOD_MS = 1000 // DRIVE_STREAM_HZ


# ── Layout ────────────────────────────────────────────────────────────────────

CANVAS_SIZE     = 260
JOYSTICK_RADIUS = 100
DOT_RADIUS      = 9

BAR_W    = 220
BAR_H    = 26
BAR_HALF = BAR_W // 2


# ── Timing / smoothing ────────────────────────────────────────────────────────

FILTER_ALPHA  = 0.25
INTERP_ALPHA  = 0.20
GUI_UPDATE_MS = 33
AGE_UPDATE_MS = 100


# ── Colours (dark-theme canvas palette) ───────────────────────────────────────

C_CANVAS_BG   = "#1e1e2e"
C_CANVAS_GRID = "#313244"
C_CIRCLE      = "#585b70"
C_DOT_FILL    = "#89b4fa"
C_DOT_OUTLINE = "#1e66f5"
C_CENTER_LINE = "#585b70"

C_BAR_BG_L  = "#3d1111"    # reverse side
C_BAR_BG_R  = "#113d11"    # forward side
C_FWD       = "#a6e3a1"    # forward fill — soft green
C_REV       = "#f38ba8"    # reverse fill — soft red/pink
C_NEUTRAL   = "#585b70"    # stopped bar fill
C_MUTED     = "#6c7086"    # secondary / muted text


# ── Argument parsing ──────────────────────────────────────────────────────────

def _pos_int(value: str) -> int:
    v = int(value)
    if v <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return v


def parse_arguments() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Wheelchair differential-drive control GUI.")
    p.add_argument("port", help="ESP32 serial port, e.g. /dev/ttyUSB1")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--gui-update-ms", type=_pos_int, default=GUI_UPDATE_MS)
    return p.parse_args()


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


def duty_label(value: float) -> str:
    if value == 0.0:
        return "Stopped"
    pct = abs(value) * 100.0
    direction = "▶  Forward" if value > 0 else "◀  Reverse"
    return f"{direction}   {pct:.0f}%"


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
        root: ttk.Window,
        event_queue: "queue.Queue[Tuple[str, Any]]",
        stop_event: threading.Event,
        reader_thread: threading.Thread,
        *,
        gui_update_ms: int,
    ) -> None:
        self.root          = root
        self.event_queue   = event_queue
        self.stop_event    = stop_event
        self.reader_thread = reader_thread
        self.closing       = False
        self.gui_update_ms = gui_update_ms

        self._conn: Optional[Any] = None
        self._write_lock = threading.Lock()
        self._seq = 0

        self.valid_packets   = 0
        self.invalid_packets = 0
        self.last_packet_time: Optional[float] = None

        # Joystick visualiser pipeline
        self.latest_x = self.filtered_x = self.visual_x = 0.0
        self.latest_y = self.filtered_y = self.visual_y = 0.0

        # Drive output (reported by firmware)
        self.out_left = self.out_right = 0.0
        self.armed_fw = False     # firmware-reported armed gate
        self.driving  = False     # firmware-reported actually-driving

        # Drive config (GUI is the source of truth)
        self._max_duty = MAX_DUTY_DEFAULT
        self._accel    = ACCEL_DEFAULT
        self._decel    = DECEL_DEFAULT

        # Arm / safety
        self._armed = False
        self._stream_after_id: Optional[str] = None

        self._build_ui()
        self.root.after(self.gui_update_ms, self._gui_frame)
        self.root.after(AGE_UPDATE_MS,      self._tick_age)

    # ── Build UI ──────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.root.title("Wheelchair Differential Drive  v0.9.0")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        container = ttk.Frame(self.root)
        container.grid(row=0, column=0, sticky="nsew")
        container.rowconfigure(0, weight=1)
        container.columnconfigure(0, weight=1)

        self._scroll_canvas = tk.Canvas(
            container, highlightthickness=0,
            background=self.root.style.colors.bg)
        self._scroll_canvas.grid(row=0, column=0, sticky="nsew")

        vscroll = ttk.Scrollbar(
            container, orient=VERTICAL, command=self._scroll_canvas.yview,
            bootstyle="round-secondary")
        vscroll.grid(row=0, column=1, sticky="ns")
        self._scroll_canvas.configure(yscrollcommand=vscroll.set)

        outer = ttk.Frame(self._scroll_canvas, padding=12)
        self._canvas_win_id = self._scroll_canvas.create_window(
            (0, 0), window=outer, anchor="nw")

        outer.bind("<Configure>", self._on_inner_configure)
        self._scroll_canvas.bind("<Configure>", self._on_canvas_configure)
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self._scroll_canvas.bind_all(seq, self._on_mousewheel)

        self._build_status_bar(outer)    # row 0
        self._build_view_toolbar(outer)  # row 1

        joy_frame = ttk.Labelframe(
            outer, text="Joystick (físico)", padding=10, bootstyle="secondary")
        joy_frame.grid(row=2, column=0, sticky="n", padx=(0, 8), pady=(0, 8))
        self._build_joystick_panel(joy_frame)

        mon_frame = ttk.Labelframe(
            outer, text="Saída dos motores", padding=10, bootstyle="secondary")
        mon_frame.grid(row=2, column=1, sticky="n", padx=(0, 8), pady=(0, 8))
        self._build_motor_monitor_panel(mon_frame)

        drive_frame = ttk.Labelframe(
            outer, text="Controle de tração", padding=12, bootstyle="secondary")
        drive_frame.grid(row=2, column=2, sticky="nsew", pady=(0, 8))
        self._build_drive_control_panel(drive_frame)

        self._build_safety_panel(outer)  # row 3

        self.root.bind("<F11>",    lambda _e: self._toggle_fullscreen())
        self.root.bind("<Escape>", lambda _e: self._exit_fullscreen())

    # ── Scroll helpers ────────────────────────────────────────────────────────

    def _on_inner_configure(self, _event: Any) -> None:
        self._scroll_canvas.configure(
            scrollregion=self._scroll_canvas.bbox("all"))

    def _on_canvas_configure(self, event: Any) -> None:
        self._scroll_canvas.itemconfigure(self._canvas_win_id, width=event.width)

    def _on_mousewheel(self, event: Any) -> None:
        if event.num == 4:
            self._scroll_canvas.yview_scroll(-1, "units")
        elif event.num == 5:
            self._scroll_canvas.yview_scroll(1, "units")
        else:
            self._scroll_canvas.yview_scroll(
                int(-1 * (event.delta / 120)), "units")

    # ── Status bar ────────────────────────────────────────────────────────────

    def _build_status_bar(self, parent: ttk.Frame) -> None:
        bar = ttk.Frame(parent, padding=(10, 6))
        bar.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 8))

        stripe = ttk.Frame(bar, width=4, bootstyle="info")
        stripe.pack(side=LEFT, fill=Y, padx=(0, 10))

        self._sv_conn  = tk.StringVar(value="connecting…")
        self._sv_fw    = tk.StringVar(value="—")
        self._sv_hb    = tk.StringVar(value="—")
        self._sv_valid = tk.StringVar(value="0")
        self._sv_age   = tk.StringVar(value="—")
        self._sv_rx    = tk.StringVar(value="—")

        fields = [
            ("Port",     self._sv_conn,  22, "info"),
            ("FW",       self._sv_fw,     8, "success"),
            ("HB",       self._sv_hb,     5, None),
            ("Valid",    self._sv_valid,  6, "success"),
            ("Last pkt", self._sv_age,    8, None),
            ("Last RX",  self._sv_rx,    26, None),
        ]
        for label, var, width, style in fields:
            ttk.Label(bar, text=f"{label}:", foreground=C_MUTED,
                      font=("", 8)).pack(side=LEFT, padx=(0, 2))
            kw: Dict[str, Any] = dict(textvariable=var, width=width,
                                      font=("", 8, "bold"))
            if style:
                kw["bootstyle"] = style
            ttk.Label(bar, **kw).pack(side=LEFT, padx=(0, 14))

    # ── View toolbar ──────────────────────────────────────────────────────────

    def _build_view_toolbar(self, parent: ttk.Frame) -> None:
        tb = ttk.Frame(parent)
        tb.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        ttk.Label(tb, text="View", foreground=C_MUTED, font=("", 8)).pack(
            side=LEFT, padx=(0, 8))
        ttk.Button(tb, text="Fullscreen", bootstyle="secondary-outline",
                   command=self._toggle_fullscreen, padding=(8, 4)
                   ).pack(side=LEFT, padx=(0, 6))
        ttk.Button(tb, text="✕  Exit Fullscreen", bootstyle="secondary-outline",
                   command=self._exit_fullscreen, padding=(8, 4)
                   ).pack(side=LEFT)

    # ── Joystick panel ────────────────────────────────────────────────────────

    def _build_joystick_panel(self, parent: ttk.Labelframe) -> None:
        self._joy_canvas = tk.Canvas(
            parent, width=CANVAS_SIZE, height=CANVAS_SIZE,
            background=C_CANVAS_BG, highlightthickness=1,
            highlightbackground=C_CIRCLE)
        self._joy_canvas.pack()

        c      = self._joy_canvas
        center = CANVAS_SIZE / 2
        r      = JOYSTICK_RADIUS
        c.create_oval(center - r, center - r, center + r, center + r,
                      outline=C_CIRCLE, width=1)
        c.create_line(center - r, center, center + r, center,
                      fill=C_CANVAS_GRID, width=1)
        c.create_line(center, center - r, center, center + r,
                      fill=C_CANVAS_GRID, width=1)
        for text, x, y in [
            ("+Y", center,          center - r - 12),
            ("-Y", center,          center + r + 12),
            ("-X", center - r - 14, center),
            ("+X", center + r + 14, center),
        ]:
            c.create_text(x, y, text=text, fill=C_MUTED, font=("", 8))
        self._dot = c.create_oval(
            center - DOT_RADIUS, center - DOT_RADIUS,
            center + DOT_RADIUS, center + DOT_RADIUS,
            fill=C_DOT_FILL, outline=C_DOT_OUTLINE, width=2)

        num = ttk.Frame(parent, padding=(4, 8, 0, 0))
        num.pack(fill=X)
        self._sv_x = tk.StringVar(value="—")
        self._sv_y = tk.StringVar(value="—")
        for i, (lbl, var) in enumerate([("x", self._sv_x), ("y", self._sv_y)]):
            ttk.Label(num, text=f"{lbl}:", width=4, anchor=E).grid(
                row=i, column=0, sticky=E, padx=(0, 6), pady=1)
            ttk.Label(num, textvariable=var, width=10, anchor=W,
                      font=("Courier", 9)).grid(row=i, column=1, sticky=W, pady=1)

    # ── Motor monitor panel ───────────────────────────────────────────────────

    def _build_motor_monitor_panel(self, parent: ttk.Labelframe) -> None:
        drive_row = ttk.Frame(parent)
        drive_row.pack(fill=X, pady=(0, 10))
        ttk.Label(drive_row, text="estado:", foreground=C_MUTED,
                  font=("", 8)).pack(side=LEFT)
        self._sv_drive_state = tk.StringVar(value="DESARMADO")
        self._lbl_drive_state = ttk.Label(
            drive_row, textvariable=self._sv_drive_state,
            font=("", 10, "bold"), bootstyle="secondary")
        self._lbl_drive_state.pack(side=LEFT, padx=(8, 0))

        self._left_bar,  self._sv_left_lbl  = self._build_motor_section(
            parent, "Esquerda   GPIO10 / GPIO11")
        ttk.Separator(parent, orient=HORIZONTAL, bootstyle="secondary").pack(
            fill=X, pady=8)
        self._right_bar, self._sv_right_lbl = self._build_motor_section(
            parent, "Direita    GPIO12 / GPIO13")

    def _build_motor_section(
        self, parent: tk.Widget, title: str,
    ) -> Tuple[tk.Canvas, tk.StringVar]:
        section = ttk.Frame(parent)
        section.pack(fill=X, pady=2)
        ttk.Label(section, text=title, font=("", 9, "bold")).pack(anchor=W)

        bar_row = ttk.Frame(section)
        bar_row.pack(fill=X, pady=(4, 2))
        ttk.Label(bar_row, text="REV", foreground="#f38ba8",
                  width=4, anchor=E, font=("", 8)).pack(side=LEFT, padx=(0, 3))
        bar = tk.Canvas(bar_row, width=BAR_W, height=BAR_H,
                        background=C_CANVAS_BG, highlightthickness=1,
                        highlightbackground=C_CIRCLE)
        bar.pack(side=LEFT)
        self._init_bar(bar)
        ttk.Label(bar_row, text="FWD", foreground="#a6e3a1",
                  width=4, anchor=W, font=("", 8)).pack(side=LEFT, padx=(3, 0))

        lbl_var = tk.StringVar(value="Stopped")
        ttk.Label(section, textvariable=lbl_var, font=("", 9)).pack(
            anchor=W, pady=(2, 0))
        return bar, lbl_var

    def _init_bar(self, canvas: tk.Canvas) -> None:
        w, h, half = BAR_W, BAR_H, BAR_HALF
        canvas.create_rectangle(0,    0, half, h, fill=C_BAR_BG_L, outline="")
        canvas.create_rectangle(half, 0, w,    h, fill=C_BAR_BG_R, outline="")
        canvas.create_line(half, 0, half, h, fill=C_CENTER_LINE, width=2)
        canvas.create_rectangle(half, 2, half, h - 2,
                                fill=C_NEUTRAL, outline="", tags="bar")

    # ── Drive control panel ───────────────────────────────────────────────────

    def _build_drive_control_panel(self, parent: ttk.Labelframe) -> None:
        parent.columnconfigure(1, weight=1)

        # Safety acknowledgement gates the ARM button.
        self._ack_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            parent, text="Entendo que armar move as rodas",
            variable=self._ack_var, command=self._on_ack_toggled,
            bootstyle="warning-round-toggle").grid(
            row=0, column=0, columnspan=3, sticky=W, pady=(0, 8))

        # Big ARM / DISARM button
        self._btn_arm = ttk.Button(
            parent, text="ARMAR", bootstyle="success",
            command=self._on_arm_toggle, state=DISABLED, padding=(0, 14))
        self._btn_arm.grid(row=1, column=0, columnspan=3, sticky=EW, pady=(0, 4))

        self._sv_arm_status = tk.StringVar(value="desarmado")
        self._lbl_arm_status = ttk.Label(
            parent, textvariable=self._sv_arm_status,
            font=("", 9, "bold"), bootstyle="secondary")
        self._lbl_arm_status.grid(row=2, column=0, columnspan=3, sticky=W,
                                  pady=(0, 6))

        ttk.Separator(parent, orient=HORIZONTAL, bootstyle="secondary").grid(
            row=3, column=0, columnspan=3, sticky=EW, pady=6)

        # Tuning sliders (sent live in every drive_cfg)
        self._var_max_duty = tk.DoubleVar(value=self._max_duty)
        self._var_accel    = tk.DoubleVar(value=self._accel)
        self._var_decel    = tk.DoubleVar(value=self._decel)
        self._sv_max_duty  = tk.StringVar(value=f"{self._max_duty:.2f}")
        self._sv_accel     = tk.StringVar(value=f"{self._accel:.1f}")
        self._sv_decel     = tk.StringVar(value=f"{self._decel:.1f}")
        self._sv_duty_warn = tk.StringVar(value="")

        def _on_max_duty(*_: Any) -> None:
            self._max_duty = max(MAX_DUTY_MIN,
                                 min(MAX_DUTY_MAX, round(self._var_max_duty.get(), 2)))
            self._sv_max_duty.set(f"{self._max_duty:.2f}")
            self._sv_duty_warn.set(
                f"⚠  duty máx > {MAX_DUTY_WARN:.2f} — só suspenso."
                if self._max_duty > MAX_DUTY_WARN else "")

        def _on_accel(*_: Any) -> None:
            self._accel = max(RAMP_MIN, min(RAMP_MAX, round(self._var_accel.get(), 1)))
            self._sv_accel.set(f"{self._accel:.1f}")

        def _on_decel(*_: Any) -> None:
            self._decel = max(RAMP_MIN, min(RAMP_MAX, round(self._var_decel.get(), 1)))
            self._sv_decel.set(f"{self._decel:.1f}")

        self._var_max_duty.trace_add("write", _on_max_duty)
        self._var_accel.trace_add("write",    _on_accel)
        self._var_decel.trace_add("write",    _on_decel)

        rows = [
            ("Duty máx",    self._var_max_duty, MAX_DUTY_MIN, MAX_DUTY_MAX, 0.01,
             self._sv_max_duty, "fração do PWM máximo"),
            ("Rampa accel", self._var_accel, RAMP_MIN, RAMP_MAX, 0.1,
             self._sv_accel, "duty/s ao acelerar"),
            ("Rampa decel", self._var_decel, RAMP_MIN, RAMP_MAX, 0.1,
             self._sv_decel, "duty/s ao frear"),
        ]
        for i, (label, var, lo, hi, res, sv, hint) in enumerate(rows):
            row = 4 + i
            ttk.Label(parent, text=label, width=12, anchor=E,
                      foreground=C_MUTED, font=("", 8)).grid(
                row=row, column=0, sticky=E, padx=(0, 6), pady=3)
            tk.Scale(parent, variable=var, from_=lo, to=hi, resolution=res,
                     orient=HORIZONTAL, length=180, showvalue=False,
                     bg=self.root.style.colors.bg, fg=C_MUTED,
                     troughcolor=C_CANVAS_BG, highlightthickness=0, bd=0,
                     activebackground=C_DOT_FILL).grid(
                row=row, column=1, sticky=EW, padx=(0, 6), pady=3)
            ttk.Label(parent, textvariable=sv, width=6, anchor=W,
                      font=("Courier", 10, "bold"), bootstyle="info").grid(
                row=row, column=2, sticky=W, pady=3)

        ttk.Label(parent, textvariable=self._sv_duty_warn, bootstyle="warning",
                  font=("", 8), wraplength=240).grid(
            row=7, column=0, columnspan=3, sticky=EW, pady=(2, 6))

        ttk.Separator(parent, orient=HORIZONTAL, bootstyle="secondary").grid(
            row=8, column=0, columnspan=3, sticky=EW, pady=6)

        # Emergency stop
        ttk.Button(parent, text="STOP", bootstyle="danger",
                   command=self._on_stop, padding=(0, 14)).grid(
            row=9, column=0, columnspan=3, sticky=EW, pady=(0, 8))

        # Latest response
        resp = ttk.Labelframe(parent, text="Última resposta", padding=6,
                              bootstyle="secondary")
        resp.grid(row=10, column=0, columnspan=3, sticky=EW)
        self._sv_ack = tk.StringVar(value="—")
        self._sv_err = tk.StringVar(value="—")
        for i, (lbl, var, style) in enumerate([
            ("ACK", self._sv_ack, "success"),
            ("ERR", self._sv_err, "danger"),
        ]):
            ttk.Label(resp, text=f"{lbl}:", width=4, anchor=E,
                      foreground=C_MUTED, font=("", 8)).grid(
                row=i, column=0, sticky=E, padx=(0, 6), pady=1)
            ttk.Label(resp, textvariable=var, width=26, anchor=W,
                      font=("Courier", 8), bootstyle=style).grid(
                row=i, column=1, sticky=W, pady=1)

        self._sv_seq = tk.StringVar(value="seq: 0")
        ttk.Label(parent, textvariable=self._sv_seq, foreground=C_MUTED,
                  font=("", 7)).grid(row=11, column=0, columnspan=3,
                                     sticky=W, pady=(4, 0))

    # ── Safety panel ──────────────────────────────────────────────────────────

    def _build_safety_panel(self, parent: ttk.Frame) -> None:
        safety = ttk.Labelframe(parent, text="Safety", padding=10,
                                bootstyle="warning")
        safety.grid(row=3, column=0, columnspan=3, sticky=EW, pady=(4, 0))
        ttk.Label(
            safety,
            text="⚠  Ao ARMAR, o joystick físico move as rodas. Mantenha a "
                 "cadeira suspensa / sem carga até validar o comportamento. "
                 "Soltar o joystick freia pela rampa; STOP/desarmar para "
                 "imediatamente; queda da GUI/USB desarma em <400 ms.",
            bootstyle="warning", font=("", 9, "bold"),
            wraplength=900, justify=LEFT).pack(fill=X)

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
            self._sv_ack.set(f"cmd_seq={cmd_seq}  {pkt.get('status', '?')}")
            self._sv_rx.set(f"ACK  cmd_seq={cmd_seq}")
            return
        if pkt_type == "err":
            code = pkt.get("code", "?")
            self._sv_err.set(str(code))
            self._sv_rx.set(f"ERR  {code}")
            return
        if pkt_type == "status":
            self._sv_rx.set(f"{pkt.get('event', '?')}: {pkt.get('status', '?')}")
            return
        if pkt_type != "drive":
            return

        x = finite_float(pkt.get("x"))
        y = finite_float(pkt.get("y"))
        self._sv_x.set("—" if x is None else f"{x:+.3f}")
        self._sv_y.set("—" if y is None else f"{y:+.3f}")
        if x is not None:
            self.latest_x = x
        if y is not None:
            self.latest_y = y

        ol  = finite_float(pkt.get("out_left"))
        orr = finite_float(pkt.get("out_right"))
        self.out_left  = ol  if ol  is not None else 0.0
        self.out_right = orr if orr is not None else 0.0
        self.armed_fw = bool(pkt.get("armed", False))
        self.driving  = bool(pkt.get("driving", False))

    # ── GUI frame ─────────────────────────────────────────────────────────────

    def _gui_frame(self) -> None:
        self._process_queue()

        # Joystick visualiser
        self.filtered_x = exp_step(self.filtered_x, self.latest_x, FILTER_ALPHA)
        self.filtered_y = exp_step(self.filtered_y, self.latest_y, FILTER_ALPHA)
        self.visual_x   = exp_step(self.visual_x,   self.filtered_x, INTERP_ALPHA)
        self.visual_y   = exp_step(self.visual_y,   self.filtered_y, INTERP_ALPHA)
        self._move_dot(self.visual_x, self.visual_y)

        # Motor output monitor
        self._update_bar(self._left_bar,  self.out_left)
        self._update_bar(self._right_bar, self.out_right)
        self._sv_left_lbl.set(duty_label(self.out_left))
        self._sv_right_lbl.set(duty_label(self.out_right))

        if self.driving:
            self._sv_drive_state.set("ANDANDO")
            self._lbl_drive_state.configure(bootstyle="success")
        elif self.armed_fw:
            self._sv_drive_state.set("ARMADO")
            self._lbl_drive_state.configure(bootstyle="warning")
        else:
            self._sv_drive_state.set("DESARMADO")
            self._lbl_drive_state.configure(bootstyle="secondary")

        if not self.closing:
            self.root.after(self.gui_update_ms, self._gui_frame)

    def _move_dot(self, x: float, y: float) -> None:
        xd, yd  = clamp_unit_circle(x, y)
        center  = CANVAS_SIZE / 2
        draw_r  = JOYSTICK_RADIUS - DOT_RADIUS
        cx      = center + xd * draw_r
        cy      = center - yd * draw_r
        self._joy_canvas.coords(
            self._dot,
            cx - DOT_RADIUS, cy - DOT_RADIUS,
            cx + DOT_RADIUS, cy + DOT_RADIUS)

    def _update_bar(self, canvas: tk.Canvas, value: float) -> None:
        v    = max(-1.0, min(1.0, value))
        half = BAR_HALF
        h    = BAR_H
        px   = int(v * half)
        if v > 0.0:
            x0, x1, color = half,      half + px, C_FWD
        elif v < 0.0:
            x0, x1, color = half + px, half,      C_REV
        else:
            x0, x1, color = half,      half,      C_NEUTRAL
        canvas.itemconfigure("bar", fill=color)
        canvas.coords("bar", x0, 2, x1, h - 2)

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
        with self._write_lock:
            if self._conn is None:
                self._sv_conn.set("not connected")
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

    def _send_drive_cfg(self, armed: bool) -> bool:
        return self._send_command({
            "type": "drive_cfg",
            "accel": round(self._accel, 2),
            "decel": round(self._decel, 2),
            "max_duty": round(self._max_duty, 3),
            "armed": armed,
        })

    # ── Drive-cfg keep-alive stream ───────────────────────────────────────────

    def _drive_tick(self) -> None:
        if not self._armed or self.closing:
            self._stream_after_id = None
            return
        if not self._send_drive_cfg(True):
            self._set_armed(False)
            return
        self._stream_after_id = self.root.after(
            DRIVE_STREAM_PERIOD_MS, self._drive_tick)

    # ── Handlers ──────────────────────────────────────────────────────────────

    def _toggle_fullscreen(self) -> None:
        self.root.attributes("-fullscreen",
                             not bool(self.root.attributes("-fullscreen")))

    def _exit_fullscreen(self) -> None:
        self.root.attributes("-fullscreen", False)

    def _on_ack_toggled(self) -> None:
        if not self._ack_var.get() and self._armed:
            self._set_armed(False)
        self._btn_arm.configure(state=NORMAL if self._ack_var.get() else DISABLED)

    def _on_arm_toggle(self) -> None:
        if not self._armed and not self._ack_var.get():
            return
        self._set_armed(not self._armed)

    def _set_armed(self, armed: bool) -> None:
        self._armed = armed
        if armed:
            self._btn_arm.configure(text="DESARMAR", bootstyle="danger")
            self._sv_arm_status.set("ARMADO — joystick ativo")
            self._lbl_arm_status.configure(bootstyle="warning")
            if self._stream_after_id is None:
                self._drive_tick()
        else:
            self._btn_arm.configure(text="ARMAR", bootstyle="success")
            self._sv_arm_status.set("desarmado")
            self._lbl_arm_status.configure(bootstyle="secondary")
            if self._stream_after_id is not None:
                self.root.after_cancel(self._stream_after_id)
                self._stream_after_id = None
            # Tell the firmware to disarm immediately.
            self._send_drive_cfg(False)
            self._send_command({"type": "stop"})

    def _on_stop(self) -> None:
        self._set_armed(False)

    # ── Close ─────────────────────────────────────────────────────────────────

    def _on_close(self) -> None:
        if self.closing:
            return
        self.closing = True
        self._set_armed(False)
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
        print("Missing dependency: pyserial. Run: pip install -r requirements-dev.txt",
              file=sys.stderr)
        return 2

    try:
        root = ttk.Window(themename=THEME)
    except Exception as exc:
        print(f"Unable to start GUI: {exc}", file=sys.stderr)
        return 1

    event_queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
    stop_event = threading.Event()
    reader = threading.Thread(
        target=serial_reader,
        args=(serial, args.port, args.baud, event_queue, stop_event),
        name="serial-reader", daemon=True)

    WheelchairControlGUI(
        root, event_queue, stop_event, reader,
        gui_update_ms=args.gui_update_ms)
    reader.start()
    root.mainloop()
    stop_event.set()
    reader.join(timeout=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
