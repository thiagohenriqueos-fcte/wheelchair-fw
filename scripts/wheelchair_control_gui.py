#!/usr/bin/env python3
"""Wheelchair integrated control GUI — v0.8.0

Usage (source the ROS2 env first for LIDAR, e.g. /opt/ros/jazzy/setup.bash):
    .venv/bin/python scripts/wheelchair_control_gui.py /dev/ttyACM0 --lidar /dev/ttyUSB0
    .venv/bin/python scripts/wheelchair_control_gui.py --lidar /dev/ttyUSB0   # LIDAR only

The optional RPLIDAR C1 is handled by the official sllidar_ros2 driver: the GUI
starts that driver and subscribes to its sensor_msgs/LaserScan on /scan, on its
own thread — never sharing the ESP32 telemetry link. The ESP32 port is optional.

WARNING: for suspended / no-load PWM testing ONLY.
"""

import argparse
import json
import math
import os
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from typing import Any, Dict, List, Optional, Tuple

import ttkbootstrap as ttk  # drop-in replacement for tkinter.ttk
from ttkbootstrap.constants import *


# ── Theme ─────────────────────────────────────────────────────────────────────

THEME = "darkly"


# ── Safety / Range ────────────────────────────────────────────────────────────

PWM_LIMIT_DEFAULT        = 0.30
PWM_LIMIT_MIN            = 0.00
PWM_LIMIT_MAX            = 1.00
PWM_LIMIT_WARN_THRESHOLD = 0.30


# ── Layout ────────────────────────────────────────────────────────────────────

CANVAS_SIZE     = 260
JOYSTICK_RADIUS = 100
DOT_RADIUS      = 9

BAR_W    = 220
BAR_H    = 26
BAR_HALF = BAR_W // 2


# ── Timing / smoothing defaults ───────────────────────────────────────────────

FILTER_ALPHA  = 0.25
INTERP_ALPHA  = 0.20
GUI_UPDATE_MS = 33
AGE_UPDATE_MS = 100

SMOOTHING_ALPHA_MIN = 0.01
SMOOTHING_ALPHA_MAX = 1.0
GUI_MS_MIN = 10
GUI_MS_MAX = 100

PWM_STREAM_HZ        = 10
PWM_STREAM_PERIOD_MS = 1000 // PWM_STREAM_HZ


# ── Encoder constants ─────────────────────────────────────────────────────────

ENCODER_PPR            = 2000
ENCODER_COUNTS_PER_REV = ENCODER_PPR * 4
ENCODER_DT             = 0.050

TWO_PI          = 2.0 * math.pi
OMEGA_PER_COUNT = TWO_PI / (ENCODER_COUNTS_PER_REV * ENCODER_DT)

ENC_FILTER_ALPHA = 0.30
ENC_INTERP_ALPHA = 0.20
ENC_MAX_OMEGA    = 50.0


# ── Colours (dark-theme canvas palette) ───────────────────────────────────────

C_CANVAS_BG   = "#1e1e2e"
C_CANVAS_GRID = "#313244"
C_CIRCLE      = "#585b70"
C_DOT_FILL    = "#89b4fa"
C_DOT_OUTLINE = "#1e66f5"
C_CENTER_LINE = "#585b70"

C_BAR_BG_L  = "#3d1111"    # reverse / CCW side
C_BAR_BG_R  = "#113d11"    # forward / CW side
C_FWD       = "#a6e3a1"    # forward fill — soft green
C_REV       = "#f38ba8"    # reverse fill — soft red/pink
C_NEUTRAL   = "#585b70"    # stopped bar fill
C_LIMIT     = "#fab387"    # 0.30 limit tick — peach
C_MUTED     = "#6c7086"    # secondary / muted text


# ── LIDAR (RPLIDAR C1 via ROS2) ───────────────────────────────────────────────
#
# The C1 is driven by the official sllidar_ros2 node (it owns the motor + C1
# protocol — the legacy Python `rplidar` lib can't drive the C1). The GUI starts
# that driver and subscribes to its sensor_msgs/LaserScan on /scan, on its own
# thread — fully independent of the ESP32 telemetry link.
#
# Requires a sourced ROS2 environment (e.g. `source /opt/ros/jazzy/setup.bash`
# and the sllidar workspace) before launching the GUI.

LIDAR_DEFAULT_PORT  = "/dev/ttyUSB0"   # serial_port handed to the driver launch
LIDAR_DEFAULT_TOPIC = "/scan"
LIDAR_LAUNCH_PKG    = "sllidar_ros2"
LIDAR_LAUNCH_FILE   = "sllidar_c1_launch.py"

LIDAR_CANVAS    = 360          # square polar-plot canvas (px)
LIDAR_MARGIN    = 12
LIDAR_POINT_R   = 1.6          # plotted point radius (px)
LIDAR_POOL_SIZE = 1500         # reused canvas point items (≥ points per scan)

LIDAR_MIN_RANGE_DEFAULT = 0.15   # m — discard returns closer than this
LIDAR_MAX_RANGE_DEFAULT = 6.0    # m — display scale + far gate
LIDAR_QUALITY_DEFAULT   = 10     # discard returns below this quality (intensity)
LIDAR_ANGLE_OFFSET_DEF  = 0.0    # deg — align sensor 0° to chassis front
LIDAR_FLIP_DEFAULT      = True   # ROS LaserScan is CCW; screen plot is CW

LIDAR_RANGE_MIN_LIMIT = 0.05     # C1 hardware minimum
LIDAR_RANGE_MAX_LIMIT = 12.0     # C1 hardware maximum
LIDAR_QUALITY_LIMIT   = 63

LIDAR_RINGS = (0.25, 0.5, 0.75, 1.0)   # grid rings as fraction of max range

LIDAR_DANGER_M = 0.5    # sector distance ≤ this → red
LIDAR_WARN_M   = 1.0    # sector distance ≤ this → orange

C_LIDAR_POINT   = "#89b4fa"
C_LIDAR_NEAREST = "#f38ba8"
C_LIDAR_FRONT   = "#a6e3a1"


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
    p = argparse.ArgumentParser(
        description="Wheelchair control GUI — suspended/no-load testing only.")
    p.add_argument("port", nargs="?", default=None,
                   help="ESP32 serial port, e.g. /dev/ttyACM0 "
                        "(optional — omit to run LIDAR-only)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--filter-alpha",  type=_alpha,   default=FILTER_ALPHA)
    p.add_argument("--interp-alpha",  type=_alpha,   default=INTERP_ALPHA)
    p.add_argument("--gui-update-ms", type=_pos_int, default=GUI_UPDATE_MS)
    p.add_argument("--lidar", default=None,
                   help=f"RPLIDAR C1 serial port for the driver, e.g. {LIDAR_DEFAULT_PORT}")
    p.add_argument("--lidar-topic", default=LIDAR_DEFAULT_TOPIC,
                   help=f"LaserScan topic to subscribe (default {LIDAR_DEFAULT_TOPIC})")
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


def motor_label(value: float, active: bool) -> str:
    if not active or value == 0.0:
        return "Stopped"
    pct = abs(value) * 100.0
    direction = "▶  Forward" if value > 0 else "◀  Reverse"
    return f"{direction}   {pct:.0f}%"


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


# ── LIDAR reader thread (ROS2 /scan subscriber) ───────────────────────────────

def laserscan_to_points(msg: Any) -> List[Tuple[float, float, float]]:
    """Convert a sensor_msgs/LaserScan into [(quality, angle_deg, distance_mm)].

    LaserScan angles are radians CCW from the sensor's +x; distances are metres
    with inf/nan for no-return. sllidar publishes the per-beam quality in
    `intensities`; when absent/zero we treat the beam as high quality so it is
    not dropped by the host-side quality gate.
    """
    inc   = msg.angle_increment
    a0    = msg.angle_min
    inten = msg.intensities
    n_int = len(inten)
    pts: List[Tuple[float, float, float]] = []
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        q = inten[i] if i < n_int else 0.0
        quality = q if q and q > 0 else 255.0
        ang_deg = math.degrees(a0 + i * inc) % 360.0
        pts.append((float(quality), ang_deg, r * 1000.0))
    return pts


def lidar_ros_reader(
    topic: str,
    serial_port: str,
    event_queue: "queue.Queue[Tuple[str, Any]]",
    stop_event: threading.Event,
) -> None:
    """Start the sllidar_ros2 driver and subscribe to its LaserScan topic.

    Lives for the thread's lifetime: launches the driver as a child process
    group, subscribes to `topic`, pushes ("lidar_scan", points) per message, and
    on exit shuts the subscriber down and stops the driver (and thus the motor).
    """
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
    except Exception as exc:  # noqa: BLE001 — surface any ROS import failure
        event_queue.put((
            "lidar_error",
            f"ROS2 indisponível ({exc}) — fonte o setup do ROS2 antes de abrir a GUI"))
        event_queue.put(("lidar_stopped", None))
        return

    if shutil.which("ros2") is None:
        event_queue.put(("lidar_error", "comando 'ros2' não encontrado — fonte o ROS2"))
        event_queue.put(("lidar_stopped", None))
        return

    proc: Optional[subprocess.Popen] = None
    node = None
    try:
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("wheelchair_lidar_gui")
        node.create_subscription(
            LaserScan, topic,
            lambda m: event_queue.put(("lidar_scan", laserscan_to_points(m))),
            qos_profile_sensor_data)

        # Reuse an already-running driver if one is publishing; only launch our
        # own otherwise (avoids a second node fighting for the serial port).
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0 and node.count_publishers(topic) == 0:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.count_publishers(topic) == 0:
            proc = subprocess.Popen(
                ["ros2", "launch", LIDAR_LAUNCH_PKG, LIDAR_LAUNCH_FILE,
                 f"serial_port:={serial_port}"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,   # own process group → clean group shutdown
            )
            event_queue.put(("lidar_status", f"driver iniciado ({serial_port})"))
        else:
            event_queue.put(("lidar_status", "usando driver já em execução"))
        event_queue.put(("lidar_status", f"assinando {topic}…"))

        while not stop_event.is_set():
            if proc is not None and proc.poll() is not None:   # our driver died
                event_queue.put(("lidar_error", "driver sllidar terminou (porta ocupada?)"))
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as exc:  # noqa: BLE001
        event_queue.put(("lidar_error", str(exc)))
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if proc is not None:
            _stop_process_group(proc)
        event_queue.put(("lidar_stopped", None))


def _stop_process_group(proc: subprocess.Popen) -> None:
    """SIGINT the driver's process group (graceful), escalate to SIGKILL."""
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if proc.poll() is not None:
            return
        try:
            os.killpg(pgid, sig)
            proc.wait(timeout=6)
            return
        except subprocess.TimeoutExpired:
            continue
        except Exception:
            return


# ── Main GUI class ────────────────────────────────────────────────────────────

class WheelchairControlGUI:
    def __init__(
        self,
        root: ttk.Window,
        event_queue: "queue.Queue[Tuple[str, Any]]",
        stop_event: threading.Event,
        reader_thread: threading.Thread,
        *,
        filter_alpha: float,
        interp_alpha: float,
        gui_update_ms: int,
        esp_port: Optional[str] = None,
        lidar_port: Optional[str] = None,
        lidar_topic: str = LIDAR_DEFAULT_TOPIC,
    ) -> None:
        self.root          = root
        self.event_queue   = event_queue
        self.stop_event    = stop_event
        self.reader_thread = reader_thread
        self.esp_port      = esp_port
        self.closing       = False

        self.filter_alpha  = filter_alpha
        self.interp_alpha  = interp_alpha
        self.gui_update_ms = gui_update_ms

        self._pwm_limit = PWM_LIMIT_DEFAULT
        self._conn: Optional[Any] = None
        self._write_lock = threading.Lock()
        self._seq = 0

        self._streaming: bool = False
        self._stream_after_id: Optional[str] = None

        self.valid_packets   = 0
        self.invalid_packets = 0
        self.last_packet_time: Optional[float] = None

        # Joystick pipeline
        self.latest_x = self.filtered_x = self.visual_x = 0.0
        self.latest_y = self.filtered_y = self.visual_y = 0.0

        self.motor_left = self.motor_right = 0.0
        self.motor_active = False

        self._slider_left:  Optional[tk.Scale] = None
        self._slider_right: Optional[tk.Scale] = None

        # Encoder pipeline
        self.latest_omega_left    = self.filtered_omega_left    = self.visual_omega_left    = 0.0
        self.latest_omega_right   = self.filtered_omega_right   = self.visual_omega_right   = 0.0
        self.enc_left_count  = self.enc_right_count = 0
        self.enc_left_delta  = self.enc_right_delta = 0
        self.enc_status = "—"
        self.enc_ok     = False

        self.enc_filter_alpha = ENC_FILTER_ALPHA
        self.enc_interp_alpha = ENC_INTERP_ALPHA
        self.enc_max_omega    = ENC_MAX_OMEGA

        # LIDAR pipeline (ROS2 /scan subscriber + sllidar driver — own thread,
        # independent of the ESP32 link)
        self._lidar_thread: Optional[threading.Thread] = None
        self._lidar_stop:   Optional[threading.Event]  = None
        self._lidar_running = False
        self._lidar_scan: Optional[list] = None
        self._lidar_dirty = False
        self._lidar_shown = 0           # points displayed last frame
        self.lidar_scans  = 0
        self._lidar_hz    = 0.0
        self._lidar_hz_t0: Optional[float] = None
        self._lidar_hz_n0 = 0
        self._lidar_start_t: Optional[float] = None   # when "Iniciar" was pressed
        self._lidar_nodata_warned = False

        # LIDAR calibration / filtering (applied host-side, live)
        self._lidar_angle_offset = LIDAR_ANGLE_OFFSET_DEF
        self._lidar_flip         = LIDAR_FLIP_DEFAULT
        self._lidar_min_range    = LIDAR_MIN_RANGE_DEFAULT
        self._lidar_max_range    = LIDAR_MAX_RANGE_DEFAULT
        self._lidar_quality_min  = LIDAR_QUALITY_DEFAULT
        self._lidar_init_port    = lidar_port or LIDAR_DEFAULT_PORT
        self._lidar_init_topic   = lidar_topic or LIDAR_DEFAULT_TOPIC

        self._build_ui()
        if self.esp_port is None:
            self._sv_conn.set("sem ESP — modo LIDAR")
        self.root.after(self.gui_update_ms, self._gui_frame)
        self.root.after(AGE_UPDATE_MS,      self._tick_age)

    # ── Build UI ──────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.root.title("Wheelchair Control  v0.8.0")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        # ── Scrollable container ──────────────────────────────────────────────
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

        # ── Panels ────────────────────────────────────────────────────────────
        self._build_status_bar(outer)    # row 0

        self._build_view_toolbar(outer)  # row 1

        joy_frame = ttk.Labelframe(
            outer, text="Joystick", padding=10, bootstyle="secondary")
        joy_frame.grid(row=2, column=0, sticky="n", padx=(0, 8), pady=(0, 8))
        self._build_joystick_panel(joy_frame)

        mon_frame = ttk.Labelframe(
            outer, text="Motor Monitor", padding=10, bootstyle="secondary")
        mon_frame.grid(row=2, column=1, sticky="n", padx=(0, 8), pady=(0, 8))
        self._build_motor_monitor_panel(mon_frame)

        ctrl_frame = ttk.Labelframe(
            outer, text="Motor Control", padding=12, bootstyle="secondary")
        ctrl_frame.grid(row=2, column=2, sticky="nsew", pady=(0, 8))
        self._build_motor_control_panel(ctrl_frame)

        smooth_frame = ttk.Labelframe(
            outer, text="Joystick Smoothing", padding=10, bootstyle="secondary")
        smooth_frame.grid(
            row=3, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self._build_smoothing_panel(smooth_frame)

        enc_frame = ttk.Labelframe(
            outer,
            text="Encoder  (2000 PPR / 8000 counts·rev⁻¹)",
            padding=10, bootstyle="secondary")
        enc_frame.grid(
            row=4, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self._build_encoder_panel(enc_frame)

        enc_smooth_frame = ttk.Labelframe(
            outer, text="Encoder Smoothing", padding=10, bootstyle="secondary")
        enc_smooth_frame.grid(
            row=5, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self._build_encoder_smoothing_panel(enc_smooth_frame)

        lidar_frame = ttk.Labelframe(
            outer, text="LIDAR  (RPLIDAR C1)", padding=10, bootstyle="secondary")
        lidar_frame.grid(
            row=6, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self._build_lidar_panel(lidar_frame)

        self._build_safety_panel(outer)  # row 7

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

        # Coloured left stripe
        stripe = ttk.Frame(bar, width=4, bootstyle="info")
        stripe.pack(side=LEFT, fill=Y, padx=(0, 10))

        self._sv_conn    = tk.StringVar(value="connecting…")
        self._sv_fw      = tk.StringVar(value="—")
        self._sv_hb      = tk.StringVar(value="—")
        self._sv_valid   = tk.StringVar(value="0")
        self._sv_invalid = tk.StringVar(value="0")
        self._sv_age     = tk.StringVar(value="—")
        self._sv_last_rx = tk.StringVar(value="—")

        fields = [
            ("Port",     self._sv_conn,    22, "info"),
            ("FW",       self._sv_fw,       8, "success"),
            ("HB",       self._sv_hb,       5, None),
            ("Valid",    self._sv_valid,    5, "success"),
            ("Invalid",  self._sv_invalid,  5, "warning"),
            ("Last pkt", self._sv_age,      8, None),
            ("Last RX",  self._sv_last_rx, 30, None),
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
            parent,
            width=CANVAS_SIZE, height=CANVAS_SIZE,
            background=C_CANVAS_BG,
            highlightthickness=1,
            highlightbackground=C_CIRCLE,
        )
        self._joy_canvas.pack()

        c      = self._joy_canvas
        center = CANVAS_SIZE / 2
        r      = JOYSTICK_RADIUS

        c.create_oval(
            center - r, center - r, center + r, center + r,
            outline=C_CIRCLE, width=1,
        )
        c.create_line(center - r, center, center + r, center,
                      fill=C_CANVAS_GRID, width=1)
        c.create_line(center, center - r, center, center + r,
                      fill=C_CANVAS_GRID, width=1)
        for text, x, y in [
            ("+Y", center,        center - r - 12),
            ("-Y", center,        center + r + 12),
            ("-X", center-r - 14, center),
            ("+X", center+r + 14, center),
        ]:
            c.create_text(x, y, text=text, fill=C_MUTED, font=("", 8))

        self._dot = c.create_oval(
            center - DOT_RADIUS, center - DOT_RADIUS,
            center + DOT_RADIUS, center + DOT_RADIUS,
            fill=C_DOT_FILL, outline=C_DOT_OUTLINE, width=2,
        )

        num = ttk.Frame(parent, padding=(4, 8, 0, 0))
        num.pack(fill=X)

        self._sv_raw_x  = tk.StringVar(value="—")
        self._sv_raw_y  = tk.StringVar(value="—")
        self._sv_x      = tk.StringVar(value="—")
        self._sv_y      = tk.StringVar(value="—")
        self._sv_filt_x = tk.StringVar(value="—")
        self._sv_filt_y = tk.StringVar(value="—")
        self._sv_vis_x  = tk.StringVar(value="—")
        self._sv_vis_y  = tk.StringVar(value="—")

        rows = [
            ("raw_x",  self._sv_raw_x,  False),
            ("raw_y",  self._sv_raw_y,  False),
            ("x",      self._sv_x,      False),
            ("y",      self._sv_y,      False),
            ("filt_x", self._sv_filt_x, True),
            ("filt_y", self._sv_filt_y, True),
            ("vis_x",  self._sv_vis_x,  True),
            ("vis_y",  self._sv_vis_y,  True),
        ]
        for i, (lbl, var, muted) in enumerate(rows):
            fg = C_MUTED if muted else None
            kw: Dict[str, Any] = {}
            if fg:
                kw["foreground"] = fg
            lw = ttk.Label(num, text=f"{lbl}:", width=7, anchor=E, **kw)
            lw.grid(row=i, column=0, sticky=E, padx=(0, 6), pady=1)
            vw = ttk.Label(num, textvariable=var, width=10, anchor=W,
                           font=("Courier", 9), **kw)
            vw.grid(row=i, column=1, sticky=W, pady=1)

    # ── Motor monitor panel ───────────────────────────────────────────────────

    def _build_motor_monitor_panel(self, parent: ttk.Labelframe) -> None:
        active_row = ttk.Frame(parent)
        active_row.pack(fill=X, pady=(0, 10))
        ttk.Label(active_row, text="motor_test_active",
                  foreground=C_MUTED, font=("", 8)).pack(side=LEFT)
        self._sv_active = tk.StringVar(value="false")
        self._lbl_active = ttk.Label(
            active_row, textvariable=self._sv_active,
            font=("", 10, "bold"), bootstyle="secondary")
        self._lbl_active.pack(side=LEFT, padx=(8, 0))

        self._left_bar,  self._sv_left_lbl,  self._sv_left_gpio = \
            self._build_motor_section(parent, "Left   GPIO10 / GPIO11")

        ttk.Separator(parent, orient=HORIZONTAL, bootstyle="secondary").pack(
            fill=X, pady=8)

        self._right_bar, self._sv_right_lbl, self._sv_right_gpio = \
            self._build_motor_section(parent, "Right  GPIO12 / GPIO13")

    def _build_motor_section(
        self, parent: tk.Widget, title: str,
    ) -> Tuple[tk.Canvas, tk.StringVar, tk.StringVar]:
        section = ttk.Frame(parent)
        section.pack(fill=X, pady=2)

        ttk.Label(section, text=title, font=("", 9, "bold")).pack(anchor=W)

        bar_row = ttk.Frame(section)
        bar_row.pack(fill=X, pady=(4, 2))

        ttk.Label(bar_row, text="LPWM", foreground="#f38ba8",
                  width=5, anchor=E, font=("", 8)).pack(side=LEFT, padx=(0, 3))
        bar = tk.Canvas(bar_row, width=BAR_W, height=BAR_H,
                        background=C_CANVAS_BG, highlightthickness=1,
                        highlightbackground=C_CIRCLE)
        bar.pack(side=LEFT)
        self._init_bar(bar, limit_ticks=True)
        ttk.Label(bar_row, text="RPWM", foreground="#a6e3a1",
                  width=5, anchor=W, font=("", 8)).pack(side=LEFT, padx=(3, 0))

        lbl_var  = tk.StringVar(value="Stopped")
        gpio_var = tk.StringVar(value="Active: —")

        ttk.Label(section, textvariable=lbl_var,
                  font=("", 9)).pack(anchor=W, pady=(2, 0))
        ttk.Label(section, textvariable=gpio_var,
                  foreground=C_MUTED, font=("", 8)).pack(anchor=W)

        return bar, lbl_var, gpio_var

    def _init_bar(self, canvas: tk.Canvas, *, limit_ticks: bool = False) -> None:
        w, h, half = BAR_W, BAR_H, BAR_HALF
        canvas.create_rectangle(0,    0, half, h, fill=C_BAR_BG_L, outline="")
        canvas.create_rectangle(half, 0, w,    h, fill=C_BAR_BG_R, outline="")
        if limit_ticks:
            limit_px = int(0.30 * half)
            for tick_x in (half - limit_px, half + limit_px):
                canvas.create_line(tick_x, 0, tick_x, h,
                                   fill=C_LIMIT, width=1, dash=(3, 3))
        canvas.create_line(half, 0, half, h, fill=C_CENTER_LINE, width=2)
        canvas.create_rectangle(
            half, 2, half, h - 2,
            fill=C_NEUTRAL, outline="", tags="bar")

    # ── Motor control panel ───────────────────────────────────────────────────

    def _build_motor_control_panel(self, parent: ttk.Labelframe) -> None:

        # ── PWM limit ─────────────────────────────────────────────────────────
        self._var_pwm_limit     = tk.DoubleVar(value=PWM_LIMIT_DEFAULT)
        self._sv_pwm_limit_val  = tk.StringVar(value=f"{PWM_LIMIT_DEFAULT:.2f}")
        self._sv_pwm_limit_warn = tk.StringVar(value="")

        def _fmt_limit(*_: Any) -> None:
            self._sv_pwm_limit_val.set(f"{round(self._var_pwm_limit.get(), 2):.2f}")

        self._var_pwm_limit.trace_add("write", _fmt_limit)

        ttk.Label(parent, text="PWM limit", foreground=C_MUTED,
                  font=("", 8)).grid(row=0, column=0, sticky=W, pady=(0, 2))
        limit_row = ttk.Frame(parent)
        limit_row.grid(row=1, column=0, columnspan=3, sticky=EW, pady=(0, 4))
        tk.Scale(limit_row, variable=self._var_pwm_limit,
                 from_=PWM_LIMIT_MIN, to=PWM_LIMIT_MAX,
                 resolution=0.01, orient=HORIZONTAL, length=160,
                 showvalue=False, bg=self.root.style.colors.bg,
                 fg=C_MUTED, troughcolor=C_CANVAS_BG,
                 highlightthickness=0, bd=0,
                 activebackground=C_DOT_FILL,
                 ).pack(side=LEFT, padx=(0, 6))
        ttk.Label(limit_row, textvariable=self._sv_pwm_limit_val,
                  width=5, font=("Courier", 10, "bold"),
                  bootstyle="info").pack(side=LEFT)

        btn_row = ttk.Frame(parent)
        btn_row.grid(row=2, column=0, columnspan=3, sticky=EW, pady=(0, 2))
        ttk.Button(btn_row, text="Apply Limit", bootstyle="warning-outline",
                   command=self._on_apply_pwm_limit,
                   padding=(8, 3)).pack(side=LEFT, padx=(0, 6))
        ttk.Button(btn_row, text="Reset 0.30", bootstyle="secondary-outline",
                   command=self._on_reset_pwm_limit,
                   padding=(8, 3)).pack(side=LEFT)

        self._lbl_pwm_warn = ttk.Label(
            parent, textvariable=self._sv_pwm_limit_warn,
            bootstyle="warning", font=("", 8), wraplength=260)
        self._lbl_pwm_warn.grid(
            row=3, column=0, columnspan=3, sticky=EW, pady=(0, 2))

        ttk.Separator(parent, orient=HORIZONTAL,
                      bootstyle="secondary").grid(
            row=4, column=0, columnspan=3, sticky=EW, pady=8)

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
        scale_kw = dict(
            from_=-lim, to=lim, resolution=0.01,
            orient=HORIZONTAL, length=160, showvalue=False,
            bg=self.root.style.colors.bg, fg=C_MUTED,
            troughcolor=C_CANVAS_BG, highlightthickness=0, bd=0,
            activebackground=C_DOT_FILL,
        )

        for row_off, (label, var, sv) in enumerate([
            ("Left motor",  self._var_left,  self._sv_left_cmd),
            ("Right motor", self._var_right, self._sv_right_cmd),
        ]):
            row = row_off + 5
            ttk.Label(parent, text=label, foreground=C_MUTED,
                      font=("", 8)).grid(
                row=row, column=0, sticky=W, pady=(4, 0))
            slider = tk.Scale(parent, variable=var, **scale_kw)
            slider.grid(row=row, column=1, sticky=EW, padx=(6, 6), pady=(0, 0))
            ttk.Label(parent, textvariable=sv, width=6,
                      font=("Courier", 10, "bold"), bootstyle="info").grid(
                row=row, column=2, sticky=W)
            if row_off == 0:
                self._slider_left  = slider
            else:
                self._slider_right = slider

        ttk.Separator(parent, orient=HORIZONTAL,
                      bootstyle="secondary").grid(
            row=7, column=0, columnspan=3, sticky=EW, pady=8)

        # ── Send / Stream ──────────────────────────────────────────────────────
        self._btn_send_once = ttk.Button(
            parent, text="Send Once", bootstyle="primary",
            command=self._on_send_once, state=DISABLED, padding=(0, 5))
        self._btn_send_once.grid(
            row=8, column=0, columnspan=3, sticky=EW, pady=(0, 4))

        stream_frame = ttk.Frame(parent)
        stream_frame.grid(row=9, column=0, columnspan=3, sticky=EW, pady=(0, 4))
        stream_frame.columnconfigure(0, weight=1)
        stream_frame.columnconfigure(1, weight=1)

        self._btn_start_stream = ttk.Button(
            stream_frame, text="Start Stream", bootstyle="success",
            command=self._on_start_stream, state=DISABLED, padding=(0, 5))
        self._btn_start_stream.grid(row=0, column=0, sticky=EW, padx=(0, 3))

        self._btn_stop_stream = ttk.Button(
            stream_frame, text="Stop Stream", bootstyle="secondary",
            command=self._on_stop_stream, state=DISABLED, padding=(0, 5))
        self._btn_stop_stream.grid(row=0, column=1, sticky=EW, padx=(3, 0))

        info_row = ttk.Frame(parent)
        info_row.grid(row=10, column=0, columnspan=3, sticky=EW, pady=(0, 2))
        self._sv_stream_status = tk.StringVar(value="Stream OFF")
        self._lbl_stream_status = ttk.Label(
            info_row, textvariable=self._sv_stream_status,
            font=("", 8, "bold"), bootstyle="secondary")
        self._lbl_stream_status.pack(side=LEFT, padx=(0, 12))

        tx_inner = ttk.Frame(info_row)
        tx_inner.pack(side=LEFT)
        ttk.Label(tx_inner, text="L", foreground=C_MUTED, font=("", 8)).pack(
            side=LEFT, padx=(0, 2))
        self._sv_stream_left = tk.StringVar(value="+0.00")
        ttk.Label(tx_inner, textvariable=self._sv_stream_left,
                  font=("Courier", 8)).pack(side=LEFT)
        ttk.Label(tx_inner, text="  R", foreground=C_MUTED, font=("", 8)).pack(
            side=LEFT, padx=(0, 2))
        self._sv_stream_right = tk.StringVar(value="+0.00")
        ttk.Label(tx_inner, textvariable=self._sv_stream_right,
                  font=("Courier", 8)).pack(side=LEFT)

        ttk.Separator(parent, orient=HORIZONTAL,
                      bootstyle="secondary").grid(
            row=11, column=0, columnspan=3, sticky=EW, pady=8)

        # ── STOP ──────────────────────────────────────────────────────────────
        self._btn_stop = ttk.Button(
            parent, text="STOP", bootstyle="danger",
            command=self._on_stop, padding=(0, 14))
        self._btn_stop.grid(
            row=12, column=0, columnspan=3, sticky=EW, pady=(0, 8))

        # ── Zero utilities ─────────────────────────────────────────────────────
        util = ttk.Frame(parent)
        util.grid(row=13, column=0, columnspan=3, sticky=EW)
        for col, (label, cmd) in enumerate([
            ("Zero L", self._on_zero_left),
            ("Zero R", self._on_zero_right),
            ("Zero ⬛", self._on_zero_both),
        ]):
            ttk.Button(util, text=label, bootstyle="secondary-outline",
                       command=cmd, padding=(0, 4)).grid(
                row=0, column=col, sticky=EW,
                padx=(0 if col == 0 else 3, 0))
        util.columnconfigure(0, weight=1)
        util.columnconfigure(1, weight=1)
        util.columnconfigure(2, weight=1)

        ttk.Separator(parent, orient=HORIZONTAL,
                      bootstyle="secondary").grid(
            row=14, column=0, columnspan=3, sticky=EW, pady=8)

        # ── Latest response ───────────────────────────────────────────────────
        resp = ttk.Labelframe(parent, text="Latest Response",
                              padding=6, bootstyle="secondary")
        resp.grid(row=15, column=0, columnspan=3, sticky=EW)

        self._sv_last_ack = tk.StringVar(value="—")
        self._sv_last_err = tk.StringVar(value="—")

        for row_i, (lbl, var, style) in enumerate([
            ("ACK", self._sv_last_ack, "success"),
            ("ERR", self._sv_last_err, "danger"),
        ]):
            ttk.Label(resp, text=f"{lbl}:", width=4, anchor=E,
                      foreground=C_MUTED, font=("", 8)).grid(
                row=row_i, column=0, sticky=E, padx=(0, 6), pady=1)
            ttk.Label(resp, textvariable=var, width=28, anchor=W,
                      font=("Courier", 8), bootstyle=style).grid(
                row=row_i, column=1, sticky=W, pady=1)

        self._sv_seq = tk.StringVar(value="seq: 0")
        ttk.Label(parent, textvariable=self._sv_seq,
                  foreground=C_MUTED, font=("", 7)).grid(
            row=16, column=0, columnspan=3, sticky=W, pady=(4, 0))

    # ── Joystick smoothing ────────────────────────────────────────────────────

    def _build_smoothing_panel(self, parent: ttk.Labelframe) -> None:
        parent.columnconfigure(1, weight=1)

        self._var_smooth_filter = tk.DoubleVar(value=self.filter_alpha)
        self._var_smooth_interp = tk.DoubleVar(value=self.interp_alpha)
        self._var_smooth_ms     = tk.IntVar(value=self.gui_update_ms)
        self._sv_smooth_filter  = tk.StringVar(value=f"{self.filter_alpha:.2f}")
        self._sv_smooth_interp  = tk.StringVar(value=f"{self.interp_alpha:.2f}")
        self._sv_smooth_ms      = tk.StringVar(value=str(self.gui_update_ms))

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
            ("Filter alpha",    self._var_smooth_filter,
             SMOOTHING_ALPHA_MIN, SMOOTHING_ALPHA_MAX, 0.01,
             self._sv_smooth_filter, "1.00 = raw   0.01 = very slow"),
            ("Interp alpha",    self._var_smooth_interp,
             SMOOTHING_ALPHA_MIN, SMOOTHING_ALPHA_MAX, 0.01,
             self._sv_smooth_interp, "1.00 = instant   0.01 = slow lag"),
            ("Update interval", self._var_smooth_ms,
             GUI_MS_MIN, GUI_MS_MAX, 1,
             self._sv_smooth_ms, f"{GUI_MS_MIN} ms = fast   {GUI_MS_MAX} ms = less CPU"),
        ]
        self._build_slider_rows(parent, rows, 0)
        ttk.Button(parent, text="Reset Defaults", bootstyle="secondary-outline",
                   command=self._on_reset_smoothing,
                   padding=(8, 3)).grid(
            row=3, column=0, columnspan=2, sticky=W, pady=(8, 0))

    # ── Encoder panel ─────────────────────────────────────────────────────────

    def _build_encoder_panel(self, parent: ttk.Labelframe) -> None:
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

        for col, (title, gpio) in enumerate([
            ("Left",  "GPIO4 / GPIO5"),
            ("Right", "GPIO6 / GPIO7"),
        ]):
            frame = ttk.Labelframe(
                parent, text=f"{title}  —  {gpio}",
                padding=8, bootstyle="secondary")
            frame.grid(row=0, column=col, sticky=NSEW,
                       padx=(0, 6) if col == 0 else (6, 0))

            bar = tk.Canvas(frame, width=BAR_W, height=BAR_H,
                            background=C_CANVAS_BG, highlightthickness=1,
                            highlightbackground=C_CIRCLE)

            bar_row = ttk.Frame(frame)
            bar_row.pack(fill=X, pady=(0, 8))
            ttk.Label(bar_row, text="CCW", foreground="#f38ba8",
                      width=4, anchor=E, font=("", 8)).pack(
                side=LEFT, padx=(0, 3))
            bar.pack(side=LEFT, in_=bar_row)
            self._init_bar(bar, limit_ticks=False)
            ttk.Label(bar_row, text="CW", foreground="#a6e3a1",
                      width=3, anchor=W, font=("", 8)).pack(
                side=LEFT, padx=(3, 0))

            # Big value display
            val_frame = ttk.Frame(frame)
            val_frame.pack(fill=X, pady=(0, 4))

            sv_omega = tk.StringVar(value="—")
            sv_rpm   = tk.StringVar(value="—")
            ttk.Label(val_frame, textvariable=sv_omega,
                      font=("Courier", 13, "bold"),
                      bootstyle="info").pack(side=LEFT, padx=(0, 12))
            ttk.Label(val_frame, textvariable=sv_rpm,
                      font=("Courier", 11),
                      bootstyle="warning").pack(side=LEFT)

            # Detail rows
            sv_filt  = tk.StringVar(value="—")
            sv_vis   = tk.StringVar(value="—")
            sv_count = tk.StringVar(value="—")
            sv_delta = tk.StringVar(value="—")

            detail = ttk.Frame(frame)
            detail.pack(fill=X)
            for i, (lbl, sv) in enumerate([
                ("ω filt", sv_filt),
                ("ω vis",  sv_vis),
                ("count",  sv_count),
                ("Δ/cycle", sv_delta),
            ]):
                ttk.Label(detail, text=f"{lbl}:", width=8, anchor=E,
                          foreground=C_MUTED, font=("", 8)).grid(
                    row=i, column=0, sticky=E, padx=(0, 4), pady=1)
                ttk.Label(detail, textvariable=sv, width=16, anchor=W,
                          font=("Courier", 8),
                          foreground=C_MUTED).grid(
                    row=i, column=1, sticky=W, pady=1)

            if col == 0:
                (self._enc_left_bar, self._sv_enc_left_omega,
                 self._sv_enc_left_rpm, self._sv_enc_left_filt,
                 self._sv_enc_left_vis, self._sv_enc_left_count,
                 self._sv_enc_left_delta) = (
                    bar, sv_omega, sv_rpm, sv_filt, sv_vis, sv_count, sv_delta)
            else:
                (self._enc_right_bar, self._sv_enc_right_omega,
                 self._sv_enc_right_rpm, self._sv_enc_right_filt,
                 self._sv_enc_right_vis, self._sv_enc_right_count,
                 self._sv_enc_right_delta) = (
                    bar, sv_omega, sv_rpm, sv_filt, sv_vis, sv_count, sv_delta)

        # Status row
        status_row = ttk.Frame(parent)
        status_row.grid(row=1, column=0, columnspan=2, sticky=EW, pady=(8, 0))
        ttk.Label(status_row, text="enc_status:",
                  foreground=C_MUTED, font=("", 8)).pack(side=LEFT)
        self._sv_enc_status = tk.StringVar(value="—")
        self._lbl_enc_status = ttk.Label(
            status_row, textvariable=self._sv_enc_status,
            font=("", 9, "bold"), bootstyle="secondary")
        self._lbl_enc_status.pack(side=LEFT, padx=(6, 0))

    # ── Encoder smoothing ─────────────────────────────────────────────────────

    def _build_encoder_smoothing_panel(self, parent: ttk.Labelframe) -> None:
        parent.columnconfigure(1, weight=1)

        self._var_enc_filter    = tk.DoubleVar(value=self.enc_filter_alpha)
        self._var_enc_interp    = tk.DoubleVar(value=self.enc_interp_alpha)
        self._var_enc_max_omega = tk.DoubleVar(value=self.enc_max_omega)
        self._sv_enc_filter     = tk.StringVar(value=f"{self.enc_filter_alpha:.2f}")
        self._sv_enc_interp     = tk.StringVar(value=f"{self.enc_interp_alpha:.2f}")
        self._sv_enc_max_omega  = tk.StringVar(value=f"{self.enc_max_omega:.0f}")

        def _on_f(*_: Any) -> None:
            v = round(self._var_enc_filter.get(), 2)
            self.enc_filter_alpha = max(SMOOTHING_ALPHA_MIN, min(SMOOTHING_ALPHA_MAX, v))
            self._sv_enc_filter.set(f"{self.enc_filter_alpha:.2f}")

        def _on_i(*_: Any) -> None:
            v = round(self._var_enc_interp.get(), 2)
            self.enc_interp_alpha = max(SMOOTHING_ALPHA_MIN, min(SMOOTHING_ALPHA_MAX, v))
            self._sv_enc_interp.set(f"{self.enc_interp_alpha:.2f}")

        def _on_w(*_: Any) -> None:
            v = self._var_enc_max_omega.get()
            self.enc_max_omega = max(1.0, min(500.0, round(v, 1)))
            self._sv_enc_max_omega.set(f"{self.enc_max_omega:.0f}")

        self._var_enc_filter.trace_add("write",    _on_f)
        self._var_enc_interp.trace_add("write",    _on_i)
        self._var_enc_max_omega.trace_add("write", _on_w)

        rows = [
            ("Filter alpha",  self._var_enc_filter,
             SMOOTHING_ALPHA_MIN, SMOOTHING_ALPHA_MAX, 0.01,
             self._sv_enc_filter, "1.00 = raw   0.01 = very slow"),
            ("Interp alpha",  self._var_enc_interp,
             SMOOTHING_ALPHA_MIN, SMOOTHING_ALPHA_MAX, 0.01,
             self._sv_enc_interp, "1.00 = instant   0.01 = slow lag"),
            ("Max ω (rad/s)", self._var_enc_max_omega,
             1.0, 200.0, 1.0,
             self._sv_enc_max_omega, "rad/s at bar full-scale  (50 ≈ 478 RPM)"),
        ]
        self._build_slider_rows(parent, rows, 0)
        ttk.Button(parent, text="Reset Defaults", bootstyle="secondary-outline",
                   command=self._on_reset_enc_smoothing,
                   padding=(8, 3)).grid(
            row=3, column=0, columnspan=2, sticky=W, pady=(8, 0))

    # ── LIDAR panel ───────────────────────────────────────────────────────────

    def _build_lidar_panel(self, parent: ttk.Labelframe) -> None:
        parent.columnconfigure(1, weight=1)

        # Polar plot (left)
        self._lidar_canvas = tk.Canvas(
            parent, width=LIDAR_CANVAS, height=LIDAR_CANVAS,
            background=C_CANVAS_BG, highlightthickness=1,
            highlightbackground=C_CIRCLE)
        self._lidar_canvas.grid(row=0, column=0, sticky="n", padx=(0, 12))
        self._init_lidar_canvas()

        # Metrics + controls (right)
        side = ttk.Frame(parent)
        side.grid(row=0, column=1, sticky="nsew")
        side.columnconfigure(0, weight=1)

        self._build_lidar_metrics(side)
        self._build_lidar_controls(side)

    def _init_lidar_canvas(self) -> None:
        c      = self._lidar_canvas
        cx     = cy = LIDAR_CANVAS / 2
        draw_r = LIDAR_CANVAS / 2 - LIDAR_MARGIN

        # Range rings + their metre labels (updated when max range changes)
        self._lidar_ring_labels = []
        for frac in LIDAR_RINGS:
            rr = draw_r * frac
            c.create_oval(cx - rr, cy - rr, cx + rr, cy + rr,
                          outline=C_CANVAS_GRID, width=1)
            lbl = c.create_text(cx + 3, cy - rr, anchor="sw",
                                fill=C_MUTED, font=("", 7), text="")
            self._lidar_ring_labels.append((frac, lbl))

        # Cross axes
        c.create_line(cx - draw_r, cy, cx + draw_r, cy, fill=C_CANVAS_GRID)
        c.create_line(cx, cy - draw_r, cx, cy + draw_r, fill=C_CANVAS_GRID)

        # Forward (chassis front) arrow — points up
        c.create_line(cx, cy, cx, cy - draw_r,
                      fill=C_LIDAR_FRONT, width=1, arrow="last")
        c.create_text(cx + 10, cy - draw_r + 8, anchor="w",
                      fill=C_LIDAR_FRONT, font=("", 7), text="frente")

        # Reusable point pool (hidden until used)
        self._lidar_point_items = [
            c.create_oval(-4, -4, -3, -3, fill=C_LIDAR_POINT,
                          outline="", state="hidden")
            for _ in range(LIDAR_POOL_SIZE)
        ]
        self._lidar_nearest_item = c.create_oval(
            -4, -4, -3, -3, fill=C_LIDAR_NEAREST, outline="white",
            width=1, state="hidden")

        # Chair marker (centre) on top
        c.create_oval(cx - 4, cy - 4, cx + 4, cy + 4,
                      fill=C_LIDAR_FRONT, outline="")

        self._update_lidar_ring_labels()

    def _update_lidar_ring_labels(self) -> None:
        for frac, lbl in self._lidar_ring_labels:
            self._lidar_canvas.itemconfigure(
                lbl, text=f"{frac * self._lidar_max_range:.1f} m")

    def _build_lidar_metrics(self, parent: ttk.Frame) -> None:
        mf = ttk.Labelframe(parent, text="Leitura", padding=8,
                            bootstyle="secondary")
        mf.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        mf.columnconfigure(1, weight=1)
        mf.columnconfigure(3, weight=1)

        # Nearest obstacle
        ttk.Label(mf, text="Mais próximo", foreground=C_MUTED,
                  font=("", 8)).grid(row=0, column=0, columnspan=4, sticky=W)
        self._sv_lidar_nearest = tk.StringVar(value="—")
        ttk.Label(mf, textvariable=self._sv_lidar_nearest,
                  font=("Courier", 13, "bold"), bootstyle="info").grid(
            row=1, column=0, columnspan=4, sticky=W, pady=(0, 6))

        # Sector minimums (front / back / left / right)
        self._sv_lidar_sector: Dict[str, tk.StringVar] = {}
        self._lbl_lidar_sector: Dict[str, ttk.Label] = {}
        sectors = [("F", "Frente", 2, 0), ("B", "Trás", 2, 2),
                   ("L", "Esq.", 3, 0), ("R", "Dir.", 3, 2)]
        for key, label, r, col in sectors:
            ttk.Label(mf, text=label, foreground=C_MUTED, font=("", 8)).grid(
                row=r, column=col, sticky=E, padx=(0, 4), pady=1)
            sv = tk.StringVar(value="—")
            lbl = ttk.Label(mf, textvariable=sv, width=8, anchor=W,
                            font=("Courier", 10, "bold"), bootstyle="secondary")
            lbl.grid(row=r, column=col + 1, sticky=W, pady=1)
            self._sv_lidar_sector[key]  = sv
            self._lbl_lidar_sector[key] = lbl

        # Scan health
        health = ttk.Frame(mf)
        health.grid(row=4, column=0, columnspan=4, sticky=W, pady=(6, 0))
        self._sv_lidar_hz  = tk.StringVar(value="— Hz")
        self._sv_lidar_pts = tk.StringVar(value="0 pts")
        self._sv_lidar_q   = tk.StringVar(value="q —")
        for sv in (self._sv_lidar_hz, self._sv_lidar_pts, self._sv_lidar_q):
            ttk.Label(health, textvariable=sv, foreground=C_MUTED,
                      font=("Courier", 8)).pack(side=LEFT, padx=(0, 12))

    def _build_lidar_controls(self, parent: ttk.Frame) -> None:
        cf = ttk.Labelframe(parent, text="Calibração", padding=8,
                            bootstyle="secondary")
        cf.grid(row=1, column=0, sticky="ew")
        cf.columnconfigure(1, weight=1)

        self._var_lidar_offset  = tk.DoubleVar(value=self._lidar_angle_offset)
        self._var_lidar_quality = tk.DoubleVar(value=self._lidar_quality_min)
        self._var_lidar_min     = tk.DoubleVar(value=self._lidar_min_range)
        self._var_lidar_max     = tk.DoubleVar(value=self._lidar_max_range)
        self._sv_lidar_offset   = tk.StringVar(value=f"{self._lidar_angle_offset:.0f}")
        self._sv_lidar_quality  = tk.StringVar(value=f"{self._lidar_quality_min:.0f}")
        self._sv_lidar_min      = tk.StringVar(value=f"{self._lidar_min_range:.2f}")
        self._sv_lidar_max      = tk.StringVar(value=f"{self._lidar_max_range:.1f}")

        def _on_offset(*_: Any) -> None:
            self._lidar_angle_offset = round(self._var_lidar_offset.get(), 0) % 360.0
            self._sv_lidar_offset.set(f"{self._lidar_angle_offset:.0f}")

        def _on_quality(*_: Any) -> None:
            self._lidar_quality_min = int(round(self._var_lidar_quality.get()))
            self._sv_lidar_quality.set(f"{self._lidar_quality_min:d}")

        def _on_min(*_: Any) -> None:
            v = round(self._var_lidar_min.get(), 2)
            self._lidar_min_range = max(LIDAR_RANGE_MIN_LIMIT, v)
            self._sv_lidar_min.set(f"{self._lidar_min_range:.2f}")

        def _on_max(*_: Any) -> None:
            v = round(self._var_lidar_max.get(), 1)
            self._lidar_max_range = min(LIDAR_RANGE_MAX_LIMIT, max(1.0, v))
            self._sv_lidar_max.set(f"{self._lidar_max_range:.1f}")
            self._update_lidar_ring_labels()

        self._var_lidar_offset.trace_add("write",  _on_offset)
        self._var_lidar_quality.trace_add("write", _on_quality)
        self._var_lidar_min.trace_add("write",     _on_min)
        self._var_lidar_max.trace_add("write",     _on_max)

        rows = [
            ("Offset 0° (°)", self._var_lidar_offset, 0.0, 359.0, 1.0,
             self._sv_lidar_offset, "alinha frente do chassi"),
            ("Qualidade ≥",   self._var_lidar_quality, 0.0, float(LIDAR_QUALITY_LIMIT), 1.0,
             self._sv_lidar_quality, "descarta retornos fracos"),
            ("Range mín (m)", self._var_lidar_min, LIDAR_RANGE_MIN_LIMIT, 2.0, 0.05,
             self._sv_lidar_min, "ignora ecos muito perto"),
            ("Range máx (m)", self._var_lidar_max, 1.0, LIDAR_RANGE_MAX_LIMIT, 0.5,
             self._sv_lidar_max, "escala do plot + corte longe"),
        ]
        self._build_slider_rows(cf, rows, 0)

        # Flip direction
        self._var_lidar_flip = tk.BooleanVar(value=self._lidar_flip)

        def _on_flip() -> None:
            self._lidar_flip = self._var_lidar_flip.get()

        ttk.Checkbutton(
            cf, text="Inverter sentido (CW/CCW)", variable=self._var_lidar_flip,
            command=_on_flip, bootstyle="secondary-round-toggle").grid(
            row=4, column=0, columnspan=4, sticky=W, pady=(6, 4))

        ttk.Separator(cf, orient=HORIZONTAL, bootstyle="secondary").grid(
            row=5, column=0, columnspan=4, sticky=EW, pady=6)

        # Driver launch (sllidar_ros2) + /scan subscription
        conn = ttk.Frame(cf)
        conn.grid(row=6, column=0, columnspan=4, sticky=EW)
        ttk.Label(conn, text="Porta", foreground=C_MUTED, font=("", 8)).pack(
            side=LEFT, padx=(0, 4))
        self._var_lidar_port = tk.StringVar(value=self._lidar_init_port)
        ttk.Entry(conn, textvariable=self._var_lidar_port, width=13,
                  font=("Courier", 9)).pack(side=LEFT, padx=(0, 8))
        ttk.Label(conn, text="Tópico", foreground=C_MUTED, font=("", 8)).pack(
            side=LEFT, padx=(0, 4))
        self._var_lidar_topic = tk.StringVar(value=self._lidar_init_topic)
        ttk.Entry(conn, textvariable=self._var_lidar_topic, width=8,
                  font=("Courier", 9)).pack(side=LEFT, padx=(0, 8))
        self._btn_lidar_start = ttk.Button(
            conn, text="Iniciar", bootstyle="success",
            command=self._on_lidar_start, padding=(8, 3))
        self._btn_lidar_start.pack(side=LEFT, padx=(0, 4))
        self._btn_lidar_stop = ttk.Button(
            conn, text="Parar", bootstyle="secondary",
            command=self._on_lidar_stop, state=DISABLED, padding=(8, 3))
        self._btn_lidar_stop.pack(side=LEFT)

        status_row = ttk.Frame(cf)
        status_row.grid(row=7, column=0, columnspan=4, sticky=W, pady=(6, 0))
        ttk.Label(status_row, text="status:", foreground=C_MUTED,
                  font=("", 8)).pack(side=LEFT, padx=(0, 6))
        self._sv_lidar_status = tk.StringVar(value="parado")
        self._lbl_lidar_status = ttk.Label(
            status_row, textvariable=self._sv_lidar_status,
            font=("", 9, "bold"), bootstyle="secondary")
        self._lbl_lidar_status.pack(side=LEFT)

    # ── Shared slider row builder ─────────────────────────────────────────────

    def _build_slider_rows(
        self, parent: ttk.Labelframe, rows: list, start_row: int
    ) -> None:
        for i, (label, var, from_, to_, res, sv, hint) in enumerate(rows):
            row = start_row + i
            ttk.Label(parent, text=label, width=16, anchor=E,
                      foreground=C_MUTED, font=("", 8)).grid(
                row=row, column=0, sticky=E, padx=(0, 6), pady=3)
            tk.Scale(parent, variable=var, from_=from_, to=to_,
                     resolution=res, orient=HORIZONTAL, length=220,
                     showvalue=False,
                     bg=self.root.style.colors.bg, fg=C_MUTED,
                     troughcolor=C_CANVAS_BG, highlightthickness=0, bd=0,
                     activebackground=C_DOT_FILL,
                     ).grid(row=row, column=1, sticky=EW, padx=(0, 6), pady=3)
            ttk.Label(parent, textvariable=sv, width=6, anchor=W,
                      font=("Courier", 9, "bold"),
                      bootstyle="info").grid(
                row=row, column=2, sticky=W, padx=(0, 14), pady=3)
            ttk.Label(parent, text=hint, foreground=C_MUTED,
                      font=("", 7)).grid(row=row, column=3, sticky=W, pady=3)

    # ── Safety panel ──────────────────────────────────────────────────────────

    def _build_safety_panel(self, parent: ttk.Frame) -> None:
        safety = ttk.Labelframe(
            parent, text="Safety", padding=10, bootstyle="warning")
        safety.grid(row=7, column=0, columnspan=3, sticky=EW, pady=(4, 0))

        ttk.Label(
            safety,
            text="⚠  Do not use with wheels on the ground.  "
                 "v0.7.x is for suspended / no-load PWM testing only.",
            bootstyle="warning", font=("", 9, "bold"),
            wraplength=800, justify=LEFT,
        ).pack(fill=X, pady=(0, 6))

        self._safety_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            safety,
            text="I understand — motors must be disconnected or suspended",
            variable=self._safety_var,
            command=self._on_safety_toggled,
            bootstyle="warning-round-toggle",
        ).pack(anchor=W)

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
            elif kind == "lidar_scan":
                self._lidar_scan  = payload
                self._lidar_dirty = True
                self.lidar_scans += 1
                self._lidar_nodata_warned = False
            elif kind == "lidar_status":
                self._sv_lidar_status.set(str(payload))
                self._lbl_lidar_status.configure(bootstyle="success")
            elif kind == "lidar_error":
                self._sv_lidar_status.set(f"erro: {payload}")
                self._lbl_lidar_status.configure(bootstyle="danger")
            elif kind == "lidar_stopped":
                self._lidar_running = False
                self._lidar_scan    = None
                self._lidar_dirty   = True
                self._update_lidar_buttons()
                if not self._sv_lidar_status.get().startswith("erro:"):
                    self._sv_lidar_status.set("parado")
                    self._lbl_lidar_status.configure(bootstyle="secondary")

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
                f"cmd_seq={cmd_seq}  {pkt.get('status','?')}")
            self._sv_last_rx.set(f"ACK  cmd_seq={cmd_seq}")
            return

        if pkt_type == "err":
            code = pkt.get("code", "?")
            self._sv_last_err.set(code)
            self._sv_last_rx.set(f"ERR  {code}")
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

        enc_status = pkt.get("enc_status")
        if enc_status is not None:
            self.enc_status = str(enc_status)
            self.enc_ok = enc_status == "ok"

        if self.enc_ok:
            lc = pkt.get("enc_left_count")
            rc = pkt.get("enc_right_count")
            ld = pkt.get("enc_left_delta")
            rd = pkt.get("enc_right_delta")
            if lc is not None:
                self.enc_left_count  = int(lc)
            if rc is not None:
                self.enc_right_count = int(rc)
            if ld is not None:
                self.enc_left_delta      = int(ld)
                self.latest_omega_left   = int(ld) * OMEGA_PER_COUNT
            if rd is not None:
                self.enc_right_delta     = int(rd)
                self.latest_omega_right  = int(rd) * OMEGA_PER_COUNT

    # ── GUI frame ─────────────────────────────────────────────────────────────

    def _gui_frame(self) -> None:
        self._process_queue()

        # Joystick pipeline
        self.filtered_x = exp_step(self.filtered_x, self.latest_x, self.filter_alpha)
        self.filtered_y = exp_step(self.filtered_y, self.latest_y, self.filter_alpha)
        self.visual_x   = exp_step(self.visual_x,   self.filtered_x, self.interp_alpha)
        self.visual_y   = exp_step(self.visual_y,   self.filtered_y, self.interp_alpha)
        self._move_dot(self.visual_x, self.visual_y)
        self._sv_filt_x.set(f"{self.filtered_x:+.3f}")
        self._sv_filt_y.set(f"{self.filtered_y:+.3f}")
        self._sv_vis_x.set(f"{self.visual_x:+.3f}")
        self._sv_vis_y.set(f"{self.visual_y:+.3f}")

        # Motor monitor
        el = self.motor_left  if self.motor_active else 0.0
        er = self.motor_right if self.motor_active else 0.0
        self._update_bar(self._left_bar,  el)
        self._update_bar(self._right_bar, er)
        self._sv_left_lbl.set(motor_label(el,  self.motor_active))
        self._sv_right_lbl.set(motor_label(er, self.motor_active))
        self._sv_left_gpio.set(
            f"Active: {active_gpio('left',  el, self.motor_active)}")
        self._sv_right_gpio.set(
            f"Active: {active_gpio('right', er, self.motor_active)}")
        if self.motor_active:
            self._sv_active.set("true")
            self._lbl_active.configure(bootstyle="success")
        else:
            self._sv_active.set("false")
            self._lbl_active.configure(bootstyle="secondary")

        # Encoder pipeline
        self.filtered_omega_left  = exp_step(
            self.filtered_omega_left,  self.latest_omega_left,  self.enc_filter_alpha)
        self.filtered_omega_right = exp_step(
            self.filtered_omega_right, self.latest_omega_right, self.enc_filter_alpha)
        self.visual_omega_left    = exp_step(
            self.visual_omega_left,    self.filtered_omega_left,  self.enc_interp_alpha)
        self.visual_omega_right   = exp_step(
            self.visual_omega_right,   self.filtered_omega_right, self.enc_interp_alpha)
        self._update_encoder_display()

        # LIDAR — only redraw when a fresh scan arrived (≈10 Hz, not every frame)
        if self._lidar_dirty:
            self._render_lidar()
            self._lidar_dirty = False

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
            cx + DOT_RADIUS, cy + DOT_RADIUS,
        )

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

    def _update_encoder_display(self) -> None:
        max_omega = self.enc_max_omega if self.enc_max_omega > 0 else 1.0

        sides = [
            (self.latest_omega_left,  self.filtered_omega_left,
             self.visual_omega_left,  self.enc_left_count,  self.enc_left_delta,
             self._enc_left_bar,  self._sv_enc_left_omega,  self._sv_enc_left_rpm,
             self._sv_enc_left_filt,  self._sv_enc_left_vis,
             self._sv_enc_left_count, self._sv_enc_left_delta),
            (self.latest_omega_right, self.filtered_omega_right,
             self.visual_omega_right, self.enc_right_count, self.enc_right_delta,
             self._enc_right_bar, self._sv_enc_right_omega, self._sv_enc_right_rpm,
             self._sv_enc_right_filt, self._sv_enc_right_vis,
             self._sv_enc_right_count, self._sv_enc_right_delta),
        ]

        for (o_raw, o_filt, o_vis, count, delta,
             bar, sv_omega, sv_rpm, sv_filt, sv_vis, sv_count, sv_delta) in sides:

            self._update_bar(bar, o_vis / max_omega)

            if self.enc_ok:
                sv_omega.set(f"{o_raw:+.3f} rad/s")
                sv_rpm.set(f"{o_vis * 60.0 / TWO_PI:+.1f} RPM")
                sv_filt.set(f"{o_filt:+.3f} rad/s")
                sv_vis.set(f"{o_vis:+.3f} rad/s")
                sv_count.set(str(count))
                sv_delta.set(f"{delta:+d}")
            else:
                for sv in (sv_omega, sv_rpm, sv_filt, sv_vis, sv_count, sv_delta):
                    sv.set("—")

        if self.enc_ok:
            self._sv_enc_status.set("ok")
            self._lbl_enc_status.configure(bootstyle="success")
        elif self.enc_status == "error":
            self._sv_enc_status.set("error")
            self._lbl_enc_status.configure(bootstyle="danger")
        else:
            self._sv_enc_status.set(self.enc_status)
            self._lbl_enc_status.configure(bootstyle="secondary")

    # ── LIDAR render ──────────────────────────────────────────────────────────

    def _render_lidar(self) -> None:
        c     = self._lidar_canvas
        items = self._lidar_point_items
        scan  = self._lidar_scan

        if not scan:
            # Hide everything (stopped / no data)
            for i in range(self._lidar_shown):
                c.itemconfigure(items[i], state="hidden")
            self._lidar_shown = 0
            c.itemconfigure(self._lidar_nearest_item, state="hidden")
            self._sv_lidar_nearest.set("—")
            self._sv_lidar_pts.set("0 pts")
            self._sv_lidar_q.set("q —")
            self._sv_lidar_hz.set(f"{self._lidar_hz:.1f} Hz"
                                  if self._lidar_running else "— Hz")
            for key in self._sv_lidar_sector:
                self._apply_lidar_sector(key, math.inf)
            return

        cx     = cy = LIDAR_CANVAS / 2
        draw_r = LIDAR_CANVAS / 2 - LIDAR_MARGIN
        offset = self._lidar_angle_offset
        sign   = -1.0 if self._lidar_flip else 1.0
        q_min  = self._lidar_quality_min
        r_min  = self._lidar_min_range
        r_max  = self._lidar_max_range
        pr     = LIDAR_POINT_R

        sector = {"F": math.inf, "B": math.inf, "L": math.inf, "R": math.inf}
        nearest = math.inf
        nearest_xy: Optional[Tuple[float, float]] = None
        q_sum  = 0
        n      = 0

        for quality, angle_deg, dist_mm in scan:
            if quality < q_min:
                continue
            d = dist_mm / 1000.0
            if d <= 0.0 or d < r_min or d > r_max:
                continue

            a  = (angle_deg * sign + offset) % 360.0
            th = math.radians(a)
            rr = min(d, r_max) / r_max * draw_r
            x  = cx + rr * math.sin(th)
            y  = cy - rr * math.cos(th)

            if n < len(items):
                c.coords(items[n], x - pr, y - pr, x + pr, y + pr)

            # Sector classification (0°=front, clockwise)
            if a < 45.0 or a >= 315.0:
                key = "F"
            elif a < 135.0:
                key = "R"
            elif a < 225.0:
                key = "B"
            else:
                key = "L"
            if d < sector[key]:
                sector[key] = d
            if d < nearest:
                nearest    = d
                nearest_xy = (x, y)

            q_sum += quality
            n     += 1

        # Toggle item visibility only across the changed range
        shown = min(n, len(items))
        if shown > self._lidar_shown:
            for i in range(self._lidar_shown, shown):
                c.itemconfigure(items[i], state="normal")
        elif shown < self._lidar_shown:
            for i in range(shown, self._lidar_shown):
                c.itemconfigure(items[i], state="hidden")
        self._lidar_shown = shown

        # Nearest marker
        if nearest_xy is not None:
            x, y = nearest_xy
            m = pr + 3
            c.coords(self._lidar_nearest_item, x - m, y - m, x + m, y + m)
            c.itemconfigure(self._lidar_nearest_item, state="normal")
            a_near = (math.degrees(math.atan2(x - cx, cy - y))) % 360.0
            self._sv_lidar_nearest.set(f"{nearest:.2f} m @ {a_near:3.0f}°")
        else:
            c.itemconfigure(self._lidar_nearest_item, state="hidden")
            self._sv_lidar_nearest.set("—")

        for key, d in sector.items():
            self._apply_lidar_sector(key, d)

        self._sv_lidar_hz.set(f"{self._lidar_hz:.1f} Hz")
        self._sv_lidar_pts.set(f"{n} pts")
        self._sv_lidar_q.set(f"q {q_sum / n:.0f}" if n else "q —")

    def _apply_lidar_sector(self, key: str, d: float) -> None:
        sv  = self._sv_lidar_sector[key]
        lbl = self._lbl_lidar_sector[key]
        if not math.isfinite(d):
            sv.set("—")
            lbl.configure(bootstyle="secondary")
        elif d <= LIDAR_DANGER_M:
            sv.set(f"{d:.2f}")
            lbl.configure(bootstyle="danger")
        elif d <= LIDAR_WARN_M:
            sv.set(f"{d:.2f}")
            lbl.configure(bootstyle="warning")
        else:
            sv.set(f"{d:.2f}")
            lbl.configure(bootstyle="success")

    def _update_lidar_buttons(self) -> None:
        self._btn_lidar_start.configure(
            state=DISABLED if self._lidar_running else NORMAL)
        self._btn_lidar_stop.configure(
            state=NORMAL if self._lidar_running else DISABLED)

    # ── Age ticker ────────────────────────────────────────────────────────────

    def _tick_age(self) -> None:
        if self.last_packet_time is None:
            self._sv_age.set("—")
        else:
            ms = (time.monotonic() - self.last_packet_time) * 1000
            self._sv_age.set(f"{ms:.0f} ms")

        # LIDAR scan-rate estimate over a ~1 s window
        now = time.monotonic()
        if self._lidar_hz_t0 is None:
            self._lidar_hz_t0 = now
            self._lidar_hz_n0 = self.lidar_scans
        elif now - self._lidar_hz_t0 >= 1.0:
            self._lidar_hz = (self.lidar_scans - self._lidar_hz_n0) / (now - self._lidar_hz_t0)
            self._lidar_hz_t0 = now
            self._lidar_hz_n0 = self.lidar_scans

        # Watchdog: driver up but no LaserScan messages after a grace period.
        if (self._lidar_running and not self._lidar_nodata_warned
                and self.lidar_scans == 0 and self._lidar_start_t is not None
                and now - self._lidar_start_t > 8.0
                and not self._sv_lidar_status.get().startswith("erro")):
            self._sv_lidar_status.set("sem mensagens no tópico — driver subindo? confira a porta")
            self._lbl_lidar_status.configure(bootstyle="warning")
            self._lidar_nodata_warned = True

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

    def _send_stop(self) -> bool:
        return self._send_command({"type": "stop"})

    # ── Stream ────────────────────────────────────────────────────────────────

    def _stream_tick(self) -> None:
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
        self._stream_after_id = self.root.after(PWM_STREAM_PERIOD_MS, self._stream_tick)

    def _update_stream_ui(self) -> None:
        safety    = self._safety_var.get()
        streaming = self._streaming
        idle = NORMAL if (safety and not streaming) else DISABLED
        self._btn_send_once.configure(state=idle)
        self._btn_start_stream.configure(state=idle)
        self._btn_stop_stream.configure(
            state=NORMAL if streaming else DISABLED)
        if streaming:
            self._sv_stream_status.set("Stream ON")
            self._lbl_stream_status.configure(bootstyle="success")
        else:
            self._sv_stream_status.set("Stream OFF")
            self._lbl_stream_status.configure(bootstyle="secondary")

    # ── Handlers ──────────────────────────────────────────────────────────────

    def _toggle_fullscreen(self) -> None:
        self.root.attributes("-fullscreen",
                             not bool(self.root.attributes("-fullscreen")))

    def _exit_fullscreen(self) -> None:
        self.root.attributes("-fullscreen", False)

    def _on_apply_pwm_limit(self) -> None:
        new_limit = max(PWM_LIMIT_MIN,
                        min(PWM_LIMIT_MAX, round(self._var_pwm_limit.get(), 2)))
        self._pwm_limit = new_limit
        assert self._slider_left  is not None
        assert self._slider_right is not None
        self._slider_left.configure(from_=-new_limit, to=new_limit)
        self._slider_right.configure(from_=-new_limit, to=new_limit)
        for var in (self._var_left, self._var_right):
            var.set(max(-new_limit, min(new_limit, var.get())))
        if new_limit > PWM_LIMIT_WARN_THRESHOLD:
            self._sv_pwm_limit_warn.set(
                f"⚠  Limit above {PWM_LIMIT_WARN_THRESHOLD:.2f} — "
                "suspended motor only.")
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

    def _on_reset_enc_smoothing(self) -> None:
        self._var_enc_filter.set(ENC_FILTER_ALPHA)
        self._var_enc_interp.set(ENC_INTERP_ALPHA)
        self._var_enc_max_omega.set(ENC_MAX_OMEGA)

    # ── LIDAR handlers ────────────────────────────────────────────────────────

    def _on_lidar_start(self) -> None:
        if self._lidar_running:
            return
        port  = self._var_lidar_port.get().strip()  or LIDAR_DEFAULT_PORT
        topic = self._var_lidar_topic.get().strip() or LIDAR_DEFAULT_TOPIC
        self._lidar_stop = threading.Event()
        self._lidar_thread = threading.Thread(
            target=lidar_ros_reader,
            args=(topic, port, self.event_queue, self._lidar_stop),
            name="lidar-ros-reader",
            daemon=True,
        )
        self._lidar_running = True
        self._lidar_start_t = time.monotonic()
        self._lidar_nodata_warned = False
        self._sv_lidar_status.set("iniciando driver…")
        self._lbl_lidar_status.configure(bootstyle="info")
        self._update_lidar_buttons()
        self._lidar_thread.start()

    def _on_lidar_stop(self) -> None:
        if self._lidar_stop is not None:
            self._lidar_stop.set()
        self._sv_lidar_status.set("parando…")
        self._lbl_lidar_status.configure(bootstyle="secondary")
        # Buttons re-enable when the thread emits "lidar_stopped".

    # ── Close ─────────────────────────────────────────────────────────────────

    def _on_close(self) -> None:
        if self.closing:
            return
        self.closing = True
        self._on_stop_stream()
        self._send_stop()
        self.stop_event.set()
        if self._lidar_stop is not None:
            self._lidar_stop.set()
        self._sv_conn.set("closing…")
        self._wait_for_reader()

    def _wait_for_reader(self) -> None:
        lidar_alive = (self._lidar_thread is not None
                       and self._lidar_thread.is_alive())
        if self.reader_thread.is_alive() or lidar_alive:
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

    if args.port:
        reader = threading.Thread(
            target=serial_reader,
            args=(serial, args.port, args.baud, event_queue, stop_event),
            name="serial-reader",
            daemon=True,
        )
    else:
        # LIDAR-only mode: no ESP32 — an idle thread keeps the shutdown path simple.
        reader = threading.Thread(target=lambda: None,
                                  name="serial-reader-idle", daemon=True)

    WheelchairControlGUI(
        root, event_queue, stop_event, reader,
        filter_alpha=args.filter_alpha,
        interp_alpha=args.interp_alpha,
        gui_update_ms=args.gui_update_ms,
        esp_port=args.port,
        lidar_port=args.lidar,
        lidar_topic=args.lidar_topic,
    )
    if args.port:
        reader.start()
    root.mainloop()
    stop_event.set()
    if args.port:
        reader.join(timeout=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
