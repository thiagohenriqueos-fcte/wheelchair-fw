#!/usr/bin/env python3
"""Wheelchair control GUI — differential drive + LIDAR + IMU — v0.9.0

Three independent subsystems, each on its own serial port / thread; mix and
match them freely. Source the ROS2 env first if you use the LIDAR, e.g.
`source /opt/ros/jazzy/setup.bash` plus the sllidar workspace.

How each mode is launched
-------------------------
  ESP32 / drive : positional port arg — connects immediately at launch.
      python3 scripts/wheelchair_control_gui.py /dev/ttyUSB1
  LIDAR         : --lidar PORT — pre-fills the LIDAR panel; press "Iniciar".
      python3 scripts/wheelchair_control_gui.py --lidar /dev/ttyUSB0
  IMU           : --imu PORT — pre-fills the IMU panel; press "Iniciar".
      python3 scripts/wheelchair_control_gui.py --imu /dev/ttyACM0

Only the ESP32 positional port auto-connects on start; --lidar and --imu just
PRE-FILL their panel fields (you still click "Iniciar" in each panel). Every
port can also be typed in the panels at runtime, so the flags are optional.

Possible combinations (include only what you have plugged in)
-------------------------------------------------------------
  ESP only          : ... /dev/ttyUSB1
  ESP + LIDAR       : ... /dev/ttyUSB1 --lidar /dev/ttyUSB0
  ESP + IMU         : ... /dev/ttyUSB1 --imu /dev/ttyACM0
  ESP + LIDAR + IMU : ... /dev/ttyUSB1 --lidar /dev/ttyUSB0 --imu /dev/ttyACM0
  LIDAR only        : ... --lidar /dev/ttyUSB0        (omit the ESP port)
  IMU only          : ... --imu /dev/ttyACM0          (omit the ESP port)
  LIDAR + IMU       : ... --lidar /dev/ttyUSB0 --imu /dev/ttyACM0

Typical ports on this rig: ESP32 = /dev/ttyUSB1 (CP2102), RPLIDAR C1 =
/dev/ttyUSB0 (CP2102N), IMU = /dev/ttyACM0 (Nations N32L40x USB-CDC @ 9600).

Subsystems
----------
  • ESP32 / drive — the physical joystick is read by the ESP32, which runs the
    differential-drive loop (mixing + accel/decel ramp + max-duty clamp). The
    GUI tunes those parameters and holds the operator safety gate: while ARMED
    it streams a `drive_cfg` keep-alive at 10 Hz, so if the GUI or USB link
    drops the firmware disarms within ~400 ms.
  • LIDAR — the RPLIDAR C1 is handled by the sllidar_ros2 driver: the GUI starts
    it (or reuses a running one) and subscribes to its sensor_msgs/LaserScan on
    /scan, on its own thread.
  • IMU — the Witmotion 603T is read directly (Witmotion 0x55 frames) and
    configured via register commands (unlock + register + save) for
    calibration and output rate.

WARNING: arming makes the wheels move. Use with the chair suspended / no load
until the behaviour is validated.
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
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

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


# ── LIDAR (RPLIDAR C1 via ROS2) ───────────────────────────────────────────────
#
# The C1 is driven by the official sllidar_ros2 node (it owns the motor + C1
# protocol). The GUI starts that driver and subscribes to its
# sensor_msgs/LaserScan on /scan, on its own thread. Requires a sourced ROS2
# environment (e.g. `source /opt/ros/jazzy/setup.bash` + the sllidar workspace).

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


# ── IMU (Witmotion 6xx — WIT standard protocol) ───────────────────────────────
#
# The IMU is read on its own USB-TTL port / thread (Witmotion 0x55 frames) and
# configured by writing register commands (0xFF 0xAA addr lo hi) wrapped by an
# unlock + save sequence. Connecting it straight to the Pi keeps config simple.

IMU_DEFAULT_PORT = "/dev/ttyACM0"  # Witmotion 603T = Nations N32L40x USB-CDC
IMU_DEFAULT_BAUD = 9600            # confirmed on the 603T (frames align at 9600)

IMU_FRAME_LEN = 11
IMU_HEADER    = 0x55
IMU_ACC   = 0x51
IMU_GYRO  = 0x52
IMU_ANGLE = 0x53
IMU_MAG   = 0x54

# Register write protocol
IMU_UNLOCK = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])
IMU_SAVE   = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])
IMU_REG_SAVE  = 0x00
IMU_REG_CALSW = 0x01
IMU_REG_RRATE = 0x03
# CALSW values
IMU_CAL_NORMAL = 0x00
IMU_CAL_ACCEL  = 0x01
IMU_CAL_HEIGHT = 0x03
IMU_CAL_YAW    = 0x04
IMU_CAL_MAG    = 0x07

# Return-rate menu: label → RRATE register value
IMU_RATES = [("10 Hz", 0x06), ("20 Hz", 0x07), ("50 Hz", 0x08),
             ("100 Hz", 0x09), ("200 Hz", 0x0B)]

IMU_CANVAS = 180          # attitude indicator canvas (px)

C_IMU_SKY    = "#1e3a5f"
C_IMU_GROUND = "#5f4a2a"
C_IMU_HORIZON = "#cdd6f4"
C_IMU_NEEDLE = "#f38ba8"


# ── Velocity estimation (from IMU) ────────────────────────────────────────────
#
# Linear velocity is integrated from the forward (y) accelerometer; angular
# velocity is the z gyro (already a rate). Accel integration drifts, so a tare
# (bias capture), ZUPT (zero-velocity update when still) and a gentle leak keep
# it bounded — treat the value as a reference, not a precise measurement.

VEL_GRAVITY      = 9.80665      # m/s² per g
VEL_HIST_SECONDS = 10.0         # strip-chart window
VEL_SAMPLE_HZ    = 20           # ≈ imu_data emit rate
VEL_HIST_LEN     = int(VEL_HIST_SECONDS * VEL_SAMPLE_HZ)
VEL_LEAK         = 0.05         # gentle drift bound [1/s] (exp decay)
VEL_ZUPT_A       = 0.15         # m/s² — "still" accel threshold
VEL_ZUPT_W       = 2.0          # °/s — "still" yaw-rate threshold
VEL_ZUPT_HOLD    = 0.30         # s still before zeroing velocity
VEL_DT_MAX       = 0.50         # s — skip integration across longer gaps

VEL_PLOT_W = 420
VEL_PLOT_H = 110

C_VEL_LIN = "#89b4fa"           # linear-velocity trace
C_VEL_ANG = "#f9e2af"           # angular-velocity trace


# ── Argument parsing ──────────────────────────────────────────────────────────

def _pos_int(value: str) -> int:
    v = int(value)
    if v <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return v


def parse_arguments() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Wheelchair control GUI — differential drive + LIDAR + IMU.")
    p.add_argument("port", nargs="?", default=None,
                   help="ESP32 serial port, e.g. /dev/ttyUSB1 "
                        "(optional — omit to run LIDAR-only)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--gui-update-ms", type=_pos_int, default=GUI_UPDATE_MS)
    p.add_argument("--lidar", default=None,
                   help=f"RPLIDAR C1 serial port for the driver, e.g. {LIDAR_DEFAULT_PORT}")
    p.add_argument("--lidar-topic", default=LIDAR_DEFAULT_TOPIC,
                   help=f"LaserScan topic to subscribe (default {LIDAR_DEFAULT_TOPIC})")
    p.add_argument("--imu", default=None,
                   help=f"Witmotion IMU serial port to pre-fill, e.g. {IMU_DEFAULT_PORT}")
    p.add_argument("--imu-baud", type=int, default=IMU_DEFAULT_BAUD)
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


# ── IMU reader thread (Witmotion 0x55 frames) ─────────────────────────────────

def _imu_word(lo: int, hi: int) -> int:
    """Little-endian signed 16-bit from a low/high byte pair."""
    return int.from_bytes(bytes((lo, hi)), "little", signed=True)


def parse_witmotion_frame(frame: bytes, state: Dict[str, float]) -> None:
    """Decode one validated 11-byte 0x55 frame into `state` (physical units)."""
    t = frame[1]
    d = frame[2:10]
    if t == IMU_ACC:
        state["ax"]   = _imu_word(d[0], d[1]) / 32768.0 * 16.0
        state["ay"]   = _imu_word(d[2], d[3]) / 32768.0 * 16.0
        state["az"]   = _imu_word(d[4], d[5]) / 32768.0 * 16.0
        state["temp"] = _imu_word(d[6], d[7]) / 100.0
    elif t == IMU_GYRO:
        state["wx"] = _imu_word(d[0], d[1]) / 32768.0 * 2000.0
        state["wy"] = _imu_word(d[2], d[3]) / 32768.0 * 2000.0
        state["wz"] = _imu_word(d[4], d[5]) / 32768.0 * 2000.0
    elif t == IMU_ANGLE:
        state["roll"]  = _imu_word(d[0], d[1]) / 32768.0 * 180.0
        state["pitch"] = _imu_word(d[2], d[3]) / 32768.0 * 180.0
        state["yaw"]   = _imu_word(d[4], d[5]) / 32768.0 * 180.0
    elif t == IMU_MAG:
        state["mx"] = float(_imu_word(d[0], d[1]))
        state["my"] = float(_imu_word(d[2], d[3]))
        state["mz"] = float(_imu_word(d[4], d[5]))


def imu_reader(
    port: str,
    baud: int,
    event_queue: "queue.Queue[Tuple[str, Any]]",
    stop_event: threading.Event,
) -> None:
    """Read Witmotion frames; emit ("imu_data", state) ~20 Hz.

    Hands the open serial connection back via ("imu_conn_ready", conn) so the GUI
    can write config/calibration commands on it (under its own write lock) while
    this thread keeps reading.
    """
    import serial as serial_module
    try:
        conn = serial_module.Serial(port=port, baudrate=baud, timeout=0.1)
    except serial_module.SerialException as exc:
        event_queue.put(("imu_error", str(exc)))
        event_queue.put(("imu_stopped", None))
        return

    event_queue.put(("imu_conn_ready", conn))
    event_queue.put(("imu_status", f"conectado: {port} @ {baud}"))

    buf = bytearray()
    state: Dict[str, float] = {}
    last_emit = 0.0
    try:
        while not stop_event.is_set():
            try:
                data = conn.read(64)
            except serial_module.SerialException as exc:
                event_queue.put(("imu_error", str(exc)))
                break
            if not data:
                continue
            buf.extend(data)
            while len(buf) >= IMU_FRAME_LEN:
                if buf[0] != IMU_HEADER:
                    del buf[0]
                    continue
                frame = bytes(buf[:IMU_FRAME_LEN])
                if (sum(frame[:10]) & 0xFF) != frame[10]:
                    del buf[0]           # bad checksum → resync one byte
                    continue
                parse_witmotion_frame(frame, state)
                del buf[:IMU_FRAME_LEN]
            if len(buf) > 4096:
                del buf[:-IMU_FRAME_LEN]
            now = time.monotonic()
            if state and now - last_emit > 0.05:
                event_queue.put(("imu_data", dict(state)))
                last_emit = now
    finally:
        try:
            conn.close()
        except Exception:
            pass
        event_queue.put(("imu_conn_gone", None))
        event_queue.put(("imu_stopped", None))


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
        esp_port: Optional[str] = None,
        lidar_port: Optional[str] = None,
        lidar_topic: str = LIDAR_DEFAULT_TOPIC,
        imu_port: Optional[str] = None,
        imu_baud: int = IMU_DEFAULT_BAUD,
    ) -> None:
        self.root          = root
        self.event_queue   = event_queue
        self.stop_event    = stop_event
        self.reader_thread = reader_thread
        self.esp_port      = esp_port
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

        # LIDAR pipeline (ROS2 /scan subscriber + sllidar driver — own thread)
        self._lidar_thread: Optional[threading.Thread] = None
        self._lidar_stop:   Optional[threading.Event]  = None
        self._lidar_running = False
        self._lidar_scan: Optional[list] = None
        self._lidar_dirty = False
        self._lidar_shown = 0
        self.lidar_scans  = 0
        self._lidar_hz    = 0.0
        self._lidar_hz_t0: Optional[float] = None
        self._lidar_hz_n0 = 0
        self._lidar_start_t: Optional[float] = None
        self._lidar_nodata_warned = False

        # LIDAR calibration / filtering (applied host-side, live)
        self._lidar_angle_offset = LIDAR_ANGLE_OFFSET_DEF
        self._lidar_flip         = LIDAR_FLIP_DEFAULT
        self._lidar_min_range    = LIDAR_MIN_RANGE_DEFAULT
        self._lidar_max_range    = LIDAR_MAX_RANGE_DEFAULT
        self._lidar_quality_min  = LIDAR_QUALITY_DEFAULT
        self._lidar_init_port    = lidar_port or LIDAR_DEFAULT_PORT
        self._lidar_init_topic   = lidar_topic or LIDAR_DEFAULT_TOPIC

        # IMU pipeline (Witmotion over USB-TTL — own thread; GUI writes config)
        self._imu_thread: Optional[threading.Thread] = None
        self._imu_stop:   Optional[threading.Event]  = None
        self._imu_running = False
        self._imu_conn: Optional[Any] = None
        self._imu_write_lock = threading.Lock()
        self._imu: Dict[str, float] = {}      # latest decoded values
        self._imu_dirty = False
        self._imu_mag_calibrating = False
        self._imu_init_port = imu_port or IMU_DEFAULT_PORT
        self._imu_init_baud = imu_baud

        # Velocity estimation (integrated from the IMU) + pop-out window
        self._vel_win: Optional[Any] = None
        self._vel_v   = 0.0           # integrated linear velocity [m/s]
        self._vel_wz  = 0.0           # angular velocity (z gyro) [°/s]
        self._vel_ay_bias = 0.0       # forward-accel bias [g] captured by tare
        self._vel_last_t: Optional[float] = None
        self._vel_zupt = True
        self._vel_still_since: Optional[float] = None
        self._vel_hist_v: "deque[float]" = deque(maxlen=VEL_HIST_LEN)
        self._vel_hist_w: "deque[float]" = deque(maxlen=VEL_HIST_LEN)
        self._vel_dirty = False

        self._build_ui()
        if self.esp_port is None:
            self._sv_conn.set("sem ESP — modo LIDAR")
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

        lidar_frame = ttk.Labelframe(
            outer, text="LIDAR  (RPLIDAR C1)", padding=10, bootstyle="secondary")
        lidar_frame.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self._build_lidar_panel(lidar_frame)

        imu_frame = ttk.Labelframe(
            outer, text="IMU  (Witmotion)", padding=10, bootstyle="secondary")
        imu_frame.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self._build_imu_panel(imu_frame)

        self._build_safety_panel(outer)  # row 5

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
        ttk.Button(tb, text="📈  Velocidades (IMU)", bootstyle="info-outline",
                   command=self._open_velocity_window, padding=(8, 4)
                   ).pack(side=LEFT, padx=(12, 0))

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
            parent, "Esquerda   GPIO12 / GPIO13")
        ttk.Separator(parent, orient=HORIZONTAL, bootstyle="secondary").pack(
            fill=X, pady=8)
        self._right_bar, self._sv_right_lbl = self._build_motor_section(
            parent, "Direita    GPIO14 / GPIO27")

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

    # ── LIDAR panel ───────────────────────────────────────────────────────────

    def _build_lidar_panel(self, parent: ttk.Labelframe) -> None:
        parent.columnconfigure(1, weight=1)

        self._lidar_canvas = tk.Canvas(
            parent, width=LIDAR_CANVAS, height=LIDAR_CANVAS,
            background=C_CANVAS_BG, highlightthickness=1,
            highlightbackground=C_CIRCLE)
        self._lidar_canvas.grid(row=0, column=0, sticky="n", padx=(0, 12))
        self._init_lidar_canvas()

        side = ttk.Frame(parent)
        side.grid(row=0, column=1, sticky="nsew")
        side.columnconfigure(0, weight=1)

        self._build_lidar_metrics(side)
        self._build_lidar_controls(side)

    def _init_lidar_canvas(self) -> None:
        c      = self._lidar_canvas
        cx     = cy = LIDAR_CANVAS / 2
        draw_r = LIDAR_CANVAS / 2 - LIDAR_MARGIN

        self._lidar_ring_labels = []
        for frac in LIDAR_RINGS:
            rr = draw_r * frac
            c.create_oval(cx - rr, cy - rr, cx + rr, cy + rr,
                          outline=C_CANVAS_GRID, width=1)
            lbl = c.create_text(cx + 3, cy - rr, anchor="sw",
                                fill=C_MUTED, font=("", 7), text="")
            self._lidar_ring_labels.append((frac, lbl))

        c.create_line(cx - draw_r, cy, cx + draw_r, cy, fill=C_CANVAS_GRID)
        c.create_line(cx, cy - draw_r, cx, cy + draw_r, fill=C_CANVAS_GRID)

        c.create_line(cx, cy, cx, cy - draw_r,
                      fill=C_LIDAR_FRONT, width=1, arrow="last")
        c.create_text(cx + 10, cy - draw_r + 8, anchor="w",
                      fill=C_LIDAR_FRONT, font=("", 7), text="frente")

        self._lidar_point_items = [
            c.create_oval(-4, -4, -3, -3, fill=C_LIDAR_POINT,
                          outline="", state="hidden")
            for _ in range(LIDAR_POOL_SIZE)
        ]
        self._lidar_nearest_item = c.create_oval(
            -4, -4, -3, -3, fill=C_LIDAR_NEAREST, outline="white",
            width=1, state="hidden")

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

        ttk.Label(mf, text="Mais próximo", foreground=C_MUTED,
                  font=("", 8)).grid(row=0, column=0, columnspan=4, sticky=W)
        self._sv_lidar_nearest = tk.StringVar(value="—")
        ttk.Label(mf, textvariable=self._sv_lidar_nearest,
                  font=("Courier", 13, "bold"), bootstyle="info").grid(
            row=1, column=0, columnspan=4, sticky=W, pady=(0, 6))

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

        self._var_lidar_flip = tk.BooleanVar(value=self._lidar_flip)

        def _on_flip() -> None:
            self._lidar_flip = self._var_lidar_flip.get()

        ttk.Checkbutton(
            cf, text="Inverter sentido (CW/CCW)", variable=self._var_lidar_flip,
            command=_on_flip, bootstyle="secondary-round-toggle").grid(
            row=4, column=0, columnspan=4, sticky=W, pady=(6, 4))

        ttk.Separator(cf, orient=HORIZONTAL, bootstyle="secondary").grid(
            row=5, column=0, columnspan=4, sticky=EW, pady=6)

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

    # ── IMU panel ─────────────────────────────────────────────────────────────

    def _build_imu_panel(self, parent: ttk.Labelframe) -> None:
        parent.columnconfigure(1, weight=1)

        self._imu_canvas = tk.Canvas(
            parent, width=IMU_CANVAS, height=IMU_CANVAS,
            background=C_CANVAS_BG, highlightthickness=1,
            highlightbackground=C_CIRCLE)
        self._imu_canvas.grid(row=0, column=0, sticky="n", padx=(0, 12))
        self._init_imu_canvas()

        side = ttk.Frame(parent)
        side.grid(row=0, column=1, sticky="nsew")
        side.columnconfigure(0, weight=1)
        self._build_imu_readings(side)
        self._build_imu_controls(side)

    def _init_imu_canvas(self) -> None:
        c  = self._imu_canvas
        cx = cy = IMU_CANVAS / 2
        c.create_oval(4, 4, IMU_CANVAS - 4, IMU_CANVAS - 4, outline=C_CIRCLE)
        # Dynamic horizon line (rotates with roll, shifts with pitch)
        self._imu_horizon = c.create_line(
            0, cy, IMU_CANVAS, cy, fill=C_IMU_HORIZON, width=2)
        # Fixed aircraft reference symbol
        c.create_line(cx - 32, cy, cx - 10, cy, fill=C_IMU_NEEDLE, width=3)
        c.create_line(cx + 10, cy, cx + 32, cy, fill=C_IMU_NEEDLE, width=3)
        c.create_oval(cx - 2, cy - 2, cx + 2, cy + 2,
                      fill=C_IMU_NEEDLE, outline="")
        self._imu_yaw_txt = c.create_text(
            cx, IMU_CANVAS - 12, fill=C_MUTED, font=("", 8), text="yaw —")

    def _build_imu_readings(self, parent: ttk.Frame) -> None:
        rf = ttk.Labelframe(parent, text="Leitura", padding=8,
                            bootstyle="secondary")
        rf.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        rf.columnconfigure(1, weight=1)

        self._sv_imu_acc  = tk.StringVar(value="—")
        self._sv_imu_gyro = tk.StringVar(value="—")
        self._sv_imu_ang  = tk.StringVar(value="—")
        self._sv_imu_temp = tk.StringVar(value="—")
        rows = [
            ("Accel (g)",  self._sv_imu_acc),
            ("Giro (°/s)", self._sv_imu_gyro),
            ("Ângulo (°)", self._sv_imu_ang),
            ("Temp",       self._sv_imu_temp),
        ]
        for i, (lbl, sv) in enumerate(rows):
            ttk.Label(rf, text=lbl, width=10, anchor=E, foreground=C_MUTED,
                      font=("", 8)).grid(row=i, column=0, sticky=E,
                                         padx=(0, 6), pady=1)
            ttk.Label(rf, textvariable=sv, anchor=W,
                      font=("Courier", 9)).grid(row=i, column=1, sticky=W, pady=1)

    def _build_imu_controls(self, parent: ttk.Frame) -> None:
        cf = ttk.Labelframe(parent, text="Calibração & Configuração", padding=8,
                            bootstyle="secondary")
        cf.grid(row=1, column=0, sticky="ew")
        for col in range(3):
            cf.columnconfigure(col, weight=1)

        # Calibration buttons
        ttk.Button(cf, text="Calibrar acel. (nivelado)",
                   bootstyle="secondary-outline", padding=(4, 3),
                   command=self._on_imu_cal_accel).grid(
            row=0, column=0, columnspan=2, sticky=EW, padx=(0, 3), pady=2)
        ttk.Button(cf, text="Zerar yaw", bootstyle="secondary-outline",
                   padding=(4, 3), command=self._on_imu_zero_yaw).grid(
            row=0, column=2, sticky=EW, pady=2)
        self._btn_imu_mag = ttk.Button(
            cf, text="Calib. magnetômetro", bootstyle="secondary-outline",
            padding=(4, 3), command=self._on_imu_cal_mag)
        self._btn_imu_mag.grid(row=1, column=0, columnspan=2, sticky=EW,
                               padx=(0, 3), pady=2)
        ttk.Button(cf, text="Reset altura", bootstyle="secondary-outline",
                   padding=(4, 3), command=self._on_imu_height).grid(
            row=1, column=2, sticky=EW, pady=2)

        ttk.Separator(cf, orient=HORIZONTAL, bootstyle="secondary").grid(
            row=2, column=0, columnspan=3, sticky=EW, pady=6)

        # Output rate + save/factory
        ttk.Label(cf, text="Taxa", foreground=C_MUTED, font=("", 8)).grid(
            row=3, column=0, sticky=E, padx=(0, 4))
        self._var_imu_rate = tk.StringVar(value=IMU_RATES[0][0])
        rate_box = ttk.Combobox(cf, textvariable=self._var_imu_rate, width=8,
                                state="readonly",
                                values=[r[0] for r in IMU_RATES])
        rate_box.grid(row=3, column=1, sticky=W)
        rate_box.bind("<<ComboboxSelected>>", self._on_imu_rate)
        ttk.Button(cf, text="Salvar", bootstyle="success-outline",
                   padding=(4, 3), command=self._on_imu_save).grid(
            row=3, column=2, sticky=EW, pady=2)
        ttk.Button(cf, text="Padrão de fábrica", bootstyle="danger-outline",
                   padding=(4, 3), command=self._on_imu_factory).grid(
            row=4, column=0, columnspan=3, sticky=EW, pady=2)

        ttk.Separator(cf, orient=HORIZONTAL, bootstyle="secondary").grid(
            row=5, column=0, columnspan=3, sticky=EW, pady=6)

        # Connection
        conn = ttk.Frame(cf)
        conn.grid(row=6, column=0, columnspan=3, sticky=EW)
        ttk.Label(conn, text="Porta", foreground=C_MUTED, font=("", 8)).pack(
            side=LEFT, padx=(0, 4))
        self._var_imu_port = tk.StringVar(value=self._imu_init_port)
        ttk.Entry(conn, textvariable=self._var_imu_port, width=12,
                  font=("Courier", 9)).pack(side=LEFT, padx=(0, 6))
        ttk.Label(conn, text="Baud", foreground=C_MUTED, font=("", 8)).pack(
            side=LEFT, padx=(0, 4))
        self._var_imu_baud = tk.StringVar(value=str(self._imu_init_baud))
        ttk.Entry(conn, textvariable=self._var_imu_baud, width=7,
                  font=("Courier", 9)).pack(side=LEFT, padx=(0, 8))
        self._btn_imu_start = ttk.Button(
            conn, text="Iniciar", bootstyle="success", padding=(8, 3),
            command=self._on_imu_start)
        self._btn_imu_start.pack(side=LEFT, padx=(0, 4))
        self._btn_imu_stop = ttk.Button(
            conn, text="Parar", bootstyle="secondary", padding=(8, 3),
            state=DISABLED, command=self._on_imu_stop)
        self._btn_imu_stop.pack(side=LEFT)

        status_row = ttk.Frame(cf)
        status_row.grid(row=7, column=0, columnspan=3, sticky=W, pady=(6, 0))
        ttk.Label(status_row, text="status:", foreground=C_MUTED,
                  font=("", 8)).pack(side=LEFT, padx=(0, 6))
        self._sv_imu_status = tk.StringVar(value="parado")
        self._lbl_imu_status = ttk.Label(
            status_row, textvariable=self._sv_imu_status,
            font=("", 9, "bold"), bootstyle="secondary")
        self._lbl_imu_status.pack(side=LEFT)

    # ── IMU render / display ──────────────────────────────────────────────────

    def _update_imu_display(self) -> None:
        d = self._imu
        self._sv_imu_acc.set(
            f"{d.get('ax', 0.0):+6.2f} {d.get('ay', 0.0):+6.2f} {d.get('az', 0.0):+6.2f}")
        self._sv_imu_gyro.set(
            f"{d.get('wx', 0.0):+7.1f} {d.get('wy', 0.0):+7.1f} {d.get('wz', 0.0):+7.1f}")
        self._sv_imu_ang.set(
            f"{d.get('roll', 0.0):+6.1f} {d.get('pitch', 0.0):+6.1f} {d.get('yaw', 0.0):+6.1f}")
        self._sv_imu_temp.set(f"{d.get('temp', 0.0):.1f} °C")
        self._draw_imu_attitude(
            d.get("roll", 0.0), d.get("pitch", 0.0), d.get("yaw", 0.0))

    def _draw_imu_attitude(self, roll: float, pitch: float, yaw: float) -> None:
        cx = cy = IMU_CANVAS / 2
        rr  = math.radians(roll)
        off = max(-cy, min(cy, pitch * 2.0))   # px, clamped to canvas
        dx, dy = math.cos(rr), math.sin(rr)
        px, py = -math.sin(rr) * off, math.cos(rr) * off
        self._imu_canvas.coords(
            self._imu_horizon,
            cx - dx * IMU_CANVAS + px, cy - dy * IMU_CANVAS + py,
            cx + dx * IMU_CANVAS + px, cy + dy * IMU_CANVAS + py)
        self._imu_canvas.itemconfigure(self._imu_yaw_txt, text=f"yaw {yaw:+.0f}°")

    # ── IMU command helpers ───────────────────────────────────────────────────

    def _imu_write(self, *frames: bytes) -> bool:
        with self._imu_write_lock:
            if self._imu_conn is None:
                self._sv_imu_status.set("não conectado")
                return False
            try:
                for f in frames:
                    self._imu_conn.write(bytes(f))
                    time.sleep(0.01)
                self._imu_conn.flush()
                return True
            except Exception as exc:
                self._sv_imu_status.set(f"erro write: {exc}")
                return False

    def _imu_cmd(self, addr: int, lo: int, hi: int = 0x00,
                 save: bool = False) -> bool:
        frames = [IMU_UNLOCK, bytes([0xFF, 0xAA, addr, lo, hi])]
        if save:
            frames.append(IMU_SAVE)
        return self._imu_write(*frames)

    def _update_imu_buttons(self) -> None:
        self._btn_imu_start.configure(
            state=DISABLED if self._imu_running else NORMAL)
        self._btn_imu_stop.configure(
            state=NORMAL if self._imu_running else DISABLED)

    # ── IMU handlers ──────────────────────────────────────────────────────────

    def _on_imu_cal_accel(self) -> None:
        if self._imu_cmd(IMU_REG_CALSW, IMU_CAL_ACCEL):
            self._sv_imu_status.set("calibrando accel — nivelado e parado…")
            self.root.after(3000, self._finish_imu_accel)

    def _finish_imu_accel(self) -> None:
        if self._imu_cmd(IMU_REG_CALSW, IMU_CAL_NORMAL, save=True):
            self._sv_imu_status.set("acelerômetro calibrado e salvo")

    def _on_imu_cal_mag(self) -> None:
        if not self._imu_mag_calibrating:
            if self._imu_cmd(IMU_REG_CALSW, IMU_CAL_MAG):
                self._imu_mag_calibrating = True
                self._btn_imu_mag.configure(text="Concluir mag.",
                                            bootstyle="warning")
                self._sv_imu_status.set("calib. mag — gire em todas as direções…")
        else:
            self._imu_cmd(IMU_REG_CALSW, IMU_CAL_NORMAL, save=True)
            self._imu_mag_calibrating = False
            self._btn_imu_mag.configure(text="Calib. magnetômetro",
                                        bootstyle="secondary-outline")
            self._sv_imu_status.set("magnetômetro calibrado e salvo")

    def _on_imu_zero_yaw(self) -> None:
        if self._imu_cmd(IMU_REG_CALSW, IMU_CAL_YAW, save=True):
            self._sv_imu_status.set("yaw zerado")

    def _on_imu_height(self) -> None:
        if self._imu_cmd(IMU_REG_CALSW, IMU_CAL_HEIGHT, save=True):
            self._sv_imu_status.set("altura zerada")

    def _on_imu_rate(self, _event: Any = None) -> None:
        value = dict(IMU_RATES).get(self._var_imu_rate.get())
        if value is not None and self._imu_cmd(IMU_REG_RRATE, value, save=True):
            self._sv_imu_status.set(f"taxa → {self._var_imu_rate.get()}")

    def _on_imu_save(self) -> None:
        if self._imu_write(IMU_UNLOCK, IMU_SAVE):
            self._sv_imu_status.set("config salva")

    def _on_imu_factory(self) -> None:
        # Factory reset = SAVE register written with value 0x0001.
        if self._imu_write(IMU_UNLOCK, bytes([0xFF, 0xAA, IMU_REG_SAVE, 0x01, 0x00])):
            self._sv_imu_status.set("padrão de fábrica restaurado")

    def _on_imu_start(self) -> None:
        if self._imu_running:
            return
        port = self._var_imu_port.get().strip() or IMU_DEFAULT_PORT
        try:
            baud = int(self._var_imu_baud.get())
        except ValueError:
            baud = IMU_DEFAULT_BAUD
        self._imu_stop = threading.Event()
        self._imu_thread = threading.Thread(
            target=imu_reader, args=(port, baud, self.event_queue, self._imu_stop),
            name="imu-reader", daemon=True)
        self._imu_running = True
        self._sv_imu_status.set("conectando…")
        self._lbl_imu_status.configure(bootstyle="info")
        self._update_imu_buttons()
        self._imu_thread.start()

    def _on_imu_stop(self) -> None:
        if self._imu_stop is not None:
            self._imu_stop.set()
        self._sv_imu_status.set("parando…")
        self._lbl_imu_status.configure(bootstyle="secondary")

    # ── Velocity estimation (from IMU) ────────────────────────────────────────

    def _integrate_velocity(self, imu: Dict[str, float]) -> None:
        """Integrate forward accel → linear velocity; take z gyro as ω. Runs on
        every imu_data sample with real wall-clock dt (drift-bounded via leak +
        ZUPT)."""
        now = time.monotonic()
        if self._vel_last_t is None:
            self._vel_last_t = now
            return
        dt = now - self._vel_last_t
        self._vel_last_t = now
        if dt <= 0.0 or dt > VEL_DT_MAX:
            return

        a  = (imu.get("ay", 0.0) - self._vel_ay_bias) * VEL_GRAVITY   # m/s²
        wz = imu.get("wz", 0.0)                                       # °/s

        self._vel_v += a * dt
        self._vel_v *= math.exp(-VEL_LEAK * dt)                       # gentle leak

        if self._vel_zupt:
            if abs(a) < VEL_ZUPT_A and abs(wz) < VEL_ZUPT_W:
                if self._vel_still_since is None:
                    self._vel_still_since = now
                elif now - self._vel_still_since > VEL_ZUPT_HOLD:
                    self._vel_v = 0.0
            else:
                self._vel_still_since = None

        self._vel_wz = wz
        self._vel_hist_v.append(self._vel_v)
        self._vel_hist_w.append(wz)
        self._vel_dirty = True

    def _open_velocity_window(self) -> None:
        if self._vel_win is not None and self._vel_win.winfo_exists():
            self._vel_win.lift()
            return
        win = ttk.Toplevel(self.root)
        win.title("Velocidades estimadas (IMU)")
        win.protocol("WM_DELETE_WINDOW", self._close_velocity_window)
        self._vel_win = win
        self._build_velocity_window(win)
        self._draw_velocity_plots()

    def _close_velocity_window(self) -> None:
        if self._vel_win is not None:
            self._vel_win.destroy()
            self._vel_win = None

    def _build_velocity_window(self, win: Any) -> None:
        frm = ttk.Frame(win, padding=12)
        frm.pack(fill=BOTH, expand=YES)

        self._sv_vel_v = tk.StringVar(value="—")
        self._sv_vel_w = tk.StringVar(value="—")

        hv = ttk.Frame(frm)
        hv.pack(fill=X)
        ttk.Label(hv, text="Velocidade linear   v = ∫aᵧ dt",
                  foreground=C_MUTED, font=("", 9)).pack(side=LEFT)
        ttk.Label(hv, textvariable=self._sv_vel_v, font=("Courier", 13, "bold"),
                  bootstyle="info").pack(side=RIGHT)
        self._vel_canvas_v = tk.Canvas(
            frm, width=VEL_PLOT_W, height=VEL_PLOT_H, background=C_CANVAS_BG,
            highlightthickness=1, highlightbackground=C_CIRCLE)
        self._vel_canvas_v.pack(pady=(2, 10))
        self._vel_line_v, self._vel_scale_v = self._init_strip(
            self._vel_canvas_v, C_VEL_LIN)

        hw = ttk.Frame(frm)
        hw.pack(fill=X)
        ttk.Label(hw, text="Velocidade angular   ω_z (giroscópio)",
                  foreground=C_MUTED, font=("", 9)).pack(side=LEFT)
        ttk.Label(hw, textvariable=self._sv_vel_w, font=("Courier", 13, "bold"),
                  bootstyle="warning").pack(side=RIGHT)
        self._vel_canvas_w = tk.Canvas(
            frm, width=VEL_PLOT_W, height=VEL_PLOT_H, background=C_CANVAS_BG,
            highlightthickness=1, highlightbackground=C_CIRCLE)
        self._vel_canvas_w.pack(pady=(2, 10))
        self._vel_line_w, self._vel_scale_w = self._init_strip(
            self._vel_canvas_w, C_VEL_ANG)

        ctl = ttk.Frame(frm)
        ctl.pack(fill=X)
        ttk.Button(ctl, text="Tara (parado)", bootstyle="secondary-outline",
                   command=self._on_vel_tare, padding=(8, 3)).pack(
            side=LEFT, padx=(0, 6))
        ttk.Button(ctl, text="Zerar v", bootstyle="secondary-outline",
                   command=self._on_vel_reset, padding=(8, 3)).pack(
            side=LEFT, padx=(0, 6))
        self._var_vel_zupt = tk.BooleanVar(value=self._vel_zupt)
        ttk.Checkbutton(ctl, text="ZUPT (zera v parado)",
                        variable=self._var_vel_zupt, command=self._on_vel_zupt,
                        bootstyle="round-toggle").pack(side=LEFT)

        ttk.Label(
            frm,
            text="⚠  v é integrada do acelerômetro e sofre deriva. Use Tara com "
                 "a cadeira parada e nivelada; ZUPT zera v ao detectar parada. "
                 "Trate como referência, não medida precisa.",
            foreground=C_MUTED, font=("", 8), wraplength=VEL_PLOT_W,
            justify=LEFT).pack(fill=X, pady=(10, 0))

    def _init_strip(self, canvas: tk.Canvas, color: str) -> Tuple[int, int]:
        mid = VEL_PLOT_H / 2
        canvas.create_line(0, mid, VEL_PLOT_W, mid, fill=C_CANVAS_GRID)
        line  = canvas.create_line(0, mid, 0, mid, fill=color, width=2)
        scale = canvas.create_text(VEL_PLOT_W - 4, 4, anchor="ne",
                                   fill=C_MUTED, font=("", 7), text="")
        return line, scale

    def _draw_velocity_plots(self) -> None:
        if self._vel_win is None or not self._vel_win.winfo_exists():
            return
        self._sv_vel_v.set(f"{self._vel_v:+.2f} m/s")
        self._sv_vel_w.set(
            f"{self._vel_wz:+.1f} °/s   ({math.radians(self._vel_wz):+.2f} rad/s)")
        self._draw_strip(self._vel_canvas_v, self._vel_line_v,
                         self._vel_scale_v, self._vel_hist_v, 0.5, "m/s")
        self._draw_strip(self._vel_canvas_w, self._vel_line_w,
                         self._vel_scale_w, self._vel_hist_w, 50.0, "°/s")

    def _draw_strip(self, canvas: tk.Canvas, line: int, scale: int,
                    hist: "deque[float]", min_range: float, unit: str) -> None:
        mid = VEL_PLOT_H / 2
        if len(hist) < 2:
            canvas.coords(line, 0, mid, VEL_PLOT_W, mid)
            canvas.itemconfigure(scale, text="")
            return
        m = max(min_range, max(abs(v) for v in hist))
        span = VEL_HIST_LEN - 1
        amp = mid - 4
        coords = []
        for i, v in enumerate(hist):
            x = i / span * VEL_PLOT_W
            y = mid - (v / m) * amp
            coords += [x, y]
        canvas.coords(line, *coords)
        canvas.itemconfigure(scale, text=f"escala ±{m:.1f} {unit}")

    def _on_vel_tare(self) -> None:
        self._vel_ay_bias = self._imu.get("ay", 0.0)
        self._vel_v = 0.0
        self._vel_still_since = None

    def _on_vel_reset(self) -> None:
        self._vel_v = 0.0
        self._vel_still_since = None

    def _on_vel_zupt(self) -> None:
        self._vel_zupt = self._var_vel_zupt.get()

    # ── Safety panel ──────────────────────────────────────────────────────────

    def _build_safety_panel(self, parent: ttk.Frame) -> None:
        safety = ttk.Labelframe(parent, text="Safety", padding=10,
                                bootstyle="warning")
        safety.grid(row=5, column=0, columnspan=3, sticky=EW, pady=(4, 0))
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
            elif kind == "imu_conn_ready":
                self._imu_conn = payload
            elif kind == "imu_conn_gone":
                self._imu_conn = None
            elif kind == "imu_data":
                self._imu = payload
                self._imu_dirty = True
                self._integrate_velocity(payload)
            elif kind == "imu_status":
                self._sv_imu_status.set(str(payload))
                self._lbl_imu_status.configure(bootstyle="success")
            elif kind == "imu_error":
                self._sv_imu_status.set(f"erro: {payload}")
                self._lbl_imu_status.configure(bootstyle="danger")
            elif kind == "imu_stopped":
                self._imu_running = False
                self._imu_conn = None
                self._update_imu_buttons()
                if not self._sv_imu_status.get().startswith("erro:"):
                    self._sv_imu_status.set("parado")
                    self._lbl_imu_status.configure(bootstyle="secondary")

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

        # LIDAR — only redraw when a fresh scan arrived (≈10 Hz, not every frame)
        if self._lidar_dirty:
            self._render_lidar()
            self._lidar_dirty = False

        # IMU — refresh readings/attitude when fresh data arrived
        if self._imu_dirty:
            self._update_imu_display()
            self._imu_dirty = False

        # Velocity window strip charts (only when open)
        if self._vel_dirty:
            self._vel_dirty = False
            self._draw_velocity_plots()

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

    # ── LIDAR render ──────────────────────────────────────────────────────────

    def _render_lidar(self) -> None:
        c     = self._lidar_canvas
        items = self._lidar_point_items
        scan  = self._lidar_scan

        if not scan:
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

        shown = min(n, len(items))
        if shown > self._lidar_shown:
            for i in range(self._lidar_shown, shown):
                c.itemconfigure(items[i], state="normal")
        elif shown < self._lidar_shown:
            for i in range(shown, self._lidar_shown):
                c.itemconfigure(items[i], state="hidden")
        self._lidar_shown = shown

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
            name="lidar-ros-reader", daemon=True)
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
        self._set_armed(False)
        self.stop_event.set()
        if self._lidar_stop is not None:
            self._lidar_stop.set()
        if self._imu_stop is not None:
            self._imu_stop.set()
        self._sv_conn.set("closing…")
        self._wait_for_reader()

    def _wait_for_reader(self) -> None:
        lidar_alive = (self._lidar_thread is not None
                       and self._lidar_thread.is_alive())
        imu_alive = (self._imu_thread is not None
                     and self._imu_thread.is_alive())
        if self.reader_thread.is_alive() or lidar_alive or imu_alive:
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
            name="serial-reader", daemon=True)
    else:
        # LIDAR-only mode: no ESP32 — idle thread keeps the shutdown path simple.
        reader = threading.Thread(target=lambda: None,
                                  name="serial-reader-idle", daemon=True)

    WheelchairControlGUI(
        root, event_queue, stop_event, reader,
        gui_update_ms=args.gui_update_ms,
        esp_port=args.port,
        lidar_port=args.lidar,
        lidar_topic=args.lidar_topic,
        imu_port=args.imu,
        imu_baud=args.imu_baud)
    if args.port:
        reader.start()
    root.mainloop()
    stop_event.set()
    if args.port:
        reader.join(timeout=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
