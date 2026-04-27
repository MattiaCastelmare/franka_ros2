#!/usr/bin/env python3
"""ROS 2 experiment logger for Franka EE-pentagon + CBF avoidance experiments.

It subscribes to joint states, CBF distance messages, nominal/safe velocity
commands and optional torque commands. During the run it writes a wide CSV.
On shutdown it generates plots in the same output folder.
"""

from __future__ import annotations

import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import MultiDistance

try:
    from scipy.signal import savgol_filter as _savgol
    _SCIPY_OK = True
except ImportError:
    _SCIPY_OK = False


NUM_JOINTS = 7

# Plot only joints influenced by segments with control points
PLOT_JOINT_START = 3   # J3..J7
PLOT_JOINT_INDICES = range(PLOT_JOINT_START, NUM_JOINTS + 1)

DEFAULT_JOINT_NAMES = [f"fr3_joint{i}" for i in range(1, NUM_JOINTS + 1)]

# Default paths/topics for direct execution without terminal parameters
from pathlib import Path


def resolve_source_nodes_dir() -> Path:
    p = Path(__file__).resolve()

    for parent in p.parents:
        candidate = parent / "src" / "franka_experiments" / "franka_experiments" / "nodes"
        if candidate.exists():
            return candidate

    # fallback se lanci direttamente dal source
    return Path(__file__).resolve().parent


DEFAULT_OUTPUT_DIR = str(resolve_source_nodes_dir() / "experiment_logs")

DEFAULT_EXPERIMENT_NAME = "bag_02_dynamic_test"

DEFAULT_JOINT_STATE_TOPIC = "/NS_1/joint_states"
DEFAULT_MULTI_DISTANCE_TOPIC = "/human_robot/multi_distance"

DEFAULT_QDOT_NOM_TOPIC = "/NS_1/tracking_qdot"
DEFAULT_QDOT_CMD_TOPIC = "/NS_1/qdot_cmd"
DEFAULT_TORQUE_CMD_TOPIC = "/NS_1/torque_cmd"


def _now_sec(node: Node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


def _stamp_to_sec(msg_stamp, fallback: float) -> float:
    sec = float(msg_stamp.sec) + float(msg_stamp.nanosec) * 1e-9
    return sec if sec > 0.0 else fallback


def _safe_float(x, default=np.nan) -> float:
    try:
        y = float(x)
        return y if math.isfinite(y) else default
    except Exception:
        return default


def _as_list7(data: Sequence[float]) -> np.ndarray:
    arr = np.full(NUM_JOINTS, np.nan, dtype=float)
    n = min(NUM_JOINTS, len(data))
    if n > 0:
        arr[:n] = np.asarray(data[:n], dtype=float)
    return arr


class ExperimentLogger(Node):
    def __init__(self):
        super().__init__("experiment_logger")

        # Try to reuse your YAML config defaults, but keep the node standalone.
        cfg_topics: Dict[str, str] = {}
        cfg_dsafe: Optional[float] = None
        try:
            from franka_experiments.utils.cbf_utils import load_robot_config
            control_cfg = load_robot_config("control")
            distance_cfg = load_robot_config("distance")
            cfg_topics.update(control_cfg.get("topics", {}))
            cfg_topics.update(distance_cfg.get("topics", {}))
            cfg_dsafe = float(control_cfg.get("params", {}).get("d_safe"))
        except Exception:
            pass

        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("run_dir_override", "")
        self.declare_parameter("experiment_name", DEFAULT_EXPERIMENT_NAME)
        self.declare_parameter("sample_rate_hz", 100.0)
        self.declare_parameter("d_safe", 0.20 if cfg_dsafe is None else cfg_dsafe)
        self.declare_parameter("max_cbf_entries", 12)
        self.declare_parameter("accel_lpf_alpha", 0.35)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)

        self.declare_parameter("joint_state_topic", DEFAULT_JOINT_STATE_TOPIC)
        self.declare_parameter("multi_distance_topic", DEFAULT_MULTI_DISTANCE_TOPIC)
        self.declare_parameter("qdot_nom_topic", DEFAULT_QDOT_NOM_TOPIC)
        self.declare_parameter("qdot_cmd_topic", DEFAULT_QDOT_CMD_TOPIC)
        self.declare_parameter("torque_cmd_topic", DEFAULT_TORQUE_CMD_TOPIC)

        self.output_root = Path(str(self.get_parameter("output_dir").value)).expanduser()
        exp_name = str(self.get_parameter("experiment_name").value)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        _override = str(self.get_parameter("run_dir_override").value).strip()
        if _override:
            self.run_dir = Path(_override).expanduser()
        else:
            self.run_dir = self.output_root / f"{stamp}_{exp_name}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.run_dir / "experiment_log.csv"

        self.sample_rate_hz = float(self.get_parameter("sample_rate_hz").value)
        self.d_safe = float(self.get_parameter("d_safe").value)
        self.max_cbf_entries = int(self.get_parameter("max_cbf_entries").value)
        self.accel_lpf_alpha = float(self.get_parameter("accel_lpf_alpha").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        if len(self.joint_names) != NUM_JOINTS:
            self.get_logger().warn("joint_names must contain 7 names; using fr3_joint1..fr3_joint7")
            self.joint_names = DEFAULT_JOINT_NAMES

        self.t0 = _now_sec(self)
        self.last_q = np.full(NUM_JOINTS, np.nan)
        self.last_qdot = np.full(NUM_JOINTS, np.nan)
        self.last_qddot = np.full(NUM_JOINTS, np.nan)
        self.last_tau_effort = np.full(NUM_JOINTS, np.nan)
        self.last_tau_cmd = np.full(NUM_JOINTS, np.nan)
        self.last_qdot_nom = np.full(NUM_JOINTS, np.nan)
        self.last_qdot_cmd = np.full(NUM_JOINTS, np.nan)

        self._prev_qdot: Optional[np.ndarray] = None
        self._prev_js_time: Optional[float] = None
        self._prev_qddot_filt: Optional[np.ndarray] = None

        self.cbf_entries: List[Dict[str, object]] = []
        self.rows: List[Dict[str, object]] = []
        self._closed = False

        self.header = self._make_header()
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.header)
        self.writer.writeheader()

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.joint_state_cb,
            50,
        )
        self.create_subscription(
            MultiDistance,
            str(self.get_parameter("multi_distance_topic").value),
            self.multi_distance_cb,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("qdot_nom_topic").value),
            self.qdot_nom_cb,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("qdot_cmd_topic").value),
            self.qdot_cmd_cb,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("torque_cmd_topic").value),
            self.torque_cmd_cb,
            20,
        )

        self.timer = self.create_timer(1.0 / self.sample_rate_hz, self.sample)

        self.get_logger().info(
            "ExperimentLogger started\n"
            f"  csv                  : {self.csv_path}\n"
            f"  d_safe               : {self.d_safe:.3f} m\n"
            f"  joint_state_topic    : {self.get_parameter('joint_state_topic').value}\n"
            f"  multi_distance_topic : {self.get_parameter('multi_distance_topic').value}\n"
            f"  qdot_nom_topic       : {self.get_parameter('qdot_nom_topic').value}\n"
            f"  qdot_cmd_topic       : {self.get_parameter('qdot_cmd_topic').value}\n"
            f"  torque_cmd_topic     : {self.get_parameter('torque_cmd_topic').value}"
        )

    def _make_header(self) -> List[str]:
        header = ["t"]
        for prefix in ["q", "qdot", "qddot", "tau_effort", "tau_cmd", "qdot_nom", "qdot_cmd"]:
            header += [f"{prefix}_{i}" for i in range(1, NUM_JOINTS + 1)]
        header += [
            "qdot_delta_norm",
            "qdot_nom_norm",
            "qdot_cmd_norm",
            "min_distance",
            "min_h",
            "min_h_link",
            "min_h_zone",
            "min_h_confidence",
            "valid_cbf_count",
            "cbf_violation_count",
        ]
        for k in range(1, self.max_cbf_entries + 1):
            header += [
                f"cbf{k}_link",
                f"cbf{k}_distance",
                f"cbf{k}_h",
                f"cbf{k}_zone",
                f"cbf{k}_confidence",
                f"cbf{k}_valid",
            ]
        return header

    def _vector_from_joint_state(self, msg: JointState, field: Sequence[float]) -> np.ndarray:
        out = np.full(NUM_JOINTS, np.nan, dtype=float)
        if not field:
            return out
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        used_named = True
        for j, name in enumerate(self.joint_names):
            idx = name_to_idx.get(name)
            if idx is None or idx >= len(field):
                used_named = False
                break
            out[j] = _safe_float(field[idx])
        if not used_named:
            n = min(NUM_JOINTS, len(field))
            out[:n] = np.asarray(field[:n], dtype=float)
        return out

    def joint_state_cb(self, msg: JointState):
        now = _now_sec(self)
        msg_time = _stamp_to_sec(msg.header.stamp, now)

        q = self._vector_from_joint_state(msg, msg.position)
        qdot = self._vector_from_joint_state(msg, msg.velocity)
        tau = self._vector_from_joint_state(msg, msg.effort)

        if self._prev_qdot is not None and self._prev_js_time is not None:
            dt = msg_time - self._prev_js_time
            if dt > 1e-5 and np.all(np.isfinite(qdot)) and np.all(np.isfinite(self._prev_qdot)):
                qddot_raw = (qdot - self._prev_qdot) / dt
                if self._prev_qddot_filt is None:
                    qddot = qddot_raw
                else:
                    a = float(np.clip(self.accel_lpf_alpha, 0.0, 1.0))
                    qddot = a * qddot_raw + (1.0 - a) * self._prev_qddot_filt
                self._prev_qddot_filt = qddot.copy()
                self.last_qddot = qddot

        self.last_q = q
        self.last_qdot = qdot
        self.last_tau_effort = tau
        self._prev_qdot = qdot.copy()
        self._prev_js_time = msg_time

    def multi_distance_cb(self, msg: MultiDistance):
        entries: List[Dict[str, object]] = []
        for item in msg.distances:
            d = _safe_float(item.distance)
            valid = bool(item.valid) and math.isfinite(d)
            h = d - self.d_safe if valid else np.nan
            entries.append({
                "link": str(item.robot_link_name),
                "distance": d,
                "h": h,
                "zone": str(item.zone),
                "confidence": _safe_float(item.confidence),
                "valid": valid,
            })
        entries.sort(key=lambda e: e["h"] if math.isfinite(float(e["h"])) else np.inf)
        self.cbf_entries = entries[: self.max_cbf_entries]

    def qdot_nom_cb(self, msg: Float64MultiArray):
        self.last_qdot_nom = _as_list7(msg.data)

    def qdot_cmd_cb(self, msg: Float64MultiArray):
        self.last_qdot_cmd = _as_list7(msg.data)

    def torque_cmd_cb(self, msg: Float64MultiArray):
        self.last_tau_cmd = _as_list7(msg.data)

    @staticmethod
    def _norm(v: np.ndarray) -> float:
        return float(np.linalg.norm(v)) if np.all(np.isfinite(v)) else np.nan

    def sample(self):
        if self._closed:
            return
        t = _now_sec(self) - self.t0
        row: Dict[str, object] = {"t": t}

        vectors = {
            "q": self.last_q,
            "qdot": self.last_qdot,
            "qddot": self.last_qddot,
            "tau_effort": self.last_tau_effort,
            "tau_cmd": self.last_tau_cmd,
            "qdot_nom": self.last_qdot_nom,
            "qdot_cmd": self.last_qdot_cmd,
        }
        for prefix, vec in vectors.items():
            for i in range(NUM_JOINTS):
                row[f"{prefix}_{i + 1}"] = vec[i]

        if np.all(np.isfinite(self.last_qdot_nom)) and np.all(np.isfinite(self.last_qdot_cmd)):
            delta = self.last_qdot_cmd - self.last_qdot_nom
            row["qdot_delta_norm"] = self._norm(delta)
        else:
            row["qdot_delta_norm"] = np.nan
        row["qdot_nom_norm"] = self._norm(self.last_qdot_nom)
        row["qdot_cmd_norm"] = self._norm(self.last_qdot_cmd)

        valid_entries = [e for e in self.cbf_entries if bool(e["valid"])]
        if valid_entries:
            e0 = valid_entries[0]
            row["min_distance"] = e0["distance"]
            row["min_h"] = e0["h"]
            row["min_h_link"] = e0["link"]
            row["min_h_zone"] = e0["zone"]
            row["min_h_confidence"] = e0["confidence"]
            row["valid_cbf_count"] = len(valid_entries)
            row["cbf_violation_count"] = sum(1 for e in valid_entries if float(e["h"]) <= 0.0)
        else:
            row["min_distance"] = np.nan
            row["min_h"] = np.nan
            row["min_h_link"] = ""
            row["min_h_zone"] = ""
            row["min_h_confidence"] = np.nan
            row["valid_cbf_count"] = 0
            row["cbf_violation_count"] = 0

        for k in range(1, self.max_cbf_entries + 1):
            if k <= len(self.cbf_entries):
                e = self.cbf_entries[k - 1]
                row[f"cbf{k}_link"] = e["link"]
                row[f"cbf{k}_distance"] = e["distance"]
                row[f"cbf{k}_h"] = e["h"]
                row[f"cbf{k}_zone"] = e["zone"]
                row[f"cbf{k}_confidence"] = e["confidence"]
                row[f"cbf{k}_valid"] = int(bool(e["valid"]))
            else:
                row[f"cbf{k}_link"] = ""
                row[f"cbf{k}_distance"] = np.nan
                row[f"cbf{k}_h"] = np.nan
                row[f"cbf{k}_zone"] = ""
                row[f"cbf{k}_confidence"] = np.nan
                row[f"cbf{k}_valid"] = 0

        self.writer.writerow(row)
        self.rows.append(row)

    def finalize(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.timer.cancel()
        except Exception:
            pass
        try:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()
        except Exception:
            pass

        if len(self.rows) < 2:
            self.get_logger().warn(f"Not enough samples to plot. CSV: {self.csv_path}")
            return

        self.get_logger().info(f"Generating plots in {self.run_dir}")
        self._plot_joint_family("q", "Joint position", "rad", "joint_positions.png")
        self._plot_joint_family("qdot", "Joint velocity", "rad/s", "joint_velocities.png")
        self._plot_joint_family("qddot", "Joint acceleration", "rad/s²", "joint_accelerations.png")
        self._plot_joint_family("tau_effort", "Joint effort / measured torque", "Nm", "joint_effort_torques.png")
        self._plot_joint_family("tau_cmd", "Commanded torque", "Nm", "joint_commanded_torques.png")
        self._plot_cbf_min_h()
        self._plot_cbf_links()
        self._plot_min_distance()
        self._plot_velocity_filter_effect()
        self._plot_norms()
        self.get_logger().info(f"Done. CSV and plots saved in: {self.run_dir}")

    def _col(self, name: str) -> np.ndarray:
        vals = []
        for r in self.rows:
            v = r.get(name, np.nan)
            vals.append(_safe_float(v))
        return np.asarray(vals, dtype=float)

    def _time(self) -> np.ndarray:
        return self._col("t")

    @staticmethod
    def _has_data(y: np.ndarray) -> bool:
        return np.any(np.isfinite(y))
    
    def _smooth(self, y: np.ndarray, alpha: float = 0.18) -> np.ndarray:
        """
        Savitzky-Golay smoothing (falls back to EMA if scipy missing).
        Preserves peak positions, removes high-freq noise.
        window_length scales with data length, min 11 samples.
        """
        out = y.copy()
        idx = np.where(np.isfinite(y))[0]
        if len(idx) < 5:
            return out

        # ── Savitzky-Golay path ──────────────────────────────────────────
        if _SCIPY_OK:
            # window ~ 1 s of data; data rate ≈ 100 Hz → 101 samples,
            # clipped to data length and forced odd.
            n = len(idx)
            wl = min(51, n if n % 2 == 1 else n - 1)
            wl = max(wl, 11)   # at least 11 samples
            if wl % 2 == 0:
                wl -= 1
            segment = y[idx]
            smoothed = _savgol(segment, window_length=wl, polyorder=3)
            out[idx] = smoothed
            return out

        # ── EMA fallback ─────────────────────────────────────────────────
        first = idx[0]
        out[first] = y[first]
        for i in range(first + 1, len(y)):
            if np.isfinite(y[i]):
                prev = out[i - 1] if np.isfinite(out[i - 1]) else y[i]
                out[i] = alpha * y[i] + (1.0 - alpha) * prev
            else:
                out[i] = out[i - 1]
        return out

    def _savefig(self, name: str):
        plt.tight_layout()
        plt.savefig(self.run_dir / name, dpi=160)
        plt.close()

    def _plot_joint_family(self, prefix: str, title: str, ylabel: str, filename: str):
        t = self._time()

        ys = [self._col(f"{prefix}_{i}") for i in PLOT_JOINT_INDICES]

        if not any(self._has_data(y) for y in ys):
            return

        plt.figure(figsize=(12, 6))

        for j_idx, y in zip(PLOT_JOINT_INDICES, ys):
            if self._has_data(y):
                y_plot = self._smooth(y)
                plt.plot(t, y_plot, label=f"J{j_idx}", linewidth=1.5)

        plt.title(title)
        plt.xlabel("time [s]")
        plt.ylabel(ylabel)
        plt.grid(True, alpha=0.3)
        plt.legend(ncol=3)
        self._savefig(filename)

    def _plot_cbf_min_h(self):
        t = self._time()
        h = self._col("min_h")
        if not self._has_data(h):
            return
        plt.figure(figsize=(12, 5))
        plt.plot(t, h, label="min h = min(distance - d_safe)", linewidth=1.5)
        plt.axhline(0.0, linestyle="--", linewidth=1.2, label="safety boundary h = 0")
        plt.title("Minimum CBF margin over time")
        plt.xlabel("time [s]")
        plt.ylabel("h [m]")
        plt.grid(True, alpha=0.3)
        plt.legend()
        self._savefig("cbf_min_h.png")

    def _plot_cbf_links(self):
        t = self._time()
        link_names = set()
        for r in self.rows:
            for k in range(1, self.max_cbf_entries + 1):
                link = str(r.get(f"cbf{k}_link", ""))
                if link:
                    link_names.add(link)
        if not link_names:
            return
        plt.figure(figsize=(12, 5))
        for link in sorted(link_names):
            y = np.full(len(self.rows), np.nan)
            for idx, r in enumerate(self.rows):
                vals = []
                for k in range(1, self.max_cbf_entries + 1):
                    if str(r.get(f"cbf{k}_link", "")) == link:
                        vals.append(_safe_float(r.get(f"cbf{k}_h")))
                vals = [v for v in vals if math.isfinite(v)]
                if vals:
                    y[idx] = min(vals)
            if self._has_data(y):
                plt.plot(t, y, label=link, linewidth=1.2)
        plt.axhline(0.0, linestyle="--", linewidth=1.2, label="h = 0")
        plt.title("CBF margin h per robot link / segment")
        plt.xlabel("time [s]")
        plt.ylabel("h [m]")
        plt.grid(True, alpha=0.3)
        plt.legend()
        self._savefig("cbf_h_by_link.png")

    def _plot_min_distance(self):
        t = self._time()
        d = self._col("min_distance")
        if not self._has_data(d):
            return
        plt.figure(figsize=(12, 5))
        plt.plot(t, d, label="minimum measured distance", linewidth=1.5)
        plt.axhline(self.d_safe, linestyle="--", linewidth=1.2, label=f"d_safe = {self.d_safe:.3f} m")
        plt.title("Minimum human-robot distance")
        plt.xlabel("time [s]")
        plt.ylabel("distance [m]")
        plt.grid(True, alpha=0.3)
        plt.legend()
        self._savefig("min_distance.png")

    def _plot_velocity_filter_effect(self):
        t = self._time()
        nom = self._col("qdot_nom_norm")
        cmd = self._col("qdot_cmd_norm")
        delta = self._col("qdot_delta_norm")
        if not (self._has_data(nom) or self._has_data(cmd) or self._has_data(delta)):
            return
        plt.figure(figsize=(12, 5))
        if self._has_data(nom):
            plt.plot(t, nom, label="||qdot_nom||", linewidth=1.4)
        if self._has_data(cmd):
            plt.plot(t, cmd, label="||qdot_cmd||", linewidth=1.4)
        if self._has_data(delta):
            plt.plot(t, delta, label="||qdot_cmd - qdot_nom||", linewidth=1.4)
        plt.title("CBF intervention on velocity command")
        plt.xlabel("time [s]")
        plt.ylabel("norm [rad/s]")
        plt.grid(True, alpha=0.3)
        plt.legend()
        self._savefig("qdot_nom_vs_safe_norms.png")

    def _plot_norms(self):
        t = self._time()
        qdot_norm = []
        qddot_norm = []
        tau_norm = []
        for r in self.rows:
            qdot = np.array([_safe_float(r.get(f"qdot_{i}")) for i in PLOT_JOINT_INDICES])
            qddot = np.array([_safe_float(r.get(f"qddot_{i}")) for i in PLOT_JOINT_INDICES])
            tau = np.array([_safe_float(r.get(f"tau_effort_{i}")) for i in PLOT_JOINT_INDICES])
            qdot_norm.append(self._norm(qdot))
            qddot_norm.append(self._norm(qddot))
            tau_norm.append(self._norm(tau))
        qdot_norm = self._smooth(np.asarray(qdot_norm))
        qddot_norm = self._smooth(np.asarray(qddot_norm))
        tau_norm = self._smooth(np.asarray(tau_norm))
        if not (self._has_data(qdot_norm) or self._has_data(qddot_norm) or self._has_data(tau_norm)):
            return
        plt.figure(figsize=(12, 5))
        if self._has_data(qdot_norm):
            plt.plot(t, qdot_norm, label="||qdot||", linewidth=1.3)
        if self._has_data(qddot_norm):
            plt.plot(t, qddot_norm, label="||qddot||", linewidth=1.3)
        if self._has_data(tau_norm):
            plt.plot(t, tau_norm, label="||tau_effort||", linewidth=1.3)
        plt.title("Global motion/effort norms")
        plt.xlabel("time [s]")
        plt.ylabel("norm")
        plt.grid(True, alpha=0.3)
        plt.legend()
        self._savefig("motion_effort_norms.png")


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
