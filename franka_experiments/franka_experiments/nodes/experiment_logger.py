#!/usr/bin/env python3
"""ROS 2 experiment logger for Franka EE-pentagon + CBF avoidance experiments.

It subscribes to joint states, CBF distance messages, nominal/safe velocity
commands and optional torque commands. During the run it writes a wide CSV.
On shutdown it generates plots in the same output folder.
"""

from __future__ import annotations

import csv
import json
import subprocess
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
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import (
    FrankaRobotState, MultiDistance, MultiLinkDistance)

from franka_experiments.utils.math_utils import _safe_float  # noqa: F401
from franka_experiments.utils.node_runtime import (  # noqa: F401
    _as_list7,
    _now_sec,
    _stamp_to_sec,
)
from franka_experiments.utils.config import resolve_log_dir
from franka_experiments.utils.params import (
    declare_float, declare_int, declare_str)

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

# Where a run lands by default. NOT $HOME any more: the container does not
# mount it, so every run used to end up somewhere the operator could not reach
# from their own file manager and had to `docker cp` out. resolve_log_dir()
# prefers the repo's franka_logs/, which IS mounted and IS gitignored.
# Override with $FRANKA_LOGS_DIR or the output_dir parameter.
DEFAULT_OUTPUT_DIR = resolve_log_dir()

DEFAULT_EXPERIMENT_NAME = "bag_02_dynamic_test"

DEFAULT_JOINT_STATE_TOPIC = "/NS_1/joint_states"
DEFAULT_MULTI_DISTANCE_TOPIC = "/human_robot/multi_distance"

DEFAULT_QDOT_NOM_TOPIC = "/NS_1/tracking_qdot"
DEFAULT_QDOT_CMD_TOPIC = "/NS_1/qdot_cmd"
DEFAULT_TORQUE_CMD_TOPIC = "/NS_1/torque_cmd"

# FCI health feed.  Set the parameter to "" to skip the subscription entirely:
# it is the joint_state_broadcaster's 1 kHz sibling, and with shared memory off
# (fastdds_no_shm.xml) every subscriber costs UDP loopback work next to the RT
# loop.  The callback here is deliberately three assignments long.
DEFAULT_ROBOT_STATE_TOPIC = "/NS_1/franka_robot_state_broadcaster/robot_state"

# ── ISO 10218 layer (roadmap Step 10) ────────────────────────────────────────
# Every ISO quantity that exists at runtime has to exist in the bag too, or the
# only record of a stop is a log line nobody kept. All three are optional: a
# topic that never publishes leaves its columns NaN, which is how a CSV says
# "that channel was not running" as distinct from "it was running and read 0".
DEFAULT_CBF_STATUS_TOPIC = "/NS_1/cbf_status"
DEFAULT_ISO_SAFETY_TOPIC = "/NS_1/iso_safety"
DEFAULT_TORQUE_SAT_TOPIC = "/NS_1/torque_saturation"

# ── Acceleration pipeline + Cartesian ────────────────────────────────────────
# The torque stack publishes q̈, not q̇: pentagon_qddot_commander -> qddot_nom,
# cbf_safety_filter -> qddot_safe. The qdot_nom_* / qdot_cmd_* columns below are
# the VELOCITY pipeline's and stay EMPTY in a torque run — they are kept because
# cbf_velocity_filter still fills them, not because they mean anything here.
DEFAULT_QDDOT_NOM_TOPIC = "/NS_1/qddot_nom"
DEFAULT_QDDOT_SAFE_TOPIC = "/NS_1/qddot_safe"
# The barrier's own input, per control point — richer than the legacy
# MultiDistance: one entry per CP rather than per link, with the tracked
# obstacle velocity on it.
DEFAULT_PER_LINK_TOPIC = "/cbf/per_link_distances"
#: Frame whose Cartesian pose and speed are logged. The TCP speed is the
#: quantity ISO limits (reduced speed, v_PFL) and NOTHING in this package logged
#: it before — joint speed is not a substitute, a folded arm decouples the two.
DEFAULT_TCP_LINK = "fr3_link8"


class ExperimentLogger(Node):
    def __init__(self):
        super().__init__("experiment_logger")

        # Try to reuse your YAML config defaults, but keep the node standalone.
        # TODO[LEGACY]: cfg_topics is built from two YAML files and never read; its keys (joint_states_topic) would not match the parameter names (joint_state_topic) even if wired | confidence: high | superseded-by: none | flagged: 2026-09-01
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
        # Range-checked on startup: a non-positive rate would divide by zero in
        # create_timer, a bad alpha would silently corrupt every logged trace.
        # Declared + range-checked here; the values are read back below through
        # get_parameter(), exactly as before.
        declare_float(self, "sample_rate_hz", 100.0,
                      positive=True, maximum=10000.0)
        declare_float(self, "d_safe", 0.20 if cfg_dsafe is None else cfg_dsafe,
                      minimum=0.0, maximum=2.0)
        declare_int(self, "max_cbf_entries", 12, positive=True)
        declare_float(self, "accel_lpf_alpha", 0.35, minimum=0.0, maximum=1.0)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)

        self.declare_parameter("joint_state_topic", DEFAULT_JOINT_STATE_TOPIC)
        self.declare_parameter("multi_distance_topic", DEFAULT_MULTI_DISTANCE_TOPIC)
        self.declare_parameter("qdot_nom_topic", DEFAULT_QDOT_NOM_TOPIC)
        self.declare_parameter("qdot_cmd_topic", DEFAULT_QDOT_CMD_TOPIC)
        self.declare_parameter("torque_cmd_topic", DEFAULT_TORQUE_CMD_TOPIC)
        self.declare_parameter("robot_state_topic", DEFAULT_ROBOT_STATE_TOPIC)
        self.declare_parameter("cbf_status_topic", DEFAULT_CBF_STATUS_TOPIC)
        self.declare_parameter("iso_safety_topic", DEFAULT_ISO_SAFETY_TOPIC)
        self.declare_parameter("torque_saturation_topic", DEFAULT_TORQUE_SAT_TOPIC)
        self.declare_parameter("qddot_nom_topic", DEFAULT_QDDOT_NOM_TOPIC)
        self.declare_parameter("qddot_safe_topic", DEFAULT_QDDOT_SAFE_TOPIC)
        self.declare_parameter("per_link_distances_topic", DEFAULT_PER_LINK_TOPIC)
        self.tcp_link = declare_str(self, "tcp_link", DEFAULT_TCP_LINK)

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

        # FCI communication health.  control_command_success_rate is libfranka's
        # own rolling fraction of accepted commands over the last 100 packets —
        # the quantity the firmware uses to decide a
        # communication_constraints_violation reflex.  Sampling it at
        # sample_rate_hz loses nothing: the metric is already a 100 ms average.
        self.last_comm_success = float("nan")
        self.last_robot_mode = -1
        self.comm_success_min = float("nan")
        self.comm_degraded_count = 0

        # ── Acceleration pipeline, Cartesian, per-CP avoidance ───────────
        # NaN until the matching topic publishes: a column of zeros would read
        # as "measured, and zero", which is a different statement from "that
        # channel was not running" and the one that misleads.
        self.last_qddot_nom = np.full(NUM_JOINTS, np.nan)
        self.last_qddot_safe = np.full(NUM_JOINTS, np.nan)
        self.cp_stats: Dict[str, object] = {}
        self._kin = None
        self._tcp_fid = None
        self._build_kinematics()

        # ISO layer, all NaN until their topic publishes.
        self.last_cbf_n_rows = float("nan")
        self.last_cbf_slack = float("nan")
        self.last_cbf_fault = float("nan")
        self.last_cbf_n_viol = float("nan")
        self.last_cbf_d_min = float("nan")
        self.last_cbf_sp = float("nan")
        self.last_cbf_vcap = float("nan")
        self.last_cbf_vcls = float("nan")
        self.last_cbf_isostop = float("nan")
        self.last_iso_latched = float("nan")
        self.last_iso_reason = float("nan")
        self.last_iso_sp = float("nan")
        self.last_iso_vcap = float("nan")
        self.last_iso_vcls = float("nan")
        self.last_tau_sat = np.full(NUM_JOINTS, np.nan)
        self.iso_stop_count = 0

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
        self._write_manifest()

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

        _rs_topic = str(self.get_parameter("robot_state_topic").value).strip()
        if _rs_topic:
            # depth 1 + BEST_EFFORT: this publisher runs at 1 kHz and we only
            # ever want its newest value, never a backlog.
            self.create_subscription(
                FrankaRobotState,
                _rs_topic,
                self.robot_state_cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

        _pl = str(self.get_parameter("per_link_distances_topic").value).strip()
        if _pl:
            # BEST_EFFORT depth 1, matching the publisher: this is a ~30 Hz
            # sensor stream and we only ever want its newest frame.
            self.create_subscription(
                MultiLinkDistance, _pl, self.per_link_cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        for topic_param, cb in (("cbf_status_topic", self.cbf_status_cb),
                                ("iso_safety_topic", self.iso_safety_cb),
                                ("torque_saturation_topic", self.torque_sat_cb),
                                ("qddot_nom_topic", self.qddot_nom_cb),
                                ("qddot_safe_topic", self.qddot_safe_cb)):
            topic = str(self.get_parameter(topic_param).value).strip()
            if topic:
                self.create_subscription(Float64MultiArray, topic, cb, 10)

        self.timer = self.create_timer(1.0 / self.sample_rate_hz, self.sample)

        self.get_logger().info(
            "ExperimentLogger started\n"
            f"  csv                  : {self.csv_path}\n"
            f"  d_safe               : {self.d_safe:.3f} m\n"
            f"  joint_state_topic    : {self.get_parameter('joint_state_topic').value}\n"
            f"  multi_distance_topic : {self.get_parameter('multi_distance_topic').value}\n"
            f"  qdot_nom_topic       : {self.get_parameter('qdot_nom_topic').value}\n"
            f"  qdot_cmd_topic       : {self.get_parameter('qdot_cmd_topic').value}\n"
            f"  torque_cmd_topic     : {self.get_parameter('torque_cmd_topic').value}\n"
            f"  robot_state_topic    : {_rs_topic or '(disabled)'}"
        )

    def _build_kinematics(self) -> None:
        """Pinocchio, for the Cartesian block. Fails SOFT.

        A missing model must cost the Cartesian columns, not the whole run: by
        the time this node starts the robot is usually already moving, and a
        logger that refuses to start is a logger that was not there when it
        mattered. The columns then stay NaN, which the summary reports as
        "not recorded" rather than as zero.
        """
        try:
            import pinocchio as pin
            from franka_experiments.utils.kinematics import (
                CBFKinematics, build_urdf_no_hand)
            self._pin = pin
            self._kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
            self._tcp_fid = self._kin.resolve_frame_id(self.tcp_link)
            if self._tcp_fid is None:
                raise RuntimeError(f'frame {self.tcp_link!r} not in the model')
        except Exception as exc:                       # noqa: BLE001
            self._kin = self._tcp_fid = None
            self.get_logger().warn(
                f'Cartesian block disabled ({exc}) — tcp_* columns stay NaN. '
                f'Joint speed is NOT a substitute: a folded or near-singular '
                f'arm decouples joint speed from task speed in both directions.')

    def _write_manifest(self) -> None:
        """The run's own description, next to its data.

        A CSV of numbers with no record of which configuration produced them is
        not a measurement, it is a pile of numbers. This is what makes a run
        readable six months later, and comparable with another one.
        """
        def _git(*a):
            # The installed copy is not a git checkout (colcon copies, it does
            # not symlink), so try the source tree first and the working
            # directory second — a run launched from the workspace resolves
            # there. Provenance is worth two attempts: "which code produced
            # this run" is the first question anyone asks of a log.
            for cwd in (os.path.dirname(os.path.abspath(__file__)), os.getcwd()):
                try:
                    out = subprocess.run(a, cwd=cwd, capture_output=True,
                                         text=True, timeout=5)
                    if out.returncode == 0 and out.stdout.strip():
                        return out.stdout.strip()
                except Exception:                      # noqa: BLE001
                    continue
            return None

        man = {
            "schema": "franka_experiments/experiment_log/2",
            "created": datetime.now().isoformat(timespec="seconds"),
            "csv": str(self.csv_path),
            "sample_rate_hz": self.sample_rate_hz,
            "d_safe": self.d_safe,
            "tcp_link": self.tcp_link,
            "cartesian_available": self._kin is not None,
            "joint_names": list(self.joint_names),
            "git": {"sha": _git("git", "rev-parse", "HEAD"),
                    "branch": _git("git", "rev-parse", "--abbrev-ref", "HEAD"),
                    "dirty": bool(_git("git", "status", "--porcelain"))},
            "topics": {k: str(self.get_parameter(k).value) for k in (
                "joint_state_topic", "multi_distance_topic", "per_link_distances_topic",
                "qdot_nom_topic", "qdot_cmd_topic", "qddot_nom_topic",
                "qddot_safe_topic", "torque_cmd_topic", "robot_state_topic",
                "cbf_status_topic", "iso_safety_topic", "torque_saturation_topic")},
            "note": (
                "qdot_nom_* and qdot_cmd_* belong to the VELOCITY pipeline and "
                "stay empty in a torque/acceleration run — the stack publishes "
                "qddot_nom / qddot_safe instead. NaN means the channel was not "
                "running; 0 means it was and read zero."),
        }
        try:
            with open(self.run_dir / "run_manifest.json", "w") as fh:
                json.dump(man, fh, indent=2, sort_keys=True, default=str)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f"could not write run_manifest.json: {exc}")

    def _make_header(self) -> List[str]:
        header = ["t"]
        # qdot_nom / qdot_cmd are the VELOCITY pipeline's and stay empty in a
        # torque run; qddot_nom / qddot_safe are the acceleration pipeline's and
        # are the ones that carry the command in this stack.
        for prefix in ["q", "qdot", "qddot", "tau_effort", "tau_cmd",
                       "qdot_nom", "qdot_cmd", "qddot_nom", "qddot_safe"]:
            header += [f"{prefix}_{i}" for i in range(1, NUM_JOINTS + 1)]
        header += [
            "qdot_delta_norm",
            "qdot_nom_norm",
            "qdot_cmd_norm",
            # How hard the barrier bent the commander's intent, in the space
            # the torque stack actually commands. This is THE avoidance number:
            # 0 means the CBF passed the task through untouched.
            "qddot_nom_norm",
            "qddot_safe_norm",
            "qddot_delta_norm",
            # ── Cartesian: what ISO limits, and what joint speed cannot say ──
            "tcp_x", "tcp_y", "tcp_z",
            "tcp_qx", "tcp_qy", "tcp_qz", "tcp_qw",
            "tcp_speed",            # [m/s] linear speed of the TCP
            "tcp_omega",            # [rad/s] angular speed of the TCP
            # ── Avoidance, per control point (the barrier's own input) ───────
            "cp_n_valid", "cp_n_total",
            "cp_min_distance", "cp_min_link",
            "cp_v_obs_max",         # [m/s] fastest tracked obstacle
            "cp_n_tracked",         # entries carrying a confirmed track
            "min_distance",
            "min_h",
            "min_h_link",
            "min_h_zone",
            "min_h_confidence",
            "valid_cbf_count",
            "cbf_violation_count",
            "comm_success_rate",
            "comm_success_min",
            "robot_mode",
            # ── The barrier's own status, data[0..4] ─────────────────────
            # These are the avoidance numbers anyone actually reads after a run:
            # how many rows were in the QP, how much slack the solver paid, was
            # the chain faulted, how many barriers were VIOLATED, and the
            # closest gap. They were missing — only the ISO tail below was
            # logged — so a run recorded that the filter was bending the command
            # without recording what it was bending it around.
            "cbf_n_rows",
            "cbf_slack",
            "cbf_fault",
            "cbf_n_violated",
            "cbf_d_min",
            # ── ISO layer ────────────────────────────────────────────────
            # cbf_* are the filter's view (cbf_status data[5..8]); iso_* are the
            # independent monitor's own (/NS_1/iso_safety). Logged SEPARATELY
            # and not merged: when the two disagree, that disagreement is the
            # most interesting thing in the run.
            "cbf_S_p",
            "cbf_v_cap_min",
            "cbf_v_closing_max",
            "cbf_iso_stop",
            "iso_stop_latched",
            "iso_trip_reason",
            "iso_S_p",
            "iso_v_cap_min",
            "iso_v_closing_max",
            "iso_stop_count",
        ]
        header += [f"tau_sat_{i}" for i in range(1, NUM_JOINTS + 1)]
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

    def qddot_nom_cb(self, msg: Float64MultiArray):
        self.last_qddot_nom = _as_list7(msg.data)

    def qddot_safe_cb(self, msg: Float64MultiArray):
        self.last_qddot_safe = _as_list7(msg.data)

    def per_link_cb(self, msg: MultiLinkDistance):
        """The barrier's own input, summarised.

        One entry per CONTROL POINT, not per link, so this counts what the QP
        actually received — including the entries it dropped as invalid, which
        is the difference between "nothing was near" and "perception failed".
        """
        from franka_experiments.utils.perception_msgs import labelled_links
        n_valid = n_total = n_tracked = 0
        d_min, d_lbl, v_max = float("inf"), "", 0.0
        for label, ld in labelled_links(msg):
            n_total += 1
            if not ld.valid:
                continue
            n_valid += 1
            d = _safe_float(ld.distance)
            if math.isfinite(d) and d < d_min:
                d_min, d_lbl = d, label
            v = math.sqrt(ld.obstacle_velocity.x ** 2
                          + ld.obstacle_velocity.y ** 2
                          + ld.obstacle_velocity.z ** 2)
            if v > v_max:
                v_max = v
            if int(getattr(ld, "track_id", 0)) > 0:
                n_tracked += 1
        self.cp_stats = {
            "cp_n_valid": n_valid, "cp_n_total": n_total,
            "cp_min_distance": d_min if math.isfinite(d_min) else float("nan"),
            "cp_min_link": d_lbl, "cp_v_obs_max": v_max,
            "cp_n_tracked": n_tracked,
        }

    def cbf_status_cb(self, msg: Float64MultiArray):
        """The barrier's status: the core data[0..4] and the ISO tail data[5..8].

        Two separate length guards, not one: a filter built before the ISO layer
        publishes exactly 5 elements, and gating the whole callback on 9 would
        have thrown away the core fields too — which is precisely what it did
        until this was fixed.
        """
        d = msg.data
        if len(d) >= 5:
            self.last_cbf_n_rows = _safe_float(d[0])
            self.last_cbf_slack = _safe_float(d[1])
            self.last_cbf_fault = _safe_float(d[2])
            self.last_cbf_n_viol = _safe_float(d[3])
            self.last_cbf_d_min = _safe_float(d[4])
        if len(d) >= 9:
            self.last_cbf_sp = _safe_float(d[5])
            self.last_cbf_vcap = _safe_float(d[6])
            self.last_cbf_vcls = _safe_float(d[7])
            self.last_cbf_isostop = _safe_float(d[8])

    def iso_safety_cb(self, msg: Float64MultiArray):
        """The monitor's own row, and a running count of stops.

        The count lives here rather than in sample() for the same reason
        comm_success_min does: a stop that latches and is reset between two
        samples still has to appear in the CSV.
        """
        d = msg.data
        if len(d) >= 5:
            latched = _safe_float(d[0])
            if latched >= 1.0 and not (self.last_iso_latched >= 1.0):
                self.iso_stop_count += 1
            self.last_iso_latched = latched
            self.last_iso_reason = _safe_float(d[1])
            self.last_iso_sp = _safe_float(d[2])
            self.last_iso_vcap = _safe_float(d[3])
            self.last_iso_vcls = _safe_float(d[4])

    def torque_sat_cb(self, msg: Float64MultiArray):
        self.last_tau_sat = _as_list7(msg.data)

    def robot_state_cb(self, msg: FrankaRobotState):
        """Runs at 1 kHz — keep it to assignments, no numpy, no logging.

        The running minimum lives here rather than in sample() so a dip
        shorter than the sampling period still lands in the CSV.
        """
        rate = float(msg.control_command_success_rate)
        self.last_comm_success = rate
        self.last_robot_mode = int(msg.robot_mode)
        if rate < 1.0:
            self.comm_degraded_count += 1
        if not (self.comm_success_min <= rate):     # nan-safe: nan <= x is False
            self.comm_success_min = rate

    def _cartesian(self) -> Dict[str, object]:
        """TCP pose and the two speeds ISO actually limits.

        ``‖J_v q̇‖`` from the Jacobian rather than a finite difference of the
        position: the derivative of a sampled pose is dominated by differencing
        noise at 100 Hz, and this number is compared against a 250 mm/s limit.
        """
        nan = float("nan")
        blank = {k: nan for k in ("tcp_x", "tcp_y", "tcp_z", "tcp_qx", "tcp_qy",
                                  "tcp_qz", "tcp_qw", "tcp_speed", "tcp_omega")}
        if self._kin is None or not np.all(np.isfinite(self.last_q)):
            return blank
        try:
            qdot = (self.last_qdot if np.all(np.isfinite(self.last_qdot))
                    else np.zeros(NUM_JOINTS))
            self._kin.update(self.last_q, qdot, with_jdot=False)
            oMf = self._kin.data.oMf[self._tcp_fid]
            p = np.asarray(oMf.translation)
            quat = self._pin.Quaternion(oMf.rotation)
            J6 = self._pin.getFrameJacobian(
                self._kin.model, self._kin.data, self._tcp_fid,
                self._pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            v6 = J6 @ qdot
            return {
                "tcp_x": float(p[0]), "tcp_y": float(p[1]), "tcp_z": float(p[2]),
                "tcp_qx": float(quat.x), "tcp_qy": float(quat.y),
                "tcp_qz": float(quat.z), "tcp_qw": float(quat.w),
                "tcp_speed": float(np.linalg.norm(v6[:3])),
                "tcp_omega": float(np.linalg.norm(v6[3:])),
            }
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f"Cartesian sample failed: {exc}",
                                   throttle_duration_sec=10.0)
            return blank

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
            "qddot_nom": self.last_qddot_nom,
            "qddot_safe": self.last_qddot_safe,
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

        # ── The acceleration pipeline, and how hard the barrier bent it ──
        row["qddot_nom_norm"] = self._norm(self.last_qddot_nom)
        row["qddot_safe_norm"] = self._norm(self.last_qddot_safe)
        if (np.all(np.isfinite(self.last_qddot_nom))
                and np.all(np.isfinite(self.last_qddot_safe))):
            row["qddot_delta_norm"] = self._norm(
                self.last_qddot_safe - self.last_qddot_nom)
        else:
            row["qddot_delta_norm"] = np.nan

        # ── Cartesian ────────────────────────────────────────────────────
        row.update(self._cartesian())

        # ── Avoidance, per control point ─────────────────────────────────
        for k in ("cp_n_valid", "cp_n_total", "cp_min_distance",
                  "cp_v_obs_max", "cp_n_tracked"):
            row[k] = self.cp_stats.get(k, np.nan)
        row["cp_min_link"] = self.cp_stats.get("cp_min_link", "")

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

        row["comm_success_rate"] = self.last_comm_success
        row["comm_success_min"] = self.comm_success_min
        row["robot_mode"] = self.last_robot_mode

        row["cbf_n_rows"] = self.last_cbf_n_rows
        row["cbf_slack"] = self.last_cbf_slack
        row["cbf_fault"] = self.last_cbf_fault
        row["cbf_n_violated"] = self.last_cbf_n_viol
        row["cbf_d_min"] = self.last_cbf_d_min
        row["cbf_S_p"] = self.last_cbf_sp
        row["cbf_v_cap_min"] = self.last_cbf_vcap
        row["cbf_v_closing_max"] = self.last_cbf_vcls
        row["cbf_iso_stop"] = self.last_cbf_isostop
        row["iso_stop_latched"] = self.last_iso_latched
        row["iso_trip_reason"] = self.last_iso_reason
        row["iso_S_p"] = self.last_iso_sp
        row["iso_v_cap_min"] = self.last_iso_vcap
        row["iso_v_closing_max"] = self.last_iso_vcls
        row["iso_stop_count"] = self.iso_stop_count
        for i in range(NUM_JOINTS):
            row[f"tau_sat_{i + 1}"] = self.last_tau_sat[i]

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
        self._plot_comm_health()
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

    def _plot_comm_health(self):
        """FCI link health: the metric the firmware reflexes on.

        A run that ends in communication_constraints_violation shows the rate
        sagging BEFORE the abort; a run that ends on a control fault (velocity
        or self-collision reflex) keeps it pinned at 1.0 and moves robot_mode
        to REFLEX(4) instead.  That is the whole point of logging both.
        """
        t = self._time()
        rate = self._col("comm_success_rate")
        mode = self._col("robot_mode")
        if not self._has_data(rate):
            return

        fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True,
                               gridspec_kw={"height_ratios": [3, 1]})
        ax[0].plot(t, rate, linewidth=1.4, label="control_command_success_rate")
        ax[0].axhline(1.0, linestyle="--", linewidth=1.0, color="grey",
                      label="1.0 = every command accepted")
        finite = rate[np.isfinite(rate)]
        if finite.size:
            ax[0].set_ylim(min(0.9, float(finite.min()) - 0.01), 1.005)
            ax[0].set_title(
                f"FCI communication health — min {float(finite.min()):.3f}, "
                f"{self.comm_degraded_count} degraded packets")
        ax[0].set_ylabel("accepted fraction")
        ax[0].grid(True, alpha=0.3)
        ax[0].legend(loc="lower left")

        ax[1].plot(t, mode, linewidth=1.4, drawstyle="steps-post")
        ax[1].set_yticks([1, 2, 4, 5])
        ax[1].set_yticklabels(["IDLE", "MOVE", "REFLEX", "USER_STOP"], fontsize=8)
        ax[1].set_ylabel("robot_mode")
        ax[1].set_xlabel("time [s]")
        ax[1].grid(True, alpha=0.3)
        self._savefig("comm_health.png")

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
