#!/usr/bin/env python3
"""Pentagon trajectory for cbf_torque_controller — no CBF filter.

Unified phase-variable design
─────────────────────────────
There is no APPROACH/TRACK state machine.  A single dimensionless phase
``s`` drives one continuous geometric path:

    s, s_dot, s_ddot = timing.step(dt)      # how fast       (TimingLaw)
    p_d = path.position(s)                   # where          (GeometricPath)
    v_d = path.velocity(s, s_dot)            # = P'(s)·s_dot
    a_d = path.acceleration(s, s_dot, s_ddot)# = P''(s)·s_dot² + P'(s)·s_ddot

The path is a :class:`CompositePath`:

  * phase ``s ∈ [0, S_a)`` — straight-line approach lead-in whose ``P(0)`` is the
    *current EE position* captured at start-up (no position jump, no impulsive
    command), running into
  * phase ``s ≥ S_a``       — the cyclic :class:`PentagonPath` (C¹ Bézier corner
    blending), one loop per ``Δs = 1``.

Initial smoothing lives in the timing law (a C² soft-start ramps ``s_dot`` from
0), not in a separate state.  A constant phase rate ``1/cycle_time`` after the
ramp reproduces the original constant-speed pentagon exactly.

The feedback controller (task-space PD + adaptive damped-LS pseudo-inverse) and
the ``qddot → dq_d → q_d`` integration are unchanged.

Safety: qddot output is clamped to ±qddot_max per joint at every tick.
"""

from __future__ import annotations

import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import numpy as np
import pinocchio as pin
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS, AUTO_SENTINEL
from franka_experiments.utils.ros import get_namespace_from_config, run_node_main
from franka_experiments.utils.cbf_utils import load_robot_config
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_frame_id,
    resolve_arm_joint_ids,
    transform_ee_to_frame,
)
from sensor_msgs.msg import JointState as SensorJointState
from franka_experiments.utils.trajectory import (
    PentagonTrajectory, PentagonPath, LinearSegmentPath, CompositePath,
)
from franka_experiments.utils.timing_law import (
    LinearTimingLaw, ExponentialTimingLaw, TrapezoidalTimingLaw,
)
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str


class PentagonQddotCommander(Node):

    def __init__(self):
        super().__init__('pentagon_qddot_commander')

        self.done       = False
        self._stopping  = False
        self._stop_end  = 0.0
        self._running   = False    # False = warm-up (zeros); True = phase-driven

        # ── Load robot config (topics + per-joint limits) ────────────────
        _cfg    = load_robot_config('control')
        _topics = _cfg['topics']
        _limits = _cfg['joint_limits']
        _jnames = [f'joint{i}' for i in range(1, 8)]

        # ── Parameters ───────────────────────────────────────────────────
        # qddot_safe_topic: default reads qddot_nom from fr3_control.yaml so
        # the node connects directly to qddot_to_torque without a launch override.
        # When cbf_safety_filter is inserted, point this to topics['qddot_nom']
        # and let the CBF filter publish from qddot_nom to qddot_safe.
        self.declare_parameter('qddot_safe_topic',  _topics.get('qddot_nom', '/NS_1/qddot_nom'))
        self.declare_parameter('q_des_topic',       '/NS_1/q_des_state')
        self.declare_parameter('reset_thr_m',       0.10)
        self.declare_parameter('joint_state_topic', AUTO_SENTINEL)
        self.declare_parameter('ee_frame',          'fr3_hand_tcp')
        self.declare_parameter('rate_hz',           100.0)
        self.declare_parameter('warmup_s',            3.0)
        # Unified phase timing.  approach_time = wall-clock duration of the
        # straight-line lead-in (current EE → pentagon start), realised as the
        # soft-start window of the timing law.  timing_law selects how the phase
        # advances: 'linear' (constant rate + C² soft start, recommended for the
        # cyclic pentagon), 'trapezoidal' (constant-accel ramp then cruise), or
        # 'exponential' (one-shot ease — does not cycle).
        self.declare_parameter('approach_time',       2.5)
        self.declare_parameter('timing_law',          'linear')
        self.declare_parameter('center_xyz',          [0.4, 0.0, 0.4])
        self.declare_parameter('radius',              0.30)
        self.declare_parameter('plane',               'front')
        self.declare_parameter('plane_frame',         'fr3_link0')
        self.declare_parameter('cycle_time',          8.0)
        self.declare_parameter('smoothness',          0.20)
        # kp/kd: task-space Cartesian gains [N/m, N·s/m].
        # Conservative for torque control — overshoot propagates via M·q̈.
        # kd ≈ 2√kp for near-critical damping.
        self.declare_parameter('kp_cart',             20.0)
        self.declare_parameter('kd_cart',              9.0)
        self.declare_parameter('kp_rot',              10.0)
        self.declare_parameter('kd_rot',               6.0)
        # High-rate CSV logging: output directory + measured-effort source topic.
        self.declare_parameter('log_dir',
                               '/ros2_ws/src/franka_experiments/franka_logs')
        self.declare_parameter('joint_effort_topic', '/NS_1/joint_states')

        qddot_topic    = self.get_parameter('qddot_safe_topic').value
        q_des_topic    = self.get_parameter('q_des_topic').value
        self.reset_thr = float(self.get_parameter('reset_thr_m').value)
        js_topic_param = self.get_parameter('joint_state_topic').value
        ee_frame_name  = self.get_parameter('ee_frame').value
        self.rate_hz   = float(self.get_parameter('rate_hz').value)
        self.warmup_s  = float(self.get_parameter('warmup_s').value)
        self.approach_time = float(self.get_parameter('approach_time').value)
        self.timing_law_name = str(self.get_parameter('timing_law').value).lower()
        center_xyz     = list(self.get_parameter('center_xyz').value)
        radius         = float(self.get_parameter('radius').value)
        plane          = self.get_parameter('plane').value
        plane_frame    = self.get_parameter('plane_frame').value
        self.cycle_time = float(self.get_parameter('cycle_time').value)
        smoothness     = float(self.get_parameter('smoothness').value)
        self.kp        = float(self.get_parameter('kp_cart').value)
        self.kd        = float(self.get_parameter('kd_cart').value)
        self.kp_rot    = float(self.get_parameter('kp_rot').value)
        self.kd_rot    = float(self.get_parameter('kd_rot').value)
        self._log_dir  = str(self.get_parameter('log_dir').value)
        effort_topic   = str(self.get_parameter('joint_effort_topic').value)
        self._dt       = 1.0 / self.rate_hz

        # Per-joint q̈ clamps from fr3_control.yaml joint_limits column [3].
        # Matches franka_description deceleration_limit: [6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0]
        self.qddot_max = np.array([_limits[j][3] for j in _jnames], dtype=np.float64)

        # ── Pinocchio ────────────────────────────────────────────────────
        self.get_logger().info('Generating URDF …')
        try:
            urdf_xml = generate_urdf_from_xacro()
        except Exception as exc:
            self.get_logger().error(f'URDF generation failed: {exc}')
            raise SystemExit(1) from exc

        self.pin_model, self.pin_data = load_pinocchio_model(urdf_xml)

        try:
            self.ee_frame_id = resolve_frame_id(self.pin_model, ee_frame_name)
        except RuntimeError as exc:
            self.get_logger().error(str(exc)); raise SystemExit(1) from exc

        self._use_plane_frame = (plane == 'front')
        self._plane_frame_id  = -1
        if self._use_plane_frame:
            try:
                self._plane_frame_id = resolve_frame_id(self.pin_model, plane_frame)
            except RuntimeError as exc:
                self.get_logger().error(str(exc)); raise SystemExit(1) from exc

        try:
            self._pin_joint_ids = resolve_arm_joint_ids(self.pin_model)
        except RuntimeError as exc:
            self.get_logger().error(str(exc)); raise SystemExit(1) from exc

        self._arm_v_ids = [self.pin_model.joints[p].idx_v for p in self._pin_joint_ids]

        # ── Geometric path (pure geometry, immutable) ─────────────────────
        # PentagonTrajectory stays the time-based generator; PentagonPath wraps
        # it as a function of phase s (one loop per Δs = 1).  The runtime
        # approach lead-in + composite path are built in _start_trajectory()
        # once the current EE position is known.
        self.traj = PentagonTrajectory(
            center=np.array(center_xyz), radius=radius,
            plane=plane, cycle_time=self.cycle_time, smoothness=smoothness,
        )
        self._pentagon_path = PentagonPath(self.traj)
        # Start point = vertex 0 (top of pentagon)
        self._start_xyz = self.traj.vertices[0].copy()

        # Phase variable + path/timing, assembled in _start_trajectory()
        self._path: Optional[CompositePath] = None
        self._timing = None
        self._approach_span = 0.0

        # ── Pre-allocated buffers ─────────────────────────────────────────
        nv = self.pin_model.nv
        self._q_neutral      = pin.neutral(self.pin_model)
        self._q_full         = pin.neutral(self.pin_model)
        self._qdot_full      = np.zeros(nv)
        self._J6n            = np.zeros((6, nv))
        self._dJ6n           = np.zeros((6, nv))
        self._J_arm          = np.zeros((6, NUM_JOINTS))
        self._dJ_arm         = np.zeros((6, NUM_JOINTS))
        self._J_rot_tmp      = np.zeros((3, NUM_JOINTS))
        self._p_ee           = np.zeros(3)
        self._R_des          = np.eye(3)
        self._R_err          = np.zeros((3, 3))
        self._e_rot          = np.zeros(3)
        self._e6             = np.zeros(6)
        self._edot6          = np.zeros(6)
        self._xddot6         = np.zeros(6)
        self._tmp3           = np.zeros(3)
        self._tmp6           = np.zeros(6)
        self._JJT            = np.zeros((6, 6))
        self._JJT_reg        = np.zeros((6, 6))
        self._J_pinv         = np.zeros((NUM_JOINTS, 6))
        self._q_ddot         = np.zeros(NUM_JOINTS)
        # Adaptive damped-LS parameters (Yoshikawa manipulability)
        self._lambda_sq_min  = 1e-6          # floor (well-conditioned poses)
        self._lambda_sq_max  = 5e-2          # ceiling (near singularity)
        self._manip_thr      = 0.05          # w₀: manipulability threshold
        self._lambda_sq      = self._lambda_sq_min
        self._orient_ok      = False

        # Actual-dt tracking — compensates Python timer jitter in integration
        self._prev_tick_time = None

        # Joint-space integration buffers for q_d, dq_d
        self._q_d            = np.zeros(NUM_JOINTS)
        self._dq_d           = np.zeros(NUM_JOINTS)

        # ── Logging buffers (pre-allocated → no per-tick allocation) ──────
        # tau_des = M(q)·q̈_des + nle(q, q̇).  computeAllTerms() already fills
        # pin_data.M (upper triangle only) and pin_data.nle each tick, so we
        # only need scratch space for the matrix-vector product.
        self._qddot_full_des = np.zeros(nv)          # full-model q̈ (arm rows set)
        self._tau_full       = np.zeros(nv)          # full-model torque scratch
        self._tau_des        = np.zeros(NUM_JOINTS)  # desired torque (arm joints)
        self._tau_meas       = np.zeros(NUM_JOINTS)  # measured torque (from effort)
        # CRBA stores only the upper triangle of M; cache the lower-triangle
        # indices once so we can mirror it to a full symmetric matrix in-place.
        self._M_tril         = np.tril_indices(nv, -1)
        self._eff_imap: Optional[List[int]] = None   # effort name→index map

        # Pre-built JointState setpoint message (avoids allocation per tick)
        self._sp_msg         = SensorJointState()
        self._sp_msg.name    = list(FR3_JOINT_NAMES)
        self._sp_msg.position = [0.0] * NUM_JOINTS
        self._sp_msg.velocity = [0.0] * NUM_JOINTS
        self._sp_msg.effort   = [0.0] * NUM_JOINTS

        # Output
        self._out_msg        = Float64MultiArray()
        self._out_msg.data   = [0.0] * NUM_JOINTS
        self._zero_msg       = Float64MultiArray()
        self._zero_msg.data  = [0.0] * NUM_JOINTS

        # ── Joint state double buffer ─────────────────────────────────────
        self._js_lock  = threading.Lock()
        self._js_a     = {'q': np.zeros(NUM_JOINTS), 'qdot': np.zeros(NUM_JOINTS),
                          'q_full': pin.neutral(self.pin_model), 'valid': False}
        self._js_b     = {'q': np.zeros(NUM_JOINTS), 'qdot': np.zeros(NUM_JOINTS),
                          'q_full': pin.neutral(self.pin_model), 'valid': False}
        self._js_write = self._js_a
        self._js_read  = self._js_b
        self._js_stamp = self.get_clock().now()
        self._js_imap: Optional[List[int]] = None

        js_topic = js_topic_param
        if js_topic == AUTO_SENTINEL:
            ns = get_namespace_from_config()
            js_topic = f'/{ns}/joint_states' if ns else '/joint_states'

        self._js_sub = self.create_subscription(JointState, js_topic, self._js_cb, 10)

        # Measured-effort subscription (configurable; used only for logging)
        self._eff_sub = self.create_subscription(
            JointState, effort_topic, self._effort_cb, 10)

        # ── Publisher / timer ─────────────────────────────────────────────
        self.pub      = self.create_publisher(Float64MultiArray, qddot_topic, 10)
        self._sp_pub  = self.create_publisher(SensorJointState,  q_des_topic,  10)
        self.timer    = self.create_timer(self._dt, self._tick)
        self.t0    = self.get_clock().now()
        self._tlog = ThrottledLogger(self.get_logger())

        # High-rate CSV logger (opened once; closed on node destruction)
        self._init_logger(effort_topic)

        self.get_logger().info(
            f'pentagon_qddot_commander\n'
            f'  topic    : {qddot_topic}\n'
            f'  js_topic : {js_topic}\n'
            f'  center   : {center_xyz}  r={radius} m  plane={plane}\n'
            f'  start_xyz: {self._start_xyz.tolist()}\n'
            f'  cycle_t  : {self.cycle_time} s  Kp={self.kp} Kd={self.kd}\n'
            f'  timing   : {self.timing_law_name}  approach_time={self.approach_time} s\n'
            f'  qddot_max: {self.qddot_max.tolist()} rad/s²  (per-joint)')

    # ── Joint state callback ──────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        if self._js_imap is None:
            try:
                self._js_imap = [msg.name.index(jn) for jn in FR3_JOINT_NAMES]
            except ValueError:
                return
        if len(msg.position) <= max(self._js_imap):
            return
        buf = self._js_write
        for k, i in enumerate(self._js_imap):
            buf['q'][k]    = msg.position[i]
            buf['qdot'][k] = msg.velocity[i] if len(msg.velocity) > i else 0.0
        q_full = buf['q_full']
        np.copyto(q_full, self._q_neutral)
        for k, pid in enumerate(self._pin_joint_ids):
            q_full[self.pin_model.joints[pid].idx_q] = buf['q'][k]
        buf['valid'] = True
        with self._js_lock:
            self._js_write, self._js_read = self._js_read, self._js_write
        self._js_stamp = self.get_clock().now()

    # ── Measured effort callback (logging only) ───────────────────────────

    def _effort_cb(self, msg: JointState) -> None:
        """Cache measured joint torques (msg.effort) for the next log row."""
        if self._eff_imap is None:
            try:
                self._eff_imap = [msg.name.index(jn) for jn in FR3_JOINT_NAMES]
            except ValueError:
                return
        if len(msg.effort) <= max(self._eff_imap):
            return
        for k, i in enumerate(self._eff_imap):
            self._tau_meas[k] = msg.effort[i]

    # ── Stop ─────────────────────────────────────────────────────────────

    def request_stop(self, dur: float = 0.5):
        if self._stopping:
            return
        self._stopping = True
        self._stop_end = time.monotonic() + dur
        self.get_logger().info(f'Stopping: zero qddot for {dur} s')

    # ── Main tick ────────────────────────────────────────────────────────

    def _tick(self):
        if self._stopping:
            self.pub.publish(self._zero_msg)
            if time.monotonic() >= self._stop_end:
                self.timer.cancel()
                self.done = True
            return

        now_stamp = self.get_clock().now()
        t = (now_stamp - self.t0).nanoseconds * 1e-9

        # Measure actual elapsed time to compensate Python timer jitter.
        # Clamped to [0.5×dt, 2×dt] to avoid integration explosions on startup
        # or after a long pause.
        if self._prev_tick_time is not None:
            actual_dt = float((now_stamp - self._prev_tick_time).nanoseconds * 1e-9)
            actual_dt = float(np.clip(actual_dt, self._dt * 0.5, self._dt * 2.0))
        else:
            actual_dt = self._dt
        self._prev_tick_time = now_stamp

        # Warm-up: publish zeros until a valid joint state arrives, then build
        # the phase-driven path/timing rooted at the current EE position.
        if not self._running:
            self.pub.publish(self._zero_msg)
            if t >= self.warmup_s:
                with self._js_lock:
                    js = self._js_read
                if js['valid']:
                    self._start_trajectory(js)
            if self._tlog.due(t):
                self._tlog.info(f'[WARMUP {t:.1f}/{self.warmup_s}s]')
            return

        # Read joint state
        with self._js_lock:
            js = self._js_read
        if not js['valid']:
            self.pub.publish(self._zero_msg)
            return
        age = (self.get_clock().now() - self._js_stamp).nanoseconds * 1e-9
        if age > 0.1:
            self.pub.publish(self._zero_msg)
            if self._tlog.due(t):
                self.get_logger().warn(f'JS stale {age:.3f}s')
            return

        # Pinocchio kinematics
        np.copyto(self._q_full, js['q_full'])
        self._qdot_full[:] = 0.0
        for k, vid in enumerate(self._arm_v_ids):
            self._qdot_full[vid] = js['qdot'][k]
        qdot = js['qdot']

        pin.computeAllTerms(self.pin_model, self.pin_data,
                            self._q_full, self._qdot_full)
        pin.updateFramePlacements(self.pin_model, self.pin_data)

        self._J6n[:] = pin.getFrameJacobian(
            self.pin_model, self.pin_data, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self._dJ6n[:] = pin.getFrameJacobianTimeVariation(
            self.pin_model, self.pin_data, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        for i, vid in enumerate(self._arm_v_ids):
            self._J_arm[:, i]  = self._J6n[:, vid]
            self._dJ_arm[:, i] = self._dJ6n[:, vid]

        oMee  = self.pin_data.oMf[self.ee_frame_id]
        R_cur = np.asarray(oMee.rotation)

        if self._use_plane_frame:
            p_raw, _ = transform_ee_to_frame(
                self.pin_model, self.pin_data, self._plane_frame_id, oMee,
                self._J6n[:, self._arm_v_ids])
            np.copyto(self._p_ee, p_raw)
            R_ref = np.asarray(self.pin_data.oMf[self._plane_frame_id].rotation)
            np.dot(R_ref.T, self._J_arm[:3, :], out=self._J_rot_tmp)
            self._J_arm[:3, :] = self._J_rot_tmp
            np.dot(R_ref.T, self._dJ_arm[:3, :], out=self._J_rot_tmp)
            self._dJ_arm[:3, :] = self._J_rot_tmp
        else:
            np.copyto(self._p_ee, oMee.translation)

        # Orientation: capture once, hold throughout
        if not self._orient_ok:
            np.copyto(self._R_des, R_cur)
            self._orient_ok = True
        np.dot(self._R_des.T, R_cur, out=self._R_err)
        self._e_rot[0] = 0.5 * (self._R_err[2, 1] - self._R_err[1, 2])
        self._e_rot[1] = 0.5 * (self._R_err[0, 2] - self._R_err[2, 0])
        self._e_rot[2] = 0.5 * (self._R_err[1, 0] - self._R_err[0, 1])

        # Unified timing law + geometric path — no APPROACH/TRACK branching.
        #   s, s_dot, s_ddot = timing.step(dt)
        #   p_d = P(s) ; v_d = P'(s)·s_dot ; a_d = P''(s)·s_dot² + P'(s)·s_ddot
        s, s_dot, s_ddot = self._timing.step(actual_dt)
        p_d = self._path.position(s)
        v_d = self._path.velocity(s, s_dot)
        a_d = self._path.acceleration(s, s_dot, s_ddot)

        # 6D error
        np.subtract(p_d, self._p_ee, out=self._tmp3)
        self._e6[:3] = self._tmp3
        self._e6[3:] = self._e_rot

        # 6D velocity error: [v_d - J_lin·qdot ; -J_ang·qdot]
        np.dot(self._J_arm, qdot, out=self._edot6)
        np.subtract(v_d, self._edot6[:3], out=self._edot6[:3])
        np.negative(self._edot6[3:], out=self._edot6[3:])

        # Task-space acceleration: xddot6 = a_d + Kp*e + Kd*edot
        self._xddot6[:3] = a_d
        np.multiply(self.kp,     self._e6[:3],    out=self._tmp3)
        self._xddot6[:3] += self._tmp3
        np.multiply(self.kd,     self._edot6[:3], out=self._tmp3)
        self._xddot6[:3] += self._tmp3
        np.multiply(self.kp_rot, self._e6[3:],    out=self._xddot6[3:])
        np.multiply(self.kd_rot, self._edot6[3:], out=self._tmp3)
        self._xddot6[3:] += self._tmp3

        # Adaptive damped LS: λ² = λ_max * (1 - w/w₀)² when w < w₀
        np.dot(self._J_arm, self._J_arm.T, out=self._JJT)
        w = math.sqrt(max(0.0, float(np.linalg.det(self._JJT))))
        if w < self._manip_thr:
            self._lambda_sq = self._lambda_sq_max * (1.0 - w / self._manip_thr) ** 2
        else:
            self._lambda_sq = self._lambda_sq_min
        np.copyto(self._JJT_reg, self._JJT)
        for i in range(6):
            self._JJT_reg[i, i] += self._lambda_sq
        np.dot(self._J_arm.T, np.linalg.inv(self._JJT_reg), out=self._J_pinv)

        # qddot = J_pinv @ (xddot6 − dJ·qdot)
        np.dot(self._dJ_arm, qdot, out=self._tmp6)
        np.subtract(self._xddot6, self._tmp6, out=self._xddot6)
        np.dot(self._J_pinv, self._xddot6, out=self._q_ddot)

        # Safety clamp
        np.clip(self._q_ddot, -self.qddot_max, self.qddot_max, out=self._q_ddot)

        # ── Integrate qddot → dq_d → q_d (Euler, actual dt for jitter tolerance)
        self._dq_d += self._q_ddot * actual_dt
        self._q_d  += self._dq_d   * actual_dt

        # Reset q_d to actual joint state if EE error is too large (anti-windup)
        ee_err = float(np.linalg.norm(self._e6[:3]))
        if ee_err > self.reset_thr:
            np.copyto(self._q_d,  js['q'])
            np.copyto(self._dq_d, js['qdot'])
            if self._tlog.due(t):
                self.get_logger().warn(
                    f'q_d reset: EE error {ee_err:.3f} m > {self.reset_thr} m')

        # ── Publish Float64MultiArray (legacy / CBF filter path) ─────────────
        for i in range(NUM_JOINTS):
            self._out_msg.data[i] = float(self._q_ddot[i])
        self.pub.publish(self._out_msg)

        # ── Publish JointState setpoint (new C++ interface) ──────────────────
        self._sp_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(NUM_JOINTS):
            self._sp_msg.position[i] = float(self._q_d[i])
            self._sp_msg.velocity[i] = float(self._dq_d[i])
            self._sp_msg.effort[i]   = float(self._q_ddot[i])
        self._sp_pub.publish(self._sp_msg)

        # ── High-rate CSV log (one row per tick; no-op if logger disabled) ───
        if self._csv_writer is not None:
            self._compute_tau_des()                 # fills self._tau_des
            self._log_data(t, js['q'], qdot, p_d, v_d, a_d,
                           s, s_dot, s_ddot, w, ee_err)

        if self._tlog.due(t):
            en = float(np.linalg.norm(self._e6[:3]))
            qn = float(np.linalg.norm(self._q_ddot))
            seg = 'APPROACH' if s < self._approach_span else 'TRACK'
            self._tlog.info(
                f'[{seg} s={s:.3f} t={t:.1f}s] '
                f'p=[{vec_to_str(self._p_ee)}] p_d=[{vec_to_str(p_d)}] '
                f'|e|={en:.4f} m  |qddot|={qn:.3f} rad/s²')

    # ── Trajectory start-up (replaces the APPROACH/TRACK state machine) ─────

    def _current_ee_position(self, js: dict) -> np.ndarray:
        """FK of the measured joint state → EE position in the control frame."""
        np.copyto(self._q_full, js['q_full'])
        pin.forwardKinematics(self.pin_model, self.pin_data, self._q_full)
        pin.updateFramePlacements(self.pin_model, self.pin_data)
        oMee = self.pin_data.oMf[self.ee_frame_id]
        if self._use_plane_frame:
            p_raw, _ = transform_ee_to_frame(
                self.pin_model, self.pin_data, self._plane_frame_id, oMee,
                np.zeros((6, NUM_JOINTS)))
            return np.asarray(p_raw).copy()
        return np.asarray(oMee.translation).copy()

    def _build_timing_law(self):
        """Construct the selected timing law.  Steady phase rate = 1/cycle_time
        so a completed soft-start reproduces the original constant-speed pentagon."""
        rate = 1.0 / self.cycle_time
        name = self.timing_law_name
        if name == 'exponential':
            self.get_logger().warn(
                'timing_law=exponential is a one-shot ease (settles at the '
                'pentagon start and does NOT cycle continuously). '
                'Use linear/trapezoidal for cyclic tracking.')
            k = 1.0 / max(1e-3, self.approach_time)
            return ExponentialTimingLaw(k=k, s_target=self._approach_span)
        if name == 'trapezoidal':
            accel = rate / max(1e-3, self.approach_time)   # reach cruise in ~approach_time
            return TrapezoidalTimingLaw(rate=rate, accel=accel)
        if name != 'linear':
            self.get_logger().warn(f'Unknown timing_law "{name}" → using linear')
        return LinearTimingLaw(rate=rate, soft_start_s=self.approach_time)

    def _start_trajectory(self, js: dict) -> None:
        """Assemble the phase-driven path + timing law rooted at the current EE.

        The composite path begins exactly at the current EE position (no jump),
        runs a straight-line lead-in to the pentagon start over phase span
        ``S_a = approach_time / cycle_time``, then tracks the cyclic pentagon.
        """
        p0 = self._current_ee_position(js)

        # Target the *actual* pentagon entry (phase 0 = blend entry point), not
        # the geometric vertex, so the approach→pentagon seam is continuous.
        entry = self._pentagon_path.position(0.0).copy()

        # Approach span in phase units (constant phase rate ⇒ S_a·cycle_time wall-s)
        self._approach_span = max(0.0, self.approach_time) / self.cycle_time

        if self._approach_span > 0.0:
            approach = LinearSegmentPath(p0, entry, span=self._approach_span)
            self._path = CompositePath([
                (approach, self._approach_span),
                (self._pentagon_path, math.inf),
            ])
        else:
            self._path = CompositePath([(self._pentagon_path, math.inf)])

        self._timing = self._build_timing_law()
        self._timing.reset(0.0)

        # Seed joint-space integration from the current measured state
        np.copyto(self._q_d,  js['q'])
        np.copyto(self._dq_d, js['qdot'])

        self._running = True
        self.get_logger().info(
            f'Trajectory start: approach {vec_to_str(p0)} → '
            f'{vec_to_str(entry)}  (S_a={self._approach_span:.3f}, '
            f'timing={self.timing_law_name})')

    # ── High-rate CSV logging ──────────────────────────────────────────────

    def _init_logger(self, effort_topic: str) -> None:
        """Open a timestamped CSV file once and write the header row.

        On any failure the node keeps running with logging disabled
        (``self._csv_writer is None``) after emitting a warning.
        """
        # Disabled by default until the file is successfully opened.
        self._log_file = None
        self._csv_writer = None

        # Column layout — keep in sync with _log_data().
        header = ['time']
        header += [f'q{i}'          for i in range(1, 8)]   # real positions
        header += [f'dq{i}'         for i in range(1, 8)]   # real velocities
        header += [f'qddot_des_{i}' for i in range(1, 8)]   # commanded accel
        header += [f'q_des_{i}'     for i in range(1, 8)]   # integrated q_d
        header += [f'dq_des_{i}'    for i in range(1, 8)]   # integrated dq_d
        header += ['ee_x', 'ee_y', 'ee_z']                  # real EE position
        header += ['ee_x_des', 'ee_y_des', 'ee_z_des']      # desired EE position
        header += ['vx_des', 'vy_des', 'vz_des']            # desired Cart. velocity
        header += ['ax_des', 'ay_des', 'az_des']            # desired Cart. accel
        header += ['ex', 'ey', 'ez']                        # Cartesian error
        header += ['ee_error_norm']                         # ‖error‖
        header += ['s', 's_dot', 's_ddot']                  # phase variable
        header += ['manipulability', 'lambda_sq']           # conditioning
        header += [f'tau_des_{i}'   for i in range(1, 8)]   # desired torque
        header += [f'tau_meas_{i}'  for i in range(1, 8)]   # measured torque

        # Pre-allocated row buffer (filled in place every tick → no allocation).
        self._log_row = [0.0] * len(header)

        try:
            log_dir = Path(self._log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = log_dir / f'pentagon_run_{stamp}.csv'
            # 1 MiB buffer keeps disk I/O off the control path; newline=''
            # is required for the csv module on all platforms.
            self._log_file = open(path, 'w', newline='', buffering=1 << 20)
            self._csv_writer = csv.writer(self._log_file)
            self._csv_writer.writerow(header)
            self.get_logger().info(
                f'Logging to {path}  (effort topic: {effort_topic})')
        except Exception as exc:  # noqa: BLE001
            self._log_file = None
            self._csv_writer = None
            self.get_logger().warn(
                f'Could not open log file in {self._log_dir}: {exc} — '
                f'continuing without logging.')

    def _compute_tau_des(self) -> None:
        """tau_des = M(q)·q̈_des + nle(q, q̇), written into self._tau_des.

        Reuses M and nle already computed by computeAllTerms() this tick.
        CRBA only fills M's upper triangle, so mirror it to the lower triangle
        before the matrix-vector product.
        """
        M = self.pin_data.M
        M[self._M_tril] = M.T[self._M_tril]            # symmetrise in place
        self._qddot_full_des[:] = 0.0
        for k, vid in enumerate(self._arm_v_ids):
            self._qddot_full_des[vid] = self._q_ddot[k]
        np.dot(M, self._qddot_full_des, out=self._tau_full)
        self._tau_full += self.pin_data.nle
        for k, vid in enumerate(self._arm_v_ids):
            self._tau_des[k] = self._tau_full[vid]

    def _log_data(self, t, q, dq, p_d, v_d, a_d,
                  s, s_dot, s_ddot, w, ee_err) -> None:
        """Fill the pre-allocated row buffer and write it to the CSV file."""
        row = self._log_row
        o = 0
        row[o] = float(t);                                       o += 1
        for i in range(NUM_JOINTS): row[o + i] = float(q[i]);            # real q
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(dq[i]);           # real dq
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._q_ddot[i])  # q̈_des
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._q_d[i])     # q_des
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._dq_d[i])    # dq_des
        o += NUM_JOINTS
        row[o] = float(self._p_ee[0]); row[o+1] = float(self._p_ee[1]); row[o+2] = float(self._p_ee[2]); o += 3
        row[o] = float(p_d[0]);        row[o+1] = float(p_d[1]);        row[o+2] = float(p_d[2]);        o += 3
        row[o] = float(v_d[0]);        row[o+1] = float(v_d[1]);        row[o+2] = float(v_d[2]);        o += 3
        row[o] = float(a_d[0]);        row[o+1] = float(a_d[1]);        row[o+2] = float(a_d[2]);        o += 3
        row[o] = float(self._e6[0]);   row[o+1] = float(self._e6[1]);   row[o+2] = float(self._e6[2]);   o += 3
        row[o] = float(ee_err);                                  o += 1
        row[o] = float(s); row[o+1] = float(s_dot); row[o+2] = float(s_ddot); o += 3
        row[o] = float(w); row[o+1] = float(self._lambda_sq);    o += 2
        for i in range(NUM_JOINTS): row[o + i] = float(self._tau_des[i])  # tau_des
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._tau_meas[i]) # tau_meas
        o += NUM_JOINTS

        self._csv_writer.writerow(row)

    def _close_logger(self) -> None:
        """Flush and close the CSV file (idempotent)."""
        if self._log_file is not None:
            try:
                self._log_file.flush()
                self._log_file.close()
                self.get_logger().info('Log file closed.')
            except Exception:  # noqa: BLE001
                pass
            self._log_file = None
            self._csv_writer = None

    def destroy_node(self):
        """Ensure the log file is closed on node destruction."""
        self._close_logger()
        return super().destroy_node()


def main(args=None):
    run_node_main(PentagonQddotCommander, args=args)


if __name__ == '__main__':
    main()