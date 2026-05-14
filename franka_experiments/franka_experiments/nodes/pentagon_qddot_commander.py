#!/usr/bin/env python3
"""Pentagon trajectory for cbf_torque_controller — no CBF filter.

State machine
─────────────
WARMUP   — publish zeros, wait for a valid joint state.
APPROACH — exponential reference filter: the desired position slides from the
           current EE position toward start_xyz with time constant approach_tau.
           The robot follows this moving target with a low-gain PD law.
           Smooth by construction: position/velocity are continuous, no
           impulsive commands even if the robot is far from start_xyz.
TRACK    — cyclic PentagonTrajectory (C¹ Bézier corner blending) with PD +
           velocity feedforward.  Pentagon is traversed continuously with no
           stop at vertices.

Safety: qddot output is clamped to ±qddot_max per joint at every tick.

Transition APPROACH → TRACK: when |p_ee − start_xyz| < pos_thr.
"""

from __future__ import annotations

import math
import threading
import time
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
from franka_experiments.utils.trajectory import PentagonTrajectory
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str

_WARMUP   = 'WARMUP'
_APPROACH = 'APPROACH'
_TRACK    = 'TRACK'


class PentagonQddotCommander(Node):

    def __init__(self):
        super().__init__('pentagon_qddot_commander')

        self.done       = False
        self._stopping  = False
        self._stop_end  = 0.0
        self._phase     = _WARMUP

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
        self.declare_parameter('approach_tau',        2.5)   # exp-filter time constant [s]
        self.declare_parameter('approach_timeout_s',  5.0)   # safety fallback to TRACK [s]
        self.declare_parameter('pos_thr',             0.04)  # APPROACH→TRACK threshold [m]
        self.declare_parameter('center_xyz',          [0.4, 0.0, 0.4])
        self.declare_parameter('radius',              0.30)
        self.declare_parameter('plane',               'front')
        self.declare_parameter('plane_frame',         'fr3_link0')
        self.declare_parameter('cycle_time',          15.0)
        self.declare_parameter('smoothness',          0.20)
        # kp/kd: task-space Cartesian gains [N/m, N·s/m].
        # Conservative for torque control — overshoot propagates via M·q̈.
        # kd ≈ 2√kp for near-critical damping.
        self.declare_parameter('kp_cart',             20.0)
        self.declare_parameter('kd_cart',              9.0)
        self.declare_parameter('kp_rot',              10.0)
        self.declare_parameter('kd_rot',               6.0)

        qddot_topic    = self.get_parameter('qddot_safe_topic').value
        q_des_topic    = self.get_parameter('q_des_topic').value
        self.reset_thr = float(self.get_parameter('reset_thr_m').value)
        js_topic_param = self.get_parameter('joint_state_topic').value
        ee_frame_name  = self.get_parameter('ee_frame').value
        self.rate_hz   = float(self.get_parameter('rate_hz').value)
        self.warmup_s  = float(self.get_parameter('warmup_s').value)
        self.approach_tau     = float(self.get_parameter('approach_tau').value)
        self.approach_timeout = float(self.get_parameter('approach_timeout_s').value)
        self.pos_thr          = float(self.get_parameter('pos_thr').value)
        center_xyz     = list(self.get_parameter('center_xyz').value)
        radius         = float(self.get_parameter('radius').value)
        plane          = self.get_parameter('plane').value
        plane_frame    = self.get_parameter('plane_frame').value
        cycle_time     = float(self.get_parameter('cycle_time').value)
        smoothness     = float(self.get_parameter('smoothness').value)
        self.kp        = float(self.get_parameter('kp_cart').value)
        self.kd        = float(self.get_parameter('kd_cart').value)
        self.kp_rot    = float(self.get_parameter('kp_rot').value)
        self.kd_rot    = float(self.get_parameter('kd_rot').value)
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

        # ── Trajectory ───────────────────────────────────────────────────
        self.traj = PentagonTrajectory(
            center=np.array(center_xyz), radius=radius,
            plane=plane, cycle_time=cycle_time, smoothness=smoothness,
        )
        # Start point = vertex 0 (top of pentagon)
        self._start_xyz = self.traj.vertices[0].copy()

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

        # Approach state (exponential filter + timeout)
        self._p_ref          = np.zeros(3)
        self._p_ref_prev     = np.zeros(3)
        self._approach_start = 0.0           # time.monotonic() at APPROACH entry

        # Track state + acceleration feedforward buffer
        self._track_t0       = 0.0
        self._a_d_buf        = np.zeros(3)

        # Joint-space integration buffers for q_d, dq_d
        self._q_d            = np.zeros(NUM_JOINTS)
        self._dq_d           = np.zeros(NUM_JOINTS)

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

        # ── Publisher / timer ─────────────────────────────────────────────
        self.pub      = self.create_publisher(Float64MultiArray, qddot_topic, 10)
        self._sp_pub  = self.create_publisher(SensorJointState,  q_des_topic,  10)
        self.timer    = self.create_timer(self._dt, self._tick)
        self.t0    = self.get_clock().now()
        self._tlog = ThrottledLogger(self.get_logger())

        self.get_logger().info(
            f'pentagon_qddot_commander\n'
            f'  topic    : {qddot_topic}\n'
            f'  js_topic : {js_topic}\n'
            f'  center   : {center_xyz}  r={radius} m  plane={plane}\n'
            f'  start_xyz: {self._start_xyz.tolist()}\n'
            f'  cycle_t  : {cycle_time} s  Kp={self.kp} Kd={self.kd}\n'
            f'  approach_tau={self.approach_tau} s  pos_thr={self.pos_thr} m\n'
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

        # WARMUP
        if self._phase == _WARMUP:
            self.pub.publish(self._zero_msg)
            if t >= self.warmup_s:
                with self._js_lock:
                    js = self._js_read
                if js['valid']:
                    self._begin_approach(js)
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

        # Phase-specific desired position + velocity + acceleration feedforward
        a_d = self._a_d_buf
        if self._phase == _APPROACH:
            p_d, v_d = self._approach_step(actual_dt)
            a_d[:] = 0.0
            pos_err = float(np.linalg.norm(self._p_ee - self._start_xyz))
            timeout = (time.monotonic() - self._approach_start) > self.approach_timeout
            if pos_err < self.pos_thr or timeout:
                reason = 'timeout' if timeout else f'|e|={pos_err:.4f} m'
                self.get_logger().info(f'APPROACH done ({reason}) → TRACK')
                self._track_t0 = t
                self._phase    = _TRACK
        else:
            tr = t - self._track_t0
            p_d, v_d, a_d_traj = self.traj.evaluate_with_accel(tr)
            np.copyto(a_d, a_d_traj)

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

        if self._tlog.due(t):
            en = float(np.linalg.norm(self._e6[:3]))
            qn = float(np.linalg.norm(self._q_ddot))
            self._tlog.info(
                f'[{self._phase} t={t:.1f}s] '
                f'p=[{vec_to_str(self._p_ee)}] p_d=[{vec_to_str(p_d)}] '
                f'|e|={en:.4f} m  |qddot|={qn:.3f} rad/s²')

    # ── Approach helpers ──────────────────────────────────────────────────

    def _begin_approach(self, js: dict) -> None:
        """Initialise the exponential filter at the current EE position."""
        np.copyto(self._q_full, js['q_full'])
        pin.forwardKinematics(self.pin_model, self.pin_data, self._q_full)
        pin.updateFramePlacements(self.pin_model, self.pin_data)
        oMee = self.pin_data.oMf[self.ee_frame_id]
        if self._use_plane_frame:
            p_raw, _ = transform_ee_to_frame(
                self.pin_model, self.pin_data, self._plane_frame_id, oMee,
                np.zeros((6, NUM_JOINTS)))
            np.copyto(self._p_ref, p_raw)
        else:
            np.copyto(self._p_ref, oMee.translation)
        np.copyto(self._p_ref_prev, self._p_ref)
        # Seed joint-space integration from current state
        np.copyto(self._q_d,  js['q'])
        np.copyto(self._dq_d, js['qdot'])
        self._approach_start = time.monotonic()
        self._phase = _APPROACH
        self.get_logger().info(
            f'APPROACH: {vec_to_str(self._p_ref)} → {vec_to_str(self._start_xyz)}'
            f'  tau={self.approach_tau}s')

    def _approach_step(self, actual_dt: float):
        """Update exponential reference filter; return (p_d, v_d).

        Uses the actual measured tick period so the approach speed is
        independent of Python timer jitter.
        """
        alpha = 1.0 - math.exp(-actual_dt / self.approach_tau)
        np.copyto(self._p_ref_prev, self._p_ref)
        self._p_ref += alpha * (self._start_xyz - self._p_ref)
        # Finite-difference velocity consistent with the actual step taken
        v_d = (self._p_ref - self._p_ref_prev) / actual_dt
        return self._p_ref, v_d


def main(args=None):
    run_node_main(PentagonQddotCommander, args=args)


if __name__ == '__main__':
    main()
