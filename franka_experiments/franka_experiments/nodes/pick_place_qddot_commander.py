#!/usr/bin/env python3
"""Cyclic Cartesian pick-and-place task for the Franka FR3 (Acceleration-Space Pipeline).

The node executes the trapezoidal pick-and-place sequence in the ``fr3_link0`` frame:
    home -> pick -> drop_down -> pick -> transfer -> place -> drop_down -> place -> home -> wait

Features:
- Fixed vertical end-effector orientation (pointing downwards parallel to base -Z).
- Operational Space Control (OSC) accelerations (6D Cartesian tracking + null-space posture).
- Robust reference integration, velocity/position synchronization, and two-level anti-windup.
- Clean null-space posture targeting a nominal neutral configuration to prevent joint fighting and base drift.

DOF budget: the 6D task (3 position + 3 orientation) uses 6 of the 7 joints;
the single redundant DOF (elbow self-motion) is left to the null-space posture.

The published q̈ is only half of the loop. It must also reach the 1 kHz joint
PD of rt_torque_controller (its ``accel_topic``): the open-loop feedforward
τ = M·q̈ + C·q̇ alone is too weak on the low-inertia wrist joints to beat their
static friction, and the EE orientation drifts. easy_torque.launch.py wires it.
"""

from __future__ import annotations

import os
import math
import threading
import time
from typing import List, Optional, Tuple

import numpy as np
import pinocchio as pin

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS, AUTO_SENTINEL
from franka_experiments.utils.node_runtime import (
    get_namespace_from_config,
    run_node_main,
)
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_frame_id,
    resolve_arm_joint_ids,
    so3_log,
)
from sensor_msgs.msg import JointState as SensorJointState
from franka_experiments.utils.math_utils import cosine_ramp
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str


class TaskPhase:
    """One timed phase of the cyclic pick-and-place task."""
    def __init__(self, name: str, start: np.ndarray, goal: np.ndarray, duration: float, moving: bool):
        self.name = name
        self.start = start
        self.goal = goal
        self.duration = duration
        self.moving = moving


class PickPlaceTrajectory:
    """Timed sequence of minimum-jerk Cartesian moves and stationary waits matching the trapezoidal profile."""

    def __init__(
        self,
        home: np.ndarray,
        lateral_offset: float,
        down_offset: float,
        drop_offset: float,
        move_time: float,
        drop_time: float,
        transfer_time: float,
        return_time: float,
        wait_pick: float,
        wait_drop: float,
        wait_transfer: float,
        wait_home: float,
    ) -> None:
        home = np.asarray(home, dtype=float).copy()

        # fr3_link0 convention: +y = robot left, -y = robot right, +z = up
        pick = home + np.array([0.0, -lateral_offset, -down_offset])
        pick_down = pick - np.array([0.0, 0.0, drop_offset])
        
        place = home + np.array([0.0, lateral_offset, -down_offset])
        place_down = place - np.array([0.0, 0.0, drop_offset])

        self.home = home
        self.pick = pick
        self.pick_down = pick_down
        self.place = place
        self.place_down = place_down

        self.phases: List[TaskPhase] = [
            TaskPhase('MOVE_TO_PICK', home, pick, move_time, True),
            TaskPhase('WAIT_AT_PICK', pick, pick, wait_pick, False),
            TaskPhase('SMALL_DESCENT_1', pick, pick_down, drop_time, True),
            TaskPhase('WAIT_AT_DROP_1', pick_down, pick_down, wait_drop, False),
            TaskPhase('SMALL_ASCENT_1', pick_down, pick, drop_time, True),
            TaskPhase('WAIT_AFTER_ASCENT_1', pick, pick, wait_drop, False),
            TaskPhase('LATERAL_TRANSFER', pick, place, transfer_time, True),
            TaskPhase('WAIT_AT_PLACE', place, place, wait_transfer, False),
            TaskPhase('SMALL_DESCENT_2', place, place_down, drop_time, True),
            TaskPhase('WAIT_AT_DROP_2', place_down, place_down, wait_drop, False),
            TaskPhase('SMALL_ASCENT_2', place_down, place, drop_time, True),
            TaskPhase('WAIT_AFTER_ASCENT_2', place, place, wait_drop, False),
            TaskPhase('RETURN_HOME', place, home, return_time, True),
            TaskPhase('WAIT_AT_HOME', home, home, wait_home, False),
        ]

        for phase in self.phases:
            if phase.duration <= 0.0:
                raise ValueError(f'Phase {phase.name} must have positive duration')

        self._end_times = np.cumsum([phase.duration for phase in self.phases])
        self.cycle_time = float(self._end_times[-1])
        self._p_out = np.zeros(3)
        self._v_out = np.zeros(3)
        self._a_out = np.zeros(3)

    @staticmethod
    def _minimum_jerk(u: float) -> Tuple[float, float, float]:
        u = float(np.clip(u, 0.0, 1.0))
        u2 = u * u
        u3 = u2 * u
        u4 = u3 * u
        u5 = u4 * u
        sigma = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
        dsigma_du = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
        d2sigma_du2 = 60.0 * u - 180.0 * u2 + 120.0 * u3
        return sigma, dsigma_du, d2sigma_du2

    def evaluate(self, t: float):
        t = max(0.0, float(t))
        cycle_index = int(t // self.cycle_time)
        cycle_t = t - cycle_index * self.cycle_time

        phase_index = int(np.searchsorted(self._end_times, cycle_t, side='right'))
        if phase_index >= len(self.phases):
            phase_index = len(self.phases) - 1

        phase = self.phases[phase_index]
        phase_start_t = 0.0 if phase_index == 0 else self._end_times[phase_index - 1]
        local_t = cycle_t - phase_start_t

        if not phase.moving:
            np.copyto(self._p_out, phase.goal)
            self._v_out[:] = 0.0
            self._a_out[:] = 0.0
            return self._p_out, self._v_out, self._a_out, phase.name, cycle_index

        u = local_t / phase.duration
        sigma, dsigma_du, d2sigma_du2 = self._minimum_jerk(u)
        delta = phase.goal - phase.start

        self._p_out[:] = phase.start + sigma * delta
        self._v_out[:] = (dsigma_du / phase.duration) * delta
        self._a_out[:] = (d2sigma_du2 / (phase.duration ** 2)) * delta
        return self._p_out, self._v_out, self._a_out, phase.name, cycle_index


class PickPlaceQddotCommander(Node):

    def __init__(self):
        super().__init__('pick_place_qddot_commander')

        self.done = False
        self._stopping = False
        self._stop_end = 0.0
        self._running = False

        # ── Load config files ────────────────────────────────────────────
        config_control = os.path.join(
            get_package_share_directory("franka_experiments"),
            "config",
            "fr3_control.yaml",
        )
        config_pick = os.path.join(
            get_package_share_directory("franka_experiments"),
            "config",
            "pick_place.yaml",
        )

        # ── Load robot config (topics + per-joint limits) ────────────────
        _cfg    = load_robot_config(config_control)
        _topics = _cfg['topics']
        _limits = _cfg['joint_limits']
        _jnames = [f'joint{i}' for i in range(1, 8)]

        # ── Parameters ────────
        _pp = load_robot_config(config_pick)
        for section in _pp.values():
            for name, value in section.items():
                self.declare_parameter(name, value)

        def _f(name: str) -> float:
            return float(self.get_parameter(name).value)

        qddot_topic    = self.get_parameter('qddot_safe_topic').value
        if qddot_topic == AUTO_SENTINEL:
            qddot_topic = _topics.get('qddot_nom', '/NS_1/qddot_nom')
        q_des_topic    = self.get_parameter('q_des_topic').value
        js_topic_param = self.get_parameter('joint_state_topic').value
        ee_frame_name  = self.get_parameter('ee_frame').value
        self.rate_hz   = _f('rate_hz')
        self.warmup_s  = _f('warmup_s')
        self.ramp_s    = _f('ramp_s')

        self.lateral_offset = _f('lateral_offset')
        self.down_offset    = _f('down_offset')
        self.drop_offset    = _f('drop_offset')

        self.move_time     = _f('move_time')
        self.drop_time     = _f('drop_time')
        self.transfer_time = _f('transfer_time')
        self.return_time   = _f('return_time')
        self.wait_pick     = _f('wait_pick')
        self.wait_drop     = _f('wait_drop')
        self.wait_transfer = _f('wait_transfer')
        self.wait_home     = _f('wait_home')

        self.kp        = _f('kp_cart')
        self.kd        = _f('kd_cart')
        self.kp_rot    = _f('kp_rot')
        self.kd_rot    = _f('kd_rot')

        self.k_sync_pos       = _f('k_sync_pos')
        self.k_sync_vel       = _f('k_sync_vel')
        self.soft_reset_thr   = _f('soft_reset_thr')
        self.hard_reset_thr   = _f('hard_reset_thr')
        self.soft_reset_alpha = _f('soft_reset_alpha')
        self.k_null           = _f('k_null')
        self.d_null           = _f('d_null')
        self._lambda_sq_min   = _f('lambda_sq_min')
        self._lambda_sq_max   = _f('lambda_sq_max')
        self._manip_thr       = _f('manip_thr')
        self.cart_err_max     = _f('cart_err_max')

        self.q_des_max_error  = _f('q_des_max_error')
        self.dq_des_max       = _f('dq_des_max')
        self._dt              = 1.0 / self.rate_hz

        # Neutral reference posture for the null space
        self._q_home_cfg = np.array(self.get_parameter('q_home').value, dtype=np.float64)
        if self._q_home_cfg.shape != (NUM_JOINTS,):
            self.get_logger().error(f'q_home must have {NUM_JOINTS} entries')
            raise SystemExit(1)

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

        try:
            self._pin_joint_ids = resolve_arm_joint_ids(self.pin_model)
        except RuntimeError as exc:
            self.get_logger().error(str(exc)); raise SystemExit(1) from exc

        self._arm_v_ids = [self.pin_model.joints[p].idx_v for p in self._pin_joint_ids]

        self.trajectory: Optional[PickPlaceTrajectory] = None

        # ── Pre-allocated buffers ─────────────────────────────────────────
        nv = self.pin_model.nv
        self._q_neutral      = pin.neutral(self.pin_model)
        self._q_full         = pin.neutral(self.pin_model)
        self._qdot_full      = np.zeros(nv)
        self._J6n            = np.zeros((6, nv))
        self._dJ6n           = np.zeros((6, nv))
        self._J_arm          = np.zeros((6, NUM_JOINTS))
        self._dJ_arm         = np.zeros((6, NUM_JOINTS))
        self._p_ee           = np.zeros(3)
        
        # Fixed Vertical Orientation: EE points downwards (-Z of base frame)
        self._R_des          = np.array([
            [1.0,  0.0,  0.0],
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0]
        ])

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
        self._lambda_sq      = self._lambda_sq_min

        self._I7             = np.eye(NUM_JOINTS)
        self._N              = np.zeros((NUM_JOINTS, NUM_JOINTS))
        self._JpinvJ         = np.zeros((NUM_JOINTS, NUM_JOINTS))
        self._qddot_task     = np.zeros(NUM_JOINTS)
        self._qddot_null     = np.zeros(NUM_JOINTS)
        self._null_proj      = np.zeros(NUM_JOINTS)
        self._q_home         = np.zeros(NUM_JOINTS)
        self._tmp7           = np.zeros(NUM_JOINTS)

        self._sync_vel       = np.zeros(NUM_JOINTS)
        self._sync_pos       = np.zeros(NUM_JOINTS)
        self._q_lo           = np.zeros(NUM_JOINTS)
        self._q_hi           = np.zeros(NUM_JOINTS)

        self._q_d            = np.zeros(NUM_JOINTS)
        self._dq_d           = np.zeros(NUM_JOINTS)

        self._prev_tick_time = None
        self._last_phase     = None
        self._last_cycle     = -1

        # Messages
        self._sp_msg         = SensorJointState()
        self._sp_msg.name    = list(FR3_JOINT_NAMES)
        self._sp_msg.position = [0.0] * NUM_JOINTS
        self._sp_msg.velocity = [0.0] * NUM_JOINTS
        self._sp_msg.effort   = [0.0] * NUM_JOINTS

        self._out_msg        = Float64MultiArray()
        self._out_msg.data   = [0.0] * NUM_JOINTS
        self._zero_msg       = Float64MultiArray()
        self._zero_msg.data  = [0.0] * NUM_JOINTS

        # ── Joint state subscriber ────────────────────────────────────────
        self._js_lock  = threading.Lock()
        self._js_a     = {'q': np.zeros(NUM_JOINTS), 'qdot': np.zeros(NUM_JOINTS),
                          'q_full': pin.neutral(self.pin_model), 'valid': False}
        self._js_b     = {'q': np.zeros(NUM_JOINTS), 'qdot': np.zeros(NUM_JOINTS),
                          'q_full': pin.neutral(self.pin_model), 'valid': False}
        self._js_write = self._js_a
        self._js_read  = self._js_b
        self._js_stamp = self.get_clock().now()
        self._js_imap: Optional[List[int]] = None
        self._js_names: List[str] = []

        js_topic = js_topic_param
        if js_topic == AUTO_SENTINEL:
            # joint_states_fast = the joint_state_broadcaster's own 1 kHz output
            js_topic = _topics.get('joint_states_fast')
            if not js_topic:
                ns = get_namespace_from_config()
                js_topic = f'/{ns}/joint_states' if ns else '/joint_states'

        # BEST_EFFORT, depth 1: with shared memory off (fastdds_no_shm.xml)
        # every 1 kHz sample travels the UDP loopback, the same kernel path as
        # the FCI loop. Best-effort drops the reliable ACK/NACK/heartbeat
        # traffic, and only the latest state is ever read anyway.
        js_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self._js_sub = self.create_subscription(JointState, js_topic, self._js_cb, js_qos)
        self.pub     = self.create_publisher(Float64MultiArray, qddot_topic, 10)
        self._sp_pub = self.create_publisher(SensorJointState,  q_des_topic,  10)
        self.timer   = self.create_timer(self._dt, self._tick)
        self.t0      = self.get_clock().now()
        self._tlog   = ThrottledLogger(self.get_logger())

        self.get_logger().info(
            f'pick_place_qddot_commander started\n'
            f'  topic    : {qddot_topic}\n'
            f'  js_topic : {js_topic}\n'
            f'  R_des    : EE z along base -z (6D task, 1 redundant DOF)\n'
            f'  rate     : {self.rate_hz} Hz\n'
            f'  offsets  : lateral={self.lateral_offset}, down={self.down_offset}, drop={self.drop_offset}')

    def _js_cb(self, msg: JointState) -> None:
        # Re-map whenever the name list changes
        if self._js_imap is None or self._js_names != msg.name:
            try:
                self._js_imap = [msg.name.index(jn) for jn in FR3_JOINT_NAMES]
            except ValueError:
                self._js_imap = None
                return
            self._js_names = list(msg.name)
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

    def request_stop(self, dur: float = 0.5):
        if self._stopping:
            return
        self._stopping = True
        self._stop_end = time.monotonic() + dur
        self.get_logger().info(f'Stopping: zero qddot for {dur} s')

    def _tick(self):
        if self._stopping:
            self.pub.publish(self._zero_msg)
            if time.monotonic() >= self._stop_end:
                self.timer.cancel()
                self.done = True
            return

        now_stamp = self.get_clock().now()
        t = (now_stamp - self.t0).nanoseconds * 1e-9

        if self._prev_tick_time is not None:
            actual_dt = float((now_stamp - self._prev_tick_time).nanoseconds * 1e-9)
            actual_dt = float(np.clip(actual_dt, self._dt * 0.5, self._dt * 2.0))
        else:
            actual_dt = self._dt
        self._prev_tick_time = now_stamp

        # Warm-up phase
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

        np.copyto(self._q_full, js['q_full'])
        self._qdot_full[:] = 0.0
        for k, vid in enumerate(self._arm_v_ids):
            self._qdot_full[vid] = js['qdot'][k]
        qdot = js['qdot']

        # Only kinematics is needed: J, dJ and frame placements (no dynamics terms)
        pin.computeJointJacobiansTimeVariation(self.pin_model, self.pin_data, self._q_full, self._qdot_full)
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
        np.copyto(self._p_ee, oMee.translation)
        R_cur = np.asarray(oMee.rotation)

        # ── Fixed Vertical Orientation ──
        np.dot(self._R_des.T, R_cur, out=self._R_err)
        np.dot(self._R_des, so3_log(self._R_err), out=self._e_rot)
        np.negative(self._e_rot, out=self._e_rot)

        # Task trajectory evaluation
        task_t = (now_stamp - self._task_start).nanoseconds * 1e-9
        envelope = cosine_ramp(task_t, self.ramp_s)
        t_traj = max(0.0, task_t - self.ramp_s)

        p_des, v_des, a_des, phase, cycle = self.trajectory.evaluate(t_traj)
        if t_traj == 0.0:
            phase = "STARTUP_RAMP"

        if phase != self._last_phase or cycle != self._last_cycle:
            self.get_logger().info(f'Cycle {cycle + 1} | phase: {phase}')
            self._last_phase = phase
            self._last_cycle = cycle

        # 6D Error
        np.subtract(p_des, self._p_ee, out=self._tmp3)
        ee_err = float(np.linalg.norm(self._tmp3))
        if self.cart_err_max > 0.0 and ee_err > self.cart_err_max:
            self._tmp3 *= (self.cart_err_max / ee_err)
        self._e6[:3] = self._tmp3
        self._e6[3:] = self._e_rot

        # Velocity error
        np.dot(self._J_arm, qdot, out=self._edot6)
        np.subtract(v_des, self._edot6[:3], out=self._edot6[:3])
        np.negative(self._edot6[3:], out=self._edot6[3:])

        # Task acceleration law: ẍ = a_d + Kp*e + Kd*edot
        self._xddot6[:3] = a_des
        np.multiply(self.kp,     self._e6[:3],    out=self._tmp3)
        self._xddot6[:3] += self._tmp3
        np.multiply(self.kd,     self._edot6[:3], out=self._tmp3)
        self._xddot6[:3] += self._tmp3
        np.multiply(self.kp_rot, self._e6[3:],    out=self._xddot6[3:])
        np.multiply(self.kd_rot, self._edot6[3:], out=self._tmp3)
        self._xddot6[3:] += self._tmp3

        # Adaptive Damped Least-Squares
        np.dot(self._J_arm, self._J_arm.T, out=self._JJT)
        w = math.sqrt(max(0.0, float(np.linalg.det(self._JJT))))
        f_w = (1.0 - w / self._manip_thr) ** 2 if w < self._manip_thr else 0.0
        self._lambda_sq = self._lambda_sq_min + (self._lambda_sq_max - self._lambda_sq_min) * f_w
        
        np.copyto(self._JJT_reg, self._JJT)
        for i in range(6):
            self._JJT_reg[i, i] += self._lambda_sq

        self._J_pinv[:] = np.linalg.solve(self._JJT_reg, self._J_arm).T

        np.dot(self._dJ_arm, qdot, out=self._tmp6)
        np.subtract(self._xddot6, self._tmp6, out=self._xddot6)
        np.dot(self._J_pinv, self._xddot6, out=self._qddot_task)

        # Null-space posture control (ancoraggio base a 0 e richiamo a posa neutra sicura)
        np.dot(self._J_pinv, self._J_arm, out=self._JpinvJ)
        np.subtract(self._I7, self._JpinvJ, out=self._N)
        np.subtract(js['q'], self._q_home, out=self._qddot_null)
        self._qddot_null *= -self.k_null
        np.multiply(self.d_null, qdot, out=self._tmp7)
        self._qddot_null -= self._tmp7
        np.dot(self._N, self._qddot_null, out=self._null_proj)

        np.add(self._qddot_task, self._null_proj, out=self._q_ddot)
        # Uniform scaling instead of a per-joint clip: clipping one joint changes the DIRECTION of q̈
        np.abs(self._q_ddot, out=self._tmp7)
        np.divide(self.qddot_max, np.maximum(self._tmp7, 1e-12), out=self._tmp7)
        scale = float(self._tmp7.min())
        if scale < 1.0:
            self._q_ddot *= scale

        # Apply envelope ramp
        self._q_ddot *= envelope

        # ── Bounded reference integration (Pentagon pattern) ──────────────
        self._dq_d += self._q_ddot * actual_dt
        np.subtract(qdot, self._dq_d, out=self._sync_vel)
        self._sync_vel *= (self.k_sync_vel * actual_dt)
        self._dq_d += self._sync_vel
        np.clip(self._dq_d, -self.dq_des_max, self.dq_des_max, out=self._dq_d)

        self._q_d += self._dq_d * actual_dt
        np.subtract(js['q'], self._q_d, out=self._sync_pos)
        self._sync_pos *= (self.k_sync_pos * actual_dt)
        self._q_d += self._sync_pos

        np.subtract(js['q'], self.q_des_max_error, out=self._q_lo)
        np.add(js['q'],      self.q_des_max_error, out=self._q_hi)
        np.clip(self._q_d, self._q_lo, self._q_hi, out=self._q_d)

        # ── Two-level anti-windup on Cartesian error (Pentagon pattern) ───
        if ee_err > self.hard_reset_thr:
            np.copyto(self._q_d,  js['q'])
            np.copyto(self._dq_d, qdot)
        elif ee_err > self.soft_reset_thr:
            a = self.soft_reset_alpha
            self._q_d  *= a; self._q_d  += (1.0 - a) * js['q']
            self._dq_d *= a; self._dq_d += (1.0 - a) * qdot

        # Publish qddot output for the safety filter
        for i in range(NUM_JOINTS):
            self._out_msg.data[i] = float(self._q_ddot[i])
        self.pub.publish(self._out_msg)

        # Publish JointState setpoint
        self._sp_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(NUM_JOINTS):
            self._sp_msg.position[i] = float(self._q_d[i])
            self._sp_msg.velocity[i] = float(self._dq_d[i])
            self._sp_msg.effort[i]   = float(self._q_ddot[i])
        self._sp_pub.publish(self._sp_msg)

    def _start_trajectory(self, js: dict) -> None:
        np.copyto(self._q_full, js['q_full'])
        pin.forwardKinematics(self.pin_model, self.pin_data, self._q_full)
        pin.updateFramePlacements(self.pin_model, self.pin_data)
        oMee = self.pin_data.oMf[self.ee_frame_id]
        home = np.asarray(oMee.translation).copy()

        self.trajectory = PickPlaceTrajectory(
            home=home,
            lateral_offset=self.lateral_offset,
            down_offset=self.down_offset,
            drop_offset=self.drop_offset,
            move_time=self.move_time,
            drop_time=self.drop_time,
            transfer_time=self.transfer_time,
            return_time=self.return_time,
            wait_pick=self.wait_pick,
            wait_drop=self.wait_drop,
            wait_transfer=self.wait_transfer,
            wait_home=self.wait_home,
        )

        np.copyto(self._q_d,  js['q'])
        np.copyto(self._dq_d, js['qdot'])
        
        np.copyto(self._q_home, self._q_home_cfg)

        self._task_start = self.get_clock().now()
        self._running = True
        self.get_logger().info(f'Pick & Place trajectory started at home: {vec_to_str(home)}')


def main(args=None):
    run_node_main(PickPlaceQddotCommander, args=args)

if __name__ == '__main__':
    main()