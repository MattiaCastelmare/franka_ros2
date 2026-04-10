#!/usr/bin/env python3
"""End-effector circle trajectory tracker via joint-velocity commands.

Publishes ``std_msgs/Float64MultiArray`` (7 joints) to the
``tracking_qdot`` topic (consumed by the **rt_velocity_blender_controller**)
using Pinocchio for Jacobian-based resolved-rate control.

The end-effector traces a circle on a configurable plane around a given
centre.  A quintic (5th-order) polynomial ramp is integrated into the
trajectory phase to guarantee C2 continuity (zero velocity AND acceleration)
at startup and shutdown.

Parameters
----------
See module-level ``declare_parameter`` calls for the full list.

Shutdown
--------
On SIGINT / SIGTERM / Ctrl-C the node ramps joint velocities to zero using a
reverse quintic over 1.0 s, publishes true zeros for 0.5 s, then sets
``self.done = True`` so the executor loop in ``main()`` exits cleanly.
"""

from __future__ import annotations

import math
import time
from typing import List

import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.constants import NUM_JOINTS, AUTO_SENTINEL
from franka_experiments.utils.ros import (
    resolve_tracking_topic,
    resolve_topic_with_deprecated_alias,
    run_node_main,
)
from franka_experiments.utils.math_utils import clamp_joints, lpf
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_frame_id,
    resolve_arm_joint_ids,
    compute_ee_fk,
    compute_arm_jacobian,
    transform_ee_to_frame,
    dls_solve,
    JointStateManager,
)
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str

# ---------------------------------------------------------------------------
# Module-level default (resolved once at import time)
# ---------------------------------------------------------------------------
DEFAULT_TOPIC = resolve_tracking_topic()


# ---------------------------------------------------------------------------
# Quintic ramp — C2-continuous: s(0)=0, s'(0)=0, s''(0)=0,
#                               s(T)=1, s'(T)=0, s''(T)=0
# ---------------------------------------------------------------------------

def quintic_ramp(t: float, T: float) -> float:
    """C2-continuous ramp from 0 to 1 over [0, T]."""
    if t <= 0.0:
        return 0.0
    if t >= T:
        return 1.0
    tau = t / T
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def quintic_ramp_integral(t: float, T: float) -> float:
    """Definite integral ∫₀ᵗ quintic_ramp(τ, T) dτ.

    Used to compute the smoothly-ramped phase: φ(t) = ω · ∫₀ᵗ s(τ) dτ.
    At t=T the integral equals T/2; for t>T it grows linearly as t - T/2.
    """
    if t <= 0.0:
        return 0.0
    if t >= T:
        return t - T / 2.0  # T/2 (integral over [0,T]) + (t-T)·1
    tau = t / T
    return T * (10.0 * tau**4 / 4.0 - 15.0 * tau**5 / 5.0 + 6.0 * tau**6 / 6.0)


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class EECircleVelocityCommander(Node):
    """Resolved-rate circle tracker publishing joint velocities."""

    _STOP_RAMP = 'ramp'
    _STOP_ZEROS = 'zeros'

    def __init__(self):
        super().__init__('ee_circle_velocity_commander')

        # ---- Done flag (checked by main-loop executor) ------------------
        self.done = False

        # ---- Stopping state machine -------------------------------------
        self._stopping = False
        self._stop_phase = self._STOP_RAMP
        self._stop_t0 = 0.0
        self._stop_zero_t0 = 0.0
        self._qdot_frozen = np.zeros(NUM_JOINTS)

        # ---- Declare parameters -----------------------------------------
        self.declare_parameter('tracking_topic', DEFAULT_TOPIC)
        self.declare_parameter('command_topic', '')  # deprecated alias
        self.declare_parameter('joint_state_topic', AUTO_SENTINEL)
        self.declare_parameter('ee_frame', 'fr3_hand_tcp')
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('warmup_s', 2.0)
        self.declare_parameter('ramp_s', 3.0)
        self.declare_parameter('center_xyz', [0.4, 0.0, 0.4])
        self.declare_parameter('radius', 0.15)
        self.declare_parameter('plane', 'front')
        self.declare_parameter('plane_frame', 'fr3_link0')
        self.declare_parameter('cycle_time', 10.0)
        self.declare_parameter('kp_cart', 1.5)
        self.declare_parameter('damping', 0.05)
        self.declare_parameter('qdot_max', 0.2)
        self.declare_parameter('lpf_alpha', 0.85)
        self.declare_parameter('max_accel_local', 1.5)

        # ---- Read parameters --------------------------------------------
        self.tracking_topic: str = resolve_topic_with_deprecated_alias(
            self, 'tracking_topic', 'command_topic', DEFAULT_TOPIC)
        js_topic_param: str = self.get_parameter('joint_state_topic').value
        self.ee_frame_name: str = self.get_parameter('ee_frame').value
        self.rate_hz: float = self.get_parameter('rate_hz').value
        self.warmup_s: float = self.get_parameter('warmup_s').value
        self.ramp_s: float = self.get_parameter('ramp_s').value
        center_xyz: List[float] = list(self.get_parameter('center_xyz').value)
        self._radius: float = self.get_parameter('radius').value
        plane: str = self.get_parameter('plane').value
        self._plane_frame_name: str = self.get_parameter('plane_frame').value
        cycle_time: float = self.get_parameter('cycle_time').value
        self.kp: float = self.get_parameter('kp_cart').value
        self.damping: float = self.get_parameter('damping').value
        self.qdot_max: float = self.get_parameter('qdot_max').value
        self.lpf_alpha: float = self.get_parameter('lpf_alpha').value
        self.max_accel_local: float = self.get_parameter('max_accel_local').value

        # ---- Validate ---------------------------------------------------
        if len(center_xyz) != 3:
            self.get_logger().error('center_xyz must have 3 elements')
            raise SystemExit(1)

        self._plane = plane.lower()
        if self._plane not in ('xy', 'xz', 'yz', 'front'):
            self.get_logger().error(
                f'Unknown plane "{self._plane}", use xy/xz/yz/front')
            raise SystemExit(1)

        # ---- Derived circle parameters ----------------------------------
        self._center = np.array(center_xyz, dtype=float)
        self._omega = 2.0 * math.pi / cycle_time

        # ---- Load Pinocchio model ---------------------------------------
        self.get_logger().info('Generating URDF via xacro …')
        try:
            urdf_xml = generate_urdf_from_xacro()
        except Exception as exc:
            self.get_logger().error(f'Failed to generate URDF: {exc}')
            raise SystemExit(1) from exc

        self.get_logger().info('Building Pinocchio model …')
        self.pin_model, self.pin_data = load_pinocchio_model(urdf_xml)

        # Resolve EE frame id
        try:
            self.ee_frame_id = resolve_frame_id(
                self.pin_model, self.ee_frame_name)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            raise SystemExit(1) from exc
        self.get_logger().info(
            f'EE frame: "{self.ee_frame_name}" (id={self.ee_frame_id})')

        # ---- Resolve plane_frame (for plane="front") --------------------
        self._use_plane_frame = (self._plane == 'front')
        self._plane_frame_id = -1
        if self._use_plane_frame:
            try:
                self._plane_frame_id = resolve_frame_id(
                    self.pin_model, self._plane_frame_name)
            except RuntimeError as exc:
                self.get_logger().error(str(exc))
                raise SystemExit(1) from exc
            self.get_logger().info(
                f'Plane frame: "{self._plane_frame_name}" '
                f'(id={self._plane_frame_id})\n'
                f'  plane="front" → circle in YZ of '
                f'{self._plane_frame_name}, X={center_xyz[0]:.3f} m constant')

        # ---- Arm joint IDs ----------------------------------------------
        try:
            self._pin_joint_ids = resolve_arm_joint_ids(self.pin_model)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            raise SystemExit(1) from exc

        # ---- Joint state manager ----------------------------------------
        self._js_mgr = JointStateManager(
            self, self.pin_model, self._pin_joint_ids,
            topic_param=js_topic_param)

        # ---- Publisher + timer ------------------------------------------
        self.pub = self.create_publisher(
            Float64MultiArray, self.tracking_topic, 10)
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._timer_cb)
        self.t0 = self.get_clock().now()

        # ---- Internal state for filter ----------------------------------
        self._qdot_prev = np.zeros(NUM_JOINTS)
        self._qdot_cmd_prev = np.zeros(NUM_JOINTS)
        self._tlog = ThrottledLogger(self.get_logger())

        # ---- Prebuilt zero message --------------------------------------
        self._zero_msg = Float64MultiArray()
        self._zero_msg.data = [0.0] * NUM_JOINTS

        # ---- Startup log ------------------------------------------------
        topic_note = ('(auto-resolved from franka.config.yaml)'
                      if self.tracking_topic == DEFAULT_TOPIC
                      else '(overridden via parameter)')
        self.get_logger().info(
            f'ee_circle_velocity_commander started\n'
            f'  tracking    : {self.tracking_topic}  {topic_note}\n'
            f'  js topic    : {self._js_mgr.topic_resolved or "(auto-detecting…)"}\n'
            f'  ee frame    : {self.ee_frame_name}\n'
            f'  rate        : {self.rate_hz} Hz\n'
            f'  warmup/ramp : {self.warmup_s} / {self.ramp_s} s\n'
            f'  center      : {center_xyz}\n'
            f'  radius      : {self._radius} m\n'
            f'  plane       : {self._plane}\n'
            f'  plane_frame : {self._plane_frame_name if self._use_plane_frame else "(N/A — world-frame axes)"}\n'
            f'  cycle_time  : {cycle_time} s  (ω={self._omega:.4f} rad/s)\n'
            f'  Kp          : {self.kp}\n'
            f'  damping     : {self.damping}\n'
            f'  qdot_max    : {self.qdot_max} rad/s\n'
            f'  lpf_alpha   : {self.lpf_alpha}\n'
            f'  max_accel   : {self.max_accel_local} rad/s²')

    # ------------------------------------------------------------------
    # Stopping
    # ------------------------------------------------------------------
    def request_stop(self):
        """Enter stopping state (idempotent).

        Ramps joint velocities to zero over 1.0 s via a reverse quintic,
        then publishes true zeros for 0.5 s before signalling done.
        """
        if self._stopping:
            return
        self._stopping = True
        self._stop_phase = self._STOP_RAMP
        self._stop_t0 = time.monotonic()
        self._qdot_frozen = self._qdot_cmd_prev.copy()
        self.get_logger().info(
            'Stopping: quintic ramp-down over 1.0 s, then zeros for 0.5 s')

    # ------------------------------------------------------------------
    # Circle position / velocity in the trajectory frame
    # ------------------------------------------------------------------
    def _circle_pd_vd(self, phi: float, s_val: float):
        """Return (p_d, v_d) for phase *phi* and velocity scale *s_val*.

        For plane='front'/'yz': circle in YZ, X held constant at center[0].
        For plane='xy': circle in XY, Z constant.
        For plane='xz': circle in XZ, Y constant.
        """
        R = self._radius
        w = self._omega
        c = math.cos(phi)
        s = math.sin(phi)

        if self._plane in ('yz', 'front'):
            p_d = self._center + np.array([0.0, R * c, R * s])
            v_d = np.array([0.0, -R * w * s_val * s, R * w * s_val * c])
        elif self._plane == 'xy':
            p_d = self._center + np.array([R * c, R * s, 0.0])
            v_d = np.array([-R * w * s_val * s, R * w * s_val * c, 0.0])
        else:  # xz
            p_d = self._center + np.array([R * c, 0.0, R * s])
            v_d = np.array([-R * w * s_val * s, 0.0, R * w * s_val * c])
        return p_d, v_d

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------
    def _timer_cb(self):  # noqa: C901
        dt = 1.0 / self.rate_hz
        max_delta = self.max_accel_local * dt

        # ---- STOPPING STATE MACHINE ------------------------------------
        if self._stopping:
            t_stop = time.monotonic() - self._stop_t0

            if self._stop_phase == self._STOP_RAMP:
                if t_stop < 1.0:
                    # Reverse quintic: scale frozen velocity from 1 → 0
                    scale = 1.0 - quintic_ramp(t_stop, 1.0)
                    qdot_target = self._qdot_frozen * scale
                    # Apply rate limiter to prevent acceleration discontinuity
                    delta = qdot_target - self._qdot_cmd_prev
                    qdot_cmd = self._qdot_cmd_prev + np.clip(
                        delta, -max_delta, max_delta)
                    self._qdot_cmd_prev = qdot_cmd.copy()
                    msg = Float64MultiArray()
                    msg.data = qdot_cmd.tolist()
                    try:
                        self.pub.publish(msg)
                    except Exception:
                        pass
                else:
                    # Ramp complete → publish zeros phase
                    self._stop_phase = self._STOP_ZEROS
                    self._stop_zero_t0 = time.monotonic()
                    self._qdot_cmd_prev = np.zeros(NUM_JOINTS)
                    self._qdot_prev = np.zeros(NUM_JOINTS)
                    try:
                        self.pub.publish(self._zero_msg)
                    except Exception:
                        pass

            elif self._stop_phase == self._STOP_ZEROS:
                try:
                    self.pub.publish(self._zero_msg)
                except Exception:
                    pass
                if time.monotonic() - self._stop_zero_t0 >= 0.5:
                    self.get_logger().info('Stop complete')
                    self.timer.cancel()
                    self._js_mgr.cancel_discovery()
                    self.done = True
            return

        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        # ---- Warmup phase: publish zeros --------------------------------
        if t < self.warmup_s:
            self.pub.publish(self._zero_msg)
            self._qdot_prev = np.zeros(NUM_JOINTS)
            self._qdot_cmd_prev = np.zeros(NUM_JOINTS)
            if self._tlog.due(t):
                self._tlog.info(
                    f'[t={t:.1f}s warmup] zeros for {self.warmup_s - t:.1f} s more')
            return

        # ---- Time since trajectory start (ramp begins here) ------------
        tr = t - self.warmup_s

        # ---- Need joint state ------------------------------------------
        if self._js_mgr.q is None or self._js_mgr.q_full is None:
            self.pub.publish(self._zero_msg)
            self._qdot_prev = np.zeros(NUM_JOINTS)
            self._qdot_cmd_prev = np.zeros(NUM_JOINTS)
            if self._tlog.due(t):
                self.get_logger().warn(
                    'No joint state received yet — publishing zeros')
            return

        # Check staleness (>0.1 s without update → zeros + warn)
        js_age = (self.get_clock().now()
                  - self._js_mgr.stamp).nanoseconds * 1e-9
        if js_age > 0.1:
            self.pub.publish(self._zero_msg)
            self._qdot_prev = np.zeros(NUM_JOINTS)
            self._qdot_cmd_prev = np.zeros(NUM_JOINTS)
            if self._tlog.due(t):
                self.get_logger().warn(
                    f'Joint-state stale ({js_age:.3f} s) — publishing zeros')
            return

        # ---- Forward kinematics + Jacobian via Pinocchio ---------------
        q_full = self._js_mgr.q_full.copy()
        try:
            oMee = compute_ee_fk(
                self.pin_model, self.pin_data, q_full, self.ee_frame_id)
            J_arm = compute_arm_jacobian(
                self.pin_model, self.pin_data, q_full,
                self.ee_frame_id, self._pin_joint_ids)
            if self._use_plane_frame:
                p_ee, J_pos = transform_ee_to_frame(
                    self.pin_model, self.pin_data,
                    self._plane_frame_id, oMee, J_arm)
            else:
                p_ee = oMee.translation.copy()
                J_pos = J_arm[:3, :]
        except Exception as exc:
            self.pub.publish(self._zero_msg)
            self._qdot_prev = np.zeros(NUM_JOINTS)
            self._qdot_cmd_prev = np.zeros(NUM_JOINTS)
            if self._tlog.due(t):
                self.get_logger().warn(f'FK/Jacobian failed: {exc}')
            return

        # ---- Circle trajectory reference --------------------------------
        # Phase integrates the quintic ramp → angular velocity ramps from
        # 0 to ω smoothly (C2-continuous, no acceleration discontinuity).
        phi = self._omega * quintic_ramp_integral(tr, self.ramp_s)
        s_val = quintic_ramp(tr, self.ramp_s)  # angular velocity scale

        p_d, v_d = self._circle_pd_vd(phi, s_val)

        # ---- Cartesian control law ------------------------------------
        e_pos = p_d - p_ee
        v_cmd = v_d + self.kp * e_pos

        # ---- Damped least-squares pseudo-inverse → qdot ---------------
        qdot_raw = dls_solve(J_pos, v_cmd, self.damping)
        if qdot_raw is None:
            self.pub.publish(self._zero_msg)
            self._qdot_prev = np.zeros(NUM_JOINTS)
            self._qdot_cmd_prev = np.zeros(NUM_JOINTS)
            return

        # ---- Joint velocity clamp + LPF + rate limiter ----------------
        qdot_clamped = clamp_joints(qdot_raw, self.qdot_max)
        qdot_lpf = lpf(self._qdot_prev, qdot_clamped, self.lpf_alpha)
        self._qdot_prev = qdot_lpf.copy()

        delta = qdot_lpf - self._qdot_cmd_prev
        qdot_rate_limited = self._qdot_cmd_prev + np.clip(
            delta, -max_delta, max_delta)
        self._qdot_cmd_prev = qdot_rate_limited.copy()

        # ---- Safety guard: reject runaway commands --------------------
        if np.any(np.abs(qdot_rate_limited) > self.qdot_max * 1.1):
            self.get_logger().error(
                f'qdot exceeds qdot_max*1.1={self.qdot_max * 1.1:.3f}: '
                f'[{vec_to_str(qdot_rate_limited)}] — publishing zeros')
            self.pub.publish(self._zero_msg)
            self._qdot_prev = np.zeros(NUM_JOINTS)
            self._qdot_cmd_prev = np.zeros(NUM_JOINTS)
            return

        # ---- Publish --------------------------------------------------
        msg = Float64MultiArray()
        msg.data = qdot_rate_limited.tolist()
        self.pub.publish(msg)

        # ---- Throttled log (~1 Hz) ------------------------------------
        if self._tlog.due(t):
            phase = 'ramp' if s_val < 1.0 else 'active'
            self._tlog.info(
                f'[t={t:.1f}s {phase} s={s_val:.3f} φ={phi:.3f}rad] '
                f'p=[{vec_to_str(p_ee)}] p_d=[{vec_to_str(p_d)}] '
                f'|e|={np.linalg.norm(e_pos):.4f} '
                f'|qdot|={np.linalg.norm(qdot_rate_limited):.4f}')


# ======================================================================
# main
# ======================================================================
def main(args=None):
    run_node_main(EECircleVelocityCommander, args=args)


if __name__ == '__main__':
    main()
