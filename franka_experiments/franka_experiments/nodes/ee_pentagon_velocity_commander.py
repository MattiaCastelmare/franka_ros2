#!/usr/bin/env python3
"""End-effector pentagon trajectory tracker via joint-velocity commands.

Publishes ``std_msgs/Float64MultiArray`` (7 joints) to the
``tracking_qdot`` topic (consumed by the **velocity_blender** node) using
Pinocchio for Jacobian-based resolved-rate control.

The end-effector follows a **smooth pentagon** trajectory on a configurable
plane (default XY) around a given centre, with minimum-jerk (5th-order) time
profiles *per side* to guarantee C2 continuity at vertices.  A cosine-ramp
warm-up prevents velocity/acceleration discontinuities at startup.

Parameters
----------
See module-level ``declare_parameter`` calls for the full list.

Shutdown
--------
On SIGINT / SIGTERM / Ctrl-C the node publishes zero velocities for ~0.5 s,
then sets ``self.done = True`` so the explicit executor loop in ``main()``
exits cleanly.
"""

from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.constants import NUM_JOINTS, AUTO_SENTINEL
from franka_experiments.utils.ros import (
    resolve_tracking_topic,
    resolve_topic_with_deprecated_alias,
    run_node_main,
)
from franka_experiments.utils.math_utils import cosine_ramp, clamp_joints, lpf
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_frame_id,
    resolve_arm_joint_ids,
    compute_ee_fk,
    compute_arm_jacobian,
    transform_ee_to_frame,
    dls_solve,
)
from franka_experiments.utils.joint_state import JointStateManager
from franka_experiments.utils.trajectory import PentagonTrajectory
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str

# ---------------------------------------------------------------------------
# Module-level default (resolved once at import time)
# ---------------------------------------------------------------------------
DEFAULT_TOPIC = resolve_tracking_topic()


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class EEPentagonVelocityCommander(Node):
    """Resolved-rate pentagon tracker publishing joint velocities."""

    def __init__(self):
        super().__init__('ee_pentagon_velocity_commander')

        # ---- Done flag (checked by main-loop executor) ------------------
        self.done = False

        # ---- Stopping state machine -------------------------------------
        self._stopping = False
        self._stop_end_time = 0.0

        # ---- Declare parameters -----------------------------------------
        self.declare_parameter('tracking_topic', DEFAULT_TOPIC)
        self.declare_parameter('command_topic', '')  # deprecated alias
        self.declare_parameter('joint_state_topic', AUTO_SENTINEL)
        self.declare_parameter('ee_frame', 'fr3_hand_tcp')
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('warmup_s', 2.0)
        self.declare_parameter('ramp_s', 2.0)
        self.declare_parameter('center_xyz', [0.4, 0.0, 0.4])
        self.declare_parameter('radius', 0.2)
        self.declare_parameter('plane', 'front')
        self.declare_parameter('plane_frame', 'fr3_link0')
        self.declare_parameter('cycle_time', 15.0)
        self.declare_parameter('kp_cart', 2.0)
        self.declare_parameter('damping', 0.02)
        self.declare_parameter('qdot_max', 0.3)
        self.declare_parameter('lpf_alpha', 0.8)

        # ---- Read parameters (tracking_topic + deprecated alias) --------
        self.tracking_topic: str = resolve_topic_with_deprecated_alias(
            self, 'tracking_topic', 'command_topic', DEFAULT_TOPIC)
        js_topic_param: str = self.get_parameter('joint_state_topic').value
        self.ee_frame_name: str = self.get_parameter('ee_frame').value
        self.rate_hz: float = self.get_parameter('rate_hz').value
        self.warmup_s: float = self.get_parameter('warmup_s').value
        self.ramp_s: float = self.get_parameter('ramp_s').value
        center_xyz: List[float] = list(self.get_parameter('center_xyz').value)
        radius: float = self.get_parameter('radius').value
        plane: str = self.get_parameter('plane').value
        self._plane_frame_name: str = self.get_parameter('plane_frame').value
        cycle_time: float = self.get_parameter('cycle_time').value
        self.kp: float = self.get_parameter('kp_cart').value
        self.damping: float = self.get_parameter('damping').value
        self.qdot_max: float = self.get_parameter('qdot_max').value
        self.lpf_alpha: float = self.get_parameter('lpf_alpha').value

        # ---- Validate ----------------------------------------------------
        if len(center_xyz) != 3:
            self.get_logger().error('center_xyz must have 3 elements')
            raise SystemExit(1)

        # ---- Load Pinocchio model ----------------------------------------
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
        self._use_plane_frame = (plane == 'front')
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
                f'  plane="front" → pentagon in YZ of '
                f'{self._plane_frame_name}, X={center_xyz[0]:.3f} m constant')

        # ---- Arm joint IDs -----------------------------------------------
        try:
            self._pin_joint_ids = resolve_arm_joint_ids(self.pin_model)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            raise SystemExit(1) from exc

        # ---- Joint state manager -----------------------------------------
        self._js_mgr = JointStateManager(
            self, self.pin_model, self._pin_joint_ids,
            topic_param=js_topic_param)

        # ---- Pentagon trajectory -----------------------------------------
        self.traj = PentagonTrajectory(
            center=np.array(center_xyz),
            radius=radius,
            plane=plane,
            cycle_time=cycle_time,
        )

        # ---- Publisher + timer -------------------------------------------
        self.pub = self.create_publisher(
            Float64MultiArray, self.tracking_topic, 10)
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._timer_cb)
        self.t0 = self.get_clock().now()

        # ---- Internal state for filter -----------------------------------
        self._qdot_prev = np.zeros(NUM_JOINTS)
        self._tlog = ThrottledLogger(self.get_logger())

        # ---- Prebuilt zero message ---------------------------------------
        self._zero_msg = Float64MultiArray()
        self._zero_msg.data = [0.0] * NUM_JOINTS

        # ---- Startup log -------------------------------------------------
        topic_note = ('(auto-resolved from franka.config.yaml)'
                      if self.tracking_topic == DEFAULT_TOPIC
                      else '(overridden via parameter)')
        self.get_logger().info(
            f'ee_pentagon_velocity_commander started\n'
            f'  tracking    : {self.tracking_topic}  {topic_note}\n'
            f'  js topic    : {self._js_mgr.topic_resolved or "(auto-detecting…)"}\n'
            f'  ee frame    : {self.ee_frame_name}\n'
            f'  rate        : {self.rate_hz} Hz\n'
            f'  warmup/ramp : {self.warmup_s} / {self.ramp_s} s\n'
            f'  center      : {center_xyz}\n'
            f'  radius      : {radius} m\n'
            f'  plane       : {plane}\n'
            f'  plane_frame : {self._plane_frame_name if self._use_plane_frame else "(N/A — world-frame axes)"}\n'
            f'  cycle_time  : {cycle_time} s\n'
            f'  Kp          : {self.kp}\n'
            f'  damping     : {self.damping}\n'
            f'  qdot_max    : {self.qdot_max} rad/s\n'
            f'  lpf_alpha   : {self.lpf_alpha}')

    # ------------------------------------------------------------------
    # Stopping
    # ------------------------------------------------------------------
    def request_stop(self, stop_duration_s: float = 0.5):
        """Enter stopping state (idempotent)."""
        if self._stopping:
            return
        self._stopping = True
        self._stop_end_time = time.monotonic() + stop_duration_s
        self.get_logger().info(
            f'Stopping: publishing zero velocities for {stop_duration_s} s')

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------
    def _timer_cb(self):  # noqa: C901
        # ---- CLEAN SHUTDOWN: publish zeros, then signal done -------------
        if self._stopping:
            try:
                self.pub.publish(self._zero_msg)
            except Exception:
                pass
            if time.monotonic() >= self._stop_end_time:
                self.get_logger().info('Stop complete')
                self.timer.cancel()
                self._js_mgr.cancel_discovery()
                self.done = True
            return

        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        # ---- Warmup phase: zeros -----------------------------------------
        if t < self.warmup_s:
            self.pub.publish(self._zero_msg)
            if self._tlog.due(t):
                self._tlog.info(
                    f'[t={t:.1f}s warmup env=0.000] '
                    f'p=[?] p_d=[?] |e|=0.0000 |qdot|=0.0000')
            return

        # ---- Cosine-ramp envelope ----------------------------------------
        tr = t - self.warmup_s
        envelope = cosine_ramp(tr, self.ramp_s)

        # ---- Need joint state --------------------------------------------
        if self._js_mgr.q is None or self._js_mgr.q_full is None:
            self.pub.publish(self._zero_msg)
            if self._tlog.due(t):
                self.get_logger().warn(
                    'No joint state received yet — publishing zeros')
            return

        # Check staleness (>0.1 s without update → zeros + warn)
        js_age = (self.get_clock().now()
                  - self._js_mgr.stamp).nanoseconds * 1e-9
        if js_age > 0.1:
            self.pub.publish(self._zero_msg)
            if self._tlog.due(t):
                self.get_logger().warn(
                    f'Joint-state stale ({js_age:.3f} s) — publishing zeros')
            return

        # ---- Forward kinematics + Jacobian via Pinocchio -----------------
        q_full = self._js_mgr.q_full.copy()
        oMee = compute_ee_fk(
            self.pin_model, self.pin_data, q_full, self.ee_frame_id)
        J_arm = compute_arm_jacobian(
            self.pin_model, self.pin_data, q_full,
            self.ee_frame_id, self._pin_joint_ids)

        # ---- Plane-frame transform (for plane="front") ------------------
        if self._use_plane_frame:
            p_ee, J_pos = transform_ee_to_frame(
                self.pin_model, self.pin_data,
                self._plane_frame_id, oMee, J_arm)
        else:
            p_ee = oMee.translation.copy()
            J_pos = J_arm[:3, :]

        # ---- Trajectory reference ----------------------------------------
        p_d, v_d = self.traj.evaluate(tr)

        # ---- Cartesian control law ---------------------------------------
        e_pos = p_d - p_ee
        v_cmd = v_d + self.kp * e_pos

        # ---- Damped pseudo-inverse → qdot --------------------------------
        qdot_raw = dls_solve(J_pos, v_cmd, self.damping)
        if qdot_raw is None:
            self.pub.publish(self._zero_msg)
            return

        # ---- Apply envelope + clamp + LPF --------------------------------
        qdot_scaled = envelope * qdot_raw
        qdot_clamped = clamp_joints(qdot_scaled, self.qdot_max)
        qdot_filt = lpf(self._qdot_prev, qdot_clamped, self.lpf_alpha)
        self._qdot_prev = qdot_filt.copy()

        # ---- Publish -----------------------------------------------------
        msg = Float64MultiArray()
        msg.data = qdot_filt.tolist()
        self.pub.publish(msg)

        # ---- Throttled log (~1 Hz) ---------------------------------------
        if self._tlog.due(t):
            phase = 'ramp' if envelope < 1.0 else 'active'
            self._tlog.info(
                f'[t={t:.1f}s {phase} env={envelope:.3f}] '
                f'p=[{vec_to_str(p_ee)}] p_d=[{vec_to_str(p_d)}] '
                f'|e|={np.linalg.norm(e_pos):.4f} '
                f'|qdot|={np.linalg.norm(qdot_filt):.4f}')


# ======================================================================
# main
# ======================================================================
def main(args=None):
    run_node_main(EEPentagonVelocityCommander, args=args)


if __name__ == '__main__':
    main()
