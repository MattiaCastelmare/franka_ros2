#!/usr/bin/env python3
"""End-effector random-waypoints trajectory tracker via joint-velocity commands.

Publishes ``std_msgs/Float64MultiArray`` (7 joints) to the
``tracking_qdot`` topic (consumed by the **rt_velocity_blender_controller**)
using Pinocchio for Jacobian-based resolved-rate control.

The end-effector follows a sequence of **random Cartesian waypoints**
inside a configurable axis-aligned bounding box.  Each segment uses a
minimum-jerk (5th-order) time profile to guarantee C2 continuity, with
an optional hold phase at each waypoint.  A cosine-ramp warm-up prevents
velocity/acceleration discontinuities at startup.

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
from franka_experiments.utils.math_utils import (
    min_jerk, cosine_ramp, clamp_joints, lpf,
)
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_frame_id,
    resolve_arm_joint_ids,
    compute_ee_fk,
    compute_arm_jacobian,
    dls_solve,
    JointStateManager,
)
from franka_experiments.utils.trajectory import (
    sample_waypoints, sample_single_waypoint,
)
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str

# ---------------------------------------------------------------------------
# Module-level defaults
# ---------------------------------------------------------------------------
DEFAULT_TOPIC = resolve_tracking_topic()
DEFAULT_WORKSPACE_BOUNDS = [0.25, 0.55, -0.25, 0.25, 0.15, 0.55]


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class EERandomWaypointsVelocityCommander(Node):
    """Resolved-rate random-waypoint tracker publishing joint velocities."""

    def __init__(self):
        super().__init__('ee_random_waypoints_velocity_commander')

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

        self.declare_parameter('num_waypoints', 20)
        self.declare_parameter('workspace_bounds', DEFAULT_WORKSPACE_BOUNDS)
        self.declare_parameter('min_distance', 0.05)
        self.declare_parameter('segment_duration_s', 2.0)
        self.declare_parameter('hold_time_s', 0.3)
        self.declare_parameter('loop', False)

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

        num_waypoints: int = self.get_parameter('num_waypoints').value
        workspace_bounds: List[float] = list(
            self.get_parameter('workspace_bounds').value)
        self.min_distance: float = self.get_parameter('min_distance').value
        self.segment_duration_s: float = self.get_parameter(
            'segment_duration_s').value
        self.hold_time_s: float = self.get_parameter('hold_time_s').value
        self.loop: bool = self.get_parameter('loop').value

        self.kp: float = self.get_parameter('kp_cart').value
        self.damping: float = self.get_parameter('damping').value
        self.qdot_max: float = self.get_parameter('qdot_max').value
        self.lpf_alpha: float = self.get_parameter('lpf_alpha').value

        # ---- Validate ----------------------------------------------------
        if len(workspace_bounds) != 6:
            self.get_logger().error(
                'workspace_bounds must have 6 elements '
                '[xmin, xmax, ymin, ymax, zmin, zmax]')
            raise SystemExit(1)
        for i in range(3):
            if workspace_bounds[2 * i] >= workspace_bounds[2 * i + 1]:
                self.get_logger().error(
                    f'workspace_bounds: min >= max on axis {i}')
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

        # Arm joint IDs
        try:
            self._pin_joint_ids = resolve_arm_joint_ids(self.pin_model)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            raise SystemExit(1) from exc

        # ---- Joint state manager -----------------------------------------
        self._js_mgr = JointStateManager(
            self, self.pin_model, self._pin_joint_ids,
            topic_param=js_topic_param)

        # ---- Generate random waypoints -----------------------------------
        self._rng = np.random.default_rng()
        self._workspace_bounds = workspace_bounds
        self._waypoints = sample_waypoints(
            num_waypoints, workspace_bounds, self.min_distance, self._rng)

        # ---- Segment timing state ----------------------------------------
        self._wp_idx: int = 0
        self._phase: str = 'move'
        self._seg_start: float = 0.0
        self._seg_p_start: Optional[np.ndarray] = None
        self._trajectory_started = False

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
        bounds_str = (
            f'x=[{workspace_bounds[0]:.2f}, {workspace_bounds[1]:.2f}]  '
            f'y=[{workspace_bounds[2]:.2f}, {workspace_bounds[3]:.2f}]  '
            f'z=[{workspace_bounds[4]:.2f}, {workspace_bounds[5]:.2f}]')
        self.get_logger().info(
            f'ee_random_waypoints_velocity_commander started\n'
            f'  tracking      : {self.tracking_topic}  {topic_note}\n'
            f'  js topic      : {self._js_mgr.topic_resolved or "(auto-detecting…)"}\n'
            f'  ee frame      : {self.ee_frame_name}\n'
            f'  rate          : {self.rate_hz} Hz\n'
            f'  warmup/ramp   : {self.warmup_s} / {self.ramp_s} s\n'
            f'  num_waypoints : {num_waypoints}\n'
            f'  bounds        : {bounds_str}\n'
            f'  min_distance  : {self.min_distance} m\n'
            f'  segment_dur   : {self.segment_duration_s} s\n'
            f'  hold_time     : {self.hold_time_s} s\n'
            f'  loop          : {self.loop}\n'
            f'  Kp            : {self.kp}\n'
            f'  damping       : {self.damping}\n'
            f'  qdot_max      : {self.qdot_max} rad/s\n'
            f'  lpf_alpha     : {self.lpf_alpha}')

        for i, wp in enumerate(self._waypoints):
            self.get_logger().info(
                f'  waypoint[{i:2d}] : '
                f'[{wp[0]:.4f}, {wp[1]:.4f}, {wp[2]:.4f}]')

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
    # Waypoint management
    # ------------------------------------------------------------------
    def _advance_to_next_waypoint(self, tr: float, p_ee: np.ndarray):
        """Move to the next waypoint (or loop / finish)."""
        self._wp_idx += 1
        if self._wp_idx >= len(self._waypoints):
            if self.loop:
                self.get_logger().info(
                    'All waypoints reached — looping back to start')
                self._waypoints = sample_waypoints(
                    len(self._waypoints), self._workspace_bounds,
                    self.min_distance, self._rng)
                for i, wp in enumerate(self._waypoints):
                    self.get_logger().info(
                        f'  waypoint[{i:2d}] : '
                        f'[{wp[0]:.4f}, {wp[1]:.4f}, {wp[2]:.4f}]')
                self._wp_idx = 0
            else:
                self.get_logger().info(
                    'All waypoints reached — trajectory complete')
                self._phase = 'done'
                return

        self._phase = 'move'
        self._seg_start = tr
        self._seg_p_start = p_ee.copy()
        self.get_logger().info(
            f'Moving to waypoint [{self._wp_idx}/{len(self._waypoints)-1}]  '
            f'target=[{self._waypoints[self._wp_idx][0]:.4f}, '
            f'{self._waypoints[self._wp_idx][1]:.4f}, '
            f'{self._waypoints[self._wp_idx][2]:.4f}]')

    def _resample_current_waypoint(self):
        """Replace the current waypoint with a fresh random sample."""
        prev = (self._waypoints[self._wp_idx - 1]
                if self._wp_idx > 0 else None)
        p = sample_single_waypoint(
            self._workspace_bounds, self.min_distance, self._rng, prev)
        self._waypoints[self._wp_idx] = p
        self.get_logger().warn(
            f'Re-sampled waypoint [{self._wp_idx}] → '
            f'[{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]')

    # ------------------------------------------------------------------
    # Clamp + low-pass filter (shared pipeline)
    # ------------------------------------------------------------------
    def _clamp_and_filter(
        self, qdot_raw: np.ndarray, envelope: float,
    ) -> np.ndarray:
        qdot_scaled = envelope * qdot_raw
        qdot_clamped = clamp_joints(qdot_scaled, self.qdot_max)
        qdot_filt = lpf(self._qdot_prev, qdot_clamped, self.lpf_alpha)
        self._qdot_prev = qdot_filt.copy()
        return qdot_filt

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

        # Check staleness
        js_age = (self.get_clock().now()
                  - self._js_mgr.stamp).nanoseconds * 1e-9
        if js_age > 0.1:
            self.pub.publish(self._zero_msg)
            if self._tlog.due(t):
                self.get_logger().warn(
                    f'Joint-state stale ({js_age:.3f} s) — publishing zeros')
            return

        # ---- Forward kinematics + Jacobian --------------------------------
        q_full = self._js_mgr.q_full.copy()
        oMee = compute_ee_fk(
            self.pin_model, self.pin_data, q_full, self.ee_frame_id)
        p_ee = oMee.translation.copy()

        J_arm = compute_arm_jacobian(
            self.pin_model, self.pin_data, q_full,
            self.ee_frame_id, self._pin_joint_ids)
        J_pos = J_arm[:3, :]

        # ---- First active tick: initialise trajectory --------------------
        if not self._trajectory_started:
            self._trajectory_started = True
            self._seg_start = tr
            self._seg_p_start = p_ee.copy()
            self.get_logger().info(
                f'Trajectory started from EE position '
                f'[{p_ee[0]:.4f}, {p_ee[1]:.4f}, {p_ee[2]:.4f}]\n'
                f'Moving to waypoint [0/{len(self._waypoints)-1}]  '
                f'target=[{self._waypoints[0][0]:.4f}, '
                f'{self._waypoints[0][1]:.4f}, '
                f'{self._waypoints[0][2]:.4f}]')

        # ---- Trajectory done: hold at last position ----------------------
        if self._phase == 'done':
            p_d = self._waypoints[-1]
            v_d = np.zeros(3)
            e_pos = p_d - p_ee
            v_cmd = v_d + self.kp * e_pos
            qdot_raw = dls_solve(J_pos, v_cmd, self.damping)
            if qdot_raw is None:
                self.pub.publish(self._zero_msg)
                return
            qdot_out = self._clamp_and_filter(qdot_raw, envelope)
            msg = Float64MultiArray()
            msg.data = qdot_out.tolist()
            self.pub.publish(msg)
            self._log_throttle(
                t, phase='done', env=envelope, p_ee=p_ee, p_d=p_d,
                e_norm=np.linalg.norm(e_pos),
                qdot_norm=np.linalg.norm(qdot_out),
                wp_idx=self._wp_idx)
            return

        # ---- Move phase --------------------------------------------------
        if self._phase == 'move':
            seg_elapsed = tr - self._seg_start
            tau = (seg_elapsed / self.segment_duration_s
                   if self.segment_duration_s > 0 else 1.0)

            if tau >= 1.0:
                self._phase = 'hold'
                self._seg_start = tr
                tau = 1.0

            p_start = self._seg_p_start
            p_target = self._waypoints[self._wp_idx]

            s, sdot_norm = min_jerk(tau)
            p_d = p_start + s * (p_target - p_start)
            v_d = ((sdot_norm / self.segment_duration_s)
                   * (p_target - p_start)) if self._phase == 'move' else np.zeros(3)

            if self._phase == 'hold':
                v_d = np.zeros(3)

        # ---- Hold phase --------------------------------------------------
        if self._phase == 'hold':
            hold_elapsed = tr - self._seg_start
            p_d = self._waypoints[self._wp_idx]
            v_d = np.zeros(3)

            if hold_elapsed >= self.hold_time_s:
                self._advance_to_next_waypoint(tr, p_ee)
                if self._phase == 'done':
                    e_pos = p_d - p_ee
                    v_cmd = self.kp * e_pos
                    qdot_raw = dls_solve(J_pos, v_cmd, self.damping)
                    if qdot_raw is None:
                        self.pub.publish(self._zero_msg)
                        return
                    qdot_out = self._clamp_and_filter(qdot_raw, envelope)
                    msg = Float64MultiArray()
                    msg.data = qdot_out.tolist()
                    self.pub.publish(msg)
                    return

        # ---- Cartesian control law ---------------------------------------
        e_pos = p_d - p_ee
        v_cmd = v_d + self.kp * e_pos

        # ---- Damped pseudo-inverse → qdot --------------------------------
        qdot_raw = dls_solve(J_pos, v_cmd, self.damping)

        # Singularity / numerical failure handling
        if qdot_raw is None:
            v_cmd_scaled = 0.5 * v_cmd
            qdot_raw = dls_solve(
                J_pos, v_cmd_scaled, self.damping, damping_boost=0.1)
            if qdot_raw is None:
                self.get_logger().warn(
                    f'Singularity near waypoint [{self._wp_idx}] — '
                    f're-sampling target')
                self._resample_current_waypoint()
                self._seg_start = tr
                self._seg_p_start = p_ee.copy()
                self._phase = 'move'
                self.pub.publish(self._zero_msg)
                self._qdot_prev *= 0.0
                return

        # ---- Envelope + clamp + filter -----------------------------------
        qdot_out = self._clamp_and_filter(qdot_raw, envelope)

        # ---- Publish -----------------------------------------------------
        msg = Float64MultiArray()
        msg.data = qdot_out.tolist()
        self.pub.publish(msg)

        # ---- Throttled log (~1 Hz) ---------------------------------------
        self._log_throttle(
            t, phase=self._phase, env=envelope,
            p_ee=p_ee, p_d=p_d,
            e_norm=np.linalg.norm(e_pos),
            qdot_norm=np.linalg.norm(qdot_out),
            wp_idx=self._wp_idx)

    # ------------------------------------------------------------------
    # Throttled logger
    # ------------------------------------------------------------------
    def _log_throttle(
        self, t: float, *, phase: str, env: float,
        p_ee: Optional[np.ndarray] = None,
        p_d: Optional[np.ndarray] = None,
        e_norm: float = 0.0,
        qdot_norm: float = 0.0,
        wp_idx: int = 0,
    ):
        if not self._tlog.due(t):
            return
        self._tlog.info(
            f'[t={t:.1f}s {phase} env={env:.3f} wp={wp_idx}/'
            f'{len(self._waypoints)-1}] '
            f'p=[{vec_to_str(p_ee)}] p_d=[{vec_to_str(p_d)}] '
            f'|e|={e_norm:.4f} |qdot|={qdot_norm:.4f}')


# ======================================================================
# main
# ======================================================================
def main(args=None):
    run_node_main(EERandomWaypointsVelocityCommander, args=args)


if __name__ == '__main__':
    main()

# ======================================================================
# TEST COMMANDS
# ======================================================================
# 1) Start the RT velocity blender controller:
#
#    ros2 launch franka_experiments wrapper_forward_velocity.launch.py
#
# 2) Run this node (optionally with namespace):
#
#    ros2 run franka_experiments ee_random_waypoints_velocity_commander
#
#    # with namespace:
#    ros2 run franka_experiments ee_random_waypoints_velocity_commander \
#        --ros-args -r __ns:=/NS_1
#
#    # override parameters:
#    ros2 run franka_experiments ee_random_waypoints_velocity_commander \
#        --ros-args \
#        -p num_waypoints:=15 \
#        -p segment_duration_s:=3.0 \
#        -p hold_time_s:=0.5 \
#        -p workspace_bounds:="[0.25, 0.55, -0.3, 0.3, 0.15, 0.55]" \
#        -p loop:=true
#
# 3) Monitor topic rate:
#
#    ros2 topic hz /tracking_qdot
#    # or with namespace:
#    ros2 topic hz /NS_1/tracking_qdot
