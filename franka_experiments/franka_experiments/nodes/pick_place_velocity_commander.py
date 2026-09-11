#!/usr/bin/env python3
"""Cyclic Cartesian pick-and-place task for the Franka FR3.

The node captures the current end-effector position as ``home`` and repeatedly
executes this nominal sequence in the ``fr3_link0`` Cartesian frame:

    home -> pick -> wait -> lift -> transfer -> lower -> wait -> home -> wait

Only the end-effector position is controlled. The orientation is not regulated,
which is acceptable for the initial "fake grasp" experiment but should be added
before using a real gripper.

Joint velocities are published on the nominal tracking topic consumed by the
``rt_velocity_blender_controller``. Cartesian tracking uses Pinocchio and a
damped least-squares inverse Jacobian, following the existing pentagon example.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.constants import AUTO_SENTINEL, NUM_JOINTS
from franka_experiments.utils.ros import resolve_tracking_topic, run_node_main
from franka_experiments.utils.math_utils import clamp_joints, lpf
from franka_experiments.utils.kinematics import (
    JointStateManager,
    compute_arm_jacobian,
    compute_ee_fk,
    dls_solve,
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_arm_joint_ids,
    resolve_frame_id,
)


DEFAULT_TOPIC = resolve_tracking_topic()


@dataclass
class TaskPhase:
    """One timed phase of the cyclic task."""

    name: str
    start: np.ndarray
    goal: np.ndarray
    duration: float
    moving: bool


class PickPlaceTrajectory:
    """Timed sequence of minimum-jerk Cartesian moves and stationary waits."""

    def __init__(
        self,
        home: np.ndarray,
        lateral_offset: float,
        down_offset: float,
        lift_offset: float,
        move_time: float,
        lift_time: float,
        transfer_time: float,
        return_time: float,
        wait_pick: float,
        wait_place: float,
        wait_home: float,
    ) -> None:
        home = np.asarray(home, dtype=float).copy()

        # fr3_link0 convention used here:
        #   +y = robot left, -y = robot right, +z = up
        pick = home + np.array([0.0, -lateral_offset, -down_offset])
        place = home + np.array([0.0, lateral_offset, -down_offset])
        pick_up = pick + np.array([0.0, 0.0, lift_offset])
        place_up = place + np.array([0.0, 0.0, lift_offset])

        self.home = home
        self.pick = pick
        self.pick_up = pick_up
        self.place_up = place_up
        self.place = place

        self.phases: List[TaskPhase] = [
            TaskPhase('MOVE_TO_PICK', home, pick, move_time, True),
            TaskPhase('WAIT_AT_PICK', pick, pick, wait_pick, False),
            TaskPhase('LIFT_OBJECT', pick, pick_up, lift_time, True),
            TaskPhase('TRANSFER', pick_up, place_up, transfer_time, True),
            TaskPhase('LOWER_TO_PLACE', place_up, place, lift_time, True),
            TaskPhase('WAIT_AT_PLACE', place, place, wait_place, False),
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

    @staticmethod
    def _minimum_jerk(u: float) -> Tuple[float, float]:
        """Return normalized position sigma(u) and d sigma / d u."""
        u = float(np.clip(u, 0.0, 1.0))
        u2 = u * u
        u3 = u2 * u
        u4 = u3 * u
        u5 = u4 * u
        sigma = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
        dsigma_du = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
        return sigma, dsigma_du

    def evaluate(self, t: float):
        """Return desired position, velocity, phase name and cycle index."""
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
            return self._p_out, self._v_out, phase.name, cycle_index

        u = local_t / phase.duration
        sigma, dsigma_du = self._minimum_jerk(u)
        delta = phase.goal - phase.start

        self._p_out[:] = phase.start + sigma * delta
        self._v_out[:] = (dsigma_du / phase.duration) * delta
        return self._p_out, self._v_out, phase.name, cycle_index


class PickPlaceVelocityCommander(Node):
    """Track the cyclic pick-and-place reference with joint-velocity commands."""

    def __init__(self) -> None:
        super().__init__('pick_place_velocity_commander')

        self.done = False
        self._stopping = False
        self._stop_end_time = 0.0

        # ROS / robot interface
        self.declare_parameter('tracking_topic', "/NS_1/qdot_cmd")
        self.declare_parameter('joint_state_topic', "/NS_1/joint_states")
        self.declare_parameter('ee_frame', 'fr3_hand_tcp')
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('warmup_s', 2.0)
        self.declare_parameter('joint_state_timeout_s', 0.1)

        # Task geometry, expressed relative to the captured home position
        self.declare_parameter('lateral_offset', 0.16)
        self.declare_parameter('down_offset', 0.20)
        self.declare_parameter('lift_offset', 0.08)

        # Task timing
        self.declare_parameter('move_time', 3.0)
        self.declare_parameter('lift_time', 1.0)
        self.declare_parameter('transfer_time', 3.0)
        self.declare_parameter('return_time', 3.0)
        self.declare_parameter('wait_pick', 0.6)
        self.declare_parameter('wait_place', 0.6)
        self.declare_parameter('wait_home', 0.6)

        # Cartesian resolved-rate controller
        self.declare_parameter('kp_cart', 1.5)
        self.declare_parameter('damping', 0.05)
        self.declare_parameter('qdot_max', 0.25)
        self.declare_parameter('lpf_alpha', 0.85)
        self.declare_parameter('max_accel_local', 2.0)
        self.declare_parameter('max_cart_error', 0.20)

        self.tracking_topic = str(self.get_parameter('tracking_topic').value)
        joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.ee_frame_name = str(self.get_parameter('ee_frame').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.warmup_s = float(self.get_parameter('warmup_s').value)
        self.joint_state_timeout_s = float(
            self.get_parameter('joint_state_timeout_s').value)

        self.lateral_offset = float(self.get_parameter('lateral_offset').value)
        self.down_offset = float(self.get_parameter('down_offset').value)
        self.lift_offset = float(self.get_parameter('lift_offset').value)

        self.move_time = float(self.get_parameter('move_time').value)
        self.lift_time = float(self.get_parameter('lift_time').value)
        self.transfer_time = float(self.get_parameter('transfer_time').value)
        self.return_time = float(self.get_parameter('return_time').value)
        self.wait_pick = float(self.get_parameter('wait_pick').value)
        self.wait_place = float(self.get_parameter('wait_place').value)
        self.wait_home = float(self.get_parameter('wait_home').value)

        self.kp = float(self.get_parameter('kp_cart').value)
        self.damping = float(self.get_parameter('damping').value)
        self.qdot_max = float(self.get_parameter('qdot_max').value)
        self.lpf_alpha = float(self.get_parameter('lpf_alpha').value)
        self.max_accel_local = float(
            self.get_parameter('max_accel_local').value)
        self.max_cart_error = float(self.get_parameter('max_cart_error').value)

        if self.rate_hz <= 0.0:
            raise ValueError('rate_hz must be positive')
        if min(self.lateral_offset, self.down_offset, self.lift_offset) < 0.0:
            raise ValueError('Task offsets must be non-negative')

        self.get_logger().info('Generating URDF and loading Pinocchio model...')
        urdf_xml = generate_urdf_from_xacro()
        self.pin_model, self.pin_data = load_pinocchio_model(urdf_xml)
        self.ee_frame_id = resolve_frame_id(self.pin_model, self.ee_frame_name)
        self._pin_joint_ids = resolve_arm_joint_ids(self.pin_model)

        self._js_mgr = JointStateManager(
            self,
            self.pin_model,
            self._pin_joint_ids,
            topic_param=joint_state_topic,
        )

        self.pub = self.create_publisher(
            Float64MultiArray, self.tracking_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_cb)

        self._node_start = self.get_clock().now()
        self._task_start = None
        self._trajectory: Optional[PickPlaceTrajectory] = None
        self._last_phase: Optional[str] = None
        self._last_cycle = -1
        self._last_log_time = -1.0

        self._qdot_filtered = np.zeros(NUM_JOINTS)
        self._qdot_commanded = np.zeros(NUM_JOINTS)

        self._zero_msg = Float64MultiArray()
        self._zero_msg.data = [0.0] * NUM_JOINTS

        self.get_logger().info(
            'Pick-and-place commander started.\n'
            f'  joint state topic : {self.get_parameter("joint_state_topic").value}\n'
            f'  tracking topic : {self.tracking_topic}\n'
            f'  EE frame       : {self.ee_frame_name}\n'
            f'  rate           : {self.rate_hz:.1f} Hz\n'
            '  home           : captured from the current EE position\n'
            f'  offsets [m]    : lateral={self.lateral_offset:.3f}, '
            f'down={self.down_offset:.3f}, lift={self.lift_offset:.3f}'
        )

    def request_stop(self, stop_duration_s: float = 0.5) -> None:
        """Publish zeros briefly before terminating."""
        if self._stopping:
            return
        self._stopping = True
        self._stop_end_time = time.monotonic() + stop_duration_s
        self.get_logger().info('Stopping: publishing zero joint velocities')

    def _publish_zero(self, reset_filters: bool = True) -> None:
        self.pub.publish(self._zero_msg)
        if reset_filters:
            self._qdot_filtered[:] = 0.0
            self._qdot_commanded[:] = 0.0

    def _joint_state_is_ready(self, now) -> bool:
        if self._js_mgr.q is None or self._js_mgr.q_full is None:
            return False
        age = (now - self._js_mgr.stamp).nanoseconds * 1e-9
        return age <= self.joint_state_timeout_s

    def _capture_home(self, now) -> None:
        q_full = self._js_mgr.q_full.copy()
        oMee = compute_ee_fk(
            self.pin_model, self.pin_data, q_full, self.ee_frame_id)
        home = oMee.translation.copy()

        self._trajectory = PickPlaceTrajectory(
            home=home,
            lateral_offset=self.lateral_offset,
            down_offset=self.down_offset,
            lift_offset=self.lift_offset,
            move_time=self.move_time,
            lift_time=self.lift_time,
            transfer_time=self.transfer_time,
            return_time=self.return_time,
            wait_pick=self.wait_pick,
            wait_place=self.wait_place,
            wait_home=self.wait_home,
        )
        self._task_start = now

        tr = self._trajectory
        self.get_logger().info(
            'Home captured; nominal task enabled.\n'
            f'  home     : {tr.home.round(4).tolist()}\n'
            f'  pick     : {tr.pick.round(4).tolist()}\n'
            f'  pick_up  : {tr.pick_up.round(4).tolist()}\n'
            f'  place_up : {tr.place_up.round(4).tolist()}\n'
            f'  place    : {tr.place.round(4).tolist()}\n'
            f'  cycle    : {tr.cycle_time:.1f} s'
        )

    def _timer_cb(self) -> None:
        if self._stopping:
            self._publish_zero()
            if time.monotonic() >= self._stop_end_time:
                self.timer.cancel()
                self._js_mgr.cancel_discovery()
                self.done = True
            return

        now = self.get_clock().now()
        node_t = (now - self._node_start).nanoseconds * 1e-9

        if node_t < self.warmup_s:
            self._publish_zero()
            return

        if not self._joint_state_is_ready(now):
            self._publish_zero()
            if node_t - self._last_log_time >= 1.0:
                self.get_logger().warn(
                    'Joint state missing or stale; publishing zeros')
                self._last_log_time = node_t
            return

        if self._trajectory is None:
            self._capture_home(now)
            self._publish_zero()
            return

        q_full = self._js_mgr.q_full.copy()
        oMee = compute_ee_fk(
            self.pin_model, self.pin_data, q_full, self.ee_frame_id)
        J_arm = compute_arm_jacobian(
            self.pin_model,
            self.pin_data,
            q_full,
            self.ee_frame_id,
            self._pin_joint_ids,
        )

        p_ee = oMee.translation.copy()
        J_pos = J_arm[:3, :]

        task_t = (now - self._task_start).nanoseconds * 1e-9
        p_des, v_des, phase, cycle = self._trajectory.evaluate(task_t)

        position_error = p_des - p_ee
        error_norm = float(np.linalg.norm(position_error))

        if error_norm > self.max_cart_error:
            self._publish_zero()
            if node_t - self._last_log_time >= 1.0:
                self.get_logger().error(
                    f'Cartesian tracking error too large: {error_norm:.3f} m; '
                    'publishing zeros')
                self._last_log_time = node_t
            return

        cartesian_velocity_cmd = v_des + self.kp * position_error
        qdot_raw = dls_solve(J_pos, cartesian_velocity_cmd, self.damping)
        if qdot_raw is None or not np.all(np.isfinite(qdot_raw)):
            self._publish_zero()
            return

        qdot_clamped = clamp_joints(qdot_raw, self.qdot_max)
        qdot_filtered = lpf(
            self._qdot_filtered, qdot_clamped, self.lpf_alpha)
        self._qdot_filtered = qdot_filtered.copy()

        dt = 1.0 / self.rate_hz
        max_delta = self.max_accel_local * dt
        delta = np.clip(
            qdot_filtered - self._qdot_commanded,
            -max_delta,
            max_delta,
        )
        qdot_command = self._qdot_commanded + delta
        self._qdot_commanded = qdot_command.copy()

        msg = Float64MultiArray()
        msg.data = qdot_command.tolist()
        self.pub.publish(msg)

        if phase != self._last_phase or cycle != self._last_cycle:
            self.get_logger().info(
                f'Cycle {cycle + 1} | phase: {phase}')
            self._last_phase = phase
            self._last_cycle = cycle

        if node_t - self._last_log_time >= 1.0:
            self.get_logger().info(
                f'{phase} | error={error_norm:.4f} m | '
                f'|qdot|={np.linalg.norm(qdot_command):.3f} rad/s')
            self._last_log_time = node_t


def main(args=None) -> None:
    run_node_main(PickPlaceVelocityCommander, args=args)

if __name__ == '__main__':
    main()