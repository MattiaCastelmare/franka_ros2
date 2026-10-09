#!/usr/bin/env python3
"""Handover commander: the Pentagon qddot controller with the reference moved to the hand.

  target   palm + standoff_m along +z of the base (same x / y), never farther than max_target_step_m
           from the TCP; hand not trusted (measured, known physical hand, confidence >= min_confidence,
           <= hand_timeout_s old) -> target on the TCP, the path brakes
  path     second-order reference (target_response_s) bounded in velocity and acceleration
  test     follow_hand false: hold the start pose + test_offset_xyz
  gripper  open / close through gripper_controller/set_gripper (std_srvs/SetBool), the code of the
           former workspace_gripper_cycle test
"""

import time

import numpy as np
from franka_msgs.msg import HandState
from std_srvs.srv import SetBool

from franka_experiments.nodes.pentagon_qddot_commander import PentagonQddotCommander
from franka_experiments.utils.node_runtime import run_node_main


class Gripper:
    """Franka hand through the gripper_controller service: close (True) / open (False)."""

    TIMEOUT_S = 5.0

    def __init__(self, node, service):
        self.node, self.client = node, node.create_client(SetBool, service)
        self.future, self.deadline = None, 0.0

    def command(self, close):
        if not self.client.service_is_ready():
            self.node.get_logger().warn(f'{self.client.srv_name} not ready — gripper command skipped')
            return
        self.future = self.client.call_async(SetBool.Request(data=bool(close)))
        self.deadline = time.monotonic() + self.TIMEOUT_S
        self.node.get_logger().info(f'Gripper {"CLOSE" if close else "OPEN"}')

    def settled(self):
        """True once the last request answered, timed out, or never went."""
        if self.future is None:
            return True
        if self.future.done():
            try:
                response = self.future.result()
                if response is not None and not response.success:
                    self.node.get_logger().warn(f'Gripper refused: {response.message}')
            except Exception as exc:
                self.node.get_logger().warn(f'Gripper call failed: {exc}')
        elif time.monotonic() < self.deadline:
            return False
        else:
            self.node.get_logger().warn('Gripper did not answer — continuing')
        self.future = None
        return True


class HandoverPath:
    """Reference for the Pentagon controller: p, v, a towards desired_position()."""

    def __init__(self, node):
        self.node = node
        self.p, self.v, self.a = None, np.zeros(3), np.zeros(3)
        self.stamp_ns = None

    def position(self, s):
        now = self.node.get_clock().now().nanoseconds
        dt = (now - self.stamp_ns) * 1e-9 if self.stamp_ns is not None else 0.0
        self.stamp_ns = now
        if self.p is None or not 0.0 < dt <= 0.1:
            self.p = self.node._p_ee.copy()
            self.v.fill(0.0)
            self.a.fill(0.0)
            return self.p
        # one continuous reference, through hand switches and reacquisition: bounded deceleration
        # instead of an abrupt v = 0 when the hand is lost
        param = lambda name: float(self.node.get_parameter(name).value)
        target = self.node.desired_position()
        omega = 1.0 / max(0.01, param('target_response_s'))
        accel = omega**2 * (target - self.p) - 2.0 * omega * self.v
        accel *= min(1.0, max(0.0, param('max_target_acceleration_m_s2')) / max(float(np.linalg.norm(accel)), 1e-12))
        velocity = self.v + accel * dt
        velocity *= min(1.0, max(0.0, param('max_target_velocity_m_s')) / max(float(np.linalg.norm(velocity)), 1e-12))
        self.a[:] = (velocity - self.v) / dt
        self.p += 0.5 * (self.v + velocity) * dt
        self.v[:] = velocity
        return self.p

    def velocity(self, s, s_dot):
        return self.v

    def acceleration(self, s, s_dot, s_ddot):
        return self.a


class HandoverQddotCommander(PentagonQddotCommander):

    def __init__(self):
        super().__init__()
        for name, value in (('follow_hand', False), ('test_offset_xyz', [0.0, 0.0, 0.0]), ('standoff_m', 0.20),
                            ('hand_timeout_s', 0.20), ('min_confidence', 0.70), ('max_target_step_m', 0.10),
                            ('max_target_velocity_m_s', 0.40), ('max_target_acceleration_m_s2', 1.5),
                            ('target_response_s', 0.10), ('gripper_service', 'gripper_controller/set_gripper')):
            self.declare_parameter(name, value)
        self.gripper = Gripper(self, self.get_parameter('gripper_service').value)
        self._hold_position = None
        self._hand_position, self._hand_ok, self._hand_stamp_ns = np.zeros(3), False, 0
        self._handover_path = HandoverPath(self)
        self.create_subscription(HandState, '/handover/hand_state', self._hand_cb, 1)
        self.get_logger().info('Handover qddot commander ready - follow_hand=False')

    def _hand_cb(self, msg):
        p = np.array([msg.palm_position.x, msg.palm_position.y, msg.palm_position.z], dtype=float)
        self._hand_ok = bool(msg.position_valid and int(msg.position_source) == int(HandState.POSITION_SOURCE_MEASURED)
                             and int(msg.physical_hand) != int(HandState.HAND_UNKNOWN)
                             and float(msg.tracking_confidence) >= float(self.get_parameter('min_confidence').value)
                             and np.isfinite(p).all())
        if self._hand_ok:
            self._hand_position[:] = p
            self._hand_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def hand_trusted(self):
        if not self._hand_ok:
            return False
        age = (self.get_clock().now().nanoseconds - self._hand_stamp_ns) * 1e-9
        return 0.0 <= age <= float(self.get_parameter('hand_timeout_s').value)

    def desired_position(self):
        if self._hold_position is None:  # first acquisition: hold the start pose
            return self._p_ee.copy()
        if not bool(self.get_parameter('follow_hand').value):  # test: small offset from the start pose
            return self._hold_position + np.asarray(self.get_parameter('test_offset_xyz').value, dtype=float)
        if not self.hand_trusted():
            return self._p_ee.copy()
        target = self._hand_position.copy()
        target[2] += float(self.get_parameter('standoff_m').value)
        return self._step_to(target, self.max_step())

    def max_step(self):
        return float(self.get_parameter('max_target_step_m').value)

    def _step_to(self, target, max_step):
        """Virtual target never farther than max_step from the TCP (speed bound)."""
        move = target - self._p_ee
        n = float(np.linalg.norm(move))
        return self._p_ee + move * (max_step / n) if n > max_step else target.copy()

    def _start_trajectory(self, js):
        # the whole Pentagon initialisation (q_home, integrators, orientation hold, timing);
        # only the reference generator changes
        super()._start_trajectory(js)
        self._hold_position = self._current_ee_position(js).copy()
        self._path = self._handover_path
        self._approach_span = 0.0
        self.get_logger().info(f'Handover control started. Hold={self._hold_position.tolist()}')


def main(args=None):
    run_node_main(HandoverQddotCommander, args=args)


if __name__ == '__main__':
    main()
