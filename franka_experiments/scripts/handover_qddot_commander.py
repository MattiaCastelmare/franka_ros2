#!/usr/bin/env python3

import numpy as np

from franka_msgs.msg import HandState

from franka_experiments.nodes.pentagon_qddot_commander import (
    PentagonQddotCommander,
)
from franka_experiments.utils.node_runtime import run_node_main


class HandoverPath:

    def __init__(self, node):
        self.node = node
        self.p = None
        self.v = np.zeros(3)
        self.a = np.zeros(3)
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

        # One continuous reference, including hand switches and reacquisition.
        # Invalid hands select the current TCP in desired_position(), not a
        # stale hand target. Bounded deceleration replaces an abrupt v_d = 0.
        target = self.node.desired_position()
        omega = 1.0 / max(0.01, float(
            self.node.get_parameter('target_response_s').value))
        accel = omega**2 * (target - self.p) - 2.0 * omega * self.v
        amax = max(0.0, float(
            self.node.get_parameter('max_target_acceleration_m_s2').value))
        accel *= min(1.0, amax / max(float(np.linalg.norm(accel)), 1e-12))
        velocity = self.v + accel * dt
        vmax = max(0.0, float(
            self.node.get_parameter('max_target_velocity_m_s').value))
        velocity *= min(1.0, vmax / max(float(np.linalg.norm(velocity)), 1e-12))
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

        self.declare_parameter('follow_hand', False)
        self.declare_parameter('test_offset_xyz', [0.0, 0.0, 0.0])

        self.declare_parameter('standoff_m', 0.20)
        self.declare_parameter('hand_timeout_s', 0.20)
        self.declare_parameter('min_confidence', 0.70)

        # Target virtuale massimo rispetto all'EE corrente.
        self.declare_parameter('max_target_step_m', 0.10)
        self.declare_parameter('max_target_velocity_m_s', 0.40)
        self.declare_parameter('max_target_acceleration_m_s2', 1.5)
        self.declare_parameter('target_response_s', 0.10)

        self._hold_position = None

        self._hand_position = np.zeros(3)

        self._hand_ok = False
        self._hand_stamp_ns = 0

        self._handover_path = HandoverPath(self)

        self.create_subscription(
            HandState,
            '/handover/hand_state',
            self._hand_cb,
            1,
        )

        self.get_logger().info(
            'Handover qddot commander ready - follow_hand=False'
        )

    def _hand_cb(self, msg):

        p = np.array([
            msg.palm_position.x,
            msg.palm_position.y,
            msg.palm_position.z,
        ], dtype=float)

        self._hand_ok = bool(
            msg.position_valid
            and
            int(msg.position_source)
            == int(HandState.POSITION_SOURCE_MEASURED)
            and
            int(msg.physical_hand)
            != int(HandState.HAND_UNKNOWN)
            and
            float(msg.tracking_confidence)
            >= float(self.get_parameter('min_confidence').value)
            and
            np.isfinite(p).all()
        )

        if self._hand_ok:
            self._hand_position[:] = p
            self._hand_stamp_ns = (
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            )

    def hand_trusted(self):

        if not self._hand_ok:
            return False

        age = (self.get_clock().now().nanoseconds - self._hand_stamp_ns) * 1e-9

        return 0.0 <= age <= float(
            self.get_parameter('hand_timeout_s').value
        )

    def desired_position(self):

        # Prima acquisizione: hold posizione iniziale.
        if self._hold_position is None:
            return self._p_ee.copy()

        follow = bool(
            self.get_parameter('follow_hand').value
        )

        # TEST MODE: piccolo offset cartesiano dal punto iniziale.
        if not follow:

            offset = np.asarray(
                self.get_parameter('test_offset_xyz').value,
                dtype=float,
            )

            return self._hold_position + offset

        # Mano persa/non trusted -> target sul TCP; il path frena gradualmente.
        if not self.hand_trusted():
            return self._p_ee.copy()

        # Standoff solo lungo +Z della base: stessa X/Y del palmo.
        standoff = float(
            self.get_parameter('standoff_m').value
        )

        target = self._hand_position.copy()
        target[2] += standoff

        # Non dare mai al controller un target virtuale
        # troppo lontano dall'EE corrente.
        move = target - self._p_ee
        move_norm = float(np.linalg.norm(move))

        max_step = float(
            self.get_parameter('max_target_step_m').value
        )

        if move_norm > max_step:
            move *= max_step / move_norm
            target = self._p_ee + move

        return target

    def _start_trajectory(self, js):

        # Riusa TUTTA l'inizializzazione del controller esistente:
        # q_home, integratori, orientation hold, timing, ecc.
        super()._start_trajectory(js)

        self._hold_position = (
            self._current_ee_position(js).copy()
        )

        # Cambiamo SOLO il generatore del riferimento cartesiano.
        self._path = self._handover_path
        self._approach_span = 0.0

        self.get_logger().info(
            f'Handover control started. '
            f'Hold={self._hold_position.tolist()}'
        )


def main(args=None):
    run_node_main(
        HandoverQddotCommander,
        args=args,
    )


if __name__ == '__main__':
    main()
