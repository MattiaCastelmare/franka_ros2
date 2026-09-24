#!/usr/bin/env python3

import time
import numpy as np

from franka_msgs.msg import HandState

from franka_experiments.nodes.pentagon_qddot_commander import (
    PentagonQddotCommander,
)
from franka_experiments.utils.node_runtime import run_node_main


class HandoverPath:

    def __init__(self, node):
        self.node = node
        self.zero = np.zeros(3)

    def position(self, s):
        return self.node.desired_position()

    def velocity(self, s, s_dot):
        return self.node.desired_velocity()

    def acceleration(self, s, s_dot, s_ddot):
        return self.zero


class HandoverQddotCommander(PentagonQddotCommander):

    def __init__(self):
        super().__init__()

        self.declare_parameter('follow_hand', False)
        self.declare_parameter('test_offset_xyz', [0.0, 0.0, 0.0])

        self.declare_parameter('standoff_m', 0.20)
        self.declare_parameter('hand_timeout_s', 0.15)
        self.declare_parameter('min_confidence', 0.70)

        # Target virtuale massimo rispetto all'EE corrente.
        self.declare_parameter('max_target_step_m', 0.05)
        self.declare_parameter('max_target_velocity_m_s', 0.25)

        self._hold_position = None

        self._hand_position = np.zeros(3)
        self._hand_velocity = np.zeros(3)

        self._hand_ok = False
        self._hand_velocity_ok = False
        self._hand_rx_time = 0.0

        self._handover_path = HandoverPath(self)

        self.create_subscription(
            HandState,
            '/handover/hand_state',
            self._hand_cb,
            10,
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
            self._hand_rx_time = time.monotonic()

        v = np.array([
            msg.palm_velocity.x,
            msg.palm_velocity.y,
            msg.palm_velocity.z,
        ], dtype=float)

        self._hand_velocity_ok = bool(
            self._hand_ok
            and
            msg.velocity_valid
            and
            np.isfinite(v).all()
        )

        if self._hand_velocity_ok:
            self._hand_velocity[:] = v

    def hand_trusted(self):

        if not self._hand_ok:
            return False

        age = time.monotonic() - self._hand_rx_time

        return age <= float(
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

        # Mano persa/non trusted -> stop cartesiano dove siamo.
        if not self.hand_trusted():
            return self._p_ee.copy()

        # Target = punto a standoff dalla mano,
        # lungo la linea mano <-> EE.
        delta = self._p_ee - self._hand_position
        distance = float(np.linalg.norm(delta))

        if distance < 1e-6:
            return self._p_ee.copy()

        standoff = float(
            self.get_parameter('standoff_m').value
        )

        target = (
            self._hand_position
            + standoff * delta / distance
        )

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

    def desired_velocity(self):

        if not bool(
            self.get_parameter('follow_hand').value
        ):
            return np.zeros(3)

        if (
            not self.hand_trusted()
            or not self._hand_velocity_ok
        ):
            return np.zeros(3)

        v = self._hand_velocity.copy()

        vmax = float(
            self.get_parameter(
                'max_target_velocity_m_s'
            ).value
        )

        speed = float(np.linalg.norm(v))

        if speed > vmax:
            v *= vmax / speed

        return v

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
