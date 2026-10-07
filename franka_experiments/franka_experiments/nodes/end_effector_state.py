#!/usr/bin/env python3

import os
import time

import numpy as np
import pinocchio as pin
import rclpy

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

from franka_msgs.msg import EndEffectorState

from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS
from franka_experiments.utils.cbf_utils import load_robot_config as load_control_config
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_arm_joint_ids,
)


class EndEffectorStateNode(Node):

    def __init__(self):
        super().__init__('end_effector_state')

        # Same geometric definition used by HandoverDistance.
        cfg_path = os.path.join(
            get_package_share_directory('franka_experiments'),
            'config',
            'fr3_complete.yaml',
        )

        cfg = load_robot_config(cfg_path)
        robot_cfg = cfg['robot']
        distance_cfg = cfg['distance']

        self.base_frame = robot_cfg['base_frame']
        self.ee_link = robot_cfg.get('ee_link', 'fr3_link8')

        self.tip_axis = int(distance_cfg['ee_tip_axis'])
        self.tip_offset = float(distance_cfg['ee_tip_offset'])

        # Same Pinocchio model used by qddot_to_torque.
        urdf_xml = generate_urdf_from_xacro()
        self.model, self.data = load_pinocchio_model(urdf_xml)

        pin_jids = resolve_arm_joint_ids(self.model)

        self.arm_q_ids = [
            self.model.joints[j].idx_q
            for j in pin_jids
        ]

        self.arm_v_ids = [
            self.model.joints[j].idx_v
            for j in pin_jids
        ]

        self.frame_id = self.model.getFrameId(self.ee_link)

        self.q_neutral = pin.neutral(self.model)
        self.q_full = pin.neutral(self.model)
        self.qdot_full = np.zeros(self.model.nv)

        self.last = None  # (q, qdot, stamp) of the newest valid joint state

        # Prefer the real-robot fast joint-state stream.
        # Automatically fall back to the recorded/standard stream
        # when the primary is not producing valid arm states.
        self.primary_last_valid_wall = None
        self.primary_hold_s = 0.20

        control_cfg = load_control_config('control')
        topics = control_cfg['topics']

        self.primary_joint_topic = topics.get(
            'joint_states_fast',
            topics['joint_states_topic'],
        )
        self.fallback_joint_topic = topics['joint_states_topic']

        self.pub = self.create_publisher(
            EndEffectorState,
            '/handover/end_effector_state',
            10,
        )

        # The fast joint stream runs at ~1 kHz: a callback per message kept the
        # Python executor busy (~30-40 % of a core). The subscriptions live on a
        # node that is never spun; publish_state takes the newest message from
        # their DDS queue (depth 1) at the publishing rate.
        self.reader = rclpy.create_node('end_effector_state_joint_reader')
        self.subs = {'primary': self.reader.create_subscription(
            JointState, self.primary_joint_topic, lambda msg: None, 1)}
        if self.fallback_joint_topic != self.primary_joint_topic:
            self.subs['fallback'] = self.reader.create_subscription(
                JointState, self.fallback_joint_topic, lambda msg: None, 1)

        # 100 Hz is plenty for the handover observer/control layer.
        self.create_timer(
            0.01,
            self.publish_state,
        )

        self.get_logger().info(
            f'EndEffectorState: {self.ee_link}, '
            f'v_ee = J_ee(q) qdot'
        )

    def take(self, key):
        """Newest message waiting in a subscription queue, or None."""
        sub, msg = self.subs.get(key), None
        if sub is None:
            return None
        with sub.handle:
            while True:
                taken = sub.handle.take_message(sub.msg_type, sub.raw)
                if taken is None:
                    return msg
                msg = taken[0]

    def latest_state(self):
        """Newest valid joint state: the fast primary stream, the fallback stream
        only while the primary has been silent / invalid for primary_hold_s."""
        now = time.monotonic()
        msg = self.take('primary')
        if msg is not None:
            state = self.parse(msg)
            if state is not None:
                self.primary_last_valid_wall = now
                self.last = state
                return state
        primary_alive = (
            self.primary_last_valid_wall is not None
            and now - self.primary_last_valid_wall <= self.primary_hold_s
        )
        msg = self.take('fallback')
        if msg is not None and not primary_alive:
            state = self.parse(msg)
            if state is not None:
                self.last = state
        return self.last

    @staticmethod
    def parse(msg):

        index = {
            name: i
            for i, name in enumerate(msg.name)
        }

        q = np.zeros(NUM_JOINTS)
        qdot = np.zeros(NUM_JOINTS)

        for k, name in enumerate(FR3_JOINT_NAMES):

            i = index.get(name)

            if (
                i is None
                or i >= len(msg.position)
                or i >= len(msg.velocity)
            ):
                return None

            q[k] = msg.position[i]
            qdot[k] = msg.velocity[i]

        return q, qdot, msg.header.stamp

    def publish_state(self):

        state = self.latest_state()
        if state is None:
            return
        q, qdot, stamp = state

        np.copyto(
            self.q_full,
            self.q_neutral,
        )

        self.qdot_full[:] = 0.0

        for k, (iq, iv) in enumerate(
            zip(
                self.arm_q_ids,
                self.arm_v_ids,
            )
        ):
            self.q_full[iq] = q[k]
            self.qdot_full[iv] = qdot[k]

        pin.forwardKinematics(
            self.model,
            self.data,
            self.q_full,
            self.qdot_full,
        )

        pin.updateFramePlacements(
            self.model,
            self.data,
        )

        placement = self.data.oMf[self.frame_id]

        tip_local = np.zeros(3)
        tip_local[self.tip_axis] = self.tip_offset

        r_world = placement.rotation @ tip_local

        position = (
            np.asarray(placement.translation)
            + r_world
        )

        J = pin.computeFrameJacobian(
            self.model,
            self.data,
            self.q_full,
            self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

        # End-effector Jacobian:
        # v_ee = v_frame + omega x r
        J_ee = (
            J[:3, :]
            - pin.skew(r_world) @ J[3:, :]
        )

        velocity = J_ee @ self.qdot_full

        out = EndEffectorState()

        out.header.stamp = stamp
        out.header.frame_id = self.base_frame

        out.valid = bool(
            np.all(np.isfinite(position))
            and np.all(np.isfinite(velocity))
        )

        out.q = q.tolist()
        out.qdot = qdot.tolist()

        out.position.x = float(position[0])
        out.position.y = float(position[1])
        out.position.z = float(position[2])

        out.velocity.x = float(velocity[0])
        out.velocity.y = float(velocity[1])
        out.velocity.z = float(velocity[2])

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = EndEffectorStateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reader.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
