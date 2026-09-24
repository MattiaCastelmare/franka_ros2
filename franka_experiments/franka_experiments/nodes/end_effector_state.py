#!/usr/bin/env python3

import os
import threading

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

        self.q = np.zeros(NUM_JOINTS)
        self.qdot = np.zeros(NUM_JOINTS)
        self.stamp = None
        self.has_state = False

        self.lock = threading.Lock()

        control_cfg = load_control_config('control')
        topics = control_cfg['topics']

        self.pub = self.create_publisher(
            EndEffectorState,
            '/handover/end_effector_state',
            10,
        )

        self.create_subscription(
            JointState,
            topics.get(
                'joint_states_fast',
                topics['joint_states_topic'],
            ),
            self.on_joint_state,
            1,
        )

        # 100 Hz is plenty for the handover observer/control layer.
        self.create_timer(
            0.01,
            self.publish_state,
        )

        self.get_logger().info(
            f'EndEffectorState: {self.ee_link}, '
            f'v_ee = J_ee(q) qdot'
        )

    def on_joint_state(self, msg):

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
                return

            q[k] = msg.position[i]
            qdot[k] = msg.velocity[i]

        with self.lock:
            self.q[:] = q
            self.qdot[:] = qdot
            self.stamp = msg.header.stamp
            self.has_state = True

    def publish_state(self):

        with self.lock:

            if not self.has_state:
                return

            q = self.q.copy()
            qdot = self.qdot.copy()
            stamp = self.stamp

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
