#!/usr/bin/env python3
"""
Human Distance Node.
Reads filtered human arm states (supports both single and dual arms dynamically)
and robot joint states to compute real-time geometric distances using a
capsule-based representation for CBF control.

Each LinkDistance also carries the obstacle "track" fields that cbf_safety_filter
consumes with obstacle_velocity_source: tracker. For a closest point at parameter
alpha on the capsule axis between keypoints a and b (independent Kalman filters):
    v_h   = (1 - alpha) v_a + alpha v_b
    Sigma = (1 - alpha)^2 Sigma_a + alpha^2 Sigma_b
"""

import os
import rclpy
from functools import partial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import numpy as np
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy

from franka_msgs.msg import HumanArmState, LinkDistance, MultiLinkDistance
from franka_experiments.utils.capsule_geometry import HumanArmGeometry, RobotGeometry
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_utils import (
    extract_human_keypoints, extract_human_covariances, init_pinocchio_from_xacro,
    define_control_points, format_topic, get_side, fill_track_fields
)


class HumanDistance(Node):
    def __init__(self):
        super().__init__('human_distance_node')

        # Configs
        config_path = os.path.join(
            get_package_share_directory("franka_experiments"), 
            "config", 
            "fr3_complete.yaml"
        )
        tracker_path = os.path.join(
            get_package_share_directory("franka_experiments"), 
            "config", 
            "human_params.yaml"
        )

        # Load YAML params
        self.config = load_robot_config(config_path)
        self.robot_cfg = self.config['robot']
        self.zones_cfg = self.config['zones']
        self.dist_cfg = self.config['distance']

        self.declare_parameter('mode', 'capsules')
        self.mode = self.get_parameter('mode').value
        # /NS_1/joint_states is what the recorded bags contain; on the robot prefer the
        # 1 kHz /NS_1/franka/joint_states (the 30 Hz topic is republished with fresh stamps)
        self.declare_parameter('joint_state_topic', '/NS_1/joint_states')
        # LinkDistance.distance is a surface gap clamped at 0 by contract (the CBF reads it);
        # /human_robot/distance keeps the signed value
        self.declare_parameter('clamp_distance', True)
        joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.clamp_distance = bool(self.get_parameter('clamp_distance').value)
        self.tracker_config = load_robot_config(tracker_path)['human_tracker']
        self.pose_side = str(self.tracker_config["pose_side"]).lower()
        self.active_sides = ["left", "right"] if self.pose_side == "both" else [self.pose_side]

        # Internal State
        self.latest_joint_state = None
        self.latest_arm_states = {side: None for side in self.active_sides}

        # Initialize Pinocchio
        self.pin_ok, self.model, self.data = init_pinocchio_from_xacro(self)
        if not self.pin_ok:
            raise RuntimeError('Pinocchio initialization failed.')

        # Arm joint names (fingers are locked) and cached link frame ids
        self.joint_names = list(self.model.names)[1:]
        self.link_frame_ids = {
            f.name: self.model.getFrameId(f.name)
            for f in self.model.frames if f.name.startswith("fr3_link")
        }
        self.robot_geom = RobotGeometry(definitions=[])

        # Geometries
        self.human_geometry = HumanArmGeometry(
            upper_arm_radius=0.075, 
            forearm_radius=0.065, 
            hand_radius=0.075
        )

        # Subscribers (best effort, latest only: works with the 1 kHz best-effort feed and with bags)
        self.joint_states_sub = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        # Dynamic subscriptions to human arm states based on tracking mode
        self.arm_state_subs = {}
        for side in self.active_sides:
            prefix = f"{side}_" if self.pose_side == "both" else ""

            s_topic = format_topic('/human/arm_state', prefix)
            self.arm_state_subs[side] = self.create_subscription(
                HumanArmState,
                s_topic,
                partial(self.arm_state_callback, side=side),
                10
            )

        # Publishers
        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.per_link_pub = self.create_publisher(MultiLinkDistance, '/human/per_link_distances', latest_qos)
        self.global_dist_pub = self.create_publisher(Float32, '/human_robot/distance', 10)
        self.get_logger().info(
            f'Human Distance node ready — mode: {self.mode}, tracking: {self.pose_side}, '
            f'joint states: {joint_state_topic}'
        )


    def arm_state_callback(self, msg: HumanArmState, side: str):
        """Stores the latest arm state; computes once all sides share its stamp."""
        self.latest_arm_states[side] = msg
        # The tracker publishes all sides per frame with the same stamp
        if all(
            s is not None and s.header.stamp == msg.header.stamp
            for s in self.latest_arm_states.values()
        ):
            self.distance_loop()

    def joint_state_callback(self, msg: JointState):
        """Stores the latest robot joint states."""
        self.latest_joint_state = msg

    def joint_positions(self):
        """Arm joint positions ordered as the Pinocchio model (None if incomplete)."""
        js = self.latest_joint_state
        name_to_pos = dict(zip(js.name, js.position))
        try:
            return np.array([name_to_pos[n] for n in self.joint_names], dtype=float)
        except KeyError:
            self.get_logger().warn('Joint state misses arm joints.', throttle_duration_sec=2.0)
            return None

    def get_zone(self, distance: float) -> str:
        """Determines the safety zone based on distance."""
        if distance <= self.zones_cfg['critical']:
            return 'critical'
        if distance <= self.zones_cfg['danger']:
            return 'danger'
        return 'warning'


    def distance_loop(self):
        """Hybrid distance computation: Robot Control Points vs All Valid Human Capsules."""
        # Stamp with the camera frame the arm states come from
        header = next(iter(self.latest_arm_states.values())).header

        all_human_capsules = []
        confidence_sum = 0.0
        valid_arms_count = 0

        # Aggregate Capsules from all tracked arms
        for side_index, side in enumerate(self.active_sides):
            state = self.latest_arm_states[side]

            # Any valid keypoint is enough: forearm and hand matter even with the shoulder occluded
            if state is None or not any(state.keypoint_valid):
                continue

            human_kpts, human_vels, human_valid = extract_human_keypoints(state)
            P_pp, P_vv, P_pv, frames_seen = extract_human_covariances(state)
            capsules = self.human_geometry.build_capsules(human_kpts, valid=human_valid)

            # Prefix capsule names to distinguish left/right in distance logs
            prefix = f"{side}_" if len(self.active_sides) > 1 else ""
            for cap in capsules:
                cap['name'] = f"{prefix}{cap['name']}"
                # Capsule k spans keypoints (k, k+1): a stable, nonzero id per side and capsule
                cap['track_id'] = 1 + 3 * side_index + cap['indices'][0]
                cap['velocities'] = human_vels
                cap['covariances'] = (P_pp, P_vv, P_pv)
                cap['frames_seen'] = frames_seen
                
            all_human_capsules.extend(capsules)
            confidence_sum += state.confidence
            valid_arms_count += 1

        # No valid human capsule: empty heartbeat ("nothing near"), no robot state needed
        if not all_human_capsules:
            self.per_link_pub.publish(MultiLinkDistance(header=header))
            return

        # Human present but no robot state: publish nothing, so the controller sees a stale channel
        if self.latest_joint_state is None:
            self.get_logger().warn('No joint states yet: distances not published.', throttle_duration_sec=2.0)
            return
        q = self.joint_positions()
        if q is None:
            return

        avg_confidence = confidence_sum / valid_arms_count

        # Robot Kinematics (Pinocchio FK)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        # Extract Transforms (Rotation and Translation) for all relevant frames
        transforms = {}
        for name, frame_id in self.link_frame_ids.items():
            oMf = self.data.oMf[frame_id]
            transforms[name] = (oMf.rotation.copy(), oMf.translation.copy())

        # Robot Control Points
        robot_cps = define_control_points(transforms, self.robot_cfg, self.dist_cfg)
        
        msg = MultiLinkDistance(header=header)
        global_min_dist = float('inf')
        closest_capsule_name = ""

        # One entry per control point, in the fixed control-point order
        for cp in robot_cps:
            # Compares the single control point against all aggregated human capsules
            info = self.robot_geom.minimum_distance_to_human([cp], all_human_capsules)

            ld = LinkDistance()
            ld.robot_link_name = cp['source_capsule']
            ld.human_capsule = str(info['human_capsule'])
            signed_distance = float(info['distance'])
            ld.distance = max(0.0, signed_distance) if self.clamp_distance else signed_distance

            if signed_distance < global_min_dist:
                global_min_dist = signed_distance
                closest_capsule_name = ld.human_capsule

            fill_track_fields(ld, info['capsule'], info['alpha'])

            # Repulsion direction vector (points from Human to Robot)
            direction_vec = info['robot_position'] - info['closest_human_point']
            norm = np.linalg.norm(direction_vec)
            if norm > 1e-9:
                direction_vec = direction_vec / norm

            # Closest point on the robot (control point, on the segment axis)
            ld.closest_point_robot.x = float(info['robot_position'][0])
            ld.closest_point_robot.y = float(info['robot_position'][1])
            ld.closest_point_robot.z = float(info['robot_position'][2])

            # Closest point on the human
            ld.closest_point_human.x = float(info['closest_human_point'][0])
            ld.closest_point_human.y = float(info['closest_human_point'][1])
            ld.closest_point_human.z = float(info['closest_human_point'][2])

            ld.direction.x = float(direction_vec[0])
            ld.direction.y = float(direction_vec[1])
            ld.direction.z = float(direction_vec[2])
            
            ld.valid = True
            ld.confidence = avg_confidence
            ld.zone = self.get_zone(signed_distance)
            
            msg.links.append(ld)

        self.per_link_pub.publish(msg)
        
        # Publish global distance
        if global_min_dist != float('inf'):
            dist_msg = Float32()
            dist_msg.data = global_min_dist
            self.global_dist_pub.publish(dist_msg)

        side_prefix = get_side(self.active_sides, closest_capsule_name)
        self.get_logger().info(
            f"{side_prefix}Global min: {global_min_dist:.3f} m "
            f"(tracking {valid_arms_count} arms, "
            f"avg confidence: {avg_confidence:.2f})",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = HumanDistance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()