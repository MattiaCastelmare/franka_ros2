#!/usr/bin/env python3
"""
Human Distance Node.
Reads filtered human arm states (supports both single and dual arms dynamically) 
and robot joint states to compute real-time geometric distances using a 
capsule-based representation for CBF control.
"""

import os
import rclpy
from functools import partial
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
    extract_human_keypoints, init_pinocchio_from_xacro, 
    define_control_points, format_topic, get_side
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
        self.tracker_config = load_robot_config(tracker_path)['human_tracker']
        self.distance_loop_rate = float(self.tracker_config["inference_hz"])
        self.pose_side = str(self.tracker_config["pose_side"]).lower()
        self.active_sides = ["left", "right"] if self.pose_side == "both" else [self.pose_side]

        # Internal State
        self.latest_joint_state = None
        self.latest_arm_states = {side: None for side in self.active_sides}

        # Initialize Pinocchio
        self.pin_ok, self.model, self.data = init_pinocchio_from_xacro(self)
        if not self.pin_ok:
            self.get_logger().error('Pinocchio initialization failed. Shutting down.')
            return

        # Geometries
        self.human_geometry = HumanArmGeometry(
            upper_arm_radius=0.075, 
            forearm_radius=0.065, 
            hand_radius=0.075
        )

        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState, 
            '/NS_1/joint_states',
            self.joint_state_callback,
            10,
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
        self.per_link_pub = self.create_publisher(MultiLinkDistance, '/cbf/per_link_distances', latest_qos)
        self.global_dist_pub = self.create_publisher(Float32, '/human_robot/distance', 10)
        self.get_logger().info(f'Human Distance node ready — mode: {self.mode}, tracking: {self.pose_side}')


    def arm_state_callback(self, msg: HumanArmState, side: str):
        """Stores the latest filtered human arm state for a specific side."""
        self.latest_arm_states[side] = msg
        # Trigger distance computation after receiving a new arm state
        self.distance_loop()

    def joint_state_callback(self, msg: JointState):
        """Stores the latest robot joint states."""
        self.latest_joint_state = msg

    def get_zone(self, distance: float) -> str:
        """Determines the safety zone based on distance."""
        if distance <= self.zones_cfg['critical']:
            return 'critical'
        if distance <= self.zones_cfg['danger']:
            return 'danger'
        return 'warning'


    def distance_loop(self):
        """Hybrid distance computation: Robot Control Points vs All Valid Human Capsules."""
        if self.latest_joint_state is None:
            return
            
        all_human_capsules = []
        confidence_sum = 0.0
        valid_arms_count = 0

        # Aggregate Capsules from all tracked arms
        for side in self.active_sides:
            state = self.latest_arm_states[side]
            
            # Check basic human tracking validity (shoulder must be valid)
            if state is None or not state.keypoint_valid[0]:
                continue
                
            human_kpts, human_vels, human_valid = extract_human_keypoints(state)
            capsules = self.human_geometry.build_capsules(human_kpts, valid=human_valid)
            
            # Prefix capsule names to distinguish left/right in distance logs
            prefix = f"{side}_" if len(self.active_sides) > 1 else ""
            for cap in capsules:
                cap['name'] = f"{prefix}{cap['name']}"
                
            all_human_capsules.extend(capsules)
            confidence_sum += state.confidence
            valid_arms_count += 1

        # If no valid human arms are currently tracked, skip distance computation
        if not all_human_capsules:
            self.per_link_pub.publish(MultiLinkDistance())
            return

        avg_confidence = confidence_sum / valid_arms_count

        # Robot Kinematics (Pinocchio FK)
        q = np.array(self.latest_joint_state.position[:7])
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        # Extract Transforms (Rotation and Translation) for all relevant frames
        transforms = {}
        for frame in self.model.frames:
            if frame.name.startswith("fr3_link"):
                frame_id = self.model.getFrameId(frame.name)
                oMf = self.data.oMf[frame_id]
                transforms[frame.name] = (oMf.rotation.copy(), oMf.translation.copy())

        # Robot Control Points
        robot_cps = define_control_points(transforms, self.robot_cfg, self.dist_cfg)
        
        # We reuse the internal geometry logic of RobotGeometry for point-to-capsule math
        robot_geom = RobotGeometry(definitions=[])
        msg = MultiLinkDistance()
        links_dict = {}

        # Minimum Distance Computation (Robot Control Points vs Human Capsules)
        for cp in robot_cps:
            # Compares the single control point against all aggregated human capsules
            best_dist_info = robot_geom.minimum_distance_to_human([cp], all_human_capsules)
            
            if best_dist_info is not None:
                link_name = cp['source_capsule']
                if link_name not in links_dict or best_dist_info['distance'] < links_dict[link_name]['distance']:
                    links_dict[link_name] = best_dist_info
        
        global_min_dist = float('inf')
        closest_capsule_name = ""

        # Populate CBF Message
        for link_name, info in links_dict.items():
            ld = LinkDistance()
            
            ld.robot_link_name = link_name
            ld.human_capsule = str(info['human_capsule'])
            ld.distance = float(info['distance'])
            
            if ld.distance < global_min_dist:
                global_min_dist = ld.distance
                closest_capsule_name = ld.human_capsule

            # Closest point on the robot
            ld.closest_point_robot.x = float(info['robot_position'][0])
            ld.closest_point_robot.y = float(info['robot_position'][1])
            ld.closest_point_robot.z = float(info['robot_position'][2])
            
            # Closest point on the human
            ld.closest_point_human.x = float(info['closest_human_point'][0])
            ld.closest_point_human.y = float(info['closest_human_point'][1])
            ld.closest_point_human.z = float(info['closest_human_point'][2])
            
            # Repulsion direction vector (points from Human to Robot)
            direction_vec = info['robot_position'] - info['closest_human_point']
            norm = np.linalg.norm(direction_vec)
            if norm > 1e-9:
                direction_vec = direction_vec / norm
                
            ld.direction.x = float(direction_vec[0])
            ld.direction.y = float(direction_vec[1])
            ld.direction.z = float(direction_vec[2])
            
            ld.valid = True
            ld.confidence = avg_confidence
            ld.zone = self.get_zone(ld.distance)
            
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()