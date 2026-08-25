#!/usr/bin/env python3
"""
Human Distance Node.
Reads filtered human arm state and robot joint states to compute real-time
geometric distances using a capsule-based representation for CBF control.
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import numpy as np
import pinocchio as pin

from franka_msgs.msg import HumanArmState, LinkDistance, MultiLinkDistance
from franka_experiments.utils.cbf_utils import load_robot_config
from franka_experiments.utils.human_utils import extract_human_keypoints, init_pinocchio_from_xacro
from franka_experiments.utils.capsule_geometry import HumanArmGeometry, RobotGeometry


class HumanDistance(Node):
    def __init__(self):
        super().__init__('human_distance_node')

        # --- Parameters ---
        self.distance_loop_rate = 30.0 
        
        # --- Internal State ---
        self.latest_arm_state = None
        self.latest_joint_state = None

        self.config = load_robot_config('complete')
        self.robot_cfg = self.config['robot']
        self.zones_cfg = self.config['zones']

        self.declare_parameter('mode', 'capsules')
        self.mode = self.get_parameter('mode').value

        # -- Initialize Pinocchio ---
        self.pin_ok, self.model, self.data = init_pinocchio_from_xacro(self)
        if not self.pin_ok:
            self.get_logger().error('Pinocchio initialization failed. Shutting down.')
            return

        # --- Geometries ---
        self.human_geometry = HumanArmGeometry(
            upper_arm_radius=0.075, 
            forearm_radius=0.065, 
            hand_radius=0.075
        )
        
        # --- Build Robot Capsules dynamically ---
        robot_defs = []
        for seg in self.robot_cfg.get('segments', []):
            if seg.get('control_points', 0) > 0:
                robot_defs.append({
                    "name": f"{seg['start_link']}_to_{seg['end_link']}",
                    "start": seg['start_link'],
                    "end": seg['end_link'],
                    "radius": seg['radius'],
                    "num_points": seg.get('control_points', 3)
                })
        
        self.robot_geometry = RobotGeometry(definitions=robot_defs)

        # --- Subscribers ---
        self.joint_states_sub = self.create_subscription(
            JointState, 
            '/NS_1/joint_states',
            self.joint_state_callback,
            10,
        )

        self.sub_state = self.create_subscription(
            HumanArmState,
            '/human/arm_state',
            self.arm_state_callback,
            10
        )

        # --- Publishers ---
        self.per_link_pub = self.create_publisher(MultiLinkDistance, '/cbf/per_link_distances', 10)
        self.global_dist_pub = self.create_publisher(Float32, '/human_robot/distance', 10)
        
        # --- Timer ---
        self.timer = self.create_timer(1.0 / self.distance_loop_rate, self.distance_loop)

        self.get_logger().info(f'Human Distance node ready — mode: {self.mode}')


    def arm_state_callback(self, msg: HumanArmState):
        """Stores the latest filtered human arm state."""
        self.latest_arm_state = msg

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
        """Hybrid distance computation: Robot Control Points vs Human Capsules."""
        if self.latest_arm_state is None or self.latest_joint_state is None:
            return
            
        # Check basic human tracking validity (shoulder must be valid)
        if not self.latest_arm_state.keypoint_valid[0]: 
            return

        # Human Capsules
        human_kpts, human_vels, human_valid = extract_human_keypoints(self.latest_arm_state)
        human_capsules = self.human_geometry.build_capsules(human_kpts, valid=human_valid)

        # Robot Kinematics
        q = np.array(self.latest_joint_state.position[:7])
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        # Extract cartesian positions of the frames specified in the robot configuration
        frame_points = {}
        for frame in self.model.frames:
            if frame.name.startswith("fr3_link"):
                frame_points[frame.name] = self.data.oMf[frame.parent].translation

        # Robot Control Points
        robot_capsules = self.robot_geometry.build_capsules(frame_points)
        robot_cps = self.robot_geometry.build_control_points(robot_capsules, points_per_capsule=3)

        msg = MultiLinkDistance()
        links_dict = {}
        
        # Group control points by link to find the absolute minimum distance for each robot segment
        for cp in robot_cps:
            best_dist_info = self.robot_geometry.minimum_distance_to_human([cp], human_capsules)
            
            if best_dist_info is not None:
                link_name = cp['source_capsule']
                # Store it if it's the first measurement or the closest one found so far
                if link_name not in links_dict or best_dist_info['distance'] < links_dict[link_name]['distance']:
                    links_dict[link_name] = best_dist_info
        
        # Translate the dictionary into the CBF compatible ROS array
        global_min_dist = float('inf')

        for link_name, info in links_dict.items():
            ld = LinkDistance()
            
            # The start_link acts as the main reference for the Jacobian in the CBF filter
            ld.robot_link_name = link_name.split('_to_')[0] 
            ld.distance = float(info['distance'])
            
            # Track global minimum for visualization/logging
            if ld.distance < global_min_dist:
                global_min_dist = ld.distance

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
            ld.confidence = self.latest_arm_state.confidence
            ld.zone = self.get_zone(ld.distance)
            
            msg.links.append(ld)

        self.per_link_pub.publish(msg)
        
        # Publish global distance
        if global_min_dist != float('inf'):
            dist_msg = Float32()
            dist_msg.data = global_min_dist
            self.global_dist_pub.publish(dist_msg)

        self.get_logger().info(
            f"Global min: {global_min_dist:.3f} m",
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