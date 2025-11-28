#!/usr/bin/env python3
"""
Obstacle Synchronizer v3 - Pubblica su entrambi i topic
========================================================

Pubblica gli ostacoli su:
1. /obstacle_scene - per l'online_avoidance_controller
2. /planning_scene - per la visualizzazione in RViz

MoveIt NON usa più /planning_scene per il collision checking perché
abbiamo disabilitato avoid_collisions nel motion server.

Author: Modificato per architettura collision avoidance dinamica
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
from urdf_parser_py.urdf import URDF
from rclpy.time import Duration
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os
import subprocess
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


PLANNING_SCENE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ObstacleSynchronizer(Node):
    def __init__(self):
        super().__init__('obstacle_synchronizer')

        # === Parametri di configurazione ===
        self.declare_parameter('obstacles_namespace', '/obstacle')
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('reference_frame', 'base')
        self.declare_parameter('urdf_xacro_path', '')
        self.declare_parameter('obstacle_scene_topic', '/obstacle_scene')
        self.declare_parameter('publish_to_planning_scene', True)  # Per RViz

        self.namespace = self.get_parameter('obstacles_namespace').value
        self.update_rate = self.get_parameter('update_rate').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.obstacle_scene_topic = self.get_parameter('obstacle_scene_topic').value
        self.publish_to_planning_scene = self.get_parameter('publish_to_planning_scene').value

        # Percorso del file Xacro
        urdf_xacro_param = self.get_parameter('urdf_xacro_path').get_parameter_value().string_value
        if urdf_xacro_param:
            self.urdf_xacro_path = urdf_xacro_param
        else:
            self.urdf_xacro_path = os.path.join(
                get_package_share_directory('franka_simulation'),
                'urdf', 'obstacles', 'collision_box.urdf.xacro'
            )

        # === TF ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # === Publisher 1: per online_avoidance_controller ===
        self.obstacle_scene_pub = self.create_publisher(
            PlanningScene,
            self.obstacle_scene_topic,
            PLANNING_SCENE_QOS,
        )

        # === Publisher 2: per RViz (visualizzazione) ===
        if self.publish_to_planning_scene:
            self.planning_scene_pub = self.create_publisher(
                PlanningScene,
                '/planning_scene',
                PLANNING_SCENE_QOS,
            )
        else:
            self.planning_scene_pub = None

        # === Timer per pubblicare periodicamente ===
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_obstacles)

        self.robot = None
        self.get_logger().info(f"🚀 ObstacleSynchronizer v3 avviato")
        self.get_logger().info(f"   📁 File: {self.urdf_xacro_path}")
        self.get_logger().info(f"   📤 Avoidance topic: {self.obstacle_scene_topic}")
        self.get_logger().info(f"   📺 RViz topic: /planning_scene ({'enabled' if self.publish_to_planning_scene else 'disabled'})")
        self.get_logger().info(f"   🔄 Update rate: {self.update_rate} Hz")

    def publish_obstacles(self):
        """Carica l'URDF e pubblica su entrambi i topic."""
        try:
            urdf_xml = subprocess.check_output(['xacro', self.urdf_xacro_path], text=True)
            self.robot = URDF.from_xml_string(urdf_xml)
        except Exception as e:
            self.get_logger().error(f"❌ Errore caricamento Xacro: {e}")
            return

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        count = 0

        for link in self.robot.links:
            if link.name == 'world' or not link.collision:
                continue

            pose = self.extract_pose_from_urdf(link)
            if pose is None:
                continue

            geom = link.collision.geometry
            primitive = SolidPrimitive()

            if hasattr(geom, 'size'):  # Box
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = list(map(float, geom.size))
            elif hasattr(geom, 'radius') and not hasattr(geom, 'length'):  # Sphere
                primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = [float(geom.radius)]
            elif hasattr(geom, 'length') and hasattr(geom, 'radius'):  # Cylinder
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = [float(geom.length), float(geom.radius)]
            else:
                continue

            collision_obj = CollisionObject()
            collision_obj.header.frame_id = self.reference_frame
            collision_obj.header.stamp = self.get_clock().now().to_msg()
            collision_obj.id = link.name
            collision_obj.operation = CollisionObject.ADD
            collision_obj.primitives.append(primitive)
            collision_obj.primitive_poses.append(pose)

            planning_scene.world.collision_objects.append(collision_obj)
            count += 1

        # Pubblica su /obstacle_scene (per avoidance controller)
        self.obstacle_scene_pub.publish(planning_scene)

        # Pubblica su /planning_scene (per RViz)
        if self.planning_scene_pub is not None:
            self.planning_scene_pub.publish(planning_scene)

    def extract_pose_from_urdf(self, link):
        pose = Pose()
        for joint in self.robot.joints:
            if joint.child == link.name and joint.origin:
                def to_floats(v):
                    return [float(x) for x in (v if isinstance(v, (list, tuple)) else str(v).split())]

                xyz = to_floats(joint.origin.xyz)
                pose.position.x, pose.position.y, pose.position.z = xyz

                if joint.origin.rpy:
                    rpy = to_floats(joint.origin.rpy)
                    quat = self.rpy_to_quaternion(*rpy)
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
                else:
                    pose.orientation.w = 1.0
                return pose

        pose.orientation.w = 1.0
        return pose

    def rpy_to_quaternion(self, roll, pitch, yaw):
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSynchronizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()