#!/usr/bin/env python3

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


class ObstacleSynchronizer(Node):
    def __init__(self):
        super().__init__('obstacle_synchronizer')

        # === Parametri di configurazione ===
        self.declare_parameter('obstacles_namespace', '/obstacle')
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('reference_frame', 'base')
        self.declare_parameter('urdf_xacro_path', '')

        self.namespace = self.get_parameter('obstacles_namespace').value
        self.update_rate = self.get_parameter('update_rate').value
        self.reference_frame = self.get_parameter('reference_frame').value

        # Percorso del file Xacro da cui generare l'URDF
        urdf_xacro_param = self.get_parameter('urdf_xacro_path').get_parameter_value().string_value
        if urdf_xacro_param:
            self.urdf_xacro_path = urdf_xacro_param
        else:
            # default (collision_box)
            self.urdf_xacro_path = os.path.join(
                get_package_share_directory('franka_simulation'),
                'urdf', 'obstacles', 'collision_box.urdf.xacro'
            )

        # === TF ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # === Publisher per la scena ===
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # === Timer per pubblicare periodicamente ===
        self.timer = self.create_timer(self.update_rate, self.publish_obstacle_from_xacro)

        self.robot = None
        self.get_logger().info(f"🚀 ObstacleSynchronizer avviato (file: {self.urdf_xacro_path})")

    # ---------------------------------------------------------------------
    def publish_obstacle_from_xacro(self):
        """Carica l'URDF dal file .xacro, interpreta le pose e pubblica la scena"""
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

        self.scene_pub.publish(planning_scene)
        self.get_logger().info(f"🔄 Pubblicati {count} ostacoli dalla scena {self.urdf_xacro_path}")

    # ---------------------------------------------------------------------
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

    # ---------------------------------------------------------------------
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
