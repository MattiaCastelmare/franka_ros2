#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from urdf_parser_py.urdf import URDF
import numpy as np
import time
from rclpy.time import Time, Duration


class ObstacleSynchronizer(Node):
    def __init__(self):
        super().__init__('obstacle_synchronizer')
        
        # Parametri
        self.declare_parameter('obstacles_namespace', '/obstacle')
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('reference_frame', 'base')
        
        namespace = self.get_parameter('obstacles_namespace').value
        update_rate = self.get_parameter('update_rate').value
        self.reference_frame = self.get_parameter('reference_frame').value

        self.namespace = namespace
        self.robot = None

        # TF buffer e listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ✅ SUBSCRIBE SUBITO all'URDF (prima di aspettare il TF)
        self.urdf_sub = self.create_subscription(
            String,
            f'{namespace}/robot_description',
            self.urdf_callback,
            10
        )

        # (facoltativo ma utile) piccolo delay per far nascere i TF publishers
        time.sleep(1.0)

        # Attendi TF, ma la subscription è già attiva, non perdi l'URDF
        timeout = 10.0
        start = time.time()
        # while not self.tf_buffer.can_transform('world', 'obstacle/world', Time()):
        #     if time.time() - start > timeout:
        #         self.get_logger().warn("Timeout: TF 'world->obstacle/world' non disponibile, procedo comunque.")
        #         break
        #     self.get_logger().info("Aspettando TF 'world->obstacle/world'...")
        #     time.sleep(0.5)
        self.get_logger().info("🚀 Starting without waiting for TF (assuming it will appear soon).")


        # Publisher per planning scene
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Timer per aggiornare periodicamente
        self.timer = self.create_timer(update_rate, self.sync_urdf_to_planning_scene)

        self.get_logger().info(f'Obstacle Synchronizer started')
        self.get_logger().info(f'Reading from: {namespace}/robot_description')
        self.get_logger().info(f'Reference frame: {self.reference_frame}')


    # ---------------------------------------------------------------------

    def urdf_callback(self, msg):
        try:
            self.robot = URDF.from_xml_string(msg.data)
            obstacle_links = [link.name for link in self.robot.links
                              if link.name != 'world' and link.collision]
            self.get_logger().info(f'Parsed URDF: found {len(obstacle_links)} obstacles {obstacle_links}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse URDF: {str(e)}')

    # ---------------------------------------------------------------------

    def sync_urdf_to_planning_scene(self):
        if not self.robot:
            return

        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        for link in self.robot.links:
            if link.name == 'world' or not link.collision:
                continue

            pose = self.get_pose_from_tf_or_urdf(link)
            if pose is None:
                continue

            collision_obj = CollisionObject()
            collision_obj.header.frame_id = self.reference_frame
            collision_obj.header.stamp = self.get_clock().now().to_msg()
            collision_obj.id = link.name
            collision_obj.operation = CollisionObject.ADD

            geom = link.collision.geometry
            primitive = SolidPrimitive()

            if hasattr(geom, 'size'):  # Box
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = list(geom.size)
            elif hasattr(geom, 'radius') and not hasattr(geom, 'length'):  # Sphere
                primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = [geom.radius]
            elif hasattr(geom, 'length') and hasattr(geom, 'radius'):  # Cylinder
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = [geom.length, geom.radius]
            else:
                continue

            collision_obj.primitives.append(primitive)
            collision_obj.primitive_poses.append(pose)
            planning_scene.world.collision_objects.append(collision_obj)

        if planning_scene.world.collision_objects:
            self.scene_pub.publish(planning_scene)
            self.get_logger().info(
                f"Updated planning scene with {len(planning_scene.world.collision_objects)} obstacles"
            )

    # ---------------------------------------------------------------------

    def get_pose_from_tf_or_urdf(self, link):
        """Prova a leggere la posizione dinamica da TF, fallback al valore URDF."""
        pose = Pose()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                f"{self.namespace}/{link.name}",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except Exception:
            # Fallback statico dal modello URDF
            for joint in self.robot.joints:
                if joint.child == link.name and joint.origin:
                    pose.position.x, pose.position.y, pose.position.z = joint.origin.xyz
                    if joint.origin.rpy:
                        quat = self.rpy_to_quaternion(*joint.origin.rpy)
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
