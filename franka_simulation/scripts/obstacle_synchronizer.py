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
from rclpy.time import Time
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
        # IMPORTANT:
        # The online avoidance controller and its visualization assume obstacle poses are in "world".
        # If obstacles are published in another frame without transforming, avoidance will not trigger.
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('urdf_xacro_path', '')
        self.declare_parameter('obstacle_scene_topic', '/obstacle_scene')
        self.declare_parameter('publish_to_planning_scene', True)  # Per RViz
        # Also publish individual CollisionObject messages (MoveIt PlanningSceneInterface).
        # This is often the most robust way to feed obstacles into MoveIt for collision-aware planning.
        self.declare_parameter('publish_to_collision_object', True)
        self.declare_parameter('collision_object_topic', '/collision_object')
        # Frame used for the RViz/MoveIt planning scene publication.
        # Using the robot base frame avoids TF-time issues with simulated time and stops
        # MoveIt from spamming "Unknown frame: world".
        self.declare_parameter('planning_scene_frame', 'fr3_link0')

        self.namespace = self.get_parameter('obstacles_namespace').value
        self.update_rate = self.get_parameter('update_rate').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.obstacle_scene_topic = self.get_parameter('obstacle_scene_topic').value
        self.publish_to_planning_scene = self.get_parameter('publish_to_planning_scene').value
        self.planning_scene_frame = self.get_parameter('planning_scene_frame').value
        self.publish_to_collision_object = bool(self.get_parameter('publish_to_collision_object').value)
        self.collision_object_topic = str(self.get_parameter('collision_object_topic').value)

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

        # Publisher 3: /collision_object (MoveIt PlanningSceneInterface)
        # Use TRANSIENT_LOCAL so late-joining nodes still get the latest objects.
        if self.publish_to_collision_object:
            self.collision_object_pub = self.create_publisher(
                CollisionObject,
                self.collision_object_topic,
                PLANNING_SCENE_QOS,
            )
        else:
            self.collision_object_pub = None

        # === Timer per pubblicare periodicamente ===
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_obstacles)

        self.robot = None
        self.get_logger().info(f"🚀 ObstacleSynchronizer v3 avviato")
        self.get_logger().info(f"   📁 File: {self.urdf_xacro_path}")
        self.get_logger().info(f"   📤 Avoidance topic: {self.obstacle_scene_topic}")
        self.get_logger().info(f"   📺 RViz topic: /planning_scene ({'enabled' if self.publish_to_planning_scene else 'disabled'})")
        self.get_logger().info(
            f"   🧱 collision_object topic: {self.collision_object_topic} ({'enabled' if self.publish_to_collision_object else 'disabled'})"
        )
        self.get_logger().info(f"   🔄 Update rate: {self.update_rate} Hz")
        self.get_logger().info(f"   🧭 reference_frame: {self.reference_frame} (published in world frame)")
        if self.publish_to_planning_scene:
            self.get_logger().info(f"   🧭 planning_scene_frame: {self.planning_scene_frame}")

    @staticmethod
    def _quat_multiply(q1, q2):
        """Quaternion multiply (x,y,z,w). Returns q = q1*q2."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array(
            [
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            ],
            dtype=float,
        )

    @staticmethod
    def _quat_to_rot(q):
        """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
        x, y, z, w = [float(v) for v in q]
        n = np.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-12:
            return np.eye(3)
        x /= n
        y /= n
        z /= n
        w /= n
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=float,
        )

    def _transform_pose(self, pose_in: Pose, from_frame: str, to_frame: str) -> Pose:
        """Transform a Pose between frames using TF2.

        NOTE: We use Time() (latest) to avoid tight coupling to message stamps.
        """
        if from_frame == to_frame:
            return pose_in

        try:
            tf = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().warn(
                f"⚠️ TF unavailable: cannot transform pose {from_frame} -> {to_frame}: {e}. "
                "Publishing pose without transform."
            )
            return pose_in

        t = tf.transform.translation
        r = tf.transform.rotation
        t_xyz = np.array([t.x, t.y, t.z], dtype=float)
        q_tf = np.array([r.x, r.y, r.z, r.w], dtype=float)
        R = self._quat_to_rot(q_tf)

        p = np.array([pose_in.position.x, pose_in.position.y, pose_in.position.z], dtype=float)
        p_out = t_xyz + R @ p

        q_in = np.array(
            [pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z, pose_in.orientation.w],
            dtype=float,
        )
        q_out = self._quat_multiply(q_tf, q_in)

        out = Pose()
        out.position.x = float(p_out[0])
        out.position.y = float(p_out[1])
        out.position.z = float(p_out[2])
        out.orientation.x = float(q_out[0])
        out.orientation.y = float(q_out[1])
        out.orientation.z = float(q_out[2])
        out.orientation.w = float(q_out[3])
        return out

    def _transform_pose_to_world(self, pose_in: Pose) -> Pose:
        """Transform a Pose expressed in self.reference_frame into the 'world' frame."""
        return self._transform_pose(pose_in, self.reference_frame, 'world')

    def publish_obstacles(self):
        """Carica l'URDF e pubblica su entrambi i topic."""
        try:
            urdf_xml = subprocess.check_output(['xacro', self.urdf_xacro_path], text=True)
            self.robot = URDF.from_xml_string(urdf_xml)
        except Exception as e:
            self.get_logger().error(f"❌ Errore caricamento Xacro: {e}")
            return

        # 1) World-framed scene for online avoidance (/obstacle_scene)
        planning_scene_world = PlanningScene()
        planning_scene_world.is_diff = True

        # 2) Robot-base-framed scene for MoveIt/RViz (/planning_scene)
        planning_scene_moveit = PlanningScene()
        planning_scene_moveit.is_diff = True
        count = 0

        for link in self.robot.links:
            if link.name == 'world' or not link.collision:
                continue

            pose = self.extract_pose_from_urdf(link)
            if pose is None:
                continue

            # Ensure published poses are in world frame for consumers that assume world.
            pose_world = self._transform_pose_to_world(pose)

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

            # World publication (for avoidance controller)
            co_world = CollisionObject()
            co_world.header.frame_id = 'world'
            co_world.header.stamp = self.get_clock().now().to_msg()
            co_world.id = link.name
            co_world.operation = CollisionObject.ADD
            co_world.primitives.append(primitive)
            co_world.primitive_poses.append(pose_world)
            planning_scene_world.world.collision_objects.append(co_world)

            # MoveIt/RViz publication (use robot base frame to avoid TF time issues)
            if self.planning_scene_pub is not None:
                pose_ps = self._transform_pose(pose_world, 'world', str(self.planning_scene_frame))
                co_ps = CollisionObject()
                co_ps.header.frame_id = str(self.planning_scene_frame)
                co_ps.header.stamp = self.get_clock().now().to_msg()
                co_ps.id = link.name
                co_ps.operation = CollisionObject.ADD
                co_ps.primitives.append(primitive)
                co_ps.primitive_poses.append(pose_ps)
                planning_scene_moveit.world.collision_objects.append(co_ps)

                # Also publish via /collision_object so MoveIt planning reliably sees it.
                if self.collision_object_pub is not None:
                    try:
                        self.collision_object_pub.publish(co_ps)
                    except Exception:
                        pass

            count += 1

        # Pubblica su /obstacle_scene (per avoidance controller)
        self.obstacle_scene_pub.publish(planning_scene_world)

        # Pubblica su /planning_scene (per RViz)
        if self.planning_scene_pub is not None:
            self.planning_scene_pub.publish(planning_scene_moveit)

        # Lightweight rate-limited log
        self.get_logger().debug(f"📦 Published {count} obstacles to {self.obstacle_scene_topic} (frame=world)")

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
        try:
            node.destroy_node()
        except Exception:
            pass
        # ros2 launch can already have shut down the default context.
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()