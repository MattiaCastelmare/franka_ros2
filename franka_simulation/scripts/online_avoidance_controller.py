#!/usr/bin/env python3
"""
ONLINE AVOIDANCE CONTROLLER — NULL SPACE VERSION
===============================================

• Capsule-based distance estimation
• Potential field used ONLY as direction metric
• Avoidance projected in EE null space
• Tracking task is NEVER opposed
• No local minima blocking

Author: Maurizio (Null-space refactor)
"""

import os
import tempfile
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from moveit_msgs.msg import PlanningScene
from rcl_interfaces.srv import GetParameters

import pinocchio as pin


class NullSpaceAvoidance(Node):

    def __init__(self):
        super().__init__("online_avoidance_controller")

        # ================= PARAMETERS =================
        # Dichiara i parametri con defaults (Humble requirement, override dal YAML)
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("influence_distance", 0.30)
        self.declare_parameter("safety_margin", 0.08)
        self.declare_parameter("nullspace_gain", 0.15)
        self.declare_parameter("max_joint_velocity", 0.25)
        self.declare_parameter("excluded_obstacles", ["ground_plane", "ground", "floor", "plane"])
        
        # Carica i valori dal YAML (ora il nodo sa dove trovarli)
        self.rate = float(self.get_parameter("control_rate").value)
        self.d_infl = float(self.get_parameter("influence_distance").value)
        self.d_safe = float(self.get_parameter("safety_margin").value)
        self.k_null = float(self.get_parameter("nullspace_gain").value)
        self.max_qdot = float(self.get_parameter("max_joint_velocity").value)
        self.excluded = list(self.get_parameter("excluded_obstacles").value)

        self.get_logger().info(f"📊 Parametri CARICATI (da file YAML o default):")
        self.get_logger().info(f"   d_infl (influence_distance): {self.d_infl}")
        self.get_logger().info(f"   d_safe (safety_margin): {self.d_safe}")
        self.get_logger().info(f"   k_null (nullspace_gain): {self.k_null}")
        self.get_logger().info(f"   max_qdot (max_joint_velocity): {self.max_qdot}")

        # ================= CAPSULE GEOMETRY =================
        self.link_pairs = [
            ("fr3_link1", "fr3_link2"),
            ("fr3_link2", "fr3_link3"),
            ("fr3_link3", "fr3_link4"),
            ("fr3_link4", "fr3_link5"),
            ("fr3_link5", "fr3_link6"),
            ("fr3_link6", "fr3_link7"),
            ("fr3_link7", "fr3_link8"),
        ]

        # Raggi delle 3 capsule sovrapposte per ogni link
        # [zona giunto, corpo, verso giunto successivo]
        self.capsule_radii = [0.15, 0.12, 0.13]
        self.capsules = {}

        # ================= STATE =================
        self.q = None
        self.frame_ids = {}
        self.obstacles = []
        self.pin_ok = False
        self.marker_id_counter = 0  # Contatore stabile per marker ID
        self.distances_data = []    # Lista di (capsula_p0, capsula_p1, obs_point, distance)

        # ================= PINOCCHIO =================
        self._init_pinocchio_and_capsules()

        # ================= RViz CAPSULE VISUALIZATION =================
        # QoS standard per compatibilità con RViz
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.capsule_marker_pub = self.create_publisher(
            MarkerArray,
            "/robot_capsules_markers",
            marker_qos
        )
        self.last_marker_array = MarkerArray()  # Cache ultimi marker

        # ================= ROS =================
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(PlanningScene, "/obstacle_scene", self._obstacle_cb, 1)

        self.pub = self.create_publisher(Float64MultiArray, "/avoidance/velocity", 10)
        self.min_dist_pub = self.create_publisher(Float64MultiArray, "/avoidance/min_distance", 10)

        # Control loop @ 100 Hz
        self.create_timer(1.0 / self.rate, self._control_loop)
        # Marker visualization @ 10 Hz (ridotto per evitare DDS buffer overflow)
        self.create_timer(0.1, self._publish_markers_only)

        self.get_logger().info("🟢 Null-Space Avoidance Controller READY")

    # ======================================================
    # PINOCCHIO + CAPSULES
    # ======================================================
    def _init_pinocchio_and_capsules(self):
        cli = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        cli.wait_for_service()

        req = GetParameters.Request()
        req.names = ["robot_description"]
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        urdf = future.result().values[0].string_value

        with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as f:
            f.write(urdf.encode())
            urdf_path = f.name

        model_full = pin.buildModelFromUrdf(urdf_path)
        os.unlink(urdf_path)

        lock = [model_full.getJointId(n) for n in model_full.names if "finger" in n]
        self.model = pin.buildReducedModel(model_full, lock, pin.neutral(model_full))
        self.data = self.model.createData()

        for parent, child in self.link_pairs:
            for link in (parent, child):
                if link not in self.frame_ids:
                    self.frame_ids[link] = self.model.getFrameId(link)

        q0 = pin.neutral(self.model)
        pin.forwardKinematics(self.model, self.data, q0)
        pin.updateFramePlacements(self.model, self.data)

        for parent, child in self.link_pairs:
            fid_p = self.frame_ids[parent]
            fid_c = self.frame_ids[child]

            oMp = self.data.oMf[fid_p]
            oMc = self.data.oMf[fid_c]

            p_child_local = oMp.rotation.T @ (oMc.translation - oMp.translation)

            # Catena di 3 capsule sovrapposte
            self.capsules[parent] = [
                {
                    "p0": 0.00 * p_child_local,
                    "p1": 0.35 * p_child_local,
                    "radius": self.capsule_radii[0],
                },
                {
                    "p0": 0.25 * p_child_local,
                    "p1": 0.75 * p_child_local,
                    "radius": self.capsule_radii[1],
                },
                {
                    "p0": 0.60 * p_child_local,
                    "p1": 0.95 * p_child_local,
                    "radius": self.capsule_radii[2],
                },
            ]

        self.pin_ok = True

    # ======================================================
    # CALLBACKS
    # ======================================================
    def _joint_cb(self, msg: JointState):
        order = [
            "fr3_joint1", "fr3_joint3", "fr3_joint2",
            "fr3_joint4", "fr3_joint6", "fr3_joint5", "fr3_joint7"
        ]

        try:
            q_ros = np.array([msg.position[msg.name.index(n)] for n in order])
        except ValueError:
            return

        self.q = np.array([
            q_ros[0], q_ros[2], q_ros[1],
            q_ros[3], q_ros[5], q_ros[4], q_ros[6]
        ])

    def _obstacle_cb(self, msg: PlanningScene):
        self.obstacles = [
            o for o in msg.world.collision_objects
            if not any(ex in o.id.lower() for ex in self.excluded)
        ]

    # ======================================================
    # CAPSULE ↔ BOX DISTANCE
    # ======================================================
    def _distance_capsule_to_box(self, p0, p1, r, obs):
        best_d = 1e6
        best_dir = None
        best_p_seg = None  # Punto sulla capsula
        best_p_box = None  # Punto sull'ostacolo

        v = p1 - p0
        L = np.linalg.norm(v)
        if L < 1e-6:
            return best_d, None, None, None

        v /= L

        for i, prim in enumerate(obs.primitives):
            if prim.type != prim.BOX:
                continue

            pose = obs.primitive_poses[i]
            center = np.array([pose.position.x, pose.position.y, pose.position.z])
            half = np.array(prim.dimensions) / 2.0

            t = np.clip(np.dot(center - p0, v), 0, L)
            p_seg = p0 + t * v
            p_box = center + np.clip(p_seg - center, -half, +half)

            diff = p_seg - p_box
            dist = np.linalg.norm(diff) - r

            if dist < best_d:
                best_d = dist
                best_dir = diff / (np.linalg.norm(diff) + 1e-6)
                best_p_seg = p_seg
                best_p_box = p_box

        return best_d, best_dir, best_p_seg, best_p_box

    # ======================================================
    # CAPSULE MARKER (RViz)
    # ======================================================
    def _make_capsule_markers(self, p0, p1, radius, marker_id):
        markers = []

        # ----- cilindro -----
        cyl = Marker()
        cyl.header.frame_id = "world"
        cyl.header.stamp = self.get_clock().now().to_msg()
        cyl.ns = "capsules"
        cyl.id = marker_id
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD

        center = (p0 + p1) / 2.0
        height = float(np.linalg.norm(p1 - p0))

        cyl.pose.position.x = float(center[0])
        cyl.pose.position.y = float(center[1])
        cyl.pose.position.z = float(center[2])

        direction = p1 - p0
        norm_dir = np.linalg.norm(direction)

        if norm_dir < 1e-6:
            q = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            z_axis = np.array([0.0, 0.0, 1.0])
            v = np.cross(z_axis, direction / norm_dir)
            c = np.dot(z_axis, direction / norm_dir)

            if c <= -1.0 + 1e-8:
                q = np.array([1.0, 0.0, 0.0, 0.0])
            else:
                s = np.sqrt((1.0 + c) * 2.0)
                q = np.array([v[0] / s, v[1] / s, v[2] / s, s / 2.0])

        cyl.pose.orientation.x = float(q[0])
        cyl.pose.orientation.y = float(q[1])
        cyl.pose.orientation.z = float(q[2])
        cyl.pose.orientation.w = float(q[3])

        cyl.scale.x = 2.0 * radius
        cyl.scale.y = 2.0 * radius
        cyl.scale.z = height
        cyl.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.5)

        markers.append(cyl)

        # ----- semisfere -----
        for idx, pos in enumerate([p0, p1], start=1):
            sph = Marker()
            sph.header.frame_id = "world"
            sph.header.stamp = self.get_clock().now().to_msg()
            sph.ns = "capsules"
            sph.id = marker_id + idx
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position.x = float(pos[0])
            sph.pose.position.y = float(pos[1])
            sph.pose.position.z = float(pos[2])
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 2.0 * radius
            sph.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.5)
            markers.append(sph)

        return markers

    # ======================================================
    # CONTROL LOOP (NULL SPACE)
    # ======================================================
    def _control_loop(self):
        zero = Float64MultiArray()
        zero.data = [0.0] * 7

        if not (self.pin_ok and isinstance(self.q, np.ndarray)):
            self.pub.publish(zero)
            return

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        qdot_ns = np.zeros(7)
        d_min = 999.0
        self.distances_data = []  # Resetta la lista di distanze

        for parent in self.capsules:
            fid = self.frame_ids[parent]

            for caps in self.capsules[parent]:
                oMp = self.data.oMf[fid]
                p0 = oMp.translation + oMp.rotation @ caps["p0"]
                p1 = oMp.translation + oMp.rotation @ caps["p1"]

                for obs in self.obstacles:
                    d, dir_vec, p_seg, p_box = self._distance_capsule_to_box(p0, p1, caps["radius"], obs)
                    if dir_vec is None:
                        continue

                    d_min = min(d_min, d)
                    
                    # Accumula i dati di distanza per la visualizzazione
                    self.distances_data.append({
                        "p_capsule": p_seg,
                        "p_obstacle": p_box,
                        "distance": d,
                    })

                    if d >= self.d_infl:
                        continue

                    alpha = np.clip((self.d_infl - d) / (self.d_infl - self.d_safe), 0, 1)
                    xdot_avoid = self.k_null * alpha * dir_vec

                    J = pin.computeFrameJacobian(
                        self.model, self.data, self.q, fid, pin.ReferenceFrame.WORLD
                    )[:3, :]

                    J_pinv = np.linalg.pinv(J)
                    N = np.eye(7) - J_pinv @ J

                    qdot_raw = J.T @ xdot_avoid
                    qdot_ns += N @ qdot_raw

        if np.linalg.norm(qdot_ns) > self.max_qdot:
            qdot_ns *= self.max_qdot / np.linalg.norm(qdot_ns)

        self.pub.publish(Float64MultiArray(data=qdot_ns.tolist()))
        self.min_dist_pub.publish(Float64MultiArray(data=[float(d_min)]))

        # ================= RViz CAPSULE VISUALIZATION =================
        marker_array = MarkerArray()
        marker_id = 0

        for parent in self.capsules:
            fid = self.frame_ids[parent]

            for caps in self.capsules[parent]:
                oMp = self.data.oMf[fid]
                p0 = oMp.translation + oMp.rotation @ caps["p0"]
                p1 = oMp.translation + oMp.rotation @ caps["p1"]

                markers = self._make_capsule_markers(
                    p0, p1, caps["radius"], marker_id
                )

                marker_array.markers.extend(markers)
                marker_id += len(markers)

        # ================= RViz DISTANCE VISUALIZATION =================
        debug_count = 0
        for dist_data in self.distances_data:
            p_cap = dist_data["p_capsule"]
            p_obs = dist_data["p_obstacle"]
            d = dist_data["distance"]

            # Determina il colore in base alla distanza di influenza
            
            if d < self.d_infl:
                # Rosso: dentro la zona di influenza (avoidance attiva)
                color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
                color_name = "RED"
            else:
                # Blu: fuori dalla zona di influenza (distanza sicura)
                color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
                color_name = "BLUE"

            # Log per debug (una volta ogni 50 cicli per non spammare)
            if debug_count == 0:
                self.get_logger().debug(f"   Distance: {d:.4f}m (d_infl={self.d_infl:.4f}) → {color_name}")
            debug_count = (debug_count + 1) % 50

            # Linea di distanza (capsula ↔ ostacolo)
            line_marker = Marker()
            line_marker.header.frame_id = "world"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "distances"
            line_marker.id = marker_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.005  # Spessore linea
            line_marker.color = color

            p1_point = Point()
            p1_point.x, p1_point.y, p1_point.z = float(p_cap[0]), float(p_cap[1]), float(p_cap[2])
            p2_point = Point()
            p2_point.x, p2_point.y, p2_point.z = float(p_obs[0]), float(p_obs[1]), float(p_obs[2])

            line_marker.points = [p1_point, p2_point]
            marker_array.markers.append(line_marker)
            marker_id += 1

            # Marker di testo con il valore della distanza
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "distances_text"
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.02  # Altezza del testo
            text_marker.color = color

            # Posiziona il testo al punto medio
            mid_point = (p_cap + p_obs) / 2.0
            text_marker.pose.position.x = float(mid_point[0])
            text_marker.pose.position.y = float(mid_point[1])
            text_marker.pose.position.z = float(mid_point[2])
            text_marker.pose.orientation.w = 1.0
            text_marker.text = f"{d:.3f}m"

            marker_array.markers.append(text_marker)
            marker_id += 1

        # Cache i marker per pubblicarli a frequenza ridotta (10 Hz)
        self.last_marker_array = marker_array

    def _publish_markers_only(self):
        """Pubblica marker a 10 Hz (non 100 Hz) per ridurre DDS buffer overflow."""
        if len(self.last_marker_array.markers) > 0:
            self.capsule_marker_pub.publish(self.last_marker_array)
            self.get_logger().debug(f"📍 Pubblicati {len(self.last_marker_array.markers)} marker")


# ======================================================
# MAIN
# ======================================================
def main(args=None):
    rclpy.init(args=args)
    node = NullSpaceAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
