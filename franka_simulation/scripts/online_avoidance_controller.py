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
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from moveit_msgs.msg import PlanningScene
from rcl_interfaces.srv import GetParameters

import pinocchio as pin


class NullSpaceAvoidance(Node):

    def __init__(self):
        super().__init__("online_avoidance_controller")

        # Nomi giunti (ordine canonico usato anche dal controller ros2_control)
        self.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ]

        # ================= PARAMETERS =================
        # Dichiara i parametri con defaults (Humble requirement, override dal YAML)
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("influence_distance", 0.30)
        self.declare_parameter("safety_margin", 0.08)
        self.declare_parameter("nullspace_gain", 0.15)
        self.declare_parameter("max_joint_velocity", 0.25)
        self.declare_parameter("excluded_obstacles", ["ground_plane", "ground", "floor", "plane"])

        # Extra safety layers (approximate but fast): ground + self-collision
        self.declare_parameter("enable_ground_avoidance", True)
        self.declare_parameter("ground_z", 0.0)  # world Z of the floor plane
        self.declare_parameter("ground_influence_distance", 0.15)
        self.declare_parameter("ground_safety_margin", 0.05)
        self.declare_parameter("ground_gain", 0.25)

        self.declare_parameter("enable_self_collision_avoidance", True)
        self.declare_parameter("self_influence_distance", 0.12)
        self.declare_parameter("self_safety_margin", 0.03)
        self.declare_parameter("self_gain", 0.25)
        # Skip capsule pairs belonging to links closer than this in the kinematic chain
        self.declare_parameter("self_skip_adjacent_links", 1)
        
        # Carica i valori dal YAML (ora il nodo sa dove trovarli)
        self.rate = float(self.get_parameter("control_rate").value)
        self.d_infl = float(self.get_parameter("influence_distance").value)
        self.d_safe = float(self.get_parameter("safety_margin").value)
        self.k_null = float(self.get_parameter("nullspace_gain").value)
        self.max_qdot = float(self.get_parameter("max_joint_velocity").value)
        self.excluded = list(self.get_parameter("excluded_obstacles").value)

        self.enable_ground = bool(self.get_parameter("enable_ground_avoidance").value)
        self.ground_z = float(self.get_parameter("ground_z").value)
        self.ground_infl = float(self.get_parameter("ground_influence_distance").value)
        self.ground_safe = float(self.get_parameter("ground_safety_margin").value)
        self.k_ground = float(self.get_parameter("ground_gain").value)

        self.enable_self = bool(self.get_parameter("enable_self_collision_avoidance").value)
        self.self_infl = float(self.get_parameter("self_influence_distance").value)
        self.self_safe = float(self.get_parameter("self_safety_margin").value)
        self.k_self = float(self.get_parameter("self_gain").value)
        self.self_skip_adjacent = int(self.get_parameter("self_skip_adjacent_links").value)

        self.get_logger().info(f"📊 Parametri CARICATI (da file YAML o default):")
        self.get_logger().info(f"   d_infl (influence_distance): {self.d_infl}")
        self.get_logger().info(f"   d_safe (safety_margin): {self.d_safe}")
        self.get_logger().info(f"   k_null (nullspace_gain): {self.k_null}")
        self.get_logger().info(f"   max_qdot (max_joint_velocity): {self.max_qdot}")
        self.get_logger().info(
            "   extra safety: "
            f"ground(enable={self.enable_ground}, z={self.ground_z}, d_infl={self.ground_infl}, d_safe={self.ground_safe}, k={self.k_ground}) | "
            f"self(enable={self.enable_self}, d_infl={self.self_infl}, d_safe={self.self_safe}, k={self.k_self}, skip_adj={self.self_skip_adjacent})"
        )


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
        self.capsules = {}
        # Manteniamo i raggi originali (come richiesto):
        # NOTE: se l'autocollisione è abilitata, questi raggi possono generare vincoli molto conservativi.
        self.capsule_radii = [0.15, 0.12, 0.13]

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
        # riga di Jacobiano (1x7) del punto più critico: d_dot ≈ j_row @ qdot
        self.jac_pub = self.create_publisher(Float64MultiArray, "/avoidance/jacobian", 10)
        # Debug/diagnostics: which hazard is currently the most critical (helps explain stalls)
        self.hazard_pub = self.create_publisher(String, "/avoidance/hazard", 10)

        # Control loop @ 100 Hz
        self.create_timer(1.0 / self.rate, self._control_loop)
        # Marker visualization @ 10 Hz (ridotto per evitare DDS buffer overflow)
        self.create_timer(0.1, self._publish_markers_only)

        self.get_logger().info("🟢 Null-Space Avoidance Controller READY")

    # ======================================================
    # MATH UTILS
    # ======================================================
    @staticmethod
    def _skew(v: np.ndarray) -> np.ndarray:
        """Skew-symmetric matrix such that skew(v) @ w == v x w."""
        return np.array(
            [
                [0.0, -v[2], v[1]],
                [v[2], 0.0, -v[0]],
                [-v[1], v[0], 0.0],
            ],
            dtype=float,
        )

    @staticmethod
    def _quat_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
        """Quaternion (x,y,z,w) to 3x3 rotation matrix."""
        # Normalize to avoid numerical issues
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

    def _point_jacobian_world(self, fid: int, p_world: np.ndarray) -> np.ndarray:
        """3x7 Jacobian of a point rigidly attached to frame fid, expressed in WORLD."""
        J6 = pin.computeFrameJacobian(
            self.model, self.data, self.q, fid, pin.ReferenceFrame.WORLD
        )
        Jv = J6[:3, :]
        Jw = J6[3:, :]

        oMf = self.data.oMf[fid]
        r = (p_world - oMf.translation).reshape(3)
        # v_point = v_origin + w x r = v_origin - skew(r) @ w
        return Jv - (self._skew(r) @ Jw)

    @staticmethod
    def _closest_points_on_segments(p0: np.ndarray, p1: np.ndarray,
                                    q0: np.ndarray, q1: np.ndarray):
        """Return closest points (cp_p, cp_q) between segments p0-p1 and q0-q1."""
        u = p1 - p0
        v = q1 - q0
        w0 = p0 - q0

        a = float(u @ u)
        b = float(u @ v)
        c = float(v @ v)
        d = float(u @ w0)
        e = float(v @ w0)

        denom = a * c - b * b
        s = 0.0
        t = 0.0

        if denom > 1e-12:
            s = (b * e - c * d) / denom
            t = (a * e - b * d) / denom

        s = float(np.clip(s, 0.0, 1.0))
        t = float(np.clip(t, 0.0, 1.0))

        cp_p = p0 + s * u
        cp_q = q0 + t * v
        return cp_p, cp_q

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
        """Estrae q in ordine [fr3_joint1..fr3_joint7] usando i nomi (non l'ordine del messaggio)."""
        try:
            name_to_idx = {n: i for i, n in enumerate(msg.name)}
            self.q = np.array([msg.position[name_to_idx[n]] for n in self.joint_names], dtype=float)
        except (KeyError, IndexError):
            # Messaggio incompleto o senza alcuni giunti: ignora.
            return

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
            center = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            half = np.array(prim.dimensions, dtype=float) / 2.0

            # Oriented box: use pose orientation
            q = pose.orientation
            R = self._quat_to_rot_matrix(q.x, q.y, q.z, q.w)

            # Sample points along the capsule segment to approximate closest point to OBB.
            # This is robust for OBBs and still cheap at 100 Hz for a small number of obstacles.
            n_samples = 7
            for s in np.linspace(0.0, L, n_samples):
                p_seg = p0 + s * v
                p_local = R.T @ (p_seg - center)
                p_clamped = np.clip(p_local, -half, +half)
                p_box = center + R @ p_clamped

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

        jac_zero = Float64MultiArray()
        jac_zero.data = [0.0] * 7

        if not (self.pin_ok and isinstance(self.q, np.ndarray)):
            self.pub.publish(zero)
            self.jac_pub.publish(jac_zero)
            return

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        qdot_avoid = np.zeros(7)
        d_min = 999.0
        self.distances_data = []  # Resetta la lista di distanze
        # Jacobiano del vincolo di distanza per il caso peggiore (min d)
        best_j_row = np.zeros(7)

        # Human-readable info about the most critical hazard
        best_hazard = "none"

        best_fid = None
        best_p_seg = None
        best_dir = None
        best_pair = None  # (fid_a, p_a, fid_b, p_b) for self-collision

        # Precompute world capsules (for ground + self-collision)
        segments = []
        link_to_index = {f"fr3_link{i}": i for i in range(1, 9)}

        for parent in self.capsules:
            fid = self.frame_ids[parent]
            link_idx = int(link_to_index.get(parent, 0))

            for caps in self.capsules[parent]:
                oMp = self.data.oMf[fid]
                p0 = oMp.translation + oMp.rotation @ caps["p0"]
                p1 = oMp.translation + oMp.rotation @ caps["p1"]
                segments.append(
                    {
                        "parent": parent,
                        "fid": fid,
                        "link_idx": link_idx,
                        "p0": p0,
                        "p1": p1,
                        "radius": float(caps["radius"]),
                    }
                )

                # ===== External obstacles (PlanningScene boxes) =====
                for obs in self.obstacles:
                    d, dir_vec, p_seg, p_box = self._distance_capsule_to_box(p0, p1, caps["radius"], obs)
                    if dir_vec is None:
                        continue

                    if d < d_min:
                        d_min = d
                        best_fid = fid
                        best_p_seg = p_seg
                        best_dir = dir_vec
                        best_pair = None
                        best_hazard = f"external:{obs.id}"

                    self.distances_data.append({
                        "p_capsule": p_seg,
                        "p_obstacle": p_box,
                        "distance": d,
                    })

                    if d >= self.d_infl:
                        continue

                    alpha = np.clip((self.d_infl - d) / (self.d_infl - self.d_safe), 0, 1)
                    xdot_avoid = self.k_null * alpha * dir_vec
                    Jp = self._point_jacobian_world(fid, p_seg)
                    qdot_avoid += Jp.T @ xdot_avoid

                # ===== Ground (floor plane z = ground_z) =====
                if self.enable_ground:
                    # closest point to the plane is the endpoint with minimum z
                    p_low = p0 if p0[2] <= p1[2] else p1
                    d_ground = float((p_low[2] - self.ground_z) - caps["radius"])
                    # Project point on the plane for visualization
                    p_plane = np.array([p_low[0], p_low[1], self.ground_z], dtype=float)

                    if d_ground < d_min:
                        d_min = d_ground
                        best_fid = fid
                        best_p_seg = p_low
                        best_dir = np.array([0.0, 0.0, 1.0], dtype=float)
                        best_pair = None
                        best_hazard = "ground:plane"

                    self.distances_data.append({
                        "p_capsule": p_low,
                        "p_obstacle": p_plane,
                        "distance": d_ground,
                    })

                    if d_ground < self.ground_infl:
                        alpha_g = np.clip((self.ground_infl - d_ground) / (self.ground_infl - self.ground_safe), 0, 1)
                        dir_g = np.array([0.0, 0.0, 1.0], dtype=float)
                        xdot_g = self.k_ground * alpha_g * dir_g
                        Jp_g = self._point_jacobian_world(fid, p_low)
                        qdot_avoid += Jp_g.T @ xdot_g

        # ===== Self-collision (capsule-capsule) =====
        if self.enable_self and len(segments) >= 2:
            for i in range(len(segments)):
                si = segments[i]
                for j in range(i + 1, len(segments)):
                    sj = segments[j]

                    # Skip nearby links to avoid false positives on adjacent geometry
                    if abs(int(si["link_idx"]) - int(sj["link_idx"])) <= self.self_skip_adjacent:
                        continue

                    cp_i, cp_j = self._closest_points_on_segments(si["p0"], si["p1"], sj["p0"], sj["p1"])
                    diff = cp_i - cp_j
                    dist = float(np.linalg.norm(diff) - (si["radius"] + sj["radius"]))

                    if dist < d_min:
                        d_min = dist
                        best_fid = None
                        best_p_seg = None
                        best_dir = None
                        best_pair = (si["fid"], cp_i, sj["fid"], cp_j, diff)
                        best_hazard = f"self:{si['parent']}<->{sj['parent']}"

                    self.distances_data.append({
                        "p_capsule": cp_i,
                        "p_obstacle": cp_j,
                        "distance": dist,
                    })

                    if dist >= self.self_infl:
                        continue

                    # Repel the two points away from each other
                    n = diff / (np.linalg.norm(diff) + 1e-9)
                    alpha_s = np.clip((self.self_infl - dist) / (self.self_infl - self.self_safe), 0, 1)
                    xdot_s = self.k_self * alpha_s * n

                    J_i = self._point_jacobian_world(si["fid"], cp_i)
                    J_j = self._point_jacobian_world(sj["fid"], cp_j)
                    J_rel = (J_i - J_j)  # relative point velocity wrt qdot
                    qdot_avoid += J_rel.T @ xdot_s

        # Jacobiano row associato al minimo (hazard più critico)
        # - external/ground: d_dot ≈ dir^T J_point qdot
        # - self:            d_dot ≈ n^T (Jp_i - Jp_j) qdot
        if best_pair is not None:
            fid_i, cp_i, fid_j, cp_j, diff = best_pair
            n = diff / (np.linalg.norm(diff) + 1e-9)
            J_i = self._point_jacobian_world(fid_i, cp_i)
            J_j = self._point_jacobian_world(fid_j, cp_j)
            best_j_row = (n.reshape(1, 3) @ (J_i - J_j)).reshape(-1)
        elif (best_fid is not None) and (best_p_seg is not None) and (best_dir is not None):
            J_best = self._point_jacobian_world(best_fid, best_p_seg)
            best_j_row = (best_dir.reshape(1, 3) @ J_best).reshape(-1)
        else:
            best_j_row = np.zeros(7)

        # Saturazione avoidance
        norm_qdot = np.linalg.norm(qdot_avoid)
        if norm_qdot > self.max_qdot:
            qdot_avoid *= self.max_qdot / norm_qdot

        self.pub.publish(Float64MultiArray(data=qdot_avoid.tolist()))
        self.min_dist_pub.publish(Float64MultiArray(data=[float(d_min)]))

        # Publish hazard label (avoid confusion when far from any influence zone)
        any_infl = [self.d_infl]
        if self.enable_ground:
            any_infl.append(self.ground_infl)
        if self.enable_self:
            any_infl.append(self.self_infl)
        infl_min = float(min(any_infl)) if any_infl else self.d_infl
        hazard_msg = String()
        hazard_msg.data = best_hazard if d_min < infl_min else "none"
        self.hazard_pub.publish(hazard_msg)

        # Se siamo fuori da ogni zona di influenza, azzera il jacobiano per evitare valori "stale"
        if d_min >= self.d_infl:
            self.jac_pub.publish(jac_zero)
        else:
            self.jac_pub.publish(Float64MultiArray(data=best_j_row.tolist()))

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
