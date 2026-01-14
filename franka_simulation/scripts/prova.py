#!/usr/bin/env python3
import os
import tempfile
import subprocess
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from moveit_msgs.msg import PlanningScene
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import pinocchio as pin


class RobotCapsuleVisualizer(Node):
    """
    Visualizza una rappresentazione REALISTICA del Franka FR3 usando CAPSULES
    (cilindro + due semisfere) allineate alla geometria dei link.

    Per ogni link i:
      - la capsula parte dall'origine di fr3_link{i}
      - e arriva all'origine di fr3_link{i+1} espressa nel frame locale di fr3_link{i}
      - il raggio è scelto in modo conservativo (un po' più grande della mesh)

    Inoltre:
      - visualizza una linea blu che collega il punto della capsula più vicino
        all'ostacolo e il punto corrispondente sul box
      - logga la distanza minima capsula ↔ ostacolo
    """

    def __init__(self):
        super().__init__("robot_capsule_visualizer")

        # --------------------------------------------------
        # Coppie di link (parent → child) per costruire le capsule
        # --------------------------------------------------
        self.link_pairs = [
            ("fr3_link1", "fr3_link2"),
            ("fr3_link2", "fr3_link3"),
            ("fr3_link3", "fr3_link4"),
            ("fr3_link4", "fr3_link5"),
            ("fr3_link5", "fr3_link6"),
            ("fr3_link6", "fr3_link7"),
            ("fr3_link7", "fr3_link8"),
        ]

        # Raggio "ragionevole" per ogni link (metri)
        # (leggermente sovrastimato per sicurezza)
        self.link_radius = {
            "fr3_link1": 0.1,
            "fr3_link2": 0.1,
            "fr3_link3": 0.1,
            "fr3_link4": 0.1,
            "fr3_link5": 0.1,
            "fr3_link6": 0.1,
            "fr3_link7": 0.1,
            "fr3_link8": 0.1,
        }

        # Dizionario finale: link -> {"p0": np.array, "p1": np.array, "radius": float}
        self.capsules = {}

        # Ostacoli da ignorare
        self.excluded_obs = ["ground", "floor", "plane"]

        # Stato runtime
        self.joint_positions = None
        self.obstacles = []
        self.frame_ids = {}
        self.pin_ok = False

        # Nome del frame di base usato in RViz
        self.base_frame = "world"

        # Inizializza Pinocchio e ricava le capsule dal tuo URDF
        self._init_pinocchio_and_build_capsules()

        # --- ROS IO ---
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(PlanningScene, "/obstacle_scene", self._obstacle_cb, 1)

        self.marker_pub = self.create_publisher(MarkerArray, "/robot_capsules_markers", 10)

        # Timer 10 Hz
        self.create_timer(0.1, self._update)

        self.get_logger().info("🤖 RobotCapsuleVisualizer started (capsule allineate ai link)")

    # ======================================================
    # PINOCCHIO + COSTRUZIONE CAPSULE
    # ======================================================
    def _init_pinocchio_and_build_capsules(self):
        try:
            from ament_index_python.packages import get_package_share_directory

            urdf_path = os.path.join(
                get_package_share_directory("franka_description"),
                "robots", "fr3", "fr3.urdf.xacro"
            )

            urdf_xml = subprocess.check_output([
                "xacro", urdf_path,
                "ros2_control:=false",
                "hand:=true",
                "arm_id:=fr3"
            ]).decode()


            with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf", mode="w") as f:
                f.write(urdf_xml)
                urdf_path = f.name

            # ---- Costruisco modello Pinocchio completo ----
            model_full = pin.buildModelFromUrdf(urdf_path)
            os.unlink(urdf_path)

            # Blocco le dita (come nel resto del tuo codice)
            locked = [model_full.getJointId(n)
                      for n in model_full.names
                      if "finger" in n]
            self.model = pin.buildReducedModel(model_full, locked, pin.neutral(model_full))
            self.data = self.model.createData()

            # Mappo i frame-id dei link che mi interessano
            for parent, child in self.link_pairs:
                for link in (parent, child):
                    if link in self.frame_ids:
                        continue
                    try:
                        fid = self.model.getFrameId(link)
                        self.frame_ids[link] = fid
                        self.get_logger().info(f"✔ Frame {link} = {fid}")
                    except Exception:
                        self.get_logger().warn(f"⚠ Frame NON trovato: {link}")

            # ---- Calcolo p0, p1 locali per ogni link (parent) ----
            # Uso la configurazione neutra del robot (q0)
            q0 = pin.neutral(self.model)
            pin.forwardKinematics(self.model, self.data, q0)
            pin.updateFramePlacements(self.model, self.data)

            for parent, child in self.link_pairs:
                if parent not in self.frame_ids or child not in self.frame_ids:
                    self.get_logger().warn(
                        f"⛔ Skip capsule {parent}->{child}: frame mancante"
                    )
                    continue

                fid_parent = self.frame_ids[parent]
                fid_child = self.frame_ids[child]

                oMp = self.data.oMf[fid_parent]   # world → parent
                oMc = self.data.oMf[fid_child]    # world → child

                p_w_parent = oMp.translation
                R_w_parent = oMp.rotation
                p_w_child = oMc.translation

                # posizione del child espressa nel frame locale del parent
                # p_parent_child = R_parent^T * (p_child_world - p_parent_world)
                p_parent_child = R_w_parent.T @ (p_w_child - p_w_parent)

                # p0 locale: origine del frame parent
                p0_local = np.zeros(3)

                # p1 locale: verso child; accorcio leggermente (0.95) per non uscire dal link
                p1_local = 0.95 * p_parent_child

                radius = self.link_radius.get(parent, 0.04)

                self.capsules[parent] = {
                    "p0": p0_local,
                    "p1": p1_local,
                    "radius": radius,
                }

                self.get_logger().info(
                    f"🧩 Capsule {parent}: |p1|={np.linalg.norm(p1_local):.3f} m, r={radius:.3f} m"
                )

            self.pin_ok = True

        except Exception as e:
            self.get_logger().error(f"❌ Pinocchio / capsule init error: {e}")
            self.pin_ok = False

    # ======================================================
    # CALLBACKS
    # ======================================================
    def _joint_cb(self, msg: JointState):
        # Ordine canonico dei giunti (come ros2_control): [1..7]
        joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ]

        try:
            name_to_idx = {n: i for i, n in enumerate(msg.name)}
            self.joint_positions = np.array(
                [msg.position[name_to_idx[n]] for n in joint_names],
                dtype=float,
            )
        except (KeyError, IndexError):
            return  # messaggio incompleto


    def _obstacle_cb(self, msg: PlanningScene):
        self.obstacles = []
        for obs in msg.world.collision_objects:
            if any(ex in obs.id.lower() for ex in self.excluded_obs):
                continue
            self.obstacles.append(obs)

    # ======================================================
    # DISTANZA PUNTO → BOX
    # (per debug: uso il punto medio della capsula)
    # ======================================================
    def _distance_point_to_box(self, point, obs):
        best_d = 999.0
        best_pt = None

        for i, prim in enumerate(obs.primitives):
            if prim.type != prim.BOX:
                continue

            pose = obs.primitive_poses[i]
            center = np.array([pose.position.x, pose.position.y, pose.position.z])
            half = np.array(prim.dimensions) / 2.0

            closest = center + np.clip(point - center, -half, +half)
            dist = np.linalg.norm(point - closest)

            if dist < best_d:
                best_d = dist
                best_pt = closest

        return best_d, best_pt

    # ======================================================
    # CREAZIONE MARKER DELLA CAPSULA
    # ======================================================
    def _make_capsule_markers(self, p0, p1, radius, marker_id):
        """
        Restituisce [cilindro, sfera_p0, sfera_p1] come Marker.
        p0, p1 sono in WORLD frame.
        """

        markers = []

        # ----- cilindro -----
        cyl = Marker()
        cyl.header.frame_id = self.base_frame
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
            # orientamento arbitrario
            q = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            # ruota asse Z (0,0,1) in 'direction'
            z_axis = np.array([0.0, 0.0, 1.0])
            v = np.cross(z_axis, direction / norm_dir)
            c = np.dot(z_axis, direction / norm_dir)
            if c <= -1.0 + 1e-8:
                # vettori opposti: rotazione di 180° attorno a X
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
            sph.header.frame_id = self.base_frame
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
    # MAIN UPDATE
    # ======================================================
    def _update(self):
        if not self.pin_ok or self.joint_positions is None:
            return

        q = self.joint_positions
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        markers = MarkerArray()
        marker_id = 0

        global_min = 999.0
        global_cp = None
        global_obs_pt = None

        # --------- Disegna tutte le capsule ---------
        for parent, child in self.link_pairs:
            if parent not in self.capsules or parent not in self.frame_ids:
                continue

            caps = self.capsules[parent]
            p0_local = caps["p0"]
            p1_local = caps["p1"]
            radius = caps["radius"]

            fid_parent = self.frame_ids[parent]
            oMp = self.data.oMf[fid_parent]  # world → parent
            R = oMp.rotation
            t = oMp.translation

            # punti in WORLD
            p0 = t + R @ p0_local
            p1 = t + R @ p1_local

            # aggiungo markers
            capsule_markers = self._make_capsule_markers(p0, p1, radius, marker_id)
            for m in capsule_markers:
                markers.markers.append(m)
            marker_id += len(capsule_markers)


            # distanza per debug: uso il punto medio della capsula
            mid = (p0 + p1) / 2.0
            d = 999.0
            pt = None
            for obs in self.obstacles:
                d0, pt0 = self._distance_point_to_box(mid, obs)
                if d0 < d:
                    d = d0
                    pt = pt0

            if d < global_min:
                global_min = d
                global_cp = mid
                global_obs_pt = pt

        # --------- Linea blu della distanza minima ---------
        if global_cp is not None and global_obs_pt is not None:
            line = Marker()
            line.header.frame_id = self.base_frame
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = "min_distance"
            line.id = 90000
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.01
            line.color = ColorRGBA(r=0.1, g=0.2, b=1.0, a=1.0)

            p1 = Point(x=float(global_cp[0]),
                       y=float(global_cp[1]),
                       z=float(global_cp[2]))
            p2 = Point(x=float(global_obs_pt[0]),
                       y=float(global_obs_pt[1]),
                       z=float(global_obs_pt[2]))

            line.points.append(p1)
            line.points.append(p2)
            markers.markers.append(line)

        # Pubblico tutti i marker
        self.marker_pub.publish(markers)

        self.get_logger().info(f"📏 Min dist capsule (punto medio) = {global_min:.3f} m")


# ======================================================
# MAIN
# ======================================================
def main(args=None):
    rclpy.init(args=args)
    node = RobotCapsuleVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
