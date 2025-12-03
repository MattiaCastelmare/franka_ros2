#!/usr/bin/env python3
"""
ONLINE AVOIDANCE CONTROLLER — BASELINE-1 (Capsule-based, Potential Field)
=========================================================================

Implementa obstacle avoidance usando una rappresentazione a CAPSULE per i link
del Franka FR3 e campi potenziali in spazio cartesiano:

 - Ogni link i è approssimato da una capsula (cilindro + due semisfere) che
   parte dall'origine di fr3_link{i} e punta verso fr3_link{i+1}, in frame locale.
 - La distanza usata è CAPSULA ↔ BOX (ostacolo), quindi con raggio esplicito.
 - Potenziale repulsivo continuo (stile Khatib):
      U(d) = 1/2 * K * ( 1/(d - d_safe) - 1/(d_infl - d_safe) )^2
 - Forza F = -∂U/∂x (direzione dal box verso la capsula)
 - Mappatura in velocità di giunto: qdot_rep = J^T * F
 - Publish:
      /avoidance/velocity  (Float64MultiArray, 7 joint vel)
      /avoidance/jacobian  (riga Jacobiano "critico" 1x7)
      /avoidance/min_distance (distanza minima attuale)

NB: nessun null-space ancora (Versione A pulita).
"""

import os
import tempfile
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import PlanningScene
from rcl_interfaces.srv import GetParameters

import pinocchio as pin


class Baseline1Avoidance(Node):

    def __init__(self):
        super().__init__("online_avoidance_controller")

        # ===== PARAMETRI ROS =====
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("influence_distance", 0.30)   # d_infl (d fuori capsula)
        self.declare_parameter("safety_margin", 0.08)        # d_safe (d fuori capsula)
        self.declare_parameter("repulsive_gain", 0.4)
        self.declare_parameter("max_joint_velocity", 0.4)
        self.declare_parameter("excluded_obstacles", ["ground", "plane", "floor"])
        # componente tangenziale del campo (deviazione laterale attorno agli ostacoli)
        self.declare_parameter("tangential_gain", 0.3)


        # Lettura parametri
        self.rate = float(self.get_parameter("control_rate").value)
        self.d_infl = float(self.get_parameter("influence_distance").value)
        self.d_safe = float(self.get_parameter("safety_margin").value)
        self.K = float(self.get_parameter("repulsive_gain").value)
        self.max_qdot = float(self.get_parameter("max_joint_velocity").value)
        self.excluded = list(self.get_parameter("excluded_obstacles").value)
        self.tangential_gain = float(self.get_parameter("tangential_gain").value)


        # ===== GEOMETRIA CAPSULE (LINK PAIRS + RAGGI) =====
        # Coppie (parent_link, child_link)
        self.link_pairs = [
            ("fr3_link1", "fr3_link2"),
            ("fr3_link2", "fr3_link3"),
            ("fr3_link3", "fr3_link4"),
            ("fr3_link4", "fr3_link5"),
            ("fr3_link5", "fr3_link6"),
            ("fr3_link6", "fr3_link7"),
            ("fr3_link7", "fr3_link8"),
        ]

        # Raggio per ogni link (m) — conservativo
        self.link_radius = {
            "fr3_link1": 0.08,
            "fr3_link2": 0.08,
            "fr3_link3": 0.08,
            "fr3_link4": 0.08,
            "fr3_link5": 0.08,
            "fr3_link6": 0.08,
            "fr3_link7": 0.08,
            "fr3_link8": 0.08,
        }

        # Dizionario finale: parent_link -> {"p0": np.array(3), "p1": np.array(3), "radius": float}
        # p0, p1 sono definiti nel frame LOCALE del parent_link
        self.capsules = {}

        # ===== STATO =====
        self.joint_positions = None          # q (ordine Pinocchio, già rimappato)
        self.frame_ids = {}                  # link_name -> frame_id
        self.obstacles = []                  # lista di CollisionObject da PlanningScene
        self.pin_ok = False
        self.loop = 0

        # ===== PINOCCHIO: MODEL + CAPSULES =====
        self._init_pinocchio_and_capsules()

        # ===== ROS I/O =====
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(PlanningScene, "/obstacle_scene", self._obstacle_cb, 1)

        self.pub = self.create_publisher(Float64MultiArray, "/avoidance/velocity", 10)
        self.jac_pub = self.create_publisher(Float64MultiArray, "/avoidance/jacobian", 10)
        self.min_dist_pub = self.create_publisher(Float64MultiArray, "/avoidance/min_distance", 10)

        # Timer di controllo
        self.create_timer(1.0 / self.rate, self._control_loop)

        self.get_logger().info("🔥 Baseline-1 Capsule-based Potential Field Avoidance READY")


    # ============================================================
    # PINOCCHIO + COSTRUZIONE CAPSULE
    # ============================================================
    def _init_pinocchio_and_capsules(self):
        try:
            # 1) Leggo robot_description
            cli = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
            cli.wait_for_service(timeout_sec=10.0)

            req = GetParameters.Request()
            req.names = ["robot_description"]
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            urdf_str = future.result().values[0].string_value

            with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf", mode="w") as f:
                f.write(urdf_str)
                urdf_path = f.name

            # 2) Modello Pinocchio completo
            model_full = pin.buildModelFromUrdf(urdf_path)
            os.unlink(urdf_path)

            # 3) Rimuovo le dita
            lock = [model_full.getJointId(n) for n in model_full.names if "finger" in n]
            self.model = pin.buildReducedModel(model_full, lock, pin.neutral(model_full))
            self.data = self.model.createData()

            # 4) Map frame_ids per tutti i link interessati
            for parent, child in self.link_pairs:
                for link in (parent, child):
                    if link in self.frame_ids:
                        continue
                    try:
                        fid = self.model.getFrameId(link)
                        self.frame_ids[link] = fid
                        self.get_logger().info(f"  ✓ frame {link}: {fid}")
                    except Exception:
                        self.get_logger().warn(f"⚠️ Frame non trovato: {link}")

            # 5) Costruisco le capsule nel frame locale dei parent, in configurazione neutra
            q0 = pin.neutral(self.model)
            pin.forwardKinematics(self.model, self.data, q0)
            pin.updateFramePlacements(self.model, self.data)

            for parent, child in self.link_pairs:
                if parent not in self.frame_ids or child not in self.frame_ids:
                    self.get_logger().warn(f"⛔ Skip capsule {parent}->{child}: frame mancante")
                    continue

                fid_parent = self.frame_ids[parent]
                fid_child = self.frame_ids[child]

                oMp = self.data.oMf[fid_parent]   # world -> parent
                oMc = self.data.oMf[fid_child]    # world -> child

                p_w_parent = oMp.translation      # posizione parent in world
                R_w_parent = oMp.rotation         # rot parent in world
                p_w_child = oMc.translation       # posizione child in world

                # posizione child nel frame locale del parent:
                # p_parent_child = R_parent^T * (p_child_world - p_parent_world)
                p_parent_child = R_w_parent.T @ (p_w_child - p_w_parent)

                p0_local = np.zeros(3)                   # origine frame parent
                p1_local = 0.95 * p_parent_child         # verso child, accorciata un po'

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
            self.get_logger().error(f"Pinocchio / capsule init ERROR: {e}")
            self.pin_ok = False


    # ============================================================
    # CALLBACKS
    # ============================================================
    def _joint_cb(self, msg: JointState):
        """
        Rimappa l'ordine "strano" pubblicato da ROS2/MoveIt all'ordine corretto
        usato da Pinocchio: [q1, q2, q3, q4, q5, q6, q7]
        """
        # ordine pubblicato da ROS2 (come hai osservato)
        raw_order = [
            "fr3_joint1",
            "fr3_joint3",
            "fr3_joint2",
            "fr3_joint4",
            "fr3_joint6",
            "fr3_joint5",
            "fr3_joint7"
        ]

        # estraggo i joint in questo ordine
        q_ros = []
        for name in raw_order:
            if name in msg.name:
                q_ros.append(msg.position[msg.name.index(name)])
            else:
                # se manca qualcosa, esco e aspetto un messaggio coerente
                return

        q_ros = np.array(q_ros)

        # conversione all’ordine corretto per Pinocchio
        q_pin = np.zeros(7)
        q_pin[0] = q_ros[0]   # joint1
        q_pin[1] = q_ros[2]   # joint2
        q_pin[2] = q_ros[1]   # joint3
        q_pin[3] = q_ros[3]   # joint4
        q_pin[4] = q_ros[5]   # joint5
        q_pin[5] = q_ros[4]   # joint6
        q_pin[6] = q_ros[6]   # joint7

        self.joint_positions = q_pin


    def _obstacle_cb(self, msg: PlanningScene):
        self.obstacles = []
        for obs in msg.world.collision_objects:
            if any(ex in obs.id.lower() for ex in self.excluded):
                continue
            self.obstacles.append(obs)


    # ============================================================
    # DISTANZA CAPSULA ↔ BOX
    # ============================================================
    def _distance_capsule_to_box(self, p0, p1, radius, obs):
        """
        Calcola distanza minima geometrica tra una CAPSULA (segmento [p0,p1] + raggio)
        e un ostacolo di tipo BOX.

        p0, p1 : punti in WORLD frame
        radius: raggio capsula

        Ritorna:
          d_eff    : distanza "effettiva" (>= 0 fuori, < 0 in penetrazione)
          dir_vec  : direzione normalizzata (dal box verso la capsula)
          best_pt  : punto più vicino sul box (WORLD)
        """
        best_d = 999.0
        best_dir = np.array([1.0, 0.0, 0.0])
        best_pt = None

        v = p1 - p0
        L = np.linalg.norm(v)
        if L < 1e-6:
            return 999.0, best_dir, None

        v_norm = v / L

        for i, prim in enumerate(obs.primitives):
            if prim.type != prim.BOX:
                continue

            pose = obs.primitive_poses[i]
            center = np.array([pose.position.x, pose.position.y, pose.position.z])
            half = np.array(prim.dimensions) / 2.0

            # proiezione del centro del box sul segmento della capsula
            t = np.dot(center - p0, v_norm)
            t_clamped = np.clip(t, 0.0, L)
            closest_capsule = p0 + v_norm * t_clamped

            # punto più vicino sul box (clamp dentro il box)
            rel = closest_capsule - center
            closest_box = center + np.clip(rel, -half, +half)

            diff = closest_capsule - closest_box
            dist_center = np.linalg.norm(diff)

            # distanza effettiva esterna (0 se tocca, negativa se penetra)
            d_eff = dist_center - radius

            if d_eff < best_d:
                if dist_center > 1e-6:
                    best_dir = diff / dist_center
                else:
                    best_dir = np.array([1.0, 0.0, 0.0])
                best_d = d_eff
                best_pt = closest_box

        return best_d, best_dir, best_pt


    # ============================================================
    # POTENTIAL FIELD
    # ============================================================
    def potential_force(self, d, dir_vec):
        """
        Campo potenziale continuo (Khatib-style) con componente tangenziale.

        d       = distanza effettiva capsula–box (>=0 fuori, ~0 contatto)
        dir_vec = direzione normalizzata (dal box verso la capsula)

        Ritorna F (3D) in WORLD frame.
        """
        # fuori zona di influenza → nessuna forza
        if d >= self.d_infl:
            return np.zeros(3)

        # se siamo dentro / troppo vicini, saturiamo d per evitare esplosioni
        if d <= self.d_safe:
            d = self.d_safe + 1e-3

        # Parte normale (radiale) come prima
        term   = (1.0 / (d - self.d_safe) - 1.0 / (self.d_infl - self.d_safe))
        dU_dd  = self.K * term * (1.0 / ((d - self.d_safe) ** 2))
        F_norm = dU_dd * dir_vec

        # ==========================
        # Parte tangenziale (NUOVA)
        # ==========================
        # Asse fisso del mondo per generare una direzione tangenziale consistente.
        # Puoi cambiare l'asse se vuoi far "girare" attorno ad un’altra direzione.
        # Scegli un asse non allineato con dir_vec
        world_axis = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(world_axis, dir_vec)) > 0.9:
            world_axis = np.array([1.0, 0.0, 0.0])

        # Direzione tangenziale = world_axis × dir_vec  (perpendicolare al gradiente)
        tang_dir = np.cross(world_axis, dir_vec)
        norm_tang = np.linalg.norm(tang_dir)

        if norm_tang > 1e-6 and self.tangential_gain > 0.0:
            tang_dir /= norm_tang
            # stessa "scala" di dU_dd, modulata da un guadagno tangenziale
            F_tan = self.tangential_gain * dU_dd * tang_dir
        else:
            F_tan = np.zeros(3)

        # Forza totale = normale + tangenziale
        F = F_norm + F_tan

        # Clip "soft" per evitare forze enormi vicino a d_safe
        F_max = 20.0  # [N] equivalente
        norm_F = np.linalg.norm(F)
        if norm_F > F_max:
            F = F / norm_F * F_max

        return F



    # ============================================================
    # CONTROL LOOP
    # ============================================================
    def _control_loop(self):
        self.loop += 1

        zero = Float64MultiArray()
        zero.data = [0.0] * 7

        if not (self.pin_ok and isinstance(self.joint_positions, np.ndarray)):
            self.pub.publish(zero)
            return

        if len(self.obstacles) == 0:
            self.pub.publish(zero)
            # distanza infinita se non ci sono ostacoli
            dist_msg = Float64MultiArray()
            dist_msg.data = [999.0]
            self.min_dist_pub.publish(dist_msg)
            return

        q = self.joint_positions

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        q_dot_rep = np.zeros(7)
        active = 0
        global_min = 999.0
        best_j_row = None

        # === PER OGNI CAPSULA DEL ROBOT ===
        for parent, child in self.link_pairs:
            if parent not in self.capsules or parent not in self.frame_ids:
                continue

            caps = self.capsules[parent]
            p0_local = caps["p0"]
            p1_local = caps["p1"]
            radius = caps["radius"]

            # FK del parent link
            fid_parent = self.frame_ids[parent]
            oMp = self.data.oMf[fid_parent]          # world -> parent
            R = oMp.rotation
            t = oMp.translation

            # punti in WORLD
            p0_world = t + R @ p0_local
            p1_world = t + R @ p1_local

            # distanza minima capsula ↔ ostacoli
            d = 999.0
            best_dir = None
            best_obs_pt = None

            for obs in self.obstacles:
                d0, dir0, pt0 = self._distance_capsule_to_box(p0_world, p1_world, radius, obs)
                if d0 < d:
                    d = d0
                    best_dir = dir0
                    best_obs_pt = pt0

            if best_dir is None:
                continue

            if d < self.d_infl:
                active += 1

                # Forza repulsiva cartesiana
                F = self.potential_force(d, best_dir)

                if np.linalg.norm(F) > 0.0:
                    # Jacobiano posizione del frame parent (approssimazione)
                    J = pin.computeFrameJacobian(
                        self.model, self.data, q, fid_parent, pin.ReferenceFrame.WORLD
                    )
                    Jpos = J[:3, :]  # prime 3 righe = posizione

                    # qdot_rep_i = J^T * F
                    q_dot_rep += Jpos.T @ F

                    # Jacobiano del vincolo distanza: d/dt(d) = (dir_vec^T * Jpos) * qdot
                    j_row = best_dir @ Jpos  # shape (7,)

                    if d < global_min:
                        global_min = d
                        best_j_row = j_row.copy()

        # Se nessuna capsula è in zona di influenza, distanza "infinita"
        if active == 0:
            global_min = 999.0

        # Debug ogni tanto
        if self.loop % 40 == 0:
            self.get_logger().info(
                f"📏 min_dist(capsule) = {global_min:.3f} m (infl={self.d_infl}) | active_capsules={active}"
            )

        # Pubblica distanza minima
        dist_msg = Float64MultiArray()
        dist_msg.data = [float(global_min)]
        self.min_dist_pub.publish(dist_msg)

        # === OUTPUT VELOCITY & JACOBIAN ===
        if active > 0:
            # Normalizza/clip della velocità di evitamento
            norm = np.linalg.norm(q_dot_rep)
            if norm > self.max_qdot and norm > 1e-8:
                q_dot_rep = q_dot_rep / norm * self.max_qdot

            vel_msg = Float64MultiArray()
            vel_msg.data = q_dot_rep.tolist()
            self.pub.publish(vel_msg)

            # Jacobiano critico
            jac_msg = Float64MultiArray()
            if best_j_row is not None:
                jac_msg.data = best_j_row.tolist()
            else:
                jac_msg.data = [0.0] * 7
            self.jac_pub.publish(jac_msg)
        else:
            # Nessuna capsula attiva → avoidance nullo
            self.pub.publish(zero)
            jac_msg = Float64MultiArray()
            jac_msg.data = [0.0] * 7
            self.jac_pub.publish(jac_msg)


# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = Baseline1Avoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
