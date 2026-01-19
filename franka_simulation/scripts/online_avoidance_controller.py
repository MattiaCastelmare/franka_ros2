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
import zlib
import math
from typing import Dict, List, Optional, Tuple

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
        # When closer than this distance, avoidance becomes intentionally more aggressive.
        # This is *not* the same as safety_margin: it is a "start pushing hard" threshold.
        self.declare_parameter("aggressive_distance", 0.20)
        self.declare_parameter("aggressive_gain_scale", 3.0)
        # Overall scaling for the avoidance twist (used for both repulsive and tangential components)
        self.declare_parameter("nullspace_gain", 0.15)
        # Extra tangential (swirl) component to break local minima near obstacles.
        # 0.0 disables tangential motion.
        self.declare_parameter("tangential_gain", 0.20)
        self.declare_parameter("max_joint_velocity", 0.25)
        self.declare_parameter("excluded_obstacles", ["ground_plane", "ground", "floor", "plane"])

        # Capsule geometry tuning (m)
        # NOTE: These directly affect d_min and therefore when avoidance triggers.
        self.declare_parameter("capsule_radii", [0.15, 0.12, 0.13])
        # Fractions along each link segment used to place 3 overlapped capsules.
        # Format: [p0_0, p1_0, p0_1, p1_1, p0_2, p1_2]
        self.declare_parameter("capsule_fractions", [0.00, 0.35, 0.25, 0.75, 0.60, 0.95])

        # Distance model knobs
        # Iterations used by the (convex) alternating projection to get closest points segment<->AABB in box frame.
        self.declare_parameter("box_projection_iters", 8)

        # Optional: spread repulsion over a small region around the closest point on the capsule.
        # This makes the avoidance less "pointy" and generally more stable.
        self.declare_parameter("repulsion_spread_enable", True)
        self.declare_parameter("repulsion_spread_samples", 5)      # odd number recommended (e.g., 3/5/7)
        self.declare_parameter("repulsion_spread_half_length", 0.10)  # m along the capsule segment

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

        # ================= CBF-QP SAFETY FILTER =================
        # NOTE: These parameters are used ONLY to filter the node's nominal output into a safe output.
        # They do not introduce new ROS topics/messages.
        #
        # Barrier function: h = d - d_safe
        # Constraint: g^T qdot >= v_obs_proj - alpha*(d - d_safe)
        self.declare_parameter("d_safe", 0.08)          # [m] safety distance
        self.declare_parameter("d_buffer", 0.30)        # [m] activation distance (influence zone for the filter)
        self.declare_parameter("d_buffer_out", 0.0)     # [m] hysteresis exit threshold (0 -> auto)
        self.declare_parameter("alpha", 5.0)            # [1/s] CBF gain
        self.declare_parameter("max_constraints", 5)    # K (top-K closest hazards)
        self.declare_parameter("lambda_reg", 1e-6)      # regularization on qdot
        self.declare_parameter("rho_slack", 100.0)      # slack penalty
        self.declare_parameter("beta_lpf", 0.80)        # LPF on output qdot (kept for backward compatibility)
        self.declare_parameter("output_accel_limit", 0.0)  # [rad/s^2] 0 disables rate limiting
        self.declare_parameter("approach_speed_limit", 0.0)  # [m/s] 0 disables extra cap on negative d_dot
        self.declare_parameter("use_qp", True)          # try OSQP if available
        self.declare_parameter("eps", 1e-9)             # numerical epsilon
        self.declare_parameter("qp_weight_diag", [1.0] * 7)  # diagonal W in ||qdot-qdot_nom||_W

        # ===== Risk-scaled zones (30/20/10/5 cm) + Stop Gate =====
        # These parameters implement the staged behavior requested:
        # - gentle deviation at 0.30m
        # - medium at 0.20m
        # - strong at 0.10m
        # - hard stop at 0.05m (release at 0.06m)
        self.declare_parameter("risk_d_far", 0.30)          # [m] start reacting
        self.declare_parameter("risk_d_mid", 0.20)          # [m] medium zone
        self.declare_parameter("risk_d_near", 0.10)         # [m] strong zone
        self.declare_parameter("stop_distance", 0.05)       # [m] hard stop enter
        self.declare_parameter("stop_release_distance", 0.06)  # [m] hard stop exit (hysteresis)

        # Risk-scaled CBF alpha: alpha(d) = lerp(alpha_min, alpha_max, w(d))
        # Defaults keep legacy behavior (alpha_min==alpha_max==alpha).
        self.declare_parameter("alpha_min", 5.0)            # [1/s]
        self.declare_parameter("alpha_max", 5.0)            # [1/s]

        # QP damping term: gamma(d) * ||qdot - qdot_prev||^2
        self.declare_parameter("qp_damping_min", 0.0)       # >= 0
        self.declare_parameter("qp_damping_max", 0.0)       # >= 0

        # Risk-scaled output LPF (beta near should be smaller for smoother motion)
        self.declare_parameter("beta_lpf_far", 0.80)        # 0..1
        self.declare_parameter("beta_lpf_near", 0.80)       # 0..1

        # Smoothing for published min distance signal (for downstream blending/visualization)
        self.declare_parameter("min_distance_lpf", 0.50)    # 0..1 (1.0 = no filtering)

        # Optional posture bias (OFF by default): pulls away from folded configurations.
        # If posture_reference is empty, pinocchio neutral is used.
        self.declare_parameter("posture_bias_gain", 0.0)    # [1/s]
        self.declare_parameter("posture_reference", [])     # 7 values (radians)
        
        # Carica i valori dal YAML (ora il nodo sa dove trovarli)
        self.rate = float(self.get_parameter("control_rate").value)
        self.d_infl = float(self.get_parameter("influence_distance").value)
        self.d_safe = float(self.get_parameter("safety_margin").value)
        self.d_aggr = float(self.get_parameter("aggressive_distance").value)
        self.k_aggr = float(self.get_parameter("aggressive_gain_scale").value)
        self.k_null = float(self.get_parameter("nullspace_gain").value)
        self.k_tan = float(self.get_parameter("tangential_gain").value)
        self.max_qdot = float(self.get_parameter("max_joint_velocity").value)
        self.excluded = list(self.get_parameter("excluded_obstacles").value)

        self.capsule_radii = [float(x) for x in list(self.get_parameter("capsule_radii").value)]
        self.capsule_fractions = [float(x) for x in list(self.get_parameter("capsule_fractions").value)]
        self.box_projection_iters = int(self.get_parameter("box_projection_iters").value)

        self.repulsion_spread_enable = bool(self.get_parameter("repulsion_spread_enable").value)
        self.repulsion_spread_samples = int(self.get_parameter("repulsion_spread_samples").value)
        self.repulsion_spread_half_length = float(self.get_parameter("repulsion_spread_half_length").value)

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

        # --- CBF-QP params (with backward compatible defaults) ---
        # If YAML doesn't specify these new keys, defaults are used.
        self.cbf_d_safe = float(self.get_parameter("d_safe").value)
        self.cbf_d_buffer_in = float(self.get_parameter("d_buffer").value)
        self.cbf_d_buffer_out = float(self.get_parameter("d_buffer_out").value)
        if self.cbf_d_buffer_out <= self.cbf_d_buffer_in + 1e-12:
            self.cbf_d_buffer_out = float(1.10 * self.cbf_d_buffer_in)

        self.cbf_alpha = float(self.get_parameter("alpha").value)
        self.cbf_K = int(self.get_parameter("max_constraints").value)
        self.cbf_lambda_reg = float(self.get_parameter("lambda_reg").value)
        self.cbf_rho_slack = float(self.get_parameter("rho_slack").value)
        self.cbf_beta_lpf = float(self.get_parameter("beta_lpf").value)
        self.cbf_output_accel_limit = float(self.get_parameter("output_accel_limit").value)
        self.cbf_approach_speed_limit = float(self.get_parameter("approach_speed_limit").value)
        self.cbf_use_qp = bool(self.get_parameter("use_qp").value)
        self.cbf_eps = float(self.get_parameter("eps").value)
        self.cbf_W_diag = np.array(list(self.get_parameter("qp_weight_diag").value), dtype=float).reshape(-1)
        if self.cbf_W_diag.shape[0] != 7:
            self.get_logger().warn("qp_weight_diag must have length 7; falling back to ones")
            self.cbf_W_diag = np.ones(7, dtype=float)
        self.cbf_W_diag = np.maximum(self.cbf_W_diag, 1e-9)

        # --- Risk-scaled staging / stop gate ---
        self.risk_d_far = float(self.get_parameter("risk_d_far").value)
        self.risk_d_mid = float(self.get_parameter("risk_d_mid").value)
        self.risk_d_near = float(self.get_parameter("risk_d_near").value)
        self.stop_d_in = float(self.get_parameter("stop_distance").value)
        self.stop_d_out = float(self.get_parameter("stop_release_distance").value)
        # Ensure monotonic thresholds
        self.risk_d_far = max(self.risk_d_far, self.risk_d_mid + 1e-6)
        self.risk_d_mid = max(self.risk_d_mid, self.risk_d_near + 1e-6)
        self.risk_d_near = max(self.risk_d_near, self.stop_d_in + 1e-6)
        self.stop_d_out = max(self.stop_d_out, self.stop_d_in + 1e-6)

        self.cbf_alpha_min = float(self.get_parameter("alpha_min").value)
        self.cbf_alpha_max = float(self.get_parameter("alpha_max").value)
        # Backward compatible: if user didn't configure min/max, treat legacy 'alpha' as both.
        if (self.cbf_alpha_min <= 0.0) and (self.cbf_alpha_max <= 0.0):
            self.cbf_alpha_min = float(self.cbf_alpha)
            self.cbf_alpha_max = float(self.cbf_alpha)
        self.cbf_alpha_min = max(0.0, float(self.cbf_alpha_min))
        self.cbf_alpha_max = max(0.0, float(self.cbf_alpha_max))
        if self.cbf_alpha_max < self.cbf_alpha_min:
            self.cbf_alpha_max = self.cbf_alpha_min

        self.cbf_qp_damping_min = float(self.get_parameter("qp_damping_min").value)
        self.cbf_qp_damping_max = float(self.get_parameter("qp_damping_max").value)

        self.cbf_beta_lpf_far = float(self.get_parameter("beta_lpf_far").value)
        self.cbf_beta_lpf_near = float(self.get_parameter("beta_lpf_near").value)
        self.min_distance_lpf = float(self.get_parameter("min_distance_lpf").value)

        self.posture_bias_gain = float(self.get_parameter("posture_bias_gain").value)
        self.posture_reference_param = list(self.get_parameter("posture_reference").value)

        self._cbf_active = False
        self._stop_gate_active = False
        self._qdot_out_prev = np.zeros(7, dtype=float)
        self._qdot_pub_prev = np.zeros(7, dtype=float)
        self._qdot_qp_prev = np.zeros(7, dtype=float)
        self._d_min_filt = 999.0
        self._qp_last_status = "disabled"
        self._qp_last_slack_max = 0.0
        self._last_debug_log_ns = 0

        # Optional QP solver (OSQP) setup
        self._qp_available = False
        self._osqp_solver = None
        self._sp = None
        self._A_data_template = None
        self._A_data_work = None
        self._A_g_slices = None
        self._qp_q_work = None
        self._qp_l_work = None
        self._qp_u_work = None
        self._P_data_template = None
        self._P_data_work = None

        if self.cbf_use_qp and self.cbf_K > 0:
            try:
                import osqp  # type: ignore
                import scipy.sparse as sp  # type: ignore

                self._sp = sp
                self._osqp_mod = osqp
                self._qp_available = True
                self._init_osqp_solver()
                self._qp_last_status = "ready"
            except Exception as e:
                self._qp_available = False
                self._osqp_solver = None
                self._qp_last_status = f"no_osqp:{e.__class__.__name__}"

        self.get_logger().info(f"📊 Parametri CARICATI (da file YAML o default):")
        self.get_logger().info(f"   d_infl (influence_distance): {self.d_infl}")
        self.get_logger().info(f"   d_safe (safety_margin): {self.d_safe}")
        self.get_logger().info(f"   k_null (nullspace_gain): {self.k_null}")
        self.get_logger().info(f"   k_tan (tangential_gain): {self.k_tan}")
        self.get_logger().info(f"   max_qdot (max_joint_velocity): {self.max_qdot}")
        self.get_logger().info(f"   d_aggr (aggressive_distance): {self.d_aggr}")
        self.get_logger().info(f"   k_aggr (aggressive_gain_scale): {self.k_aggr}")
        self.get_logger().info(
            "   capsule geometry: "
            f"radii={self.capsule_radii} | fractions={self.capsule_fractions}"
        )
        self.get_logger().info(
            "   box distance: "
            f"iters={self.box_projection_iters} | spread(enable={self.repulsion_spread_enable}, samples={self.repulsion_spread_samples}, half_len={self.repulsion_spread_half_length})"
        )
        self.get_logger().info(
            "   extra safety: "
            f"ground(enable={self.enable_ground}, z={self.ground_z}, d_infl={self.ground_infl}, d_safe={self.ground_safe}, k={self.k_ground}) | "
            f"self(enable={self.enable_self}, d_infl={self.self_infl}, d_safe={self.self_safe}, k={self.k_self}, skip_adj={self.self_skip_adjacent})"
        )

        self.get_logger().info(
            "   CBF-QP safety filter: "
            f"d_safe={self.cbf_d_safe}, d_buffer_in={self.cbf_d_buffer_in}, d_buffer_out={self.cbf_d_buffer_out}, "
            f"alpha={self.cbf_alpha} (risk-scaled [{self.cbf_alpha_min},{self.cbf_alpha_max}]), K={self.cbf_K}, use_qp={self.cbf_use_qp} (available={self._qp_available}), beta_lpf={self.cbf_beta_lpf}"
        )

        self.get_logger().info(
            "   Risk zones: "
            f"far={self.risk_d_far:.3f} mid={self.risk_d_mid:.3f} near={self.risk_d_near:.3f} "
            f"stop_in={self.stop_d_in:.3f} stop_out={self.stop_d_out:.3f} | "
            f"qp_damping=[{self.cbf_qp_damping_min},{self.cbf_qp_damping_max}] | "
            f"beta_lpf_far={self.cbf_beta_lpf_far} beta_lpf_near={self.cbf_beta_lpf_near}"
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
        # NOTE: capsule_radii is now a parameter (configurable in YAML).

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

            # Catena di 3 capsule sovrapposte (frazioni configurabili via parametro)
            fr = list(self.capsule_fractions) if isinstance(self.capsule_fractions, list) else []
            if len(fr) != 6:
                fr = [0.00, 0.35, 0.25, 0.75, 0.60, 0.95]
            r = list(self.capsule_radii) if isinstance(self.capsule_radii, list) else []
            if len(r) != 3:
                r = [0.15, 0.12, 0.13]

            self.capsules[parent] = [
                {
                    "p0": float(fr[0]) * p_child_local,
                    "p1": float(fr[1]) * p_child_local,
                    "radius": float(r[0]),
                },
                {
                    "p0": float(fr[2]) * p_child_local,
                    "p1": float(fr[3]) * p_child_local,
                    "radius": float(r[1]),
                },
                {
                    "p0": float(fr[4]) * p_child_local,
                    "p1": float(fr[5]) * p_child_local,
                    "radius": float(r[2]),
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
    @staticmethod
    def _clip_aabb(p: np.ndarray, half: np.ndarray) -> np.ndarray:
        return np.clip(p, -half, +half)

    @staticmethod
    def _outward_normal_aabb(p_inside: np.ndarray, half: np.ndarray) -> np.ndarray:
        """Best-effort outward normal when p is (numerically) inside the AABB."""
        p = p_inside.reshape(3)
        h = half.reshape(3)
        # distance to each face
        d = h - np.abs(p)
        # pick the nearest face
        axis = int(np.argmin(d))
        n = np.zeros(3, dtype=float)
        n[axis] = 1.0 if p[axis] >= 0.0 else -1.0
        return n

    @classmethod
    def _closest_points_segment_aabb(
        cls,
        a: np.ndarray,
        b: np.ndarray,
        half: np.ndarray,
        iters: int = 8,
    ):
        """Closest points between segment a-b and axis-aligned box [-half,+half] in the same frame.

        Returns (p_seg, p_box, t) where:
          p_seg = a + t*(b-a), t in [0,1]
          p_box = clip(p_seg)

        Implementation: alternating projections between convex sets (segment and AABB).
        For our small problem size this is fast and stable.
        """
        a = a.reshape(3).astype(float)
        b = b.reshape(3).astype(float)
        half = half.reshape(3).astype(float)

        d = b - a
        dd = float(d @ d)
        if dd < 1e-12:
            p = a.copy()
            q = cls._clip_aabb(p, half)
            return p, q, 0.0

        t = 0.5
        it = max(1, int(iters))
        for _ in range(it):
            p = a + t * d
            q = cls._clip_aabb(p, half)
            t = float((q - a) @ d) / dd
            t = float(np.clip(t, 0.0, 1.0))

        p = a + t * d
        q = cls._clip_aabb(p, half)
        return p, q, t

    def _distance_capsule_to_box(self, p0, p1, r, obs):
        """Distance between a capsule segment (p0-p1, radius r) and a set of OBB boxes in CollisionObject.

        Returns:
          best_d: min distance (can be negative for penetration)
          best_dir: unit vector (world) pointing from obstacle to capsule
          best_p_seg: closest point on capsule segment (world)
          best_p_box: closest point on obstacle (world)
          samples: optional list of repulsion samples (each with p_seg, p_box, dir, distance, weight)
        """
        best_d = 1e6
        best_dir = None
        best_p_seg = None
        best_p_box = None
        best_samples = []

        for i, prim in enumerate(obs.primitives):
            if prim.type != prim.BOX:
                continue

            pose = obs.primitive_poses[i]
            center = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            half = np.array(prim.dimensions, dtype=float) / 2.0

            # Oriented box: use pose orientation
            q = pose.orientation
            R = self._quat_to_rot_matrix(q.x, q.y, q.z, q.w)

            # Transform segment into box-local frame (OBB -> AABB)
            a = R.T @ (p0 - center)
            b = R.T @ (p1 - center)

            p_seg_l, p_box_l, t_star = self._closest_points_segment_aabb(
                a, b, half, iters=self.box_projection_iters
            )

            diff_l = p_seg_l - p_box_l
            diff_n = float(np.linalg.norm(diff_l))
            if diff_n < 1e-9:
                # Segment point is (numerically) on or inside the box: choose outward normal.
                dir_l = self._outward_normal_aabb(p_seg_l, half)
            else:
                dir_l = diff_l / diff_n

            p_seg_w = center + R @ p_seg_l
            p_box_w = center + R @ p_box_l
            dir_w = R @ dir_l
            dir_w = dir_w / (float(np.linalg.norm(dir_w)) + 1e-9)

            dist = float(np.linalg.norm(p_seg_w - p_box_w) - float(r))

            # Optional repulsion samples around the closest point for a smoother "region" effect.
            samples = []
            if self.repulsion_spread_enable and self.repulsion_spread_samples >= 2:
                d_ab = b - a
                L = float(np.linalg.norm(d_ab))
                if L > 1e-9:
                    half_len = max(0.0, float(self.repulsion_spread_half_length))
                    dt = float(np.clip(half_len / L, 0.0, 0.5))
                    n = int(self.repulsion_spread_samples)
                    if (n % 2) == 0:
                        n += 1
                    offsets = np.linspace(-dt, +dt, n)
                    # Gaussian weights over offsets so the "hand" effect is smooth and bounded.
                    # IMPORTANT: Do not weight by 1/(epsilon + distance). That can explode near contact
                    # and produce abrupt, overly strong repulsion.
                    sigma = max(1e-9, 0.5 * dt) if dt > 1e-9 else 1e-9
                    for off in offsets:
                        ti = float(np.clip(float(t_star) + float(off), 0.0, 1.0))
                        pi_l = a + ti * d_ab
                        qi_l = self._clip_aabb(pi_l, half)
                        di_l = pi_l - qi_l
                        di_n = float(np.linalg.norm(di_l))
                        if di_n < 1e-9:
                            ni_l = self._outward_normal_aabb(pi_l, half)
                        else:
                            ni_l = di_l / di_n
                        pi_w = center + R @ pi_l
                        qi_w = center + R @ qi_l
                        ni_w = R @ ni_l
                        ni_w = ni_w / (float(np.linalg.norm(ni_w)) + 1e-9)
                        di = float(np.linalg.norm(pi_w - qi_w) - float(r))
                        # Weight depends only on offset along the capsule segment ("region" shape), bounded in (0,1].
                        w = float(math.exp(-0.5 * float(off * off) / float(sigma * sigma)))
                        samples.append(
                            {
                                "p_seg": pi_w,
                                "p_box": qi_w,
                                "dir": ni_w,
                                "distance": di,
                                "weight": float(w),
                            }
                        )

            if dist < best_d:
                best_d = dist
                best_dir = dir_w
                best_p_seg = p_seg_w
                best_p_box = p_box_w
                best_samples = samples

        return best_d, best_dir, best_p_seg, best_p_box, best_samples

    @staticmethod
    def _stable_sign_from_id(text: str) -> float:
        """Deterministic +/-1 sign from a string id (no dependence on PYTHONHASHSEED)."""
        try:
            v = zlib.crc32(text.encode('utf-8'))
        except Exception:
            v = 0
        return 1.0 if (v % 2) == 0 else -1.0

    @staticmethod
    def _smooth_alpha(d: float, d_infl: float, d_safe: float) -> float:
        """Smooth activation 0..1 (0 at d_infl, 1 at d_safe or closer)."""
        if d_infl <= d_safe + 1e-9:
            return 0.0
        # allow negative distances (penetration): treat as fully active
        if d <= d_safe:
            return 1.0
        if d >= d_infl:
            return 0.0
        x = (d_infl - d) / (d_infl - d_safe)
        x = float(np.clip(x, 0.0, 1.0))
        # smoothstep
        return float(3.0 * x * x - 2.0 * x * x * x)

    @staticmethod
    def _tangential_dir(dir_vec: np.ndarray) -> np.ndarray:
        """Return a unit tangential direction orthogonal to dir_vec (prefer world-up swirl)."""
        d = dir_vec.reshape(3)
        n = float(np.linalg.norm(d))
        if n < 1e-9:
            return np.zeros(3)
        d = d / n
        up = np.array([0.0, 0.0, 1.0], dtype=float)
        t = np.cross(up, d)
        tn = float(np.linalg.norm(t))
        if tn < 1e-6:
            # dir ~ parallel to up, pick another axis
            ax = np.array([1.0, 0.0, 0.0], dtype=float)
            t = np.cross(ax, d)
            tn = float(np.linalg.norm(t))
        if tn < 1e-9:
            return np.zeros(3)
        return t / tn

    # ======================================================
    # RISK-SCALED ZONES (30/20/10/5 cm)
    # ======================================================
    def _risk_weight(self, d: float) -> float:
        """Continuous risk weight w(d) in [0,1], based on staged distance thresholds.

        Mapping (by default):
          d >= 30cm   -> w = 0
          30..20cm    -> w ramps 0 .. 0.25
          20..10cm    -> w ramps 0.25 .. 0.75
          10..5cm     -> w ramps 0.75 .. 1.0
          d <= 5cm    -> w = 1
        """
        df = float(self.risk_d_far)
        dm = float(self.risk_d_mid)
        dn = float(self.risk_d_near)
        ds = float(self.stop_d_in)

        d = float(d)
        if d >= df:
            return 0.0
        if d <= ds:
            return 1.0

        if d > dm:
            x = self._smooth_alpha(d, df, dm)  # 0..1
            return 0.25 * x
        if d > dn:
            x = self._smooth_alpha(d, dm, dn)
            return 0.25 + 0.50 * x

        x = self._smooth_alpha(d, dn, ds)
        return 0.75 + 0.25 * x

    def _alpha_from_distance(self, d: float) -> float:
        w = self._risk_weight(d)
        a0 = float(self.cbf_alpha_min)
        a1 = float(self.cbf_alpha_max)
        return float(a0 + w * (a1 - a0))

    def _qp_gamma_from_distance(self, d: float) -> float:
        w = self._risk_weight(d)
        g0 = float(self.cbf_qp_damping_min)
        g1 = float(self.cbf_qp_damping_max)
        return float(g0 + w * (g1 - g0))

    def _beta_lpf_from_distance(self, d: float) -> float:
        """Risk-scaled output LPF coefficient beta in [0,1].

        beta=1 -> no filtering; beta small -> smoother/laggier.
        """
        w = self._risk_weight(d)
        b_far = float(self.cbf_beta_lpf_far)
        b_near = float(self.cbf_beta_lpf_near)
        return float(np.clip(b_far + w * (b_near - b_far), 0.0, 1.0))

    def _posture_reference(self) -> Optional[np.ndarray]:
        """Return a 7D posture reference (radians) if available."""
        try:
            if isinstance(self.posture_reference_param, list) and len(self.posture_reference_param) == 7:
                return np.array([float(x) for x in self.posture_reference_param], dtype=float).reshape(7)
        except Exception:
            pass
        try:
            # Pinocchio neutral for the reduced model should match 7 DoF here.
            q0 = pin.neutral(self.model)
            q0 = np.array(q0, dtype=float).reshape(-1)
            if q0.shape[0] >= 7:
                return q0[:7].copy()
        except Exception:
            pass
        return None

    # ======================================================
    # CBF-QP SAFETY FILTER
    # ======================================================
    def compute_constraints(
        self,
        candidates: List[dict],
        active_threshold: float,
    ) -> Tuple[np.ndarray, np.ndarray, int, Optional[dict]]:
        """Build top-K CBF constraints from hazard candidates.

        IMPORTANT: constraints are computed ONLY from the closest capsule/contact geometry per hazard.

        Each candidate dict must contain:
          - kind: 'external'|'ground'|'self'
          - hazard: string label
          - d: distance (signed allowed)
          - and geometry needed to compute g:
              external/ground: fid, p (world), n (world)
              self: fid_i, p_i, fid_j, p_j, n (world)

        Returns:
          G: (K,7) stacked gradients (inactive rows are zeros)
          b: (K,) RHS for constraints (inactive are very negative)
          m_active: number of active constraints (<=K)
          active_best: most critical active candidate (min d) or None
        """
        K = max(0, int(self.cbf_K))
        G = np.zeros((K, 7), dtype=float)
        b = np.full((K,), -1e9, dtype=float)  # inactive constraints

        if K == 0:
            return G, b, 0, None

        # Filter by activation distance and sort by distance
        act = [c for c in candidates if float(c.get("d", 1e9)) <= float(active_threshold)]
        act.sort(key=lambda x: float(x.get("d", 1e9)))

        m_active = min(K, len(act))
        active_best = act[0] if m_active > 0 else None

        for i in range(m_active):
            c = act[i]
            d = float(c["d"])
            kind = str(c.get("kind", ""))
            v_obs_proj = 0.0  # obstacle velocity along normal not available -> assume static

            if kind in ("external", "ground"):
                fid = int(c["fid"])
                p = np.array(c["p"], dtype=float).reshape(3)
                n = np.array(c["n"], dtype=float).reshape(3)
                n = n / (float(np.linalg.norm(n)) + self.cbf_eps)
                Jp = self._point_jacobian_world(fid, p)
                g = (n.reshape(1, 3) @ Jp).reshape(-1)
            elif kind == "self":
                fid_i = int(c["fid_i"])
                fid_j = int(c["fid_j"])
                p_i = np.array(c["p_i"], dtype=float).reshape(3)
                p_j = np.array(c["p_j"], dtype=float).reshape(3)
                n = np.array(c["n"], dtype=float).reshape(3)
                n = n / (float(np.linalg.norm(n)) + self.cbf_eps)
                J_i = self._point_jacobian_world(fid_i, p_i)
                J_j = self._point_jacobian_world(fid_j, p_j)
                g = (n.reshape(1, 3) @ (J_i - J_j)).reshape(-1)
            else:
                continue

            # CBF RHS: g^T qdot >= v_obs_proj - alpha*(d - d_safe)
            alpha_i = float(self._alpha_from_distance(d))
            bi = float(v_obs_proj - alpha_i * (d - self.cbf_d_safe))
            # Optional cap on maximum approach speed (negative d_dot).
            # Enforces: d_dot >= -v_limit, i.e. g^T qdot >= -v_limit
            v_lim = float(self.cbf_approach_speed_limit)
            if v_lim > 0.0:
                bi = max(bi, -v_lim)
            G[i, :] = g
            b[i] = bi

        return G, b, m_active, active_best

    def solve_qp_safety_filter(
        self,
        qdot_nom: np.ndarray,
        G: np.ndarray,
        b: np.ndarray,
        gamma: float,
        qdot_prev: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, float, str]]:
        """Solve the CBF-QP using OSQP (if available). Returns (qdot, slack_max, status) or None."""
        if (not self._qp_available) or (self._osqp_solver is None):
            return None

        qdot_nom = np.array(qdot_nom, dtype=float).reshape(7)
        K = int(self.cbf_K)
        if K <= 0:
            return qdot_nom, 0.0, "no_constraints"

        # Fill OSQP update vectors in-place
        # Objective:
        #   (q-qnom)^T W (q-qnom) + lambda||q||^2 + gamma||q-qprev||^2 + rho||s||^2
        # OSQP uses: 1/2 x^T P x + q^T x
        gamma = float(max(0.0, gamma))
        qdot_prev = np.array(qdot_prev, dtype=float).reshape(7)

        # Linear cost on qdot: -2 * (W*qdot_nom + gamma*qdot_prev)
        self._qp_q_work[:] = 0.0
        self._qp_q_work[:7] = -2.0 * (self.cbf_W_diag * qdot_nom + gamma * qdot_prev)

        # Constraint lower bounds for CBF rows
        self._qp_l_work[:K] = b.reshape(-1)[:K]

        # Update A matrix values for G entries (fixed sparsity)
        # A stores columns 0..6 (qdot) first K entries as g[:,j]
        self._A_data_work[:] = self._A_data_template
        for j in range(7):
            sl = self._A_g_slices[j]
            self._A_data_work[sl[0]:sl[1]] = G[:, j]

        # Update P diagonal for qdot block (fixed sparsity: diagonal matrix)
        # Base P already includes 2*(W + lambda_reg). We add 2*gamma on the first 7 diagonal entries.
        if (self._P_data_template is not None) and (self._P_data_work is not None):
            self._P_data_work[:] = self._P_data_template
            self._P_data_work[:7] = self._P_data_template[:7] + (2.0 * gamma)

        try:
            if (self._P_data_work is not None):
                self._osqp_solver.update(Px=self._P_data_work, q=self._qp_q_work, l=self._qp_l_work, Ax=self._A_data_work)
            else:
                self._osqp_solver.update(q=self._qp_q_work, l=self._qp_l_work, Ax=self._A_data_work)
            res = self._osqp_solver.solve()
        except Exception as e:
            return None

        status = str(getattr(res.info, "status", ""))
        status_ok = status.lower().startswith("solved")
        if (not status_ok) or (res.x is None):
            return None

        x = np.array(res.x, dtype=float).reshape(-1)
        qdot = x[:7]
        slack = x[7:7 + K] if x.shape[0] >= (7 + K) else np.zeros(K)
        slack_max = float(np.max(slack)) if slack.size > 0 else 0.0
        return qdot, slack_max, status

    def fallback_projection(
        self,
        qdot_nom: np.ndarray,
        G_active: np.ndarray,
        b_active: np.ndarray,
        iters: int = 3,
    ) -> np.ndarray:
        """Deterministic fallback: sequential projection (POCS) onto half-spaces g^T qdot >= b."""
        q = np.array(qdot_nom, dtype=float).reshape(7)

        # Joint velocity box constraints
        qmin = -float(self.max_qdot)
        qmax = +float(self.max_qdot)
        q = np.clip(q, qmin, qmax)

        M = int(G_active.shape[0])
        if M <= 0:
            return q

        eps = float(self.cbf_eps)
        for _ in range(max(1, int(iters))):
            for i in range(M):
                g = G_active[i, :].reshape(7)
                bi = float(b_active[i])
                g_norm2 = float(g @ g)
                if g_norm2 < 1e-12:
                    continue
                val = float(g @ q)
                if val + 1e-12 < bi:
                    q = q + ((bi - val) / (g_norm2 + eps)) * g
                    q = np.clip(q, qmin, qmax)

        return q

    def _init_osqp_solver(self):
        """Initialize an OSQP problem with fixed size based on max_constraints (K)."""
        K = int(self.cbf_K)
        if K <= 0:
            return

        sp = self._sp
        osqp = self._osqp_mod

        n = 7 + K
        m = (2 * K) + 7

        # Quadratic cost: (q-qnom)^T W (q-qnom) + lambda||q||^2 + rho||s||^2
        # OSQP uses 1/2 x^T P x + q^T x
        P_diag = np.zeros(n, dtype=float)
        P_diag[:7] = 2.0 * (self.cbf_W_diag + float(self.cbf_lambda_reg))
        P_diag[7:] = 2.0 * float(self.cbf_rho_slack)
        P = sp.diags(P_diag, offsets=0, format='csc')

        # Build A with a fixed sparsity pattern.
        # Rows:
        #   0..K-1         : CBF constraints  G q + s >= b
        #   K..2K-1        : s >= 0
        #   2K..2K+6       : qmin <= q <= qmax

        indptr = [0]
        indices: List[int] = []
        data: List[float] = []
        self._A_g_slices = []

        # q columns (0..6)
        for j in range(7):
            col_rows = list(range(0, K)) + [2 * K + j]
            col_data = [0.0] * K + [1.0]
            start = len(data)
            indices.extend(col_rows)
            data.extend(col_data)
            end = start + K
            self._A_g_slices.append((start, end))
            indptr.append(len(data))

        # slack columns (7..7+K-1)
        for k in range(K):
            col_rows = [k, K + k]
            col_data = [1.0, 1.0]
            indices.extend(col_rows)
            data.extend(col_data)
            indptr.append(len(data))

        A = sp.csc_matrix((np.array(data, dtype=float), np.array(indices, dtype=int), np.array(indptr, dtype=int)), shape=(m, n))

        # Bounds l <= A x <= u
        l = np.full(m, -np.inf, dtype=float)
        u = np.full(m, +np.inf, dtype=float)

        # Initialize with all CBF constraints inactive
        l[:K] = -1e9

        # Slack nonnegativity
        l[K:2 * K] = 0.0

        # Joint limits
        qmin = -float(self.max_qdot)
        qmax = +float(self.max_qdot)
        l[2 * K:2 * K + 7] = qmin
        u[2 * K:2 * K + 7] = qmax

        q = np.zeros(n, dtype=float)

        solver = osqp.OSQP()
        solver.setup(
            P=P,
            q=q,
            A=A,
            l=l,
            u=u,
            warm_start=True,
            verbose=False,
            polish=False,
            max_iter=100,
        )

        # Cache arrays for fast update
        self._osqp_solver = solver
        self._A_data_template = A.data.copy()
        self._A_data_work = A.data.copy()
        self._P_data_template = P.data.copy()
        self._P_data_work = P.data.copy()
        self._qp_q_work = q.copy()
        self._qp_l_work = l.copy()
        self._qp_u_work = u.copy()  # (u is constant here; kept for completeness)

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

        # Track closest capsule per external obstacle for CBF constraints
        external_best: Dict[str, dict] = {}
        ground_best: Optional[dict] = None
        self_best: Optional[dict] = None

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
                    d, dir_vec, p_seg, p_box, samples = self._distance_capsule_to_box(p0, p1, caps["radius"], obs)
                    if dir_vec is None:
                        continue

                    # Record best (closest) capsule contact for this obstacle (used by CBF constraints)
                    obs_id = str(getattr(obs, "id", ""))
                    if obs_id not in external_best or float(d) < float(external_best[obs_id]["d"]):
                        external_best[obs_id] = {
                            "kind": "external",
                            "hazard": f"external:{obs_id}",
                            "d": float(d),
                            "fid": int(fid),
                            "p": np.array(p_seg, dtype=float).reshape(3),
                            "n": np.array(dir_vec, dtype=float).reshape(3),
                        }

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

                    sgn = self._stable_sign_from_id(str(getattr(obs, 'id', '')))

                    # If enabled, use multiple points around the closest point to create a "region" repulsion.
                    # Otherwise, fall back to the single closest point.
                    rep_points = samples if (self.repulsion_spread_enable and len(samples) > 0) else [
                        {
                            "p_seg": p_seg,
                            "dir": dir_vec,
                            "distance": float(d),
                            "weight": 1.0,
                        }
                    ]

                    # Combine region samples as a WEIGHTED AVERAGE in joint space.
                    # This keeps the repulsion magnitude bounded and avoids scaling with number of samples.
                    qdot_reg = np.zeros(7)
                    w_sum = 0.0
                    for s in rep_points:
                        ds = float(s.get("distance", d))
                        if ds >= self.d_infl:
                            continue

                        # Base activation (0 at d_infl, 1 at d_aggr or closer)
                        alpha_far = self._smooth_alpha(ds, float(self.d_infl), float(self.d_aggr))
                        # Extra aggressive scaling inside the 20cm zone down to safety_margin
                        alpha_close = self._smooth_alpha(ds, float(self.d_aggr), float(self.d_safe))
                        gain_scale = 1.0 + float(self.k_aggr) * float(alpha_close)

                        dir_s = np.array(s.get("dir", dir_vec), dtype=float).reshape(3)
                        tan = self._tangential_dir(dir_s)
                        w = float(s.get("weight", 1.0))
                        if w <= 0.0:
                            continue

                        xdot_avoid = (
                            (self.k_null * alpha_far * gain_scale) * dir_s
                            + (self.k_tan * alpha_far * gain_scale * sgn) * tan
                        )

                        Jp = self._point_jacobian_world(
                            fid,
                            np.array(s.get("p_seg", p_seg), dtype=float).reshape(3)
                        )
                        qdot_reg += w * (Jp.T @ xdot_avoid)
                        w_sum += w

                    if w_sum > 1e-9:
                        qdot_avoid += (qdot_reg / w_sum)

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

                    # Record best (closest) ground hazard for CBF constraints
                    if (ground_best is None) or (float(d_ground) < float(ground_best["d"])):
                        ground_best = {
                            "kind": "ground",
                            "hazard": "ground:plane",
                            "d": float(d_ground),
                            "fid": int(fid),
                            "p": np.array(p_low, dtype=float).reshape(3),
                            "n": np.array([0.0, 0.0, 1.0], dtype=float),
                        }

                    if d_ground < self.ground_infl:
                        alpha_g = self._smooth_alpha(float(d_ground), float(self.ground_infl), float(self.ground_safe))
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

                    # Record closest self-collision pair for CBF constraints
                    if (self_best is None) or (float(dist) < float(self_best["d"])):
                        n_self = diff / (np.linalg.norm(diff) + 1e-9)
                        self_best = {
                            "kind": "self",
                            "hazard": f"self:{si['parent']}<->{sj['parent']}",
                            "d": float(dist),
                            "fid_i": int(si["fid"]),
                            "p_i": np.array(cp_i, dtype=float).reshape(3),
                            "fid_j": int(sj["fid"]),
                            "p_j": np.array(cp_j, dtype=float).reshape(3),
                            "n": np.array(n_self, dtype=float).reshape(3),
                        }

                    self.distances_data.append({
                        "p_capsule": cp_i,
                        "p_obstacle": cp_j,
                        "distance": dist,
                    })

                    if dist >= self.self_infl:
                        continue

                    # Repel the two points away from each other
                    n = diff / (np.linalg.norm(diff) + 1e-9)
                    alpha_s = self._smooth_alpha(float(dist), float(self.self_infl), float(self.self_safe))
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

        # --------------------------
        # CBF-QP Safety Filter stage
        # --------------------------
        # Nominal command for this node = the current (potential-field) avoidance output.
        qdot_nom = np.array(qdot_avoid, dtype=float).reshape(7)

        # Filter the min distance signal (used only for risk scaling / downstream diagnostics).
        d_beta = float(np.clip(self.min_distance_lpf, 0.0, 1.0))
        self._d_min_filt = float(d_beta * float(d_min) + (1.0 - d_beta) * float(self._d_min_filt))

        # Hard stop gate at stop_d_in, release at stop_d_out (hysteresis).
        if (not self._stop_gate_active) and (float(d_min) <= float(self.stop_d_in)):
            self._stop_gate_active = True
        elif self._stop_gate_active and (float(d_min) >= float(self.stop_d_out)):
            self._stop_gate_active = False

        # Optional posture bias (OFF by default): only meaningful near obstacles.
        # This helps reduce the "fold on itself" behavior by gently pulling toward a neutral posture.
        if float(self.posture_bias_gain) > 0.0:
            q_ref = self._posture_reference()
            if (q_ref is not None) and isinstance(self.q, np.ndarray) and (self.q.shape[0] >= 7):
                w_post = float(self._risk_weight(self._d_min_filt))
                q_cur = np.array(self.q, dtype=float).reshape(-1)[:7]
                qdot_post = float(self.posture_bias_gain) * w_post * (q_ref - q_cur)
                qdot_nom = qdot_nom + qdot_post

        # Hysteresis on activation to avoid chatter/jitter
        # Use the filtered distance for stability (constraints still use raw per-candidate distances).
        d_act = float(self._d_min_filt)
        d_in = float(max(self.cbf_d_buffer_in, self.risk_d_far))
        d_out = float(max(self.cbf_d_buffer_out, self.risk_d_far))

        if (not self._cbf_active) and (d_act <= d_in):
            self._cbf_active = True
        elif self._cbf_active and (d_act >= d_out):
            self._cbf_active = False

        active_thr = float(d_out) if self._cbf_active else float(d_in)

        candidates: List[dict] = []
        if len(external_best) > 0:
            candidates.extend(list(external_best.values()))
        if ground_best is not None:
            candidates.append(ground_best)
        if self_best is not None:
            candidates.append(self_best)

        G, b_cbf, m_active, active_best = self.compute_constraints(candidates, active_thr)

        # Solve (QP preferred) or fallback
        qdot_safe = None
        slack_max = 0.0
        qp_status = "inactive"
        if self._stop_gate_active:
            qdot_safe = np.zeros(7, dtype=float)
            slack_max = 0.0
            qp_status = "stop_gate"
        elif m_active <= 0:
            qdot_safe = qdot_nom
            qp_status = "no_constraints"
        else:
            gamma = float(self._qp_gamma_from_distance(self._d_min_filt))
            # Try QP with OSQP
            if self.cbf_use_qp and self._qp_available:
                qp_res = self.solve_qp_safety_filter(qdot_nom, G, b_cbf, gamma=gamma, qdot_prev=self._qdot_qp_prev)
                if qp_res is not None:
                    qdot_safe, slack_max, qp_status = qp_res
                else:
                    qdot_safe = None

            # Robust fallback
            if qdot_safe is None:
                qdot_safe = self.fallback_projection(qdot_nom, G[:m_active, :], b_cbf[:m_active], iters=3)
                slack_max = 0.0
                qp_status = "fallback_projection"

        # Remember previous (pre-LPF) safe output for QP damping
        try:
            self._qdot_qp_prev = np.array(qdot_safe, dtype=float).reshape(7)
        except Exception:
            self._qdot_qp_prev = self._qdot_qp_prev

        # Joint velocity limits (box)
        qdot_safe = np.clip(qdot_safe, -float(self.max_qdot), +float(self.max_qdot))

        # Low-pass filter on output to reduce jitter (risk-scaled)
        beta = float(self._beta_lpf_from_distance(self._d_min_filt))
        qdot_out = beta * qdot_safe + (1.0 - beta) * self._qdot_out_prev
        self._qdot_out_prev = qdot_out.copy()

        self._qp_last_status = str(qp_status)
        self._qp_last_slack_max = float(slack_max)

        # Optional acceleration (rate) limiting on the published command to reduce "scatti".
        # Per-joint: |dqdot/dt| <= output_accel_limit
        acc_lim = float(self.cbf_output_accel_limit)
        if acc_lim > 0.0:
            dt = 1.0 / float(max(1.0, self.rate))
            dq = qdot_out - self._qdot_pub_prev
            dq_max = float(acc_lim) * float(dt)
            dq = np.clip(dq, -dq_max, +dq_max)
            qdot_out = self._qdot_pub_prev + dq
        self._qdot_pub_prev = qdot_out.copy()

        self.pub.publish(Float64MultiArray(data=qdot_out.tolist()))

        # --------------------------
        # Publish ACTIVE CBF hazard signals (consistent with constraints)
        # --------------------------
        hazard_msg = String()
        # Publish filtered global minimum distance so downstream logic can react early but smoothly.
        self.min_dist_pub.publish(Float64MultiArray(data=[float(self._d_min_filt)]))

        if self._stop_gate_active:
            hazard_msg.data = "stop_gate"
            self.hazard_pub.publish(hazard_msg)
            self.jac_pub.publish(jac_zero)
        elif (active_best is None) or (m_active <= 0):
            hazard_msg.data = "none"
            self.hazard_pub.publish(hazard_msg)
            self.jac_pub.publish(jac_zero)
        else:
            hazard_msg.data = str(active_best.get("hazard", "none"))
            self.hazard_pub.publish(hazard_msg)
            # Jacobian row for the most critical ACTIVE constraint
            self.jac_pub.publish(Float64MultiArray(data=G[0, :].reshape(-1).tolist()))

        # Debug log (throttled): useful to confirm the filter is behaving
        try:
            now_ns = int(self.get_clock().now().nanoseconds)
            if now_ns - int(self._last_debug_log_ns) >= 1_000_000_000:
                self._last_debug_log_ns = now_ns
                w_dbg = float(self._risk_weight(self._d_min_filt))
                gamma_dbg = float(self._qp_gamma_from_distance(self._d_min_filt))
                self.get_logger().debug(
                    f"CBF-QP: d_min_raw={float(d_min):.4f} d_min_filt={float(self._d_min_filt):.4f} "
                    f"w={w_dbg:.2f} gamma={gamma_dbg:.2f} stop={self._stop_gate_active} "
                    f"active={self._cbf_active} m_active={m_active} status={self._qp_last_status} "
                    f"slack_max={self._qp_last_slack_max:.3e}"
                )
        except Exception:
            pass

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


if __name__ == "__main__":
    main()
