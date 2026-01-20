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

import numpy as np
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import MarkerArray
from moveit_msgs.msg import PlanningScene
from rcl_interfaces.srv import GetParameters

import pinocchio as pin

# Shared math helpers (installed via franka_simulation/CMakeLists.txt).
from utils.avoidance_math import (
    OsqpCbfQpSolver,
    build_capsules_for_link_pairs,
    build_cbf_constraints,
    build_reduced_pinocchio_model_from_urdf,
    filtered_collision_objects_from_planning_scene,
    ordered_joint_positions_from_joint_state,
)

# Controller decomposition (keeps this file high-level).
from utils.avoidance_core import iter_world_capsule_segments, scan_external_and_ground, scan_self_collision
from utils.cbf_filter import (
    CbfFilterParams,
    CbfFilterState,
    apply_cbf_qp_safety_filter,
    debug_throttled,
)
from utils.rviz_markers import build_marker_array

# ROS-facing glue (service call + callback factories).
from utils.ros_setup import (
    init_pinocchio_and_capsules,
    make_joint_state_callback,
    make_planning_scene_callback,
)


class NullSpaceAvoidance(Node):

    def __init__(self):
        super().__init__("online_avoidance_controller")

        # Nomi giunti (ordine canonico usato anche dal controller ros2_control)
        self.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ]

        # ================= PARAMETERS =================
        # ROS2 (Humble) best practice:
        # - the node MUST declare parameters (with defaults) in code
        # - a YAML file can override them at launch time.
        #
        # Keeping all defaults in a single table makes it harder to accidentally forget a parameter
        # (and reduces boilerplate without changing behavior).
        default_params = {
            # Control
            "control_rate": 100.0,

            # Distances / gains
            "influence_distance": 0.30,
            "safety_margin": 0.08,
            # When closer than this distance, avoidance becomes intentionally more aggressive.
            # This is *not* the same as safety_margin: it is a "start pushing hard" threshold.
            "aggressive_distance": 0.20,
            "aggressive_gain_scale": 3.0,
            # Overall scaling for the avoidance twist (used for both repulsive and tangential components)
            "nullspace_gain": 0.15,
            # Extra tangential (swirl) component to break local minima near obstacles.
            "tangential_gain": 0.20,
            "max_joint_velocity": 0.25,
            "excluded_obstacles": ["ground_plane", "ground", "floor", "plane"],

            # Capsule geometry tuning (m)
            "capsule_radii": [0.15, 0.12, 0.13],
            # Format: [p0_0, p1_0, p0_1, p1_1, p0_2, p1_2]
            "capsule_fractions": [0.00, 0.35, 0.25, 0.75, 0.60, 0.95],

            # Distance model knobs
            "box_projection_iters": 8,
            "repulsion_spread_enable": True,
            "repulsion_spread_samples": 5,         # odd number recommended (e.g., 3/5/7)
            "repulsion_spread_half_length": 0.10,  # m along the capsule segment

            # Extra safety layers (approximate but fast): ground + self-collision
            "enable_ground_avoidance": True,
            "ground_z": 0.0,  # world Z of the floor plane
            "ground_influence_distance": 0.15,
            "ground_safety_margin": 0.05,
            "ground_gain": 0.25,

            "enable_self_collision_avoidance": True,
            "self_influence_distance": 0.12,
            "self_safety_margin": 0.03,
            "self_gain": 0.25,
            # Skip capsule pairs belonging to links closer than this in the kinematic chain
            "self_skip_adjacent_links": 1,

            # ================= CBF-QP SAFETY FILTER =================
            # Barrier function: h = d - d_safe
            # Constraint: g^T qdot >= v_obs_proj - alpha*(d - d_safe)
            "d_safe": 0.08,              # [m] safety distance
            "d_buffer": 0.30,            # [m] activation distance (influence zone for the filter)
            "d_buffer_out": 0.0,         # [m] hysteresis exit threshold (0 -> auto)
            "alpha": 5.0,                # [1/s] CBF gain
            "max_constraints": 5,        # K (top-K closest hazards)
            "lambda_reg": 1e-6,          # regularization on qdot
            "rho_slack": 100.0,          # slack penalty
            "beta_lpf": 0.80,            # LPF on output qdot (kept for backward compatibility)
            "output_accel_limit": 0.0,   # [rad/s^2] 0 disables rate limiting
            "approach_speed_limit": 0.0, # [m/s] 0 disables extra cap on negative d_dot
            "use_qp": True,              # try OSQP if available
            "eps": 1e-9,                 # numerical epsilon
            "qp_weight_diag": [1.0] * 7, # diagonal W in ||qdot-qdot_nom||_W

            # ===== Risk-scaled zones (30/20/10/5 cm) + Stop Gate =====
            "risk_d_far": 0.30,              # [m] start reacting
            "risk_d_mid": 0.20,              # [m] medium zone
            "risk_d_near": 0.10,             # [m] strong zone
            "stop_distance": 0.05,           # [m] hard stop enter
            "stop_release_distance": 0.06,   # [m] hard stop exit (hysteresis)

            # Risk-scaled CBF alpha: alpha(d) = lerp(alpha_min, alpha_max, w(d))
            # Defaults keep legacy behavior (alpha_min==alpha_max==alpha).
            "alpha_min": 5.0,                # [1/s]
            "alpha_max": 5.0,                # [1/s]

            # QP damping term: gamma(d) * ||qdot - qdot_prev||^2
            "qp_damping_min": 0.0,           # >= 0
            "qp_damping_max": 0.0,           # >= 0

            # Risk-scaled output LPF (beta near should be smaller for smoother motion)
            "beta_lpf_far": 0.80,            # 0..1
            "beta_lpf_near": 0.80,           # 0..1

            # Smoothing for published min distance signal (for downstream blending/visualization)
            "min_distance_lpf": 0.50,        # 0..1 (1.0 = no filtering)

            # Optional posture bias (OFF by default)
            "posture_bias_gain": 0.0,        # [1/s]
            "posture_reference": [],         # 7 values (radians)
        }

        self.declare_parameters("", [(k, v) for k, v in default_params.items()])
        
        # Carica i valori (da default o override YAML). Conversioni tipizzate per chiarezza.
        p = lambda n: self.get_parameter(n).value
        p_float = lambda n: float(p(n))
        p_int = lambda n: int(p(n))
        p_bool = lambda n: bool(p(n))
        p_list_float = lambda n: [float(x) for x in list(p(n))]
        p_list_str = lambda n: [str(x) for x in list(p(n))]

        self.rate = p_float("control_rate")
        self.d_infl = p_float("influence_distance")
        self.d_safe = p_float("safety_margin")
        self.d_aggr = p_float("aggressive_distance")
        self.k_aggr = p_float("aggressive_gain_scale")
        self.k_null = p_float("nullspace_gain")
        self.k_tan = p_float("tangential_gain")
        self.max_qdot = p_float("max_joint_velocity")
        self.excluded = p_list_str("excluded_obstacles")

        self.capsule_radii = p_list_float("capsule_radii")
        self.capsule_fractions = p_list_float("capsule_fractions")
        self.box_projection_iters = p_int("box_projection_iters")

        self.repulsion_spread_enable = p_bool("repulsion_spread_enable")
        self.repulsion_spread_samples = p_int("repulsion_spread_samples")
        self.repulsion_spread_half_length = p_float("repulsion_spread_half_length")

        self.enable_ground = p_bool("enable_ground_avoidance")
        self.ground_z = p_float("ground_z")
        self.ground_infl = p_float("ground_influence_distance")
        self.ground_safe = p_float("ground_safety_margin")
        self.k_ground = p_float("ground_gain")

        self.enable_self = p_bool("enable_self_collision_avoidance")
        self.self_infl = p_float("self_influence_distance")
        self.self_safe = p_float("self_safety_margin")
        self.k_self = p_float("self_gain")
        self.self_skip_adjacent = p_int("self_skip_adjacent_links")

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

        # These two params were added later; when running with an older installed YAML
        # (or a different overlay) they may not be initialized. Default to "off".
        try:
            self.posture_bias_gain = float(self.get_parameter("posture_bias_gain").value)
        except Exception:
            self.posture_bias_gain = 0.0

        try:
            self.posture_reference_param = list(self.get_parameter("posture_reference").value)
        except Exception:
            self.posture_reference_param = []

        # Safety filter runtime state (kept in a dedicated struct for clarity).
        self._cbf_state = CbfFilterState()

        # Optional QP solver (OSQP) setup (moved to utils wrapper)
        self._qp_solver = None
        self._qp_available = False
        if self.cbf_use_qp and self.cbf_K > 0:
            self._qp_solver = OsqpCbfQpSolver(
                K=int(self.cbf_K),
                W_diag=np.array(self.cbf_W_diag, dtype=float).reshape(-1),
                lambda_reg=float(self.cbf_lambda_reg),
                rho_slack=float(self.cbf_rho_slack),
                max_abs_vel=float(self.max_qdot),
                max_iter=100,
            )
            self._qp_available = bool(getattr(self._qp_solver, "available", False))
            self._cbf_state.qp_last_status = str(getattr(self._qp_solver, "init_status", "disabled"))

        # Bundle safety-filter params in one place (keeps _control_loop uncluttered).
        self._cbf_params = CbfFilterParams(
            rate=float(self.rate),
            max_qdot=float(self.max_qdot),
            cbf_d_safe=float(self.cbf_d_safe),
            cbf_d_buffer_in=float(self.cbf_d_buffer_in),
            cbf_d_buffer_out=float(self.cbf_d_buffer_out),
            risk_d_far=float(self.risk_d_far),
            risk_d_mid=float(self.risk_d_mid),
            risk_d_near=float(self.risk_d_near),
            stop_d_in=float(self.stop_d_in),
            stop_d_out=float(self.stop_d_out),
            cbf_eps=float(self.cbf_eps),
            cbf_K=int(self.cbf_K),
            cbf_approach_speed_limit=float(self.cbf_approach_speed_limit),
            cbf_alpha_min=float(self.cbf_alpha_min),
            cbf_alpha_max=float(self.cbf_alpha_max),
            cbf_use_qp=bool(self.cbf_use_qp),
            cbf_qp_damping_min=float(self.cbf_qp_damping_min),
            cbf_qp_damping_max=float(self.cbf_qp_damping_max),
            beta_lpf_far=float(self.cbf_beta_lpf_far),
            beta_lpf_near=float(self.cbf_beta_lpf_near),
            min_distance_lpf=float(self.min_distance_lpf),
            output_accel_limit=float(self.cbf_output_accel_limit),
            posture_bias_gain=float(self.posture_bias_gain),
            posture_reference_param=list(self.posture_reference_param),
        )

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
        self.pin_ok, self.model, self.data, self.frame_ids, self.capsules = init_pinocchio_and_capsules(
            self,
            link_pairs=list(self.link_pairs),
            capsule_fractions=list(self.capsule_fractions),
            capsule_radii=list(self.capsule_radii),
        )

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
        self.create_subscription(
            JointState,
            "/joint_states",
            make_joint_state_callback(controller=self, joint_names=self.joint_names),
            10,
        )
        self.create_subscription(
            PlanningScene,
            "/obstacle_scene",
            make_planning_scene_callback(controller=self, excluded_substrings=self.excluded),
            1,
        )

        self.pub = self.create_publisher(Float64MultiArray, "/avoidance/velocity", 10)
        self.min_dist_pub = self.create_publisher(Float64MultiArray, "/avoidance/min_distance", 10)
        # Raw, unfiltered global minimum distance (safety-critical for downstream blending).
        self.min_dist_raw_pub = self.create_publisher(Float64MultiArray, "/avoidance/min_distance_raw", 10)

        # Coherent pair (d_closest_raw, j_row_closest) to avoid mismatch between
        # filtered min distance and the active constraint Jacobian.
        # Message format: [d_closest, j_row_0..j_row_6]
        self.closest_constraint_pub = self.create_publisher(Float64MultiArray, "/avoidance/closest_constraint", 10)
        self.closest_hazard_pub = self.create_publisher(String, "/avoidance/closest_hazard", 10)

        # riga di Jacobiano (1x7) del punto più critico: d_dot ≈ j_row @ qdot
        self.jac_pub = self.create_publisher(Float64MultiArray, "/avoidance/jacobian", 10)
        # Debug/diagnostics: which hazard is currently the most critical (helps explain stalls)
        self.hazard_pub = self.create_publisher(String, "/avoidance/hazard", 10)

        # Control loop @ 100 Hz
        self.create_timer(1.0 / self.rate, self._control_loop)
        # Marker visualization @ 10 Hz (ridotto per evitare DDS buffer overflow)
        self.create_timer(0.1, self._publish_markers_only)

        self.get_logger().info("🟢 Null-Space Avoidance Controller READY")

    def _publish_cbf_diagnostics(
        self,
        jac_zero: Float64MultiArray,
        G: Optional[np.ndarray],
        m_active: int,
        active_best: Optional[dict],
        d_min_raw: float,
    ) -> None:
        """Publish min-distance, hazard string, and active constraint Jacobian row.

        NOTE: Behavior is intentionally kept identical to the previous version:
        - When stop gate is active OR there are no active constraints -> publish jac_zero
        - Otherwise publish G[0, :] (the most critical ACTIVE constraint row)
        """
        hazard_msg = String()

        # Publish BOTH raw and filtered global min distance.
        # - raw: safety-critical (avoids latency)
        # - filtered: smooth visualization / non-critical shaping
        self.min_dist_raw_pub.publish(Float64MultiArray(data=[float(d_min_raw)]))
        self.min_dist_pub.publish(Float64MultiArray(data=[float(self._cbf_state.d_min_filt)]))

        if self._cbf_state.stop_gate_active:
            hazard_msg.data = "stop_gate"
            self.hazard_pub.publish(hazard_msg)
            self.jac_pub.publish(jac_zero)
            return

        if (active_best is None) or (m_active <= 0) or (G is None):
            hazard_msg.data = "none"
            self.hazard_pub.publish(hazard_msg)
            self.jac_pub.publish(jac_zero)
            return

        hazard_msg.data = str(active_best.get("hazard", "none"))
        self.hazard_pub.publish(hazard_msg)
        self.jac_pub.publish(Float64MultiArray(data=G[0, :].reshape(-1).tolist()))

    def _control_loop(self):
        # High-level flow:
        #  1) sanity check / publish zeros if not ready
        #  2) update kinematics
        #  3) compute nominal avoidance (external + ground + self)
        #  4) apply CBF-QP safety layer (stop gate + constraints + smoothing)
        #  5) publish commands + diagnostics
        #  6) rebuild RViz marker cache (published separately at 10Hz)

        zero = Float64MultiArray(data=[0.0] * 7)
        jac_zero = Float64MultiArray(data=[0.0] * 7)

        if not (self.pin_ok and isinstance(self.q, np.ndarray)):
            self.pub.publish(zero)
            self.jac_pub.publish(jac_zero)
            # Keep downstream nodes from using stale values.
            try:
                self.min_dist_raw_pub.publish(Float64MultiArray(data=[999.0]))
                self.min_dist_pub.publish(Float64MultiArray(data=[999.0]))
                self.closest_constraint_pub.publish(Float64MultiArray(data=[999.0] + [0.0] * 7))
                msg = String()
                msg.data = "none"
                self.closest_hazard_pub.publish(msg)
            except Exception:
                pass
            return

        # Update kinematics for current q
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        # --- Build world geometry (capsule segments)
        segments = iter_world_capsule_segments(capsules=self.capsules, frame_ids=self.frame_ids, data=self.data)

        # --- Nominal avoidance (external obstacles + ground) + debug distances
        qdot_external_ground, d_min, external_best, ground_best, dist_ext_ground = scan_external_and_ground(
            segments=segments,
            obstacles=list(self.obstacles),
            model=self.model,
            data=self.data,
            q=self.q,
            box_projection_iters=int(self.box_projection_iters),
            repulsion_spread_enable=bool(self.repulsion_spread_enable),
            repulsion_spread_samples=int(self.repulsion_spread_samples),
            repulsion_spread_half_length=float(self.repulsion_spread_half_length),
            d_infl=float(self.d_infl),
            d_aggr=float(self.d_aggr),
            d_safe=float(self.d_safe),
            k_aggr=float(self.k_aggr),
            k_null=float(self.k_null),
            k_tan=float(self.k_tan),
            enable_ground=bool(self.enable_ground),
            ground_z=float(self.ground_z),
            ground_infl=float(self.ground_infl),
            ground_safe=float(self.ground_safe),
            k_ground=float(self.k_ground),
        )

        # --- Nominal avoidance (self-collision) + debug distances
        qdot_self, d_min, self_best, dist_self = scan_self_collision(
            segments=segments,
            model=self.model,
            data=self.data,
            q=self.q,
            enable_self=bool(self.enable_self),
            self_skip_adjacent_links=int(self.self_skip_adjacent),
            self_infl=float(self.self_infl),
            self_safe=float(self.self_safe),
            k_self=float(self.k_self),
            d_min_in=float(d_min),
        )

        # Distances list used only for RViz debug markers (ordering preserved)
        self.distances_data = list(dist_ext_ground) + list(dist_self)

        qdot_nom = np.array(qdot_external_ground + qdot_self, dtype=float).reshape(7)

        # --- Build candidate list for the safety filter
        candidates: List[dict] = []
        if len(external_best) > 0:
            candidates.extend(list(external_best.values()))
        if ground_best is not None:
            candidates.append(ground_best)
        if self_best is not None:
            candidates.append(self_best)

        # --- Safety filter (CBF-QP) + smoothing
        qdot_out, G, m_active, active_best, _, _, _ = apply_cbf_qp_safety_filter(
            qdot_nom=qdot_nom,
            d_min=float(d_min),
            candidates=candidates,
            model=self.model,
            data=self.data,
            q=self.q,
            params=self._cbf_params,
            state=self._cbf_state,
            qp_solver=self._qp_solver,
            qp_available=bool(self._qp_available),
        )

        # ------------------------------------------------------------------
        # Publish a *coherent* closest hazard pair for downstream blending:
        #   (d_closest_raw, j_row_closest)
        # This intentionally does NOT depend on whether the CBF is active.
        # ------------------------------------------------------------------
        try:
            if len(candidates) > 0:
                Gc, _, mc, best_c = build_cbf_constraints(
                    list(candidates),
                    float(1e9),
                    K=1,
                    cbf_eps=float(self._cbf_params.cbf_eps),
                    cbf_d_safe=float(self._cbf_params.cbf_d_safe),
                    approach_speed_limit=float(self._cbf_params.cbf_approach_speed_limit),
                    alpha_min=float(self._cbf_params.cbf_alpha_min),
                    alpha_max=float(self._cbf_params.cbf_alpha_max),
                    risk_d_far=float(self._cbf_params.risk_d_far),
                    risk_d_mid=float(self._cbf_params.risk_d_mid),
                    risk_d_near=float(self._cbf_params.risk_d_near),
                    stop_distance=float(self._cbf_params.stop_d_in),
                    model=self.model,
                    data=self.data,
                    q=self.q,
                )
                if (int(mc) > 0) and (best_c is not None):
                    d_closest = float(best_c.get("d", float(d_min)))
                    j_row_closest = np.array(Gc[0, :], dtype=float).reshape(-1)
                    self.closest_constraint_pub.publish(
                        Float64MultiArray(data=[float(d_closest)] + j_row_closest.tolist())
                    )
                    msg = String()
                    msg.data = str(best_c.get("hazard", "none"))
                    self.closest_hazard_pub.publish(msg)
                else:
                    self.closest_constraint_pub.publish(Float64MultiArray(data=[999.0] + [0.0] * 7))
                    msg = String()
                    msg.data = "none"
                    self.closest_hazard_pub.publish(msg)
            else:
                self.closest_constraint_pub.publish(Float64MultiArray(data=[999.0] + [0.0] * 7))
                msg = String()
                msg.data = "none"
                self.closest_hazard_pub.publish(msg)
        except Exception:
            pass

        # Publish the joint velocity command (same topic/type as before)
        self.pub.publish(Float64MultiArray(data=np.array(qdot_out, dtype=float).reshape(-1).tolist()))

        # Publish diagnostics (min distance, hazard string, and jacobian of the most critical active constraint)
        self._publish_cbf_diagnostics(
            jac_zero=jac_zero,
            G=G,
            m_active=int(m_active),
            active_best=active_best,
            d_min_raw=float(d_min),
        )

        # Throttled debug log (1Hz)
        debug_throttled(
            logger=self.get_logger(),
            now_ns=int(self.get_clock().now().nanoseconds),
            d_min_raw=float(d_min),
            m_active=int(m_active),
            params=self._cbf_params,
            state=self._cbf_state,
        )

        # Build and cache markers (published by the 10Hz timer)
        self.last_marker_array = build_marker_array(
            capsules=self.capsules,
            frame_ids=self.frame_ids,
            data=self.data,
            distances_data=self.distances_data,
            d_infl=float(self.d_infl),
            stamp_msg=self.get_clock().now().to_msg(),
            logger=self.get_logger(),
        )

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
