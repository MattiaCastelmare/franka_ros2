#!/usr/bin/env python3
"""
VELOCITY CONTROL BLENDER - VERSIONE SEMPLICE (con vincolo su ḋ)
================================================================

Segue la traiettoria punto per punto e combina il tracking con
l'avoidance usando una proiezione in spazio di giunto che garantisce:

    d_dot = j_row @ qdot >= d_dot_min(d)

Questo evita:
 - blocchi sull'ostacolo
 - collassi sull'ostacolo
 - spinte eccessive e rientri bruschi
"""

import numpy as np
import rclpy
from rclpy.node import Node
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray, Bool, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# Low-level helpers extracted to utils (installed via franka_simulation/CMakeLists.txt).
from utils.velocity_blender_core import (
    EmergencyParams,
    EmergencyRecoveryState,
    InfluenceParams,
    apply_output_filter_and_constraints,
    compute_influence_zone_command,
    compute_polyline_arc_lengths,
    emergency_override,
    interpolate_at_s,
    nearest_point_on_polyline,
)


class SimpleVelocityBlender(Node):

    def __init__(self):
        super().__init__("velocity_control_blender")

        # Nomi giunti
        self.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ]
        self.n_dof = 7

        # ===== PARAMETRI (da ROS/YAML) =====
        # Nota: per una vista completa/ordinata dei parametri, guarda:
        #   franka_simulation/config/velocity_blender_params.yaml
        self._declare_parameters()
        self._load_parameters()

        # ===== STATO =====
        self.q = np.zeros(self.n_dof)              # Posizione corrente
        self.qdot_avoid = np.zeros(self.n_dof)     # Velocità di avoidance
        self.qdot_avoid_filt = np.zeros(self.n_dof)  # filtered avoidance
        self.qdot_prev = np.zeros(self.n_dof)      # Velocità precedente (per smoothing)
        self.J_avoid = np.zeros((1, self.n_dof))   # Jacobiano del punto più critico (1x7)
        self.min_dist = 999.0                      # Distanza minima iniziale "lontana"
        self.min_dist_raw = 999.0                  # Distanza minima RAW (non filtrata)
        self.hazard = "none"

        # Coherent closest constraint from the avoidance controller
        # Message format: [d_closest, j_row_0..j_row_6]
        self.closest_d = 999.0
        self.closest_j_row = np.zeros(self.n_dof)
        self.closest_hazard = "none"

        # Emergency / recovery state (kept as a small struct to avoid scattering fields).
        self._er_state = EmergencyRecoveryState()

        # Traiettoria
        self.trajectory_points = []      # Lista di configurazioni target
        self.current_index = 0           # Indice del punto corrente
        self.active = False              # Traiettoria attiva?

        # Monotonic progress index (never decreases) to avoid oscillations on paths with noise.
        self._progress_index = 0
        # Continuous progress along the polyline (joint-space arc length)
        self._progress_s = 0.0
        self._traj_s = None  # cumulative arc-length at each waypoint (len=N)

        # ===== SUBSCRIBERS =====
        self.create_subscription(
            JointState, "/joint_states",
            self.joint_state_cb, 10
        )

        self.create_subscription(
            JointTrajectory, "/velocity_blender/trajectory",
            self.trajectory_cb,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        )

        self.create_subscription(
            Float64MultiArray, "/avoidance/velocity",
            self.avoidance_cb, 10
        )

        self.create_subscription(
            Float64MultiArray, "/avoidance/jacobian",
            self.avoidance_jac_cb, 10
        )

        self.create_subscription(
            Float64MultiArray, "/avoidance/min_distance",
            self.min_dist_cb, 10
        )

        # Raw global min distance (safety-critical)
        self.create_subscription(
            Float64MultiArray, "/avoidance/min_distance_raw",
            self.min_dist_raw_cb, 10
        )

        # Coherent (d_closest, j_row_closest)
        self.create_subscription(
            Float64MultiArray, "/avoidance/closest_constraint",
            self.closest_constraint_cb, 10
        )

        self.create_subscription(
            String,
            "/avoidance/closest_hazard",
            self.closest_hazard_cb,
            10,
        )

        self.create_subscription(
            String,
            "/avoidance/hazard",
            self.hazard_cb,
            10,
        )

        # External pause control (typically from an interactive test node)
        self.create_subscription(
            Bool,
            "/velocity_blender/pause",
            self.pause_cb,
            10,
        )

        # ===== PUBLISHER =====
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/fr3_velocity_controller/commands",
            10
        )

        # ===== TIMER =====
        period = float(self.control_period_s)
        if period <= 0.0:
            period = 0.01
        self.create_timer(period, self.control_loop)

        self.get_logger().info("✅ Simple Velocity Blender (ḋ-constrained) started")
        self.get_logger().info(
            f"   Kp={self.kp}, max_vel={self.max_vel}, d_safe={self.d_safe}, d_infl={self.d_infl}"
        )
        self.get_logger().info(
            f"   use_avoidance_velocity={self.use_avoidance_velocity}, avoidance_normal_only={self.avoidance_normal_only}, "
            f"null_boost_max={self.null_boost_max}, avoidance_ratio_max={self.avoidance_ratio_max}"
        )
        self.get_logger().info(
            f"   d_dot_min_far={self.d_dot_min_far}, d_dot_min_close={self.d_dot_min_close}, "
            f"avoidance_tangent_weight={self.avoidance_tangent_weight}, normal_correction_max={self.normal_correction_max}"
        )
        self.get_logger().info(
            f"   d_dot_push_gain={self.d_dot_push_gain}, d_dot_push_max={self.d_dot_push_max}, "
            f"avoidance_weight_max={self.avoidance_weight_max}"
        )
        self.get_logger().info(
            f"   tangent_escape_enable={self.tangent_escape_enable}, tangent_escape_speed={self.tangent_escape_speed}, "
            f"tangent_escape_err_min={self.tangent_escape_err_min}"
        )
        self.get_logger().info(
            f"   pause_enable={self.pause_enable} (topic: /velocity_blender/pause)"
        )


    def _declare_parameters(self):
        """Declare all ROS parameters with defaults (single source of truth)."""
        defaults = {
            # Core tracking
            "control_period_s": 0.01,
            "kp": 20.0,
            "max_vel": 0.4,
            "waypoint_threshold": 0.05,
            "final_threshold": 0.01,

            # Path rejoin / lookahead
            "rejoin_enable": True,
            "rejoin_lookahead_points": 5,
            "rejoin_lookahead_distance_rad": 0.25,
            "rejoin_search_ahead_points": 0,

            # Distance / avoidance inputs
            "influence_distance": 0.30,
            "safety_margin": 0.08,

            # Blending / constraints
            "avoidance_weight_max": 1.0,
            "slowdown_factor_max": 0.5,
            "d_dot_min_close": 0.0,
            "d_dot_push_gain": 0.5,
            "d_dot_push_max": 0.10,
            "d_dot_min_far": -0.05,
            "cbf_enable": True,
            "cbf_kappa": 2.0,
            "cbf_projection_iters": 4,
            "cbf_eps": 1e-4,

            # Emergency stop/escape
            "emergency_enable": True,
            "emergency_enter_m": 0.05,
            "emergency_exit_m": 0.10,
            "emergency_d_dot": 0.06,
            "emergency_max_vel_fraction": 1.0,
            "emergency_hazard_prefixes": ["external:"],

            # Recovery
            "recovery_enable": True,
            "recovery_time_s": 1.0,
            "recovery_tangent_speed": 0.08,

            # Blending shaping / anti-stall
            "avoidance_tangent_weight": 0.4,
            "normal_correction_max": 0.25,
            "use_avoidance_velocity": True,
            "avoidance_normal_only": True,
            "null_boost_max": 3.0,
            "avoidance_ratio_max": 1.2,
            "avoidance_input_filter_beta": 0.55,
            "avoidance_repulsion_cap_fraction": 0.65,
            "velocity_filter_beta": 0.7,
            "slowdown_gamma_min": 0.6,
            "ns_floor_fraction": 0.25,
            "diag_enable": True,
            "diag_period_s": 1.0,
            "diag_cmd_norm_eps": 0.005,
            "tangent_escape_enable": True,
            "tangent_escape_speed": 0.03,
            "tangent_escape_err_min": 0.03,

            # Pause / reactive
            "pause_enable": True,
            "reactive_enable": True,
            "reactive_deadband": 1e-3,
            "hold_position_without_trajectory": True,
        }

        for k, v in defaults.items():
            self.declare_parameter(k, v)

    def _load_parameters(self):
        """Read parameters into attributes (casts kept explicit for clarity)."""
        p = lambda name: self.get_parameter(name).value

        self.control_period_s = float(p("control_period_s"))
        self.kp = float(p("kp"))
        self.max_vel = float(p("max_vel"))
        self.waypoint_threshold = float(p("waypoint_threshold"))
        self.final_threshold = float(p("final_threshold"))

        self.rejoin_enable = bool(p("rejoin_enable"))
        self.rejoin_lookahead_points = int(p("rejoin_lookahead_points"))
        self.rejoin_lookahead_distance_rad = float(p("rejoin_lookahead_distance_rad"))
        self.rejoin_search_ahead_points = int(p("rejoin_search_ahead_points"))

        self.d_infl = float(p("influence_distance"))
        self.d_safe = float(p("safety_margin"))

        self.avoidance_weight_max = float(p("avoidance_weight_max"))
        self.slowdown_factor_max = float(p("slowdown_factor_max"))
        self.d_dot_min_close = float(p("d_dot_min_close"))
        self.d_dot_push_gain = float(p("d_dot_push_gain"))
        self.d_dot_push_max = float(p("d_dot_push_max"))
        self.d_dot_min_far = float(p("d_dot_min_far"))

        self.cbf_enable = bool(p("cbf_enable"))
        self.cbf_kappa = float(p("cbf_kappa"))
        self.cbf_projection_iters = int(p("cbf_projection_iters"))
        self.cbf_eps = float(p("cbf_eps"))

        self.emergency_enable = bool(p("emergency_enable"))
        self.emergency_enter_m = float(p("emergency_enter_m"))
        self.emergency_exit_m = float(p("emergency_exit_m"))
        self.emergency_d_dot = float(p("emergency_d_dot"))
        self.emergency_max_vel_fraction = float(p("emergency_max_vel_fraction"))
        self.emergency_hazard_prefixes = [str(x) for x in list(p("emergency_hazard_prefixes"))]

        self.recovery_enable = bool(p("recovery_enable"))
        self.recovery_time_s = float(p("recovery_time_s"))
        self.recovery_tangent_speed = float(p("recovery_tangent_speed"))

        self.avoidance_tangent_weight = float(p("avoidance_tangent_weight"))
        self.normal_correction_max = float(p("normal_correction_max"))
        self.use_avoidance_velocity = bool(p("use_avoidance_velocity"))
        self.avoidance_normal_only = bool(p("avoidance_normal_only"))
        self.null_boost_max = float(p("null_boost_max"))
        self.avoidance_ratio_max = float(p("avoidance_ratio_max"))

        self.avoidance_input_filter_beta = float(p("avoidance_input_filter_beta"))
        self.avoidance_repulsion_cap_fraction = float(p("avoidance_repulsion_cap_fraction"))

        self.velocity_filter_beta = float(p("velocity_filter_beta"))
        self.slowdown_gamma_min = float(p("slowdown_gamma_min"))
        self.ns_floor_fraction = float(p("ns_floor_fraction"))

        self.diag_enable = bool(p("diag_enable"))
        self.diag_period_s = float(p("diag_period_s"))
        self.diag_cmd_norm_eps = float(p("diag_cmd_norm_eps"))
        self._last_diag_wall = 0.0

        self.tangent_escape_enable = bool(p("tangent_escape_enable"))
        self.tangent_escape_speed = float(p("tangent_escape_speed"))
        self.tangent_escape_err_min = float(p("tangent_escape_err_min"))

        self.pause_enable = bool(p("pause_enable"))
        self.paused = False

        self.reactive_enable = bool(p("reactive_enable"))
        self.reactive_deadband = float(p("reactive_deadband"))
        self.hold_position_without_trajectory = bool(p("hold_position_without_trajectory"))

    # ======================================================================
    # CALLBACKS
    # ======================================================================

    def joint_state_cb(self, msg: JointState):
        """Legge la posizione corrente dei giunti."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q[i] = msg.position[idx]

    def trajectory_cb(self, msg: JointTrajectory):
        """Riceve una nuova traiettoria da MoveIt."""
        if not msg.points:
            return

        # Mappa nomi giunti -> indici
        # In alcune pipeline (o per bug/bridge), JointTrajectory può arrivare con joint_names vuoti.
        # In quel caso, se positions è di dimensione 7, assumiamo l'ordine canonico.
        index_map = {}
        if msg.joint_names and len(msg.joint_names) > 0:
            for i, name in enumerate(self.joint_names):
                if name in msg.joint_names:
                    index_map[i] = msg.joint_names.index(name)

        use_direct_positions = False
        if len(index_map) != self.n_dof:
            # Fallback: assume canonical order if positions look correct
            ok_shape = all((hasattr(p, "positions") and len(p.positions) == self.n_dof) for p in msg.points)
            if ok_shape:
                use_direct_positions = True
                self.get_logger().warn(
                    "JointTrajectory joint_names missing/mismatched; assuming canonical FR3 joint order."
                )
            else:
                self.get_logger().error(
                    f"Joint names mismatch in trajectory_cb! joint_names={list(msg.joint_names)}"
                )
                return

        # Estrai tutti i punti della traiettoria (prima in una lista locale)
        new_points = []
        for point in msg.points:
            if use_direct_positions:
                q_target = np.array(point.positions[: self.n_dof], dtype=float)
            else:
                q_target = np.array([point.positions[index_map[i]] for i in range(self.n_dof)], dtype=float)
            new_points.append(q_target)

        # Guard: ignore degenerate trajectories (all points ~ identical).
        # These can appear in some edge cases and would otherwise overwrite a useful trajectory.
        try:
            if len(new_points) >= 2:
                span = float(np.linalg.norm(new_points[-1] - new_points[0]))
                if span < 1e-3:
                    self.get_logger().warn(
                        f"Ignoring degenerate trajectory (span≈{span:.2e} rad, points={len(new_points)})."
                    )
                    return
        except Exception:
            pass

        self.trajectory_points = new_points

        # Precompute cumulative arc-length along the joint-space polyline
        self._traj_s = compute_polyline_arc_lengths(self.trajectory_points)

        self.current_index = 0
        self._progress_index = 0
        self._progress_s = 0.0
        self.active = True

        # Reset filtro smoothing
        self.qdot_prev = np.zeros(self.n_dof)

        self.get_logger().info(f"📈 Nuova traiettoria: {len(self.trajectory_points)} punti")

    def avoidance_cb(self, msg: Float64MultiArray):
        """Riceve velocità di avoidance (7D)."""
        if len(msg.data) == self.n_dof:
            self.qdot_avoid = np.array(msg.data)

    def avoidance_jac_cb(self, msg: Float64MultiArray):
        """Riceve la riga di Jacobiano del punto più critico (1x7)."""
        if len(msg.data) == self.n_dof:
            self.J_avoid[0, :] = np.array(msg.data)
        else:
            self.J_avoid[0, :] = np.zeros(self.n_dof)

    def min_dist_cb(self, msg: Float64MultiArray):
        """Riceve la distanza minima dall'ostacolo."""
        if len(msg.data) > 0:
            self.min_dist = float(msg.data[0])

    def min_dist_raw_cb(self, msg: Float64MultiArray):
        """Riceve la distanza minima RAW (non filtrata) dall'ostacolo."""
        if len(msg.data) > 0:
            self.min_dist_raw = float(msg.data[0])

    def closest_constraint_cb(self, msg: Float64MultiArray):
        """Riceve (d_closest, j_row_closest) coerenti dal controller di avoidance."""
        try:
            if len(msg.data) >= (1 + self.n_dof):
                self.closest_d = float(msg.data[0])
                self.closest_j_row = np.array(msg.data[1 : 1 + self.n_dof], dtype=float).reshape(self.n_dof)
        except Exception:
            pass

    def closest_hazard_cb(self, msg: String):
        if msg.data:
            self.closest_hazard = str(msg.data)

    def hazard_cb(self, msg: String):
        if msg.data:
            self.hazard = str(msg.data)

    def pause_cb(self, msg: Bool):
        if not self.pause_enable:
            return
        self.paused = bool(msg.data)

    # ======================================================================
    # CONTROL LOOP
    # ======================================================================

    def control_loop(self):
        """Loop di controllo principale."""

        # ===== PAUSE MODE =====
        # This has priority over any other mode (reactive or trajectory).
        if self.pause_enable and self.paused:
            # Reset smoothing so we do not "jump" when unpausing.
            self.qdot_prev = np.zeros(self.n_dof)
            self.publish_velocity(np.zeros(self.n_dof))
            return

        # ===== MODALITÀ REACTIVE (B) =====
        # Se non c'è traiettoria, usa direttamente qdot_avoid come comando di velocità.
        if (not self.active) or (len(self.trajectory_points) == 0):
            # Default: mantieni fermo finché non arriva una traiettoria
            if self.hold_position_without_trajectory:
                self.publish_velocity(np.zeros(self.n_dof))
                return

            if self.reactive_enable:
                qdot = self.qdot_avoid.copy()

                # deadband per evitare drift dovuto a rumore numerico
                if np.linalg.norm(qdot) < self.reactive_deadband:
                    qdot = np.zeros(self.n_dof)

                # saturazione
                qdot = np.clip(qdot, -self.max_vel, self.max_vel)

                self.publish_velocity(qdot)
            else:
                self.publish_velocity(np.zeros(self.n_dof))
            return

        # ------------------------------------------------------------------
        # EMERGENCY OVERRIDE (hard safety)
        # ------------------------------------------------------------------
        now = time.time()

        # --- Pick the most coherent safety signal available ---
        # Prefer the coherent pair published by the avoidance controller.
        use_closest = bool(np.linalg.norm(self.closest_j_row) > 1e-6) and (float(self.closest_d) < 1e6)
        if use_closest:
            d = float(self.closest_d)
            j_row = np.array(self.closest_j_row, dtype=float).reshape(-1)
            hazard_for_safety = str(self.closest_hazard or "none")
        else:
            # Fallback to legacy topics
            d = float(self.min_dist)
            j_row = self.J_avoid[0, :]
            hazard_for_safety = str(self.hazard or "none")

        # Conservative gating distance: use RAW global min if available.
        try:
            d_gate = float(min(float(self.min_dist_raw), float(d)))
        except Exception:
            d_gate = float(d)

        j_norm = float(np.linalg.norm(j_row))

        handled, qdot_em, self._er_state, reset_smoothing = emergency_override(
            now_wall=float(now),
            d=float(d_gate),
            j_row=np.array(j_row, dtype=float).reshape(-1),
            hazard=str(hazard_for_safety),
            j_norm=float(j_norm),
            max_vel=float(self.max_vel),
            state=self._er_state,
            params=EmergencyParams(
                emergency_enable=bool(self.emergency_enable),
                emergency_enter_m=float(self.emergency_enter_m),
                emergency_exit_m=float(self.emergency_exit_m),
                emergency_d_dot=float(self.emergency_d_dot),
                emergency_max_vel_fraction=float(self.emergency_max_vel_fraction),
                emergency_hazard_prefixes=list(self.emergency_hazard_prefixes),
                recovery_enable=bool(self.recovery_enable),
                recovery_time_s=float(self.recovery_time_s),
            ),
        )
        if reset_smoothing:
            self.qdot_prev = np.zeros(self.n_dof)
        if handled and (qdot_em is not None):
            self.publish_velocity(qdot_em)
            return

        # ------------------------------------------------------------------
        # PATH FOLLOWING WITH REJOIN (dynamic-obstacle friendly)
        # ------------------------------------------------------------------
        n_pts = int(len(self.trajectory_points))
        last_idx = max(0, n_pts - 1)

        # Update progress by projecting current q onto the polyline.
        # We only allow forward progress (monotonic) so we don't go backwards on noise.
        start = int(max(0, self._progress_index))
        if int(self.rejoin_search_ahead_points) > 0:
            end = int(min(last_idx, start + int(self.rejoin_search_ahead_points)))
        else:
            end = int(last_idx)

        if self.rejoin_enable and n_pts > 0:
            try:
                # Project on segments: search up to end-1
                seg_end = int(max(0, min(end - 1, last_idx - 1)))
                seg_start = int(max(0, min(start, seg_end)))
                bi, ba, bs, bq, _ = nearest_point_on_polyline(
                    q=self.q.reshape(self.n_dof),
                    pts=self.trajectory_points,
                    s_cum=self._traj_s,
                    i0=seg_start,
                    i1=seg_end,
                )
                # Monotonic progress along arc-length
                if float(bs) > float(self._progress_s):
                    self._progress_s = float(bs)
                    self._progress_index = int(max(self._progress_index, bi))
            except Exception:
                pass
        else:
            self._progress_index = int(self.current_index)

        # Select target using distance-based lookahead when available.
        lookahead_s = float(self.rejoin_lookahead_distance_rad)
        if (lookahead_s is not None) and (lookahead_s > 1e-6) and (self._traj_s is not None):
            s_target = float(self._progress_s) + float(lookahead_s)
            q_target = interpolate_at_s(pts=self.trajectory_points, s_cum=self._traj_s, s_query=s_target, n_dof=self.n_dof)
            # Maintain a conservative index (for logging/diagnostics)
            try:
                self.current_index = int(np.searchsorted(self._traj_s, float(self._progress_s), side='right') - 1)
                self.current_index = int(max(0, min(last_idx, self.current_index)))
            except Exception:
                self.current_index = int(min(last_idx, max(0, self._progress_index)))
        else:
            lookahead = max(0, int(self.rejoin_lookahead_points))
            self.current_index = int(min(last_idx, max(0, self._progress_index)))
            target_index = int(min(last_idx, self.current_index + lookahead))
            q_target = self.trajectory_points[target_index]

        error = q_target - self.q
        error_norm = float(np.linalg.norm(error))

        # Completion: close to final point (regardless of intermediate indices).
        q_final = self.trajectory_points[last_idx]
        final_err = float(np.linalg.norm(q_final - self.q))
        if final_err < float(self.final_threshold):
            self.get_logger().info(
                f"✅ Traiettoria completata! Errore finale: {final_err:.4f} rad"
            )
            self.active = False
            self.publish_velocity(np.zeros(self.n_dof))
            return

        # For compatibility with existing logic, keep a 'threshold' variable used later
        # (e.g., tangent_escape_err_min uses max(threshold, ...)).
        threshold = float(self.waypoint_threshold)

        # ===== 1) Tracking "puro" (senza avoidance) =====
        qdot_tracking = self.kp * error

        # ===== 2) Velocità di evitamento =====
        # Filter avoidance input to avoid abrupt "kicks" that can overpower tracking.
        try:
            b = float(np.clip(float(self.avoidance_input_filter_beta), 0.0, 1.0))
        except Exception:
            b = 1.0
        if b >= 0.999:
            self.qdot_avoid_filt = self.qdot_avoid.copy()
        else:
            self.qdot_avoid_filt = b * self.qdot_avoid + (1.0 - b) * self.qdot_avoid_filt

        qdot_avoid = self.qdot_avoid_filt.copy()
        avoid_norm = np.linalg.norm(qdot_avoid)

        # ===== 3) Info per safety =====
        # (d, j_row, j_norm already computed above for emergency logic)

        # For diagnostics
        dbg = {
            "active": True,
            "idx": int(self.current_index),
            "n_points": int(len(self.trajectory_points)),
            "err_norm": float(error_norm),
            "d": float(d_gate),
            "j_norm": float(j_norm),
            "w_d": 0.0,
            "gamma": 1.0,
            "d_dot": 0.0,
            "d_dot_min": 0.0,
            "track_norm": float(np.linalg.norm(qdot_tracking)),
            "avoid_norm": float(avoid_norm),
        }

        # Se nessuna informazione sensata di avoidance → tracking puro
        if (d_gate >= self.d_infl) or (j_norm < 1e-6):
            qdot_des = qdot_tracking
        else:
            qdot_des, dbg2 = compute_influence_zone_command(
                now_wall=float(now),
                d=float(d_gate),
                j_row=np.array(j_row, dtype=float).reshape(-1),
                j_norm=float(j_norm),
                qdot_tracking=np.array(qdot_tracking, dtype=float).reshape(self.n_dof),
                qdot_avoid=np.array(qdot_avoid, dtype=float).reshape(self.n_dof),
                avoid_norm=float(avoid_norm),
                error_norm=float(error_norm),
                threshold=float(threshold),
                recovery_until_wall=float(self._er_state.recovery_until_wall),
                params=InfluenceParams(
                    n_dof=int(self.n_dof),
                    d_infl=float(self.d_infl),
                    d_safe=float(self.d_safe),
                    max_vel=float(self.max_vel),
                    avoidance_weight_max=float(self.avoidance_weight_max),
                    slowdown_factor_max=float(self.slowdown_factor_max),
                    slowdown_gamma_min=float(self.slowdown_gamma_min),
                    d_dot_min_far=float(self.d_dot_min_far),
                    d_dot_min_close=float(self.d_dot_min_close),
                    cbf_enable=bool(self.cbf_enable),
                    cbf_kappa=float(self.cbf_kappa),
                    cbf_projection_iters=int(self.cbf_projection_iters),
                    cbf_eps=float(self.cbf_eps),
                    d_dot_push_gain=float(self.d_dot_push_gain),
                    d_dot_push_max=float(self.d_dot_push_max),
                    use_avoidance_velocity=bool(self.use_avoidance_velocity),
                    avoidance_normal_only=bool(self.avoidance_normal_only),
                    avoidance_tangent_weight=float(self.avoidance_tangent_weight),
                    null_boost_max=float(self.null_boost_max),
                    avoidance_ratio_max=float(self.avoidance_ratio_max),
                    avoidance_repulsion_cap_fraction=float(self.avoidance_repulsion_cap_fraction),
                    ns_floor_fraction=float(self.ns_floor_fraction),
                    normal_correction_max=float(self.normal_correction_max),
                    recovery_enable=bool(self.recovery_enable),
                    recovery_tangent_speed=float(self.recovery_tangent_speed),
                    tangent_escape_enable=bool(self.tangent_escape_enable),
                    tangent_escape_speed=float(self.tangent_escape_speed),
                    tangent_escape_err_min=float(self.tangent_escape_err_min),
                    diag_cmd_norm_eps=float(self.diag_cmd_norm_eps),
                ),
            )
            dbg.update(dbg2)


        # ------------------------------------------------------------------
        # FILTRO SULLA VELOCITÀ + SATURAZIONE (+ robust CBF enforcement under saturation)
        # ------------------------------------------------------------------
        qdot, qdot_prev_new, dbg = apply_output_filter_and_constraints(
            qdot_des=np.array(qdot_des, dtype=float).reshape(self.n_dof),
            qdot_prev=np.array(self.qdot_prev, dtype=float).reshape(self.n_dof),
            velocity_filter_beta=float(self.velocity_filter_beta),
            max_vel=float(self.max_vel),
            d=float(d_gate),
            d_infl=float(self.d_infl),
            j_row=np.array(j_row, dtype=float).reshape(-1),
            j_norm=float(j_norm),
            cbf_projection_iters=int(self.cbf_projection_iters),
            cbf_eps=float(self.cbf_eps),
            normal_correction_max=float(self.normal_correction_max),
            dbg=dbg,
        )
        self.qdot_prev = np.array(qdot_prev_new, dtype=float).reshape(self.n_dof)

        # Diagnostics: if avoidance is active (d in influence + valid jacobian) but cmd is ~0, log internal state.
        if self.diag_enable:
            try:
                cmd_n = float(np.linalg.norm(qdot))
                avoid_active = bool((dbg["d"] < float(self.d_infl)) and (dbg["j_norm"] > 1e-6))
                if avoid_active and (cmd_n <= float(self.diag_cmd_norm_eps)):
                    now = time.time()
                    if (now - float(self._last_diag_wall)) >= max(0.2, float(self.diag_period_s)):
                        self._last_diag_wall = now
                        self.get_logger().warn(
                            "[BLENDER-STALL] "
                            f"idx={dbg['idx']}/{max(0, dbg['n_points']-1)} err_norm={dbg['err_norm']:.3f} "
                            f"d={dbg['d']:.3f} j_norm={dbg['j_norm']:.3e} w_d={dbg['w_d']:.3f} gamma={dbg['gamma']:.3f} "
                            f"d_dot={dbg['d_dot']:.4f} d_dot_min={dbg['d_dot_min']:.4f} "
                            f"|track|={dbg['track_norm']:.3f} |avoid_in|={dbg['avoid_norm']:.3f} |cmd|={cmd_n:.3f} "
                            f"cbf_ok={bool(dbg.get('cbf_ok', True))}"
                        )
            except Exception:
                pass

        # Pubblica comando finale
        self.publish_velocity(qdot)

    # ======================================================================
    # UTILITIES
    # ======================================================================

    def publish_velocity(self, qdot: np.ndarray):
        """Pubblica comando di velocità sui giunti."""
        msg = Float64MultiArray()
        msg.data = qdot.tolist()
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleVelocityBlender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ferma il robot (best-effort): under ros2 launch the context may already be shutting down.
        try:
            if rclpy.ok():
                msg = Float64MultiArray()
                msg.data = [0.0] * 7
                node.cmd_pub.publish(msg)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
