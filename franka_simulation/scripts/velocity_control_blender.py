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
        self.declare_parameter("control_period_s", 0.01)  # seconds
        self.declare_parameter("kp", 20.0)             # MODIFICA: un po' meno aggressivo (prima 30)
        self.declare_parameter("max_vel", 0.4)         # MODIFICA: limita velocità globale
        self.declare_parameter("waypoint_threshold", 0.05)
        self.declare_parameter("final_threshold", 0.01)

        # Path rejoin / lookahead (for dynamic obstacles)
        # Instead of tracking the current waypoint index rigidly, we continuously project the current
        # joint configuration onto the received trajectory and pick a target ahead (lookahead).
        # This enables: deviate to avoid obstacles -> once clear, rejoin the original trajectory.
        self.declare_parameter("rejoin_enable", True)
        self.declare_parameter("rejoin_lookahead_points", 5)
        # Prefer distance-based lookahead (joint-space arc length). If > 0, it overrides lookahead_points.
        self.declare_parameter("rejoin_lookahead_distance_rad", 0.25)
        # Search range for nearest-point (0 = search full trajectory from current_index).
        self.declare_parameter("rejoin_search_ahead_points", 0)

        self.declare_parameter("influence_distance", 0.30)
        self.declare_parameter("safety_margin", 0.08)

        # MODIFICA: nuovi parametri per blending e proiezione
        self.declare_parameter("avoidance_weight_max", 1.0)      # peso max su qdot_avoid
        self.declare_parameter("slowdown_factor_max", 0.5)       # riduzione max velocità vicini (0.5 -> velocità min 50%)
        # ḋ minima quando d <= d_safe (m/s equivalente). 0.0 = evita penetrazione ma non forza "scappare".
        self.declare_parameter("d_dot_min_close", 0.0)
        # Extra push when inside the safety margin (helps avoid deadlocks at the boundary).
        # Interpreted as: d_dot_min := max(d_dot_min, d_dot_push_gain * (d_safe - d)) for d < d_safe.
        self.declare_parameter("d_dot_push_gain", 0.5)
        self.declare_parameter("d_dot_push_max", 0.10)
        # ḋ minima al bordo della zona di influenza (di solito negativa: permette avvicinarsi se lontano)
        self.declare_parameter("d_dot_min_far", -0.05)

        # CBF-style constraint shaping and robust enforcement under velocity saturation.
        # Classic CBF on h(d)=d-d_safe:  h_dot + k*h >= 0  ->  d_dot >= -k*(d-d_safe)
        # This yields a smooth lower bound: negative far away, 0 at d_safe, positive inside.
        self.declare_parameter("cbf_enable", True)
        self.declare_parameter("cbf_kappa", 2.0)
        # Enforce the half-space constraint together with joint-velocity box constraints by
        # alternating projections (fast, dependency-free). Small number of iterations is enough.
        self.declare_parameter("cbf_projection_iters", 4)
        self.declare_parameter("cbf_eps", 1e-4)

        # ------------------------------------------------------------------
        # EMERGENCY SAFETY (hard override)
        # If the minimum distance drops below enter threshold, immediately override tracking and
        # command a joint velocity that increases the distance (move away along the distance gradient).
        # Once the minimum distance is above exit threshold (hysteresis), switch to a short recovery
        # phase where we add tangential motion to help slide around the obstacle and rejoin the path.
        # ------------------------------------------------------------------
        self.declare_parameter("emergency_enable", True)
        self.declare_parameter("emergency_enter_m", 0.05)  # 5 cm
        self.declare_parameter("emergency_exit_m", 0.10)   # 10 cm
        # Desired minimum distance rate (m/s) during emergency escape.
        self.declare_parameter("emergency_d_dot", 0.06)
        # Hard cap for escape command as fraction of max_vel.
        self.declare_parameter("emergency_max_vel_fraction", 1.0)
        # Apply emergency only for these hazard prefixes (from /avoidance/hazard), e.g. external/self/ground.
        self.declare_parameter("emergency_hazard_prefixes", ["external:"])

        # Recovery (after emergency clears): inject tangential motion for a short time.
        self.declare_parameter("recovery_enable", True)
        self.declare_parameter("recovery_time_s", 1.0)
        self.declare_parameter("recovery_tangent_speed", 0.08)  # rad/s scaled by proximity

        # Bias tangenziale per "aggirare" senza bloccare il goal
        self.declare_parameter("avoidance_tangent_weight", 0.4)
        # Limite alla correzione lungo la normale (evita scatti)
        self.declare_parameter("normal_correction_max", 0.25)
        # Strategie per evitare lo "stallo" in presenza di ostacoli
        self.declare_parameter("use_avoidance_velocity", True)        # usa /avoidance/velocity nel blending
        self.declare_parameter("avoidance_normal_only", True)         # applica repulsione solo nella normale (rispetto a j_row)
        self.declare_parameter("null_boost_max", 3.0)                 # boost progress nel nullspace vicino all'ostacolo
        self.declare_parameter("avoidance_ratio_max", 1.2)            # limite: ||w_rep*qdot_avoid|| <= ratio*(||qdot_ns||+eps)

        # Gentle avoidance shaping: filter and cap avoidance so it cannot dominate tracking.
        # - avoidance_input_filter_beta: 1.0 = no filtering, lower = smoother
        # - avoidance_repulsion_cap_fraction: cap repulsion norm as fraction of max_vel
        self.declare_parameter("avoidance_input_filter_beta", 0.55)
        self.declare_parameter("avoidance_repulsion_cap_fraction", 0.65)

        # Smoothing / safety knobs (previously hard-coded)
        self.declare_parameter("velocity_filter_beta", 0.7)   # 0.7 = più reattivo, 0.5 = più liscio
        self.declare_parameter("slowdown_gamma_min", 0.6)     # lower bound for global slowdown gamma
        self.declare_parameter("ns_floor_fraction", 0.25)     # floor for ns_norm as fraction of max_vel

        # Diagnostics (helps explain stalls)
        self.declare_parameter("diag_enable", True)
        self.declare_parameter("diag_period_s", 1.0)
        self.declare_parameter("diag_cmd_norm_eps", 0.005)

        # Anti-stall: inject a small tangential component (in the nullspace of j_row)
        # if the blended command collapses close to zero while avoidance is active.
        # This helps "slide around" obstacles instead of freezing.
        self.declare_parameter("tangent_escape_enable", True)
        self.declare_parameter("tangent_escape_speed", 0.03)     # rad/s (scaled by w_d)
        self.declare_parameter("tangent_escape_err_min", 0.03)   # rad: only if not already basically at target

        # Pause (for interactive step-by-step demos/tests)
        # When paused, the blender publishes zero velocities and does not execute the trajectory.
        self.declare_parameter("pause_enable", True)
        # Modalità B (reactive): l'avoidance può muovere anche senza traiettoria
        self.declare_parameter("reactive_enable", True)
        self.declare_parameter("reactive_deadband", 1e-3)   # sotto questa norma → fermo
        # Sicurezza/UX: per default non muovere il robot finché non arriva una traiettoria
        # (evita drift/spinte iniziali dovute a avoidance o rumore)
        self.declare_parameter("hold_position_without_trajectory", True)
        self.control_period_s = float(self.get_parameter("control_period_s").value)

        self.kp = self.get_parameter("kp").value
        self.max_vel = self.get_parameter("max_vel").value
        self.waypoint_threshold = self.get_parameter("waypoint_threshold").value
        self.final_threshold = self.get_parameter("final_threshold").value

        self.rejoin_enable = bool(self.get_parameter("rejoin_enable").value)
        self.rejoin_lookahead_points = int(self.get_parameter("rejoin_lookahead_points").value)
        self.rejoin_lookahead_distance_rad = float(self.get_parameter("rejoin_lookahead_distance_rad").value)
        self.rejoin_search_ahead_points = int(self.get_parameter("rejoin_search_ahead_points").value)

        self.d_infl = self.get_parameter("influence_distance").value
        self.d_safe = self.get_parameter("safety_margin").value

        self.avoidance_weight_max = self.get_parameter("avoidance_weight_max").value
        self.slowdown_factor_max = self.get_parameter("slowdown_factor_max").value
        self.d_dot_min_close = self.get_parameter("d_dot_min_close").value
        self.d_dot_push_gain = float(self.get_parameter("d_dot_push_gain").value)
        self.d_dot_push_max = float(self.get_parameter("d_dot_push_max").value)
        self.d_dot_min_far = float(self.get_parameter("d_dot_min_far").value)

        self.cbf_enable = bool(self.get_parameter("cbf_enable").value)
        self.cbf_kappa = float(self.get_parameter("cbf_kappa").value)
        self.cbf_projection_iters = int(self.get_parameter("cbf_projection_iters").value)
        self.cbf_eps = float(self.get_parameter("cbf_eps").value)

        self.emergency_enable = bool(self.get_parameter("emergency_enable").value)
        self.emergency_enter_m = float(self.get_parameter("emergency_enter_m").value)
        self.emergency_exit_m = float(self.get_parameter("emergency_exit_m").value)
        self.emergency_d_dot = float(self.get_parameter("emergency_d_dot").value)
        self.emergency_max_vel_fraction = float(self.get_parameter("emergency_max_vel_fraction").value)
        self.emergency_hazard_prefixes = [
            str(x) for x in list(self.get_parameter("emergency_hazard_prefixes").value)
        ]

        self.recovery_enable = bool(self.get_parameter("recovery_enable").value)
        self.recovery_time_s = float(self.get_parameter("recovery_time_s").value)
        self.recovery_tangent_speed = float(self.get_parameter("recovery_tangent_speed").value)

        self.avoidance_tangent_weight = float(self.get_parameter("avoidance_tangent_weight").value)
        self.normal_correction_max = float(self.get_parameter("normal_correction_max").value)

        self.use_avoidance_velocity = bool(self.get_parameter("use_avoidance_velocity").value)
        self.avoidance_normal_only = bool(self.get_parameter("avoidance_normal_only").value)
        self.null_boost_max = float(self.get_parameter("null_boost_max").value)
        self.avoidance_ratio_max = float(self.get_parameter("avoidance_ratio_max").value)

        self.avoidance_input_filter_beta = float(self.get_parameter("avoidance_input_filter_beta").value)
        self.avoidance_repulsion_cap_fraction = float(self.get_parameter("avoidance_repulsion_cap_fraction").value)

        self.velocity_filter_beta = float(self.get_parameter("velocity_filter_beta").value)
        self.slowdown_gamma_min = float(self.get_parameter("slowdown_gamma_min").value)
        self.ns_floor_fraction = float(self.get_parameter("ns_floor_fraction").value)

        self.diag_enable = bool(self.get_parameter("diag_enable").value)
        self.diag_period_s = float(self.get_parameter("diag_period_s").value)
        self.diag_cmd_norm_eps = float(self.get_parameter("diag_cmd_norm_eps").value)
        self._last_diag_wall = 0.0

        self.tangent_escape_enable = bool(self.get_parameter("tangent_escape_enable").value)
        self.tangent_escape_speed = float(self.get_parameter("tangent_escape_speed").value)
        self.tangent_escape_err_min = float(self.get_parameter("tangent_escape_err_min").value)

        self.pause_enable = bool(self.get_parameter("pause_enable").value)
        self.paused = False

        self.reactive_enable = bool(self.get_parameter("reactive_enable").value)
        self.reactive_deadband = float(self.get_parameter("reactive_deadband").value)
        self.hold_position_without_trajectory = bool(
            self.get_parameter("hold_position_without_trajectory").value
        )

        # ===== STATO =====
        self.q = np.zeros(self.n_dof)              # Posizione corrente
        self.qdot_avoid = np.zeros(self.n_dof)     # Velocità di avoidance
        self.qdot_avoid_filt = np.zeros(self.n_dof)  # filtered avoidance
        self.qdot_prev = np.zeros(self.n_dof)      # Velocità precedente (per smoothing)
        self.J_avoid = np.zeros((1, self.n_dof))   # Jacobiano del punto più critico (1x7)
        self.min_dist = 999.0                      # Distanza minima iniziale "lontana"
        self.hazard = "none"

        # Emergency / recovery state
        self._emergency_active = False
        self._recovery_until_wall = 0.0

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

    def _enforce_halfspace_with_box(
        self,
        qdot_des: np.ndarray,
        j_row: np.ndarray,
        d_dot_min: float,
        max_abs_vel: float,
        iters: int,
        eps: float,
    ):
        """Enforce j_row @ qdot >= d_dot_min with |qdot_i|<=max_abs_vel.

        We implement a small alternating-projection loop:
          1) clamp to box
          2) if half-space violated, add minimal correction along j_row
          3) clamp again

        This is a practical, dependency-free approximation to the QP:
          min ||qdot - qdot_des||^2 s.t. j_row qdot >= d_dot_min, |qdot|<=max
        """

        qdot = np.array(qdot_des, dtype=float).reshape(self.n_dof)
        maxv = float(max_abs_vel)
        if maxv <= 0.0:
            return np.zeros(self.n_dof), False

        j = np.array(j_row, dtype=float).reshape(self.n_dof)
        jn2 = float(j @ j) + 1e-8

        # Start from box projection
        qdot = np.clip(qdot, -maxv, +maxv)

        ok = False
        for _ in range(max(1, int(iters))):
            d_dot = float(j @ qdot)
            if d_dot >= float(d_dot_min) - float(eps):
                ok = True
                break

            # Minimal correction along j
            lam = (float(d_dot_min) - d_dot) / jn2
            corr = lam * j

            # Smoothness cap (reuse existing normal_correction_max as an L2 bound)
            try:
                c_norm = float(np.linalg.norm(corr))
                if c_norm > float(self.normal_correction_max):
                    corr *= float(self.normal_correction_max) / (c_norm + 1e-9)
            except Exception:
                pass

            qdot = qdot + corr
            qdot = np.clip(qdot, -maxv, +maxv)

        # Final check
        try:
            ok = ok or (float(j @ qdot) >= float(d_dot_min) - float(eps))
        except Exception:
            ok = False

        return qdot, ok

    @staticmethod
    def _nearest_point_on_polyline(q: np.ndarray, pts: list, s_cum: np.ndarray, i0: int, i1: int):
        """Nearest point projection of q onto polyline segments [i0..i1].

        Returns (best_i, best_alpha, best_s, best_qproj, best_d2)
        where the nearest point lies on segment i->i+1 at interpolation alpha in [0,1].
        """
        n = int(len(pts))
        if n <= 0:
            return 0, 0.0, 0.0, q.copy(), float('inf')
        if n == 1:
            dq = pts[0] - q
            return 0, 0.0, float(s_cum[0]) if s_cum is not None else 0.0, pts[0].copy(), float(dq @ dq)

        i0 = int(max(0, min(n - 2, i0)))
        i1 = int(max(i0, min(n - 2, i1)))

        best_i = i0
        best_a = 0.0
        best_d2 = float('inf')
        best_qp = pts[i0].copy()
        best_s = float(s_cum[i0]) if s_cum is not None else 0.0

        qv = q.reshape(-1)

        for i in range(i0, i1 + 1):
            p0 = pts[i]
            p1 = pts[i + 1]
            v = p1 - p0
            vv = float(v @ v)
            if vv < 1e-12:
                a = 0.0
                qp = p0
            else:
                a = float(((qv - p0) @ v) / vv)
                a = float(np.clip(a, 0.0, 1.0))
                qp = p0 + a * v

            d = qp - qv
            d2 = float(d @ d)
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
                best_a = a
                best_qp = qp
                if s_cum is not None:
                    seg_len = float(np.linalg.norm(v))
                    best_s = float(s_cum[i]) + a * seg_len
                else:
                    best_s = float(i) + a

        return best_i, best_a, best_s, best_qp, best_d2

    @staticmethod
    def _interpolate_at_s(pts: list, s_cum: np.ndarray, s_query: float) -> np.ndarray:
        """Interpolate polyline at arc-length s_query (joint-space)."""
        n = int(len(pts))
        if n <= 0:
            return np.zeros(7, dtype=float)
        if n == 1 or s_cum is None or len(s_cum) != n:
            return pts[-1].copy()

        s0 = float(s_cum[0])
        sN = float(s_cum[-1])
        s = float(np.clip(s_query, s0, sN))
        j = int(np.searchsorted(s_cum, s, side='right') - 1)
        j = int(max(0, min(n - 2, j)))

        sj0 = float(s_cum[j])
        sj1 = float(s_cum[j + 1])
        if (sj1 - sj0) < 1e-12:
            return pts[j + 1].copy()
        a = (s - sj0) / (sj1 - sj0)
        return (1.0 - a) * pts[j] + a * pts[j + 1]

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
        try:
            s = [0.0]
            for i in range(1, len(self.trajectory_points)):
                ds = float(np.linalg.norm(self.trajectory_points[i] - self.trajectory_points[i - 1]))
                s.append(s[-1] + ds)
            self._traj_s = np.array(s, dtype=float)
        except Exception:
            self._traj_s = None

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
        d = float(self.min_dist)
        j_row = self.J_avoid[0, :]
        j_norm = float(np.linalg.norm(j_row))

        hazard = str(self.hazard or "none")
        hazard_ok = True
        if self.emergency_hazard_prefixes and hazard != "none":
            hazard_ok = any(hazard.startswith(p) for p in self.emergency_hazard_prefixes)

        if self.emergency_enable and hazard_ok and (j_norm > 1e-6):
            # Hysteresis on emergency state
            if (not self._emergency_active) and (d <= float(self.emergency_enter_m)):
                self._emergency_active = True
                # Reset smoothing so we react immediately
                self.qdot_prev = np.zeros(self.n_dof)

            if self._emergency_active:
                # Exit condition
                if d >= float(self.emergency_exit_m):
                    self._emergency_active = False
                    if self.recovery_enable and (self.recovery_time_s > 0.0):
                        self._recovery_until_wall = now + float(self.recovery_time_s)
                else:
                    # Escape velocity: minimal-norm solution to enforce d_dot >= emergency_d_dot
                    d_dot_des = max(0.0, float(self.emergency_d_dot))
                    alpha = d_dot_des / (j_norm * j_norm + 1e-8)
                    qdot_escape = alpha * j_row

                    # Stronger escape if deeper than enter threshold
                    try:
                        if float(self.emergency_enter_m) > 1e-6:
                            depth = max(0.0, float(self.emergency_enter_m) - d)
                            depth_gain = 1.0 + 4.0 * (depth / float(self.emergency_enter_m))
                            qdot_escape *= float(np.clip(depth_gain, 1.0, 5.0))
                    except Exception:
                        pass

                    maxv = float(self.max_vel) * float(max(0.1, self.emergency_max_vel_fraction))
                    qdot_escape = np.clip(qdot_escape, -maxv, +maxv)

                    # Publish immediately (ignore trajectory tracking while in emergency)
                    self.publish_velocity(qdot_escape)
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
                bi, ba, bs, bq, _ = self._nearest_point_on_polyline(
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
            q_target = self._interpolate_at_s(self.trajectory_points, self._traj_s, s_target)
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
            "d": float(d),
            "j_norm": float(j_norm),
            "w_d": 0.0,
            "gamma": 1.0,
            "d_dot": 0.0,
            "d_dot_min": 0.0,
            "track_norm": float(np.linalg.norm(qdot_tracking)),
            "avoid_norm": float(avoid_norm),
        }

        # Se nessuna informazione sensata di avoidance → tracking puro
        if (d >= self.d_infl) or (j_norm < 1e-6):
            qdot_des = qdot_tracking
        else:
            # --------------------------------------------------------------
            # ZONA DI INFLUENZA (smooth, goal-driven):
            #   - mantieni tracking verso goal
            #   - aggiungi un bias tangenziale (per aggirare)
            #   - applica un vincolo su ḋ in tutta la zona di influenza
            #     con correzione MINIMA lungo j_row (QP 1D)
            # --------------------------------------------------------------
            d_safe = self.d_safe
            d_infl = self.d_infl

            # Projectors for the distance normal and its nullspace
            J = j_row.reshape(1, self.n_dof)
            JT = J.T
            denom = float(J @ JT) + 1e-8
            P = (JT @ J) / denom
            N = np.eye(self.n_dof) - P

            # Smoothstep weight: 0 at d_infl, 1 at d_safe
            x = (d_infl - d) / (d_infl - d_safe)
            x = max(0.0, min(1.0, x))
            w_d = 3.0 * x * x - 2.0 * x * x * x
            dbg["w_d"] = float(w_d)

            # 1) Base: keep tracking (towards goal)
            qdot_des = qdot_tracking.copy()

            # 2) Add avoidance contribution (repulsive normal + tangential)
            if self.use_avoidance_velocity and (avoid_norm > 1e-6):
                # Split avoidance into normal/tangential w.r.t. the current distance gradient
                qdot_avoid_n = P @ qdot_avoid
                qdot_avoid_t = N @ qdot_avoid

                # Optionally keep only the normal repulsion (safer but can stall); tangential can still be added.
                qdot_avoid_use_n = qdot_avoid_n if self.avoidance_normal_only else qdot_avoid

                # Weight schedule near obstacle
                w_rep = float(self.avoidance_weight_max) * w_d
                w_tan = float(self.avoidance_tangent_weight) * w_d

                # Ratio limiter: do not let repulsion dominate the motion that can still make progress.
                # Use the tracking component in the constraint nullspace as a proxy for "go-around" capability.
                ns_norm = float(np.linalg.norm(N @ qdot_des))
                rep_vec = w_rep * qdot_avoid_use_n
                rep_norm = float(np.linalg.norm(rep_vec))

                # Absolute cap on repulsion magnitude so it can't dominate tracking.
                try:
                    cap_frac = float(np.clip(float(self.avoidance_repulsion_cap_fraction), 0.0, 2.0))
                    rep_cap = cap_frac * float(self.max_vel)
                    if (rep_cap > 0.0) and (rep_norm > rep_cap) and (rep_norm > 1e-9):
                        rep_vec *= (rep_cap / rep_norm)
                        rep_norm = float(np.linalg.norm(rep_vec))
                except Exception:
                    pass
                # IMPORTANT:
                # If the tracking component has (almost) no nullspace part, ns_norm can be ~0.
                # With a strict ratio limiter this would squash repulsion to ~0 as well, which can
                # result in cmd≈0 right when we actually need repulsion to keep moving safely.
                # We therefore apply a small floor based on the configured max velocity.
                ns_floor = max(float(self.ns_floor_fraction) * float(self.max_vel), 1e-3)
                rep_max = float(self.avoidance_ratio_max) * (max(ns_norm, ns_floor) + 1e-6)
                if rep_norm > rep_max and rep_norm > 1e-9:
                    rep_vec *= (rep_max / rep_norm)

                qdot_des = qdot_des + rep_vec + (w_tan * qdot_avoid_t)

            # 3) Optional: increase tangential progress near obstacle
            null_boost = 1.0 + self.null_boost_max * w_d
            qdot_des = (P @ qdot_des) + null_boost * (N @ qdot_des)

            # 4) Enforce a smooth lower bound on d_dot across the influence region
            #    - far (d≈d_infl): allow some approach (negative)
            #    - close (d≈d_safe): require non-decreasing distance (>= d_dot_min_close, default 0)
            d_dot_min = (1.0 - w_d) * self.d_dot_min_far + w_d * self.d_dot_min_close

            # Optional CBF shaping: d_dot >= -k*(d-d_safe). This smoothly transitions from
            # "allowed approach" (negative) to "no approach" at the safety margin and to
            # "escape" (positive) inside.
            if self.cbf_enable:
                try:
                    cbf_min = -float(self.cbf_kappa) * float(d - d_safe)
                    d_dot_min = max(float(d_dot_min), float(cbf_min))
                except Exception:
                    pass

            # Extra escape term if we are inside the safety margin: force d_dot to become positive.
            if d < d_safe:
                push = float(self.d_dot_push_gain) * float(d_safe - d)
                push = float(np.clip(push, 0.0, float(self.d_dot_push_max)))
                d_dot_min = max(float(d_dot_min), push)

            d_dot = float(j_row @ qdot_des)
            dbg["d_dot"] = float(d_dot)
            dbg["d_dot_min"] = float(d_dot_min)
            if d_dot < d_dot_min:
                # Minimal correction along j_row (QP 1D) before slowdown/saturation
                lambda_corr = (d_dot_min - d_dot) / (j_norm * j_norm + 1e-8)
                corr = lambda_corr * j_row
                # Cap the correction magnitude for smoothness
                corr_norm = float(np.linalg.norm(corr))
                if corr_norm > self.normal_correction_max:
                    corr *= self.normal_correction_max / (corr_norm + 1e-9)
                qdot_des = qdot_des + corr

            # 5) Global slowdown (gentle) close to obstacles
            gamma = 1.0 - self.slowdown_factor_max * w_d
            gamma = max(float(self.slowdown_gamma_min), gamma)
            qdot_des *= gamma
            dbg["gamma"] = float(gamma)

            # 5b) Recovery tangential injection (after emergency clears)
            if self.recovery_enable and (now < float(self._recovery_until_wall)):
                try:
                    # Build a tangential direction in the nullspace of J (doesn't worsen d_dot)
                    t_vec = N @ qdot_tracking
                    t_n = float(np.linalg.norm(t_vec))
                    if t_n < 1e-9 and (avoid_norm > 1e-9):
                        t_vec = N @ qdot_avoid
                        t_n = float(np.linalg.norm(t_vec))
                    if t_n < 1e-9:
                        # deterministic basis fallback
                        t_vec = np.zeros(self.n_dof)
                        for k in range(self.n_dof):
                            ei = np.zeros(self.n_dof)
                            ei[k] = 1.0
                            cand = N @ ei
                            cn = float(np.linalg.norm(cand))
                            if cn > 1e-6:
                                t_vec = cand
                                t_n = cn
                                break
                    if t_n > 1e-9:
                        # Scale by proximity (w_d) so it's mainly active near the obstacle
                        t_dir = t_vec / (t_n + 1e-9)
                        qdot_des = qdot_des + (float(self.recovery_tangent_speed) * float(w_d)) * t_dir
                except Exception:
                    pass

            # 6) Anti-stall tangential escape: if command collapses (due to cancellation between tracking
            #    and repulsion) while we're still far from the waypoint, add a small tangential velocity
            #    in the nullspace of J (so it does not worsen d_dot).
            if self.tangent_escape_enable:
                try:
                    if (w_d > 1e-3) and (float(error_norm) > float(max(threshold, self.tangent_escape_err_min))):
                        des_n = float(np.linalg.norm(qdot_des))
                        if des_n < float(self.diag_cmd_norm_eps):
                            # Candidate 1: tracking component in distance-nullspace
                            t_vec = N @ qdot_tracking
                            t_n = float(np.linalg.norm(t_vec))
                            if t_n < 1e-9:
                                # Candidate 2: avoidance tangential component (if any)
                                try:
                                    t_vec = N @ qdot_avoid
                                    t_n = float(np.linalg.norm(t_vec))
                                except Exception:
                                    t_vec = np.zeros(self.n_dof)
                                    t_n = 0.0

                            if t_n < 1e-9:
                                # Candidate 3: deterministic basis fallback
                                # (project e_i into nullspace until we find a usable direction)
                                t_vec = np.zeros(self.n_dof)
                                for k in range(self.n_dof):
                                    ei = np.zeros(self.n_dof)
                                    ei[k] = 1.0
                                    cand = N @ ei
                                    cn = float(np.linalg.norm(cand))
                                    if cn > 1e-6:
                                        t_vec = cand
                                        t_n = cn
                                        break

                            if t_n > 1e-9:
                                t_dir = t_vec / (t_n + 1e-9)
                                qdot_des = qdot_des + (float(self.tangent_escape_speed) * float(w_d)) * t_dir
                except Exception:
                    pass


        # ------------------------------------------------------------------
        # FILTRO SULLA VELOCITÀ + SATURAZIONE (+ robust CBF enforcement under saturation)
        # ------------------------------------------------------------------
        beta = float(self.velocity_filter_beta)
        beta = float(np.clip(beta, 0.0, 1.0))
        qdot = beta * qdot_des + (1.0 - beta) * self.qdot_prev
        self.qdot_prev = qdot.copy()

        # First box saturation
        qdot = np.clip(qdot, -self.max_vel, self.max_vel)

        # If avoidance is active, ensure the distance constraint is still satisfied after saturation.
        # This is important with dynamic obstacles: clipping can break the half-space constraint.
        if (d < float(self.d_infl)) and (j_norm > 1e-6):
            try:
                # Recompute the same d_dot_min used above (dbg already holds it).
                d_dot_min_eff = float(dbg.get("d_dot_min", 0.0))
                qdot, ok = self._enforce_halfspace_with_box(
                    qdot_des=qdot,
                    j_row=j_row,
                    d_dot_min=d_dot_min_eff,
                    max_abs_vel=float(self.max_vel),
                    iters=int(self.cbf_projection_iters),
                    eps=float(self.cbf_eps),
                )
                dbg["cbf_ok"] = bool(ok)
            except Exception:
                dbg["cbf_ok"] = False

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
