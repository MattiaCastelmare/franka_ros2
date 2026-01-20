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

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

from utils import velocity_blender_ros_helpers as vh
from utils.velocity_blender_core import (
    EmergencyRecoveryState,
    compute_polyline_arc_lengths,
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

        # Map joint_name -> index once to keep JointState parsing O(n).
        self._joint_name_to_i = vh.build_name_to_index(self.joint_names)

        # ===== PARAMETRI (da ROS/YAML) =====
        # Nota: per una vista completa/ordinata dei parametri, guarda:
        #   franka_simulation/config/velocity_blender_params.yaml
        vh.declare_velocity_blender_parameters(self)
        vh.load_velocity_blender_parameters(self, self)

        # Extra (self-contained) parameters for smoother, more "natural" obstacle bypass.
        # These are declared here (not in utils) to honor the "edit only this file" constraint.
        # Defaults are conservative and require no YAML changes.
        self.declare_parameter("tangent_tracking_escape_enable", True)
        self.declare_parameter("tangent_tracking_cmd_small", 0.02)
        self.declare_parameter("tangent_tracking_vt_min", 0.03)
        self.declare_parameter("tangent_tracking_vt_max", 0.10)
        self.declare_parameter("tangent_tracking_alpha_max", 0.80)
        self.declare_parameter("penetration_emergency_enable", True)
        self.declare_parameter("penetration_emergency_d_dot", 0.08)
        self.declare_parameter("penetration_emergency_depth_ref", 0.03)
        self.declare_parameter("penetration_emergency_max_vel_fraction", 1.0)

        self.tangent_tracking_escape_enable = bool(self.get_parameter("tangent_tracking_escape_enable").value)
        self.tangent_tracking_cmd_small = float(self.get_parameter("tangent_tracking_cmd_small").value)
        self.tangent_tracking_vt_min = float(self.get_parameter("tangent_tracking_vt_min").value)
        self.tangent_tracking_vt_max = float(self.get_parameter("tangent_tracking_vt_max").value)
        self.tangent_tracking_alpha_max = float(self.get_parameter("tangent_tracking_alpha_max").value)
        self.penetration_emergency_enable = bool(self.get_parameter("penetration_emergency_enable").value)
        self.penetration_emergency_d_dot = float(self.get_parameter("penetration_emergency_d_dot").value)
        self.penetration_emergency_depth_ref = float(self.get_parameter("penetration_emergency_depth_ref").value)
        self.penetration_emergency_max_vel_fraction = float(self.get_parameter("penetration_emergency_max_vel_fraction").value)

        # ===== STATO =====
        self.q = np.zeros(self.n_dof)              # Posizione corrente
        self.qdot_avoid = np.zeros(self.n_dof)     # Velocità di avoidance
        self.qdot_avoid_filt = np.zeros(self.n_dof)  # filtered avoidance
        self.qdot_prev = np.zeros(self.n_dof)      # Velocità precedente (per smoothing)
        self.hazard = "none"

        # Coherent closest constraint from the avoidance controller
        # Message format: [d_closest, j_row_0..j_row_6]
        self.closest_d = 999.0
        self.closest_j_row = np.zeros(self.n_dof)
        self.closest_hazard = "none"

        # Light filtering of the closest constraint Jacobian row (to avoid jitter at the influence boundary).
        # IMPORTANT: keep sign consistent (j_row and -j_row represent the same halfspace normal but can flip due
        # to numeric conventions), otherwise the constraint direction can chatter.
        self._closest_j_row_filt = np.zeros(self.n_dof)
        self._closest_j_row_filt_init = False

        # Emergency / recovery state (kept as a small struct to avoid scattering fields).
        self._er_state = EmergencyRecoveryState()

        # Traiettoria
        self.trajectory_points = []      # Lista di configurazioni target
        self.current_index = 0           # Indice del punto corrente
        self.active = False              # Traiettoria attiva?

        # Monotonic progress index (never decreases) to avoid oscillations on paths with noise.
        self._progress = vh.PolylineProgress(progress_index=0, progress_s=0.0)
        self._traj_s = None  # cumulative arc-length at each waypoint (len=N)

        # Progress-based stall detection (used to gate tangential escape):
        # only inject tangential motion when near hazards AND progress along the planned path stalls.
        self._stall_prog_s_last = None  # Optional[float]
        self._stall_prog_wall_last = None  # Optional[float]
        self._stall_prog_flag = False

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

        logger = self.get_logger()
        for line in (
            "✅ Simple Velocity Blender (ḋ-constrained) started",
            f"   Kp={self.kp}, max_vel={self.max_vel}, d_safe={self.d_safe}, d_infl={self.d_infl}",
            f"   use_avoidance_velocity={self.use_avoidance_velocity}, avoidance_normal_only={self.avoidance_normal_only}, "
            f"null_boost_max={self.null_boost_max}, avoidance_ratio_max={self.avoidance_ratio_max}",
            f"   d_dot_min_far={self.d_dot_min_far}, d_dot_min_close={self.d_dot_min_close}, "
            f"avoidance_tangent_weight={self.avoidance_tangent_weight}, normal_correction_max={self.normal_correction_max}",
            f"   d_dot_push_gain={self.d_dot_push_gain}, d_dot_push_max={self.d_dot_push_max}, "
            f"avoidance_weight_max={self.avoidance_weight_max}",
            f"   tangent_escape_enable={self.tangent_escape_enable}, tangent_escape_speed={self.tangent_escape_speed}, "
            f"tangent_escape_err_min={self.tangent_escape_err_min}",
            f"   pause_enable={self.pause_enable} (topic: /velocity_blender/pause)",
        ):
            logger.info(line)

        # Diagnostics / counters (moved out of the node)
        self._diag = vh.VelocityBlenderDiagnostics(
            last_diag_wall=0.0,
            last_robust_wall=0.0,
            infeasible_count=0,
            emergency_enter_count=0,
            stop_gate_count=0,
            prev_emergency=bool(self._er_state.emergency_active),
            prev_stop_gate=False,
        )

    # ======================================================================
    # CALLBACKS
    # ======================================================================

    def joint_state_cb(self, msg: JointState):
        """Legge la posizione corrente dei giunti."""
        vh.update_joint_positions_inplace(self.q, msg, self._joint_name_to_i)

    def trajectory_cb(self, msg: JointTrajectory):
        """Riceve una nuova traiettoria da MoveIt."""
        new_points = vh.joint_trajectory_to_points(
            msg=msg,
            joint_names=self.joint_names,
            n_dof=self.n_dof,
            logger=self.get_logger(),
        )
        if not new_points:
            return

        if vh.is_degenerate_trajectory(new_points, span_eps=1e-3):
            try:
                span = float(np.linalg.norm(new_points[-1] - new_points[0]))
                self.get_logger().warn(
                    f"Ignoring degenerate trajectory (span≈{span:.2e} rad, points={len(new_points)})."
                )
            except Exception:
                self.get_logger().warn("Ignoring degenerate trajectory.")
            return

        self.trajectory_points = list(new_points)

        # Precompute cumulative arc-length along the joint-space polyline
        self._traj_s = compute_polyline_arc_lengths(self.trajectory_points)

        self.current_index = 0
        self._progress = vh.PolylineProgress(progress_index=0, progress_s=0.0)
        self.active = True

        # Reset filtro smoothing
        self.qdot_prev = np.zeros(self.n_dof)

        self.get_logger().info(f"📈 Nuova traiettoria: {len(self.trajectory_points)} punti")

    def avoidance_cb(self, msg: Float64MultiArray):
        """Riceve velocità di avoidance (7D)."""
        if len(msg.data) == self.n_dof:
            self.qdot_avoid = np.array(msg.data)

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

        n = int(self.n_dof)

        # ===== PAUSE MODE =====
        # This has priority over any other mode (reactive or trajectory).
        handled, qdot, reset_smoothing = vh.handle_pause_mode(
            pause_enable=bool(self.pause_enable),
            paused=bool(self.paused),
            n_dof=n,
        )
        if handled:
            if reset_smoothing:
                self.qdot_prev = np.zeros(n)
            self.publish_velocity(np.array(qdot, dtype=float).reshape(n))
            return

        # ===== MODALITÀ REACTIVE (B) =====
        # Se non c'è traiettoria, usa direttamente qdot_avoid come comando di velocità.
        handled, qdot, _reset_smoothing = vh.handle_no_trajectory_mode(
            active=bool(self.active),
            trajectory_points_len=int(len(self.trajectory_points)),
            hold_position_without_trajectory=bool(self.hold_position_without_trajectory),
            reactive_enable=bool(self.reactive_enable),
            qdot_avoid=np.array(self.qdot_avoid, dtype=float).reshape(n),
            reactive_deadband=float(self.reactive_deadband),
            max_vel=float(self.max_vel),
            n_dof=n,
        )
        if handled:
            self.publish_velocity(np.array(qdot, dtype=float).reshape(n))
            return

        # ------------------------------------------------------------------
        # EMERGENCY OVERRIDE (hard safety)
        # ------------------------------------------------------------------
        now = time.time()

        safety = vh.compute_safety_signal_context(
            closest_d=float(self.closest_d),
            closest_j_row=np.array(self.closest_j_row, dtype=float).reshape(-1),
            closest_hazard=str(self.closest_hazard or "none"),
            distance_inflation=float(self.distance_inflation),
            risk_d_far=float(self.risk_d_far),
            risk_d_mid=float(self.risk_d_mid),
            risk_d_near=float(self.risk_d_near),
            stop_distance=float(self.stop_distance),
        )

        # Signed distance robustness:
        # - use a clipped effective distance for blending weights (continuous behavior)
        # - but if d_raw < 0 -> penetration emergency (push out decisively, no oscillations)
        d_raw_closest = float(safety.d_raw)
        d_eff_for_weights = float(max(0.0, float(safety.d_eff)))
        try:
            staging_clip = vh.compute_risk_staging(
                d_eff=float(d_eff_for_weights),
                d_far=float(self.risk_d_far),
                d_mid=float(self.risk_d_mid),
                d_near=float(self.risk_d_near),
                d_stop=float(self.stop_distance),
            )
        except Exception:
            staging_clip = safety.staging

        # Filter j_row for stability (use raw for emergency for fast reaction).
        j_row_raw = np.array(safety.j_row, dtype=float).reshape(-1)
        w_d = float(getattr(staging_clip, "w_total", 0.0))
        try:
            if (not self._closest_j_row_filt_init) or (self._closest_j_row_filt is None):
                self._closest_j_row_filt = np.array(j_row_raw, dtype=float).reshape(n)
                self._closest_j_row_filt_init = True
            else:
                prev = np.array(self._closest_j_row_filt, dtype=float).reshape(n)
                cur = np.array(j_row_raw, dtype=float).reshape(n)
                # Sign alignment to avoid flips.
                if float(prev @ cur) < 0.0:
                    cur = -cur
                # Risk-scaled beta: far -> smoother, near -> more responsive (but still filtered).
                beta = float(0.20 + 0.60 * float(np.clip(w_d, 0.0, 1.0)))
                self._closest_j_row_filt = beta * cur + (1.0 - beta) * prev
        except Exception:
            self._closest_j_row_filt = np.array(j_row_raw, dtype=float).reshape(n)
            self._closest_j_row_filt_init = True

        j_row_use = np.array(self._closest_j_row_filt, dtype=float).reshape(-1)
        j_norm_use = float(np.linalg.norm(j_row_use))
        if j_norm_use < 1e-9:
            j_row_use = np.array(j_row_raw, dtype=float).reshape(-1)
            j_norm_use = float(np.linalg.norm(j_row_use))

        # ------------------------------------------------------------------
        # PENETRATION EMERGENCY (signed distance < 0)
        # ------------------------------------------------------------------
        # If the signed distance becomes negative, we are in contact/penetration.
        # In that case we avoid any blending ambiguity and command an outward escape
        # along the distance gradient (j_row). This is deterministic and does not
        # depend on hazard prefixes.
        if (
            bool(self.penetration_emergency_enable)
            and (float(d_raw_closest) < 0.0)
            and (float(j_norm_use) > 1e-6)
            and (str(safety.hazard) != "none")
        ):
            try:
                d_dot_des = max(0.0, float(self.penetration_emergency_d_dot))
                alpha = float(d_dot_des) / (float(j_norm_use) * float(j_norm_use) + 1e-8)
                qdot_escape = float(alpha) * np.array(j_row_use, dtype=float).reshape(n)

                depth_ref = max(1e-6, float(self.penetration_emergency_depth_ref))
                depth = max(0.0, -float(d_raw_closest))
                depth_gain = 1.0 + 4.0 * float(np.clip(depth / depth_ref, 0.0, 2.0))
                qdot_escape *= float(np.clip(depth_gain, 1.0, 5.0))

                maxv = float(self.max_vel) * float(max(0.1, float(self.penetration_emergency_max_vel_fraction)))
                qdot_escape = np.clip(qdot_escape, -maxv, +maxv)

                # Reset smoothing so we don't fight the escape with a slow LPF state.
                self.qdot_prev = np.zeros(n)
                self.publish_velocity(qdot_escape)
                return
            except Exception:
                pass

        handled, qdot_em, self._er_state, reset_smoothing, emergency_now = vh.handle_emergency_override(
            now_wall=float(now),
            d_eff=float(d_eff_for_weights),
            j_row=np.array(j_row_raw, dtype=float).reshape(-1),
            hazard=str(safety.hazard),
            j_norm=float(np.linalg.norm(j_row_raw)),
            max_vel=float(self.max_vel),
            state=self._er_state,
            params=vh.make_emergency_params_from_attrs(self),
        )
        if reset_smoothing:
            self.qdot_prev = np.zeros(n)
        if handled and (qdot_em is not None):
            # Preserve original semantics: on emergency early-return we only count emergency enters.
            self._diag.update_edge_counters(
                emergency_now=bool(emergency_now),
                stop_gate_now=bool(self._diag.prev_stop_gate),
            )
            self.publish_velocity(qdot_em)
            return

        # Stop gate entry counter (based on effective distance)
        self._diag.update_edge_counters(
            emergency_now=bool(self._diag.prev_emergency),
            stop_gate_now=bool(getattr(staging_clip, "stop_gate", False)),
        )

        # ------------------------------------------------------------------
        # PATH FOLLOWING WITH REJOIN (dynamic-obstacle friendly)
        # ------------------------------------------------------------------
        pts = self.trajectory_points
        last_idx = max(0, int(len(pts)) - 1)

        self._progress, self.current_index, q_target = vh.update_progress_and_select_target(
            q=self.q.reshape(n),
            pts=pts,
            s_cum=self._traj_s,
            last_idx=int(last_idx),
            current_index=int(self.current_index),
            progress=self._progress,
            rejoin_enable=bool(self.rejoin_enable),
            rejoin_search_ahead_points=int(self.rejoin_search_ahead_points),
            rejoin_lookahead_distance_rad=float(self.rejoin_lookahead_distance_rad),
            rejoin_lookahead_points=int(self.rejoin_lookahead_points),
            nearest_point_fn=nearest_point_on_polyline,
            interpolate_fn=interpolate_at_s,
        )

        error = q_target - self.q
        error_norm = float(np.linalg.norm(error))

        # ------------------------------------------------------------------
        # Progress-based stall detection (very lightweight, deterministic)
        # ------------------------------------------------------------------
        try:
            prog_s = float(getattr(self._progress, "progress_s", 0.0))
            if self._stall_prog_s_last is None or self._stall_prog_wall_last is None:
                self._stall_prog_s_last = prog_s
                self._stall_prog_wall_last = float(now)
                self._stall_prog_flag = False
            else:
                dt = float(now) - float(self._stall_prog_wall_last)
                # Evaluate on a coarse window to avoid noise-triggered toggles.
                if dt >= 1.5:
                    ds = float(prog_s) - float(self._stall_prog_s_last)
                    # If we advanced less than ~0.01 rad of polyline arc-length in 1.5s, treat as stalled.
                    self._stall_prog_flag = bool(ds < 0.01)
                    self._stall_prog_s_last = prog_s
                    self._stall_prog_wall_last = float(now)
        except Exception:
            self._stall_prog_flag = False

        # Completion: close to final point (regardless of intermediate indices).
        q_final = pts[last_idx]
        final_err, complete = vh.check_trajectory_completion(
            q=self.q,
            q_final=q_final,
            final_threshold=float(self.final_threshold),
        )
        if bool(complete):
            self.get_logger().info(
                f"✅ Traiettoria completata! Errore finale: {final_err:.4f} rad"
            )
            self.active = False
            self.publish_velocity(np.zeros(n))
            return

        # For compatibility with existing logic, keep a 'threshold' variable used later
        # (e.g., tangent_escape_err_min uses max(threshold, ...)).
        threshold = float(self.waypoint_threshold)

        # ===== 1) Tracking "puro" (senza avoidance) =====
        qdot_tracking = vh.compute_tracking_command(kp=float(self.kp), q_target=q_target, q=self.q)

        # Explicit decomposition of tracking w.r.t. the closest constraint direction.
        # n = j_row/||j_row||, v_n = (n^T qdot_tracking) n, v_t = qdot_tracking - v_n
        v_n = np.zeros(n)
        v_t = np.array(qdot_tracking, dtype=float).reshape(n)
        d_dot_track = 0.0
        try:
            if float(j_norm_use) > 1e-6:
                n_dir = (np.array(j_row_use, dtype=float).reshape(n)) / float(j_norm_use)
                d_dot_track = float(n_dir @ np.array(qdot_tracking, dtype=float).reshape(n))
                v_n = float(d_dot_track) * n_dir
                v_t = np.array(qdot_tracking, dtype=float).reshape(n) - np.array(v_n, dtype=float).reshape(n)
        except Exception:
            v_n = np.zeros(n)
            v_t = np.array(qdot_tracking, dtype=float).reshape(n)
            d_dot_track = 0.0

        v_n_norm = float(np.linalg.norm(v_n))
        v_t_norm = float(np.linalg.norm(v_t))

        # ===== 2) Velocità di evitamento =====
        # Filter avoidance input to avoid abrupt "kicks" that can overpower tracking.
        b = vh.compute_avoidance_input_beta(
            beta_far=float(self.avoidance_input_filter_beta),
            beta_near=float(self.avoidance_input_filter_beta_near),
            w=float(getattr(staging_clip, "w_total", 0.0)),
        )
        self.qdot_avoid_filt = vh.update_lpf(self.qdot_avoid_filt, self.qdot_avoid, b)

        qdot_des, dbg = vh.blend_tracking_and_avoidance(
            now_wall=float(now),
            d_raw=float(safety.d_raw),
            d_eff=float(d_eff_for_weights),
            d_infl=float(self.d_infl),
            j_row=np.array(j_row_use, dtype=float).reshape(-1),
            j_norm=float(j_norm_use),
            staging=staging_clip,
            qdot_tracking=np.array(qdot_tracking, dtype=float).reshape(n),
            qdot_avoid_filt=np.array(self.qdot_avoid_filt, dtype=float).reshape(n),
            error_norm=float(error_norm),
            threshold=float(threshold),
            recovery_until_wall=float(self._er_state.recovery_until_wall),
            influence_params=vh.make_influence_params_from_attrs(self),
            stall_detected=bool(self._stall_prog_flag),
            idx=int(self.current_index),
            n_points=int(len(pts)),
        )

        # ------------------------------------------------------------------
        # ANTI-CANCELLATION FALLBACK (local, no utils changes)
        # ------------------------------------------------------------------
        # In some configurations, tracking and avoidance can become almost opposite vectors.
        # If the blender then linearly mixes them, the resulting command can collapse to a
        # very small magnitude even while both inputs are large, causing quasi-stasis.
        #
        # When this happens near obstacles, a robust deterministic choice is:
        #   qdot_des := (tracking tangential to the closest hazard) + (avoidance normal)
        # Optionally keep a tangential part of avoidance only if it helps the chosen tangential.
        try:
            if (
                (str(safety.hazard) != "none")
                and (float(d_eff_for_weights) < float(self.d_infl))
                and (float(error_norm) > float(max(threshold, float(self.tangent_escape_err_min))))
                and (float(j_norm_use) > 1e-6)
            ):
                cmd_pre = np.array(qdot_des, dtype=float).reshape(n)
                cmd_pre_norm = float(np.linalg.norm(cmd_pre))

                t_vec = np.array(qdot_tracking, dtype=float).reshape(n)
                a_vec = np.array(self.qdot_avoid_filt, dtype=float).reshape(n)
                t_norm = float(np.linalg.norm(t_vec))
                a_norm = float(np.linalg.norm(a_vec))
                eps = 1e-9

                cos_ta = float("nan")
                if (t_norm > eps) and (a_norm > eps):
                    cos_ta = float(np.dot(t_vec, a_vec) / (t_norm * a_norm))

                # Trigger when cmd collapses while track+avoid are sizable and opposing.
                cmd_small = float(max(0.03, 1.5 * float(self.tangent_tracking_cmd_small)))
                if (
                    (cmd_pre_norm < cmd_small)
                    and (t_norm > 3.0 * cmd_small)
                    and (a_norm > 3.0 * cmd_small)
                    and (np.isfinite(cos_ta) and (cos_ta < -0.35))
                ):
                    n_dir = (np.array(j_row_use, dtype=float).reshape(n)) / float(j_norm_use)
                    t_n = float(n_dir @ t_vec)
                    a_n = float(n_dir @ a_vec)
                    t_tan = t_vec - float(t_n) * n_dir
                    a_norm_vec = float(a_n) * n_dir
                    a_tan = a_vec - a_norm_vec

                    # Start with: keep tracking tangential + avoidance normal.
                    cmd_rebuilt = np.array(t_tan, dtype=float).reshape(n) + np.array(a_norm_vec, dtype=float).reshape(n)

                    # Optionally add some avoidance tangential only if it does not fight cmd_rebuilt's tangential.
                    try:
                        tan_ref = np.array(t_tan, dtype=float).reshape(n)
                        if float(np.linalg.norm(tan_ref)) < 1e-6:
                            tan_ref = np.array(cmd_rebuilt, dtype=float).reshape(n) - float((n_dir @ cmd_rebuilt)) * n_dir
                        if float(np.linalg.norm(tan_ref)) > 1e-6:
                            if float(np.dot(tan_ref, a_tan)) > 0.0:
                                cmd_rebuilt = cmd_rebuilt + float(self.avoidance_tangent_weight) * np.array(a_tan, dtype=float).reshape(n)
                    except Exception:
                        pass

                    qdot_des = np.array(cmd_rebuilt, dtype=float).reshape(n)

                    # Diagnostics
                    try:
                        if isinstance(dbg, dict):
                            dbg["anti_cancel"] = True
                            dbg["anti_cancel_cos_track_avoid"] = float(cos_ta)
                            dbg["anti_cancel_cmd_pre_norm"] = float(cmd_pre_norm)
                            dbg["anti_cancel_cmd_post_norm"] = float(np.linalg.norm(qdot_des))
                            dbg["anti_cancel_track_tan_norm"] = float(np.linalg.norm(t_tan))
                            dbg["anti_cancel_avoid_norm_norm"] = float(np.linalg.norm(a_norm_vec))
                    except Exception:
                        pass
                else:
                    try:
                        if isinstance(dbg, dict):
                            dbg["anti_cancel"] = False
                            if np.isfinite(cos_ta):
                                dbg["anti_cancel_cos_track_avoid"] = float(cos_ta)
                    except Exception:
                        pass
        except Exception:
            pass

        # ------------------------------------------------------------------
        # TRACKING TANGENTIAL ESCAPE (real sliding when normal is clipped)
        # ------------------------------------------------------------------
        # If we're inside the influence zone and the safety constraint tends to cut the
        # "normal" component of tracking (pushing into the obstacle), the final command
        # can become a near-constant small vector (sticky / quasi-stasis). Here we ensure
        # a minimum tangential "slide" component derived from tracking itself.
        try:
            if (
                bool(self.tangent_tracking_escape_enable)
                and (str(safety.hazard) != "none")
                and (float(d_eff_for_weights) < float(self.d_infl))
                and (float(error_norm) > float(max(threshold, float(self.tangent_escape_err_min))))
            ):
                cmd_norm_pre = float(np.linalg.norm(np.array(qdot_des, dtype=float).reshape(n)))
                cmd_small = float(max(0.0, float(self.tangent_tracking_cmd_small)))
                vt_min = float(max(0.0, float(self.tangent_tracking_vt_min)))

                # Trigger when tracking would reduce distance (d_dot_track < 0) and the
                # current blend is already small.
                if (cmd_norm_pre < cmd_small) and (float(d_dot_track) < 0.0) and (float(v_t_norm) > vt_min):
                    # Smooth alpha from 0 at d_infl to alpha_max near stop_distance.
                    denom = float(max(1e-6, float(self.d_infl) - float(self.stop_distance)))
                    x = float(np.clip((float(self.d_infl) - float(d_eff_for_weights)) / denom, 0.0, 1.0))
                    smooth = float(3.0 * x * x - 2.0 * x * x * x)
                    alpha_t = float(np.clip(float(self.tangent_tracking_alpha_max) * smooth, 0.0, float(self.tangent_tracking_alpha_max)))

                    vt_max = float(max(0.0, float(self.tangent_tracking_vt_max)))
                    vt_used = np.array(v_t, dtype=float).reshape(n)
                    if float(v_t_norm) > max(1e-9, vt_max):
                        vt_used *= float(vt_max) / (float(v_t_norm) + 1e-9)

                    qdot_des = np.array(qdot_des, dtype=float).reshape(n) + float(alpha_t) * vt_used

                    # Diagnostics
                    try:
                        if isinstance(dbg, dict):
                            dbg["track_vn_norm"] = float(v_n_norm)
                            dbg["track_vt_norm"] = float(v_t_norm)
                            dbg["track_d_dot"] = float(d_dot_track)
                            dbg["tangent_track_escape"] = True
                            dbg["tangent_track_alpha"] = float(alpha_t)
                            dbg["tangent_track_vt_used_norm"] = float(np.linalg.norm(vt_used))
                    except Exception:
                        pass
                else:
                    try:
                        if isinstance(dbg, dict):
                            dbg["track_vn_norm"] = float(v_n_norm)
                            dbg["track_vt_norm"] = float(v_t_norm)
                            dbg["track_d_dot"] = float(d_dot_track)
                            dbg["tangent_track_escape"] = False
                    except Exception:
                        pass
        except Exception:
            pass


        # ------------------------------------------------------------------
        # FILTRO SULLA VELOCITÀ + SATURAZIONE (+ robust CBF enforcement under saturation)
        # ------------------------------------------------------------------
        qdot, qdot_prev_new, dbg = vh.apply_final_filters_and_limits(
            qdot_des=np.array(qdot_des, dtype=float).reshape(n),
            qdot_prev=np.array(self.qdot_prev, dtype=float).reshape(n),
            velocity_filter_beta=float(self.velocity_filter_beta),
            velocity_filter_beta_near=float(self.velocity_filter_beta_near),
            max_vel=float(self.max_vel),
            d_eff=float(d_eff_for_weights),
            d_infl=float(self.d_infl),
            j_row=np.array(j_row_use, dtype=float).reshape(-1),
            j_norm=float(j_norm_use),
            cbf_projection_iters=int(self.cbf_projection_iters),
            cbf_eps=float(self.cbf_eps),
            normal_correction_max=float(self.normal_correction_max),
            qdot_tracking_hint=np.array(qdot_tracking, dtype=float).reshape(n),
            dbg=dbg,
        )
        self.qdot_prev = np.array(qdot_prev_new, dtype=float).reshape(n)

        # Diagnostics (counters + throttled logs are fully outside the node)
        self._diag.count_infeasible(dbg)
        self._diag.maybe_log_stall(
            self.get_logger(),
            diag_enable=bool(self.diag_enable),
            now_wall=float(time.time()),
            diag_period_s=float(self.diag_period_s),
            dbg=dbg,
            cmd_norm=float(np.linalg.norm(qdot)),
            d_infl=float(self.d_infl),
            diag_cmd_norm_eps=float(self.diag_cmd_norm_eps),
        )

        # Pubblica comando finale
        self.publish_velocity(qdot)

        # 1Hz robustness summary
        self._diag.maybe_log_robust(
            self.get_logger(),
            diag_enable=bool(self.diag_enable),
            now_wall=float(time.time()),
            d_raw=float(safety.d_raw),
            d_eff=float(d_eff_for_weights),
            inflation=float(safety.inflation),
            hazard=str(safety.hazard),
        )

    # ======================================================================
    # UTILITIES
    # ======================================================================

    def publish_velocity(self, qdot: np.ndarray):
        """Pubblica comando di velocità sui giunti."""
        self.cmd_pub.publish(Float64MultiArray(data=qdot.tolist()))


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
