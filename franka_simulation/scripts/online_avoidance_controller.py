#!/usr/bin/env python3
"""
ONLINE AVOIDANCE CONTROLLER — NULL SPACE VERSION
===============================================

• Capsule-based distance estimation
• Potential field used ONLY as direction metric
• Avoidance projected in EE null space
• Tracking task is NEVER opposed
• No local minima blocking

Author: Mattia (Null-space refactor)
"""

import numpy as np
from typing import List
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import MarkerArray
from moveit_msgs.msg import PlanningScene

import pinocchio as pin

# Controller decomposition (keeps this file high-level).
from utils.avoidance_core import iter_world_capsule_segments, scan_external_and_ground, scan_self_collision
from utils.cbf_filter import (
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

# High-level-only refactor: params, logging and repetitive publishing live in utils.
from utils.params import NullSpaceAvoidanceParams, load_controller_params, setup_optional_qp_solver
from utils.logging import log_controller_config
from utils.ros_publishers import PublishersBundle, publish_not_ready_outputs
from utils.closest_constraint import publish_closest_constraint
from utils.closest_constraint import ClosestConstraintHoldState
from utils.diagnostics import publish_cbf_diagnostics

# Optional posture objective (used when controller safety filter is disabled)
from utils.avoidance_math import (
    posture_reference,
    staged_risk_weight,
    build_cbf_constraints,
    smooth_alpha,
    point_jacobian_world,
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
        # Single call: declare + load + validate (keeps node readable).
        self.params: NullSpaceAvoidanceParams = load_controller_params(self)

        # Safety filter runtime state (kept in a dedicated struct for clarity).
        self._cbf_state = CbfFilterState()

        # Optional QP solver setup
        self._qp_solver, self._qp_available = setup_optional_qp_solver(params=self.params, cbf_state=self._cbf_state)

        log_controller_config(
            logger=self.get_logger(),
            params=self.params,
            qp_available=bool(self._qp_available),
            cbf_state=self._cbf_state,
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
            capsule_fractions=list(self.params.capsule_fractions),
            capsule_radii=list(self.params.capsule_radii),
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
            make_planning_scene_callback(controller=self, excluded_substrings=list(self.params.excluded)),
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
        # TODO: Confirm topic name for multi-constraint list if there is an existing convention.
        self.constraints_pub = self.create_publisher(Float64MultiArray, "/avoidance/constraints", 10)

        # riga di Jacobiano (1x7) del punto più critico: d_dot ≈ j_row @ qdot
        self.jac_pub = self.create_publisher(Float64MultiArray, "/avoidance/jacobian", 10)
        # Debug/diagnostics: which hazard is currently the most critical (helps explain stalls)
        self.hazard_pub = self.create_publisher(String, "/avoidance/hazard", 10)

        # Bundle publishers for reuse in helper functions.
        self._pubs = PublishersBundle(
            pub=self.pub,
            min_dist_pub=self.min_dist_pub,
            min_dist_raw_pub=self.min_dist_raw_pub,
            closest_constraint_pub=self.closest_constraint_pub,
            closest_hazard_pub=self.closest_hazard_pub,
            constraints_pub=self.constraints_pub,
            jac_pub=self.jac_pub,
            hazard_pub=self.hazard_pub,
            capsule_marker_pub=self.capsule_marker_pub,
        )

        # Control loop @ 100 Hz
        self.create_timer(1.0 / float(self.params.rate), self._control_loop)
        # Marker visualization @ 10 Hz (ridotto per evitare DDS buffer overflow)
        self.create_timer(0.1, self._publish_markers_only)

        self.get_logger().info("🟢 Null-Space Avoidance Controller READY")

        # --- Robustness / debug state ---
        self._closest_hold = ClosestConstraintHoldState()
        self._dbg_last_ns = 0
        self._dbg_stop_enter_count = 0
        self._dbg_stop_exit_count = 0
        self._dbg_prev_stop = bool(self._cbf_state.stop_gate_active)
        self._dbg_prev_closest_hazard = "none"
        self._dbg_closest_switch_count = 0
        # TEST-REACTIVE: throttled logs
        self._test_last_no_obs_wall = 0.0
        self._test_last_no_js_wall = 0.0

    def _control_loop(self):
        # High-level flow:
        #  1) sanity check / publish zeros if not ready
        #  2) update kinematics
        #  3) compute nominal avoidance (external + ground + self)
        #  4) apply CBF-QP safety layer (stop gate + constraints + smoothing)
        #  5) publish commands + diagnostics
        #  6) rebuild RViz marker cache (published separately at 10Hz)

        if not (self.pin_ok and isinstance(self.q, np.ndarray)):
            # TEST-REACTIVE: waiting for joint_states
            try:
                now_wall = float(time.time())
                if (now_wall - float(self._test_last_no_js_wall)) >= 1.0:
                    self._test_last_no_js_wall = now_wall
                    self.get_logger().info("[TEST-REACTIVE] waiting for joint_states")
            except Exception:
                pass
            publish_not_ready_outputs(pubs=self._pubs)
            return

        # Update kinematics for current q
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        # --- Build world geometry (capsule segments)
        # TEST-REACTIVE: warn if no obstacles have been received
        try:
            now_wall = float(time.time())
            if (len(self.obstacles) == 0) and ((now_wall - float(self._test_last_no_obs_wall)) >= 1.0):
                self._test_last_no_obs_wall = now_wall
                self.get_logger().info("[TEST-REACTIVE] no obstacles received on /obstacle_scene")
        except Exception:
            pass
        segments = iter_world_capsule_segments(capsules=self.capsules, frame_ids=self.frame_ids, data=self.data)

        # --- Nominal avoidance (external obstacles + ground) + debug distances
        avoid_diag: dict = {}
        (
            qdot_external_ground,
            d_min,
            external_best,
            ground_best,
            dist_ext_ground,
            external_candidates,
            tip_to_obstacle_distances,
        ) = scan_external_and_ground(
            segments=segments,
            obstacles=list(self.obstacles),
            model=self.model,
            data=self.data,
            q=self.q,
            box_projection_iters=int(self.params.box_projection_iters),
            repulsion_spread_enable=bool(self.params.repulsion_spread_enable),
            repulsion_spread_samples=int(self.params.repulsion_spread_samples),
            repulsion_spread_half_length=float(self.params.repulsion_spread_half_length),
            influence_distance=float(self.params.influence_distance),
            d_aggr=float(self.params.d_aggr),
            safety_margin=float(self.params.safety_margin),
            k_aggr=float(self.params.k_aggr),
            k_null=float(self.params.k_null),
            k_tan=float(self.params.k_tan),
            max_qdot=float(self.params.max_qdot),
            avoidance_contrib_max_ratio=float(getattr(self.params, "avoidance_contrib_max_ratio", 0.0)),
            enable_ground=bool(self.params.enable_ground),
            ground_z=float(self.params.ground_z),
            ground_infl=float(self.params.ground_infl),
            ground_safe=float(self.params.ground_safe),
            k_ground=float(self.params.k_ground),
            debug_stats=avoid_diag,
        )

        # --- Nominal avoidance (self-collision) + debug distances
        qdot_self, d_min, self_best, dist_self, self_candidates = scan_self_collision(
            segments=segments,
            model=self.model,
            data=self.data,
            q=self.q,
            enable_self=bool(self.params.enable_self),
            self_skip_adjacent_links=int(self.params.self_skip_adjacent),
            self_infl=float(self.params.self_infl),
            self_safe=float(self.params.self_safe),
            k_self=float(self.params.k_self),
            d_min_in=float(d_min),
        )

        # Distances list used only for RViz debug markers (ordering preserved)
        # Add tip-to-obstacle distances for visualization
        self.distances_data = list(dist_ext_ground) + list(dist_self)
        if tip_to_obstacle_distances:
            self.distances_data.extend(tip_to_obstacle_distances)

        # Active contacts for multi-constraint blending (within influence zones only)
        active_candidates: List[dict] = list(external_candidates) + list(self_candidates)

        # ------------------------------------------------------------------
        # Nominal avoidance (distance-gradient, normal-only, multi-point sum)
        # ------------------------------------------------------------------
        qdot_nom_pre = np.zeros(7, dtype=float)
        weight_sum = 0.0
        active_count = 0
        closest_label = "none"
        closest_d = float("inf")

        try:
            act = []
            for c in list(active_candidates):
                try:
                    d = float(c.get("d", 1e9))
                    if not bool(np.isfinite(d)):
                        continue
                    act.append(c)
                except Exception:
                    continue
            act.sort(key=lambda x: float(x.get("d", 1e9)))

            for idx, c in enumerate(act):
                kind = str(c.get("kind", "external"))
                d = float(c.get("d", 1e9))

                if kind == "ground":
                    influence_radius = float(self.params.ground_infl)
                    safety_limit = float(self.params.ground_safe)
                    gain = float(self.params.k_ground)
                elif kind == "self":
                    influence_radius = float(self.params.self_infl)
                    safety_limit = float(self.params.self_safe)
                    gain = float(self.params.k_self)
                else:
                    influence_radius = float(self.params.influence_distance)
                    safety_limit = float(self.params.safety_margin)
                    gain = float(self.params.k_null)

                if d >= float(influence_radius):
                    continue

                w = float(smooth_alpha(d, float(influence_radius), float(safety_limit)))
                if kind == "external":
                    if float(self.params.d_aggr) > (float(safety_limit) + 1e-9):
                        alpha_close = float(smooth_alpha(d, float(self.params.d_aggr), float(safety_limit)))
                    else:
                        alpha_close = 0.0
                    gain_scale = 1.0 + float(self.params.k_aggr) * float(alpha_close)
                else:
                    gain_scale = 1.0

                w = float(w) * float(gain) * float(gain_scale)
                if w <= 0.0:
                    continue

                j_row = None
                if kind in ("external", "ground"):
                    fid = int(c.get("fid", -1))
                    p = np.array(c.get("p", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                    n = np.array(c.get("n", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                    n = n / (float(np.linalg.norm(n)) + 1e-9)
                    if fid >= 0:
                        Jp = point_jacobian_world(self.model, self.data, self.q, fid, p)
                        j_row = (n.reshape(1, 3) @ Jp).reshape(-1)
                elif kind == "self":
                    fid_i = int(c.get("fid_i", -1))
                    fid_j = int(c.get("fid_j", -1))
                    p_i = np.array(c.get("p_i", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                    p_j = np.array(c.get("p_j", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                    n = np.array(c.get("n", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                    n = n / (float(np.linalg.norm(n)) + 1e-9)
                    if (fid_i >= 0) and (fid_j >= 0):
                        J_i = point_jacobian_world(self.model, self.data, self.q, fid_i, p_i)
                        J_j = point_jacobian_world(self.model, self.data, self.q, fid_j, p_j)
                        j_row = (n.reshape(1, 3) @ (J_i - J_j)).reshape(-1)

                if j_row is None:
                    continue

                jn = float(np.linalg.norm(np.array(j_row, dtype=float).reshape(-1)))
                if jn <= 1e-6:
                    continue

                qdot_nom_pre += float(w) * (np.array(j_row, dtype=float).reshape(-1) / float(jn))
                weight_sum += float(w)
                active_count += 1

                if float(d) < float(closest_d):
                    closest_d = float(d)
                    hazard = str(c.get("hazard", ""))
                    closest_label = hazard if len(hazard) > 0 else f"idx:{int(idx)}"
        except Exception:
            pass

        qdot_nom = np.array(qdot_nom_pre, dtype=float).reshape(7)
        avoid_diag["active_count"] = int(active_count)
        avoid_diag["weight_sum"] = float(weight_sum)
        avoid_diag["closest_label"] = str(closest_label)
        avoid_diag["qdot_nom_pre_norm"] = float(np.linalg.norm(qdot_nom_pre))

        # --- Build candidate list (RAW) for publishing + (EFFECTIVE) for safety decisions.
        candidates_raw: List[dict] = []
        if len(external_best) > 0:
            candidates_raw.extend(list(external_best.values()))
        if ground_best is not None:
            # Ground-plane can be useful when really close, but it should NOT dominate the
            # coherent safety signal when we are well above the floor (it tends to create
            # stalls because the blender treats ANY active hazard as a d_dot constraint).
            #
            # Rule:
            # - include ground as a candidate only inside its influence zone
            # - when external hazards exist and ground is not near its safety margin,
            #   de-prioritize ground so it does not steal "closest" due to tiny deltas.
            try:
                d_g = float(ground_best.get("d", 1e9))
                ground_infl = float(getattr(self.params, "ground_infl", 0.0))
                ground_safe = float(getattr(self.params, "ground_safe", 0.0))

                # Only consider ground for the safety signal when it is actually relevant.
                if d_g <= (ground_infl + 1e-9):
                    # If we have external hazards and ground is not close to its safety margin,
                    # bias it away to avoid frequent switching ground<->external.
                    min_ext = 1e9
                    try:
                        if len(external_best) > 0:
                            min_ext = float(min(float(v.get("d", 1e9)) for v in external_best.values()))
                    except Exception:
                        min_ext = 1e9

                    # If an external obstacle is within ~3cm of the ground distance, prefer external,
                    # unless ground is genuinely critical (near its safety margin).
                    if (min_ext < 1e6) and (d_g > (ground_safe + 0.01)) and (min_ext <= (d_g + 0.03)):
                        gb = dict(ground_best)
                        gb["d"] = float(d_g + 0.03)
                        candidates_raw.append(gb)
                    else:
                        candidates_raw.append(ground_best)
            except Exception:
                candidates_raw.append(ground_best)
        if self_best is not None:
            candidates_raw.append(self_best)

        inflation = float(max(0.0, float(self.params.distance_inflation)))

        # Use a conservative effective distance for ALL safety decisions.
        d_min_raw = float(d_min)
        d_min_eff = float(d_min_raw) - float(inflation)

        candidates_eff: List[dict] = []
        for c in list(candidates_raw):
            try:
                d_raw = float(c.get("d", 1e9))
                c_eff = dict(c)
                c_eff["d_raw"] = float(d_raw)
                c_eff["d"] = float(d_raw) - float(inflation)
                candidates_eff.append(c_eff)
            except Exception:
                # Best-effort: keep the candidate unchanged
                candidates_eff.append(c)

        # --- Safety filter (CBF-QP) + smoothing
        if bool(getattr(self.params, "controller_safety_filter_enable", True)):
            qdot_out, G, m_active, active_best, _, _, _ = apply_cbf_qp_safety_filter(
                qdot_nom=qdot_nom,
                d_min=float(d_min_eff),
                candidates=candidates_eff,
                model=self.model,
                data=self.data,
                q=self.q,
                params=self.params.cbf_params,
                state=self._cbf_state,
                qp_solver=self._qp_solver,
                qp_available=bool(self._qp_available),
            )
        else:
            # Architecture B (recommended): controller is nominal, blender is the single point
            # of safety enforcement for the FINAL command (tracking + avoidance).
            # Here we still publish a smoothed min-distance for visualization/debug, but we do
            # NOT apply any safety projection/stop gate to the controller output.
            try:
                d_beta = float(np.clip(float(self.params.cbf_params.min_distance_lpf), 0.0, 1.0))
                self._cbf_state.d_min_filt = float(
                    d_beta * float(d_min_eff) + (1.0 - d_beta) * float(self._cbf_state.d_min_filt)
                )
                self._cbf_state.cbf_active = False
                self._cbf_state.stop_gate_active = False
            except Exception:
                pass

            # Optional posture bias to prevent ugly redundancy usage (kept very lightweight).
            try:
                if float(getattr(self.params, "posture_bias_gain", 0.0)) > 0.0:
                    q_ref = posture_reference(getattr(self.params, "posture_reference_param", []), model=self.model)
                    if q_ref is not None and isinstance(self.q, np.ndarray) and self.q.shape[0] >= 7:
                        w_post = float(
                            staged_risk_weight(
                                float(d_min_eff),
                                d_far=float(self.params.risk_d_far),
                                d_mid=float(self.params.risk_d_mid),
                                d_near=float(self.params.risk_d_near),
                                d_stop=float(self.params.stop_d_in),
                            )
                        )
                        q_cur = np.array(self.q, dtype=float).reshape(-1)[:7]
                        qdot_nom = np.array(qdot_nom, dtype=float).reshape(7) + float(self.params.posture_bias_gain) * w_post * (q_ref - q_cur)
            except Exception:
                pass

            # Light output damping (NOT a safety layer): reduces oscillations in nominal mode.
            # Uses already-existing risk-staged LPF parameters from the CBF filter config.
            try:
                w = float(
                    staged_risk_weight(
                        float(self._cbf_state.d_min_filt),
                        d_far=float(self.params.risk_d_far),
                        d_mid=float(self.params.risk_d_mid),
                        d_near=float(self.params.risk_d_near),
                        d_stop=float(self.params.stop_d_in),
                    )
                )
                b_far = float(np.clip(float(self.params.cbf_params.beta_lpf_far), 0.0, 1.0))
                b_near = float(np.clip(float(self.params.cbf_params.beta_lpf_near), 0.0, 1.0))
                beta = float(b_far + (b_near - b_far) * float(np.clip(w, 0.0, 1.0)))
                qdot_out = beta * np.array(qdot_nom, dtype=float).reshape(7) + (1.0 - beta) * np.array(self._cbf_state.qdot_pub_prev, dtype=float).reshape(7)
                self._cbf_state.qdot_pub_prev = np.array(qdot_out, dtype=float).reshape(7)
            except Exception:
                qdot_out = np.array(qdot_nom, dtype=float).reshape(7)

            G = None
            m_active = 0
            active_best = None

        # Always respect max joint velocity at the controller output (keeps nominal avoidance bounded).
        try:
            qdot_out = np.clip(np.array(qdot_out, dtype=float).reshape(7), -float(self.params.max_qdot), +float(self.params.max_qdot))
        except Exception:
            pass

        # TEST-REACTIVE: ensure visible avoidance output inside influence zone (unless stop gate)
        try:
            if (
                (float(d_min_eff) <= float(self.params.influence_distance))
                and (float(np.linalg.norm(np.array(qdot_out, dtype=float).reshape(-1))) < 1e-6)
                and (float(np.linalg.norm(np.array(qdot_nom, dtype=float).reshape(-1))) > 1e-6)
                and (not bool(self._cbf_state.stop_gate_active))
            ):
                qdot_out = np.array(qdot_nom, dtype=float).reshape(7)
                qdot_out = np.clip(
                    np.array(qdot_out, dtype=float).reshape(7),
                    -float(self.params.max_qdot),
                    +float(self.params.max_qdot),
                )
        except Exception:
            pass

        # ------------------------------------------------------------------
        # Publish a *coherent* closest hazard pair for downstream blending:
        #   (d_closest_raw, j_row_closest)
        # This intentionally does NOT depend on whether the CBF is active.
        # ------------------------------------------------------------------
        publish_closest_constraint(
            candidates=list(candidates_raw),
            model=self.model,
            data=self.data,
            q=self.q,
            pubs=self._pubs,
            params=self.params,
            d_min_default=float(d_min_raw),
            hold_state=self._closest_hold,
            now_wall=float(time.time()),
        )

        # Publish active multi-constraints for the blender (Float64MultiArray format)
        # Format: [N, d1, j1_0..j1_6, d2, j2_0..j2_6, ...]
        try:
            act = [c for c in list(active_candidates) if float(c.get("d", 1e9)) <= float(self.params.influence_distance)]
            act.sort(key=lambda x: float(x.get("d", 1e9)))
            K = int(max(0, int(self.params.cbf_K)))
            if K > 0:
                K = min(K, int(len(act)))
            if (K <= 0) or (len(act) <= 0):
                self.constraints_pub.publish(Float64MultiArray(data=[0.0]))
            else:
                Gc, _bc_unused, mc, _best_c = build_cbf_constraints(
                    list(act),
                    float(self.params.influence_distance),
                    K=int(K),
                    cbf_eps=float(self.params.cbf_eps),
                    cbf_d_safe=float(self.params.cbf_d_safe),
                    approach_speed_limit=float(self.params.cbf_approach_speed_limit),
                    alpha_min=float(self.params.cbf_alpha_min),
                    alpha_max=float(self.params.cbf_alpha_max),
                    risk_d_far=float(self.params.risk_d_far),
                    risk_d_mid=float(self.params.risk_d_mid),
                    risk_d_near=float(self.params.risk_d_near),
                    stop_distance=float(self.params.stop_d_in),
                    model=self.model,
                    data=self.data,
                    q=self.q,
                )

                payload: List[float] = []
                for i in range(int(mc)):
                    try:
                        di = float(act[i].get("d", 1e9))
                        gi = np.array(Gc[i, :], dtype=float).reshape(-1)
                        if gi.shape[0] != 7:
                            continue
                        if not bool(np.all(np.isfinite(gi))):
                            continue
                        if float(np.linalg.norm(gi)) <= 1e-6:
                            continue
                        payload.append(float(di))
                        payload.extend(gi.tolist())
                    except Exception:
                        continue

                if len(payload) <= 0:
                    self.constraints_pub.publish(Float64MultiArray(data=[0.0]))
                else:
                    n_active = int(len(payload) / 8)
                    self.constraints_pub.publish(Float64MultiArray(data=[float(n_active)] + payload))
        except Exception:
            pass

        # If the controller safety filter is disabled, reuse the held closest-constraint
        # triplet for legacy diagnostics so downstream debug stays coherent.
        if not bool(getattr(self.params, "controller_safety_filter_enable", True)):
            try:
                if getattr(self._closest_hold, "last_j_row", None) is not None:
                    G = np.array(self._closest_hold.last_j_row, dtype=float).reshape(1, 7)
                    m_active = 1
                    active_best = {"hazard": str(getattr(self._closest_hold, "last_hazard", "none"))}
            except Exception:
                pass

        # Publish the joint velocity command (same topic/type as before)
        self.pub.publish(Float64MultiArray(data=np.array(qdot_out, dtype=float).reshape(-1).tolist()))

        # Publish diagnostics (min distance, hazard string, and jacobian of the most critical active constraint)
        publish_cbf_diagnostics(
            pubs=self._pubs,
            cbf_state=self._cbf_state,
            G=G,
            m_active=int(m_active),
            active_best=active_best,
            d_min_raw=float(d_min_raw),
        )

        # Throttled debug log (1Hz) only when the controller safety filter is active.
        if bool(getattr(self.params, "controller_safety_filter_enable", True)):
            debug_throttled(
                logger=self.get_logger(),
                now_ns=int(self.get_clock().now().nanoseconds),
                d_min_raw=float(d_min_raw),
                m_active=int(m_active),
                params=self.params.cbf_params,
                state=self._cbf_state,
            )

        # Minimal robustness diagnostics (1 Hz): raw vs effective distance and switching counts.
        try:
            now_ns = int(self.get_clock().now().nanoseconds)
            if (now_ns - int(self._dbg_last_ns)) >= 1_000_000_000:
                self._dbg_last_ns = now_ns

                # stop gate transitions
                stop_now = bool(self._cbf_state.stop_gate_active)
                if (not bool(self._dbg_prev_stop)) and stop_now:
                    self._dbg_stop_enter_count += 1
                if bool(self._dbg_prev_stop) and (not stop_now):
                    self._dbg_stop_exit_count += 1
                self._dbg_prev_stop = bool(stop_now)

                # closest hazard switching (post-hold)
                ch = str(self._closest_hold.last_hazard)
                if ch != str(self._dbg_prev_closest_hazard):
                    self._dbg_closest_switch_count += 1
                    self._dbg_prev_closest_hazard = str(ch)

                self.get_logger().info(
                    "[AVOID-ROBUST] "
                    f"d_raw={float(d_min_raw):.3f}m infl={float(inflation):.3f}m d_eff={float(d_min_eff):.3f}m "
                    f"closest='{ch}' switches={int(self._dbg_closest_switch_count)} "
                    f"stop={bool(self._cbf_state.stop_gate_active)} enter={int(self._dbg_stop_enter_count)} exit={int(self._dbg_stop_exit_count)} "
                    f"cbf_active={bool(self._cbf_state.cbf_active)}"
                )

                # TEST-REACTIVE: concise end-to-end status (1 Hz)
                try:
                    qn = float(np.linalg.norm(np.array(qdot_nom, dtype=float).reshape(-1)))
                    qo = float(np.linalg.norm(np.array(qdot_out, dtype=float).reshape(-1)))
                    self.get_logger().info(
                        "[TEST-REACTIVE] "
                        f"d_min_raw={float(d_min_raw):.3f} d_min_eff={float(d_min_eff):.3f} "
                        f"closest='{ch}' |qdot_nom|={qn:.4f} |qdot_out|={qo:.4f} "
                        f"stop_gate={bool(self._cbf_state.stop_gate_active)} cbf_active={bool(self._cbf_state.cbf_active)}"
                    )
                except Exception:
                    pass

                alpha_far_max = float(avoid_diag.get("alpha_far_max", 0.0)) if isinstance(avoid_diag, dict) else 0.0
                alpha_near_zero = bool(alpha_far_max < 1e-3)
                qdot_pre_max = float(avoid_diag.get("norm_pre_max", 0.0)) if isinstance(avoid_diag, dict) else 0.0
                qdot_post_max = float(avoid_diag.get("norm_post_max", 0.0)) if isinstance(avoid_diag, dict) else 0.0
                clamp_count = int(avoid_diag.get("clamp_count", 0)) if isinstance(avoid_diag, dict) else 0
                clamp_ratio = float(getattr(self.params, "avoidance_contrib_max_ratio", 0.0))

                active_count = int(avoid_diag.get("active_count", 0)) if isinstance(avoid_diag, dict) else 0
                weight_sum = float(avoid_diag.get("weight_sum", 0.0)) if isinstance(avoid_diag, dict) else 0.0
                closest_label = str(avoid_diag.get("closest_label", "none")) if isinstance(avoid_diag, dict) else "none"

                qdot_nom_norm = float(avoid_diag.get("qdot_nom_pre_norm", 0.0)) if isinstance(avoid_diag, dict) else 0.0
                qdot_out_norm = float(np.linalg.norm(np.array(qdot_out, dtype=float).reshape(-1)))

                self.get_logger().info(
                    "[AVOID-NOM] "
                    f"d_raw={float(d_min_raw):.3f}m alpha_max={alpha_far_max:.3f} alpha_zero={alpha_near_zero} "
                    f"active={active_count} w_sum={float(weight_sum):.3f} closest='{closest_label}' "
                    f"|qdot_avoid_pre|={qdot_nom_norm:.3f} |qdot_out|={qdot_out_norm:.3f} "
                    f"pre_max={qdot_pre_max:.3f} post_max={qdot_post_max:.3f} "
                    f"clamp_ratio={clamp_ratio:.2f} clamp_count={clamp_count} haz='{ch}'"
                )
        except Exception:
            pass

        # Build and cache markers (published by the 10Hz timer)
        self.last_marker_array = build_marker_array(
            capsules=self.capsules,
            frame_ids=self.frame_ids,
            data=self.data,
            distances_data=self.distances_data,
            influence_distance=float(self.params.influence_distance),
            distance_inflation=float(self.params.distance_inflation),
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
