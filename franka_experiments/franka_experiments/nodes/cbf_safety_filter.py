#!/usr/bin/env python3
"""CBF safety filter — acceleration-level, three rates, one process.

    I/O callbacks (event-driven)     perception group        control group
    ────────────────────────────     ────────────────        ─────────────
    /joint_states  ─┐                50 Hz: Pinocchio        100 Hz: OSQP
    /qddot_nom     ─┼► snapshots ──► FK + point Jacobians ─► solve + publish
    /per_link_dist ─┘  (atomic       ConstraintBuilder       /qddot_safe
                        swap)        (atomic swap)

Per QP tick:

    min ½‖q̈ − q̈_nom‖² + ½ Σ_g ρ_g s_g²
    s.t.  q̈_min ≤ q̈ ≤ q̈_max                     (state box, hard)
          aᵢᵀq̈ + mᵢ·s_g ≥ bᵢ   ∀ row i           (CBF rows, relaxable)

This module is the ORCHESTRATOR and nothing else. It owns the ROS wiring, the
three clocks, the staleness policy and the OSQP instance. It owns no formula:

* what a row IS            → utils.cbf_state_rows (all six families + builder)
* the rows' right-hand side→ utils.cbf_qp_assembly.build_row_rhs
* the hard state box       → utils.cbf_hard_limits
* the singularity barrier  → utils.cbf_singularity
* every parameter          → config/fr3_control.yaml, via utils.config
* the CBFDIAG line         → utils.logging_utils.format_cbf_diag

Staleness policy, all of it in one place (``_qp_tick``):

    joint state older than joint_state_timeout   → brake on last known q̇
    joint state FROZEN (identical, re-stamped)   → brake, CBF rows dropped
    distances older than distance_timeout        → brake, CBF rows dropped
    q̈_nom older than nom_timeout                 → brake
    QP not solved                                → brake, reset the warm start

Every one of those degrades toward braking, never toward passthrough, and
raises ``fault_braking`` on /cbf_status.
"""

import gc
import os
import threading
import time

import numpy as np
import osqp
import pinocchio as pin
import rclpy
import scipy.sparse as sparse
from franka_msgs.msg import MultiLinkDistance
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.cbf_hard_limits import (
    FR3_VEL_Q_REF_LOWER,
    FR3_VEL_Q_REF_UPPER,
    apply_slew_limit,
    position_velocity_accel_box,
)
from franka_experiments.utils.cbf_zones import ladder_from_params, ZONE_NAMES
from franka_experiments.utils.cbf_qp_assembly import (
    LEVEL_NAMES,
    OSQP_LEVEL_BOX,
    OSQP_LEVEL_BRAKE,
    OSQP_LEVEL_FULL,
    accept_iterate,
    box_only_solve,
    braking_command,
    build_osqp_A,
    build_osqp_bounds,
    build_row_rhs,
    pad_rows_to_block,
    tangential_bias,
)
from franka_experiments.utils.state_governor import governor_from_params
from franka_experiments.utils.livelock import LivelockDetector, ProgressWindow
from franka_experiments.utils.cbf_state_rows import (
    FR3_JOINT_KEYS,
    FR3_JOINTS,
    G_CAP,
    G_OBS,
    G_QLIM,
    G_SC,
    G_SING,
    G_SPD,
    NV,
    NX,
    N_SLACK,
    ConstraintBuilder,
    JointSnap,
    NomSnap,
    Obstacle,
    ObstacleSnap,
    build_optional_row_builders,
)
from franka_experiments.utils.config import (
    load_cbf_config,
    load_franka_joint_limits,
)
from franka_experiments.utils.kinematics import (
    CBFKinematics,
    build_urdf_no_hand,
)
from franka_experiments.utils.logging_utils import (
    format_cbf_diag,
    format_velocity_summary,
)
from franka_experiments.utils.perception_msgs import labelled_links


#: Slack price of the task-space speed family once the SSM rows drive it
#: (roadmap Step 5, **[E]**). Above ``rho_slack`` (1000.0) on purpose: below it
#: the obstacle rows could relax the separation-distance cap by paying for it.
#: Not a YAML parameter — it is a fixed relation to ``rho_slack``, and exposing
#: it would invite tuning the one number that has to stay on the correct side.
_ISO_RHO_SLACK_SPEED = 2000.0


class CBFSafetyFilter(Node):

    # ═════════════════════════════════════════════════════════════════════
    #  Construction
    # ═════════════════════════════════════════════════════════════════════

    def __init__(self):
        super().__init__('cbf_safety_filter')

        # ── 1. Configuration ────────────────────────────────────────────
        # Every knob comes from config/fr3_control.yaml and from nowhere else;
        # utils.config.CBF_PARAM_SPEC carries only the type and the validation
        # range. A missing key is a startup failure naming the key.
        topics, P = load_cbf_config(self)
        self.P = P
        # ── 1a. ISO 10218 layer ─────────────────────────────────────────
        # Entirely inert with iso_enabled: false, which is the shipped default.
        # Everything it does — the d_safe floor, the S_h parameterisation, the
        # PFL ceiling, the SSM rows' slack price — is applied HERE, on P, before
        # anything reads it, so there is exactly one place to look for "what did
        # the flag change".
        self._iso_configure(P)

        # ── 2. State limits: franka_description, then the firmware ──────
        # NOT from the joint_limits: block at the bottom of fr3_control.yaml —
        # that one is read by four other nodes and the two can drift.
        jl = load_franka_joint_limits(FR3_JOINT_KEYS)
        # Acceleration authority, capped at what the arm can actually produce.
        # decel_max is the braking number and the only q̈ scale joint_limits.yaml
        # carries; symmetrically it claims 17 rad/s² on joints 5 and 7 against a
        # rated 10 (libfranka kMaxJointAcceleration). See qddot_max_abs in
        # fr3_control.yaml for the measurement that made this necessary.
        qdd_cap = np.minimum(jl['decel_max'], P.qddot_max_abs)
        self._lb, self._ub = -qdd_cap, qdd_cap
        self._qdot_max = jl['qdot_max']

        # EFFECTIVE position limits, not the mechanical ones. The firmware's
        # velocity envelope reaches zero at a reference position that lies
        # INSIDE the mechanical stop — 4.5205 rad on joint6 against a 4.6216
        # limit, −3.0481 on joint4 against −3.0770 — so the last 0.03…0.10 rad
        # of every joint admit no motion at all: enter them moving and the
        # answer is `joint_velocity_violation`, not a soft landing.
        #
        # Anchoring the joint-limit ROWS at the mechanical limit therefore put
        # the barrier behind the wall. Run 20260915_080040 is the demonstration:
        # joint6 aborted at 4.4644 rad and +0.489 rad/s, where the firmware
        # admitted +0.435 — and the row, measuring h against 4.6216, saw 0.157
        # rad of headroom and never fired. Rows and box now share the same wall
        # as the hard envelope in cbf_hard_limits, and it is the real one.
        self._q_min = np.maximum(jl['q_min'], FR3_VEL_Q_REF_LOWER)
        self._q_max = np.minimum(jl['q_max'], FR3_VEL_Q_REF_UPPER)

        # ── 3. Kinematics and the row builders ──────────────────────────
        kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
        opt = build_optional_row_builders(P, kin, self.get_logger())

        # The one object that turns a (joint state, obstacle) pair into rows.
        # Stateful — barrier smoothing, per-track velocity filters, frame
        # counters — and driven at the constraint rate, never on the QP tick.
        self._rows = ConstraintBuilder(
            P, kin, q_min=self._q_min, q_max=self._q_max,
            acc_lb=self._lb, acc_ub=self._ub, logger=self.get_logger(),
            qdot_max=self._qdot_max, **opt)

        # ── 4. QP, preallocated once ────────────────────────────────────
        # Slack penalty is QUADRATIC (½ρs²) and one slack per FAMILY. Quadratic
        # is C¹ at s=0 and prices small violations softly; per-family because a
        # single shared slack let a joint-limit row in RADIANS relax every
        # self-collision row in METRES by the same amount, until the firmware
        # fired its own reflex.
        #
        # ISO layer (roadmap Step 5): with the SSM rows on, the speed family's
        # slack is priced ABOVE the obstacle family's (2000 vs rho_slack=1000)
        # so the barrier can no longer buy its way past the separation-distance
        # cap with slack. Still a PRICE, not a hard row: a hard speed row plus
        # the state box can be infeasible exactly when the arm is already over
        # the cap. What makes the bound enforced rather than preferred is
        # iso_safety_monitor, one channel over. [E]
        rho_spd = P.rho_slack_link_speed
        if P.iso_enabled and P.iso_ssm_speed_rows:
            rho_spd = max(rho_spd, _ISO_RHO_SLACK_SPEED)
            self.get_logger().info(
                f'ISO SSM speed rows ON: rho_slack_link_speed '
                f'{P.rho_slack_link_speed:.0f} -> {rho_spd:.0f} '
                f'(above rho_slack={P.rho_slack:.0f})')
        P_mat = np.eye(NX)
        for g, rho in ((G_OBS,  P.rho_slack),
                       (G_SC,   P.rho_slack_self_collision),
                       (G_QLIM, P.rho_slack_joint_limit),
                       (G_SING, P.rho_slack_singularity),
                       (G_CAP,  P.rho_slack_retreat),
                       (G_SPD,  rho_spd)):
            P_mat[NV + g, NV + g] = rho
        self._P_csc = sparse.csc_matrix(P_mat)
        self._qvec = np.zeros(NX)
        self._box_lb = np.concatenate([self._lb, np.zeros(N_SLACK)])
        self._box_ub = np.concatenate([self._ub, np.full(N_SLACK, 1e6)])
        self._osqp_prob = None
        self._prev_nc = -1
        self._qp_fail_count = 0
        self._fallback_count = [0, 0, 0]      # ticks answered at each rung
        self._last_level = OSQP_LEVEL_FULL
        # Phase-3c livelock detector. Owns WHEN and HOW MUCH; the builder puts
        # WHICH WAY into the snapshot (livelock_dir).
        self._livelock = LivelockDetector(
            stall_s=P.livelock_stall_s, ramp_s=P.livelock_ramp_s,
            max_s=P.livelock_max_s, cooldown_s=P.livelock_cooldown_s)
        self._lock_mag = 0.0
        self._lock_was = False
        self._lock_bias = np.zeros(NV)   # EMA state, like the other two biases
        self._progress = ProgressWindow(P.livelock_progress_window_s)
        self._dt_qp = 1.0 / P.qp_rate_hz
        # Zone ladder for the TASK SWITCH only. The constraint builder owns a
        # second instance for the row gains: two rates, two pieces of state,
        # one factory (utils.cbf_zones.ladder_from_params) so they cannot drift
        # apart in configuration. None when the flag is off.
        self._zones = ladder_from_params(P, self.get_logger())
        if self._zones is not None:
            self.get_logger().info(
                f'zone ladder (x d_safe={P.d_safe} m): {self._zones.describe()}')
        self._zone_held = False     # edge detector for the hold-zone log line
        self._diag_w_task = 1.0     # last task weight, for CBFDIAG
        # State governor: the SECOND task switch, keyed on the arm's own state
        # (velocity envelope, sigma_min, self-collision gap) rather than on the
        # obstacle gap. Composes multiplicatively with the ladder's weight —
        # they attenuate the same thing for independent reasons, so the tighter
        # of the two must win and a product of two [0, 1] scalars gives that
        # without either needing to know about the other.
        self._gov = governor_from_params(P, self.get_logger())
        self._gov_held = False      # edge detector for the suspension log line
        self._diag_gov = None       # last GovernorState, for CBFDIAG

        # ── 5. Shared state (lock-free: one immutable snapshot per producer,
        #       published by a single atomic attribute assignment) ────────
        self._js = self._nom = self._obs = self._con = None
        self._js_frozen_since = None
        self._qdot_cbf = np.zeros(NV)
        self._qddot_prev = np.zeros(NV)
        self._nom_prev = np.zeros(NV)   # last tick's (biased) nominal, for the livelock test
        self._tan_bias = np.zeros(NV)   # EMA state for tangential_bias, below
        self._esc_bias = np.zeros(NV)   # EMA state for the lateral-evasion bias
        self._outrun_bias = np.zeros(NV)  # EMA state for the Phase-2 escape bias

        # ── 6. Diagnostics ──────────────────────────────────────────────
        self._diag_slack = np.zeros(N_SLACK)
        self._diag_vel_ratio = np.zeros(NV)
        self._diag_vel_bite = np.zeros(NV, dtype=bool)
        self._diag_slew_step = np.zeros(NV)
        self._diag_slew_bite = np.zeros(NV, dtype=bool)
        self._diag_qddot_real = np.zeros(NV)
        self._diag_qdot_prev = self._diag_t_prev = None
        self._diag_cap_age = 0.0
        self._diag_caps = (0.0, 0.0, 0.0, 0.0)
        self._cap_warned = False
        self._last_diag_t = 0.0
        self._last_tick_t = None
        self._tick_count = 0
        self._priority_set = False
        self._gap_warn_thr_ms = P.tick_gap_warn_factor * 1000.0 * self._dt_qp

        # ── 7. ROS wiring ───────────────────────────────────────────────
        # ── Callback groups: QP loop must never wait on perception ──────────
        grp_io   = MutuallyExclusiveCallbackGroup()
        grp_perc = MutuallyExclusiveCallbackGroup()
        grp_ctrl = MutuallyExclusiveCallbackGroup()

        # depth=1: always consume the latest sample, never drain a backlog
        # joint_states_fast, NOT joint_states: the latter comes from a 30 Hz
        # Python republisher that re-stamps its CACHED values, so a stall in it
        # is invisible to every staleness check here (see the topics block in
        # fr3_control.yaml). Falls back to the old key if the config predates it.
        js_topic = topics.get('joint_states_fast',
                              topics['joint_states_topic'])
        self.create_subscription(
            JointState, js_topic, self._on_joint_state,
            QoSProfile(depth=1), callback_group=grp_io)
        self.create_subscription(
            Float64MultiArray, topics['qddot_nom'], self._on_qddot_nom,
            QoSProfile(depth=1), callback_group=grp_io)
        self.create_subscription(
            MultiLinkDistance, topics['per_link_distances'], self._on_distances,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=grp_io)

        self._pub = self.create_publisher(
            Float64MultiArray, topics['qddot_safe'], 10)

        # CBF activity status for downstream consumers.
        # data = [n_active_constraints, slack, fault_braking, n_active_cps,
        #         d_obstacle_min].
        # data[4] is the closest OBSTACLE surface gap [m], +inf when nothing is
        # in range. It is NOT min(h_bar) + d_safe: h_bar mixes obstacle rows
        # with joint-limit rows measured in radians. pentagon_qddot_commander
        # reads it to scale its phase rate — slow the trajectory down only when
        # something is genuinely close, never on tracking error (an error-driven
        # governor deadlocks: CBF blocks -> error grows -> phase freezes ->
        # reference parked on the blocked pose -> CBF keeps blocking).
        # data[3] (n_active_cps) counts the rows whose barrier is actually
        # VIOLATED (h̄ < 0, i.e. the CP is inside d_safe), which is the honest
        # "how many control points triggered this cycle" figure. data[0] is the
        # larger count of rows PRESENT in the QP — every CP inside
        # cbf_obstacle_horizon, most of them non-binding. Both consumers
        # (frame_grabber, rl_policy_commander) index positionally behind a
        # len() guard, so appending a 4th element is backward compatible.
        # Today the only subscriber is frame_grabber.py, which uses it to gate
        # frame saving (save while CBF active or fault-braking). NOTE: no
        # consumer currently freezes virtual time on this signal — a "freeze
        # virtual time while CBF active" motion generator is planned (roadmap)
        # but not yet implemented; do not read this comment as if it happens.
        # fault_braking=1 marks a SAFETY-CHAIN fault that forced braking with no
        # geometric CBF rows (distance stale, or QP failure) — distinct from
        # n_active_constraints=0 during normal "no obstacle nearby" operation.
        self._status_pub = self.create_publisher(
            Float64MultiArray, topics.get('cbf_status', '/NS_1/cbf_status'), 10)
        self._status_msg = Float64MultiArray()
        self._status_msg.data = [0.0, 0.0, 0.0, 0.0, float('inf'),
                                 0.0, float('inf'), 0.0, 0.0,
                                 0.0, 0.0]

        # ── ISO layer: the independent monitor's verdict ─────────────────
        # The filter SHAPES; iso_safety_monitor ENFORCES. When it latches, the
        # filter stops filtering and brakes — the rows are not consulted at all,
        # because the situation the monitor found is precisely one the rows were
        # already failing to prevent.
        #
        # Subscribed unconditionally (the topic simply never arrives when the
        # monitor is off) so the wiring does not depend on a flag read at
        # construction. _iso_stop_active() is where the flag is honoured.
        # Empty-frame run (roadmap Step 7). A distance frame with no valid entry
        # is normal when nothing is nearby and a FAULT when something was close
        # a moment ago: the difference is what the last non-empty frame said.
        # Braking-authority fault (roadmap Step 8): consecutive ticks on which
        # the REALIZED acceleration delivered less than iso_brake_frac_min of
        # the commanded one, while an obstacle row was active.
        self._brake_frac_run = 0
        self._brake_frac_warned = False
        self._diag_brake_frac = 1.0
        self._empty_since = None    # monotonic start of the current empty run
        self._empty_close = False   # was the last non-empty frame inside the zone
        self._empty_faulted = False # edge detector for the log line
        self._iso = None            # last [latched, reason, S_p, v_cap, v_cls]
        self._iso_stamp = 0.0
        self._iso_start = self._now()
        self._iso_was_latched = False
        self.create_subscription(
            Float64MultiArray, topics.get('iso_safety', '/NS_1/iso_safety'),
            self._on_iso_safety, QoSProfile(depth=1), callback_group=grp_io)

        # ── 8. Start ────────────────────────────────────────────────────
        # Warm-up FIRST: the one-shot lazy costs (Pinocchio's first FK, OSQP's
        # first factorization, numpy's first BLAS call) stalled the first real
        # tick by 400-800 ms. Pay them here, with the robot stationary.
        self._warmup()
        if P.diag_disable_gc:
            # Diagnostic only — used to test whether _qp_tick gaps coincide with
            # GC pauses. Memory can grow unbounded if cyclic garbage collects.
            gc.disable()
            self.get_logger().warn('diag_disable_gc=TRUE → gc.disable(). '
                                   'TEMPORARY diagnostic mode, not for normal use.')
        self.create_timer(1.0 / P.cbf_update_rate_hz, self._update_constraints,
                          callback_group=grp_perc)
        self.create_timer(1.0 / P.qp_rate_hz, self._qp_tick,
                          callback_group=grp_ctrl)
        self.get_logger().info(
            f'CBF filter  QP={P.qp_rate_hz:.0f} Hz  rows={P.cbf_update_rate_hz:.0f} Hz  '
            f'solver={P.qp_solver}\n'
            f'  config: {P.config_path}\n'
            f'  d_safe={P.d_safe} m  horizon={P.cbf_obstacle_horizon} m  '
            f'k0={P.k0_cbf} k1={P.k1_cbf} rho={P.rho_slack}\n'
            f'  limits from franka_description: q margin={P.position_margin_rad} rad, '
            f'brake_eta={P.position_brake_eta}, qdot at {P.velocity_box_margin:.0%}, '
            f'qddot +-{np.round(self._ub, 2).tolist()} rad/s^2')

    # ═════════════════════════════════════════════════════════════════════
    #  ISO 10218-1/-2:2025 layer — configuration-time, before anything reads P
    # ═════════════════════════════════════════════════════════════════════

    def _iso_configure(self, P) -> None:
        """Apply and CHECK the ISO layer. No-op when ``iso_enabled`` is false.

        Three things happen here, in this order, and all three RAISE rather than
        clamp. A safety envelope that quietly moves when it cannot be met is
        worse than no envelope: it reads like a guarantee and is not one.

        1. **The d_safe floor** (roadmap Step 4). ISO 10218-2:2025 Annex L makes
           ``S_p`` the sum of three motion terms plus ``C + Z_d + Z_r``. The
           motion terms are velocity-dependent and are enforced elsewhere (the
           velocity standoff carries ``S_h``, the SSM speed cap carries
           ``S_r + S_s``); ``C + Z_d + Z_r`` is the IRREDUCIBLE part, the
           distance that is required even from a standstill, and that is what
           ``d_safe`` has to be. Splitting the terms across those mechanisms
           rather than putting all of ``S_p`` into one is **[E]**, a design
           choice; the requirement ``S >= S_p`` itself is **[R]**.

           With the conformant ``iso_c_intrusion = 0.85`` (ISO 13855:2024 body
           detection, which is what an unrated depth pipeline with no
           demonstrated detection capability gets) the floor is 0.92 m and this
           check WILL fail on this cell. That is the correct outcome, not a bug.
           There is no bypass flag and there will not be one. The three
           admissible resolutions are in the message.

        2. **S_h** (Step 4). ``velocity_standoff_time_s`` becomes
           ``T_r + v_h/a_s``: the existing speed-proportional standoff then
           moves the barrier out by ``v_app * (T_r + T_s)``, which IS ``S_h``.
           **[S]** structure, **[E]** parameterisation.

        3. **The PFL ceiling** (Step 9). ``link_speed_max`` must not exceed
           ``iso_v_pfl``, and ``retreat_cap_max_speed`` must stay strictly below
           ``link_speed_max``.
        """
        if not P.iso_enabled:
            return

        # ── 1. d_safe >= C + Z_d + Z_r ──────────────────────────────────
        d_floor = P.iso_c_intrusion + P.iso_z_depth + P.iso_z_robot
        if P.d_safe < d_floor:
            msg = (
                f'ISO 10218-2:2025 Annex L: d_safe={P.d_safe:.3f} m is below the '
                f'irreducible part of the protective separation distance, '
                f'C + Z_d + Z_r = {P.iso_c_intrusion:.3f} + {P.iso_z_depth:.3f} '
                f'+ {P.iso_z_robot:.3f} = {d_floor:.3f} m.\n'
                f'  This is not a tuning problem. The three admissible '
                f'resolutions, in order:\n'
                f'  (a) demonstrate a detection capability d <= 40 mm for the '
                f'depth pipeline (scripts/iso_constants_measure.py detection) '
                f'and set iso_c_intrusion = 8*(d-14) mm;\n'
                f'  (b) raise d_safe to {d_floor:.3f} m — note this exceeds the '
                f'FR3 reach at the conformant C = 0.85 m, i.e. conformant SSM is '
                f'not achievable in this workspace;\n'
                f'  (c) record a research deviation in franka_experiments/'
                f'SAFETY.md and run with iso_enabled: false, making no ISO '
                f'claim.\n'
                f'  There is no bypass flag. See {P.config_path}')
            self.get_logger().error(msg)
            raise ValueError(msg)

        # ── 2. S_h through the existing velocity standoff ───────────────
        # T_s = v_h/a_s here rather than v_r/a_s: the standoff is evaluated once
        # per rebuild against the OBSTACLE's closing speed, and bounding the
        # robot's own stopping time by the human approach speed is the roadmap's
        # [E] parameterisation, not a term of the standard.
        t_stand = P.iso_t_reaction + P.iso_v_human / max(P.iso_a_stop, 1e-9)
        t_max = 5.0                       # CBF_PARAM_SPEC velocity_standoff_time_s
        m_max = 2.0                       # CBF_PARAM_SPEC velocity_standoff_max
        s_h_max = P.iso_v_human * t_stand
        if t_stand > t_max or s_h_max > m_max:
            self.get_logger().warn(
                f'ISO S_h clamped: T_r + v_h/a_s = {t_stand:.2f} s and '
                f'v_h*(T_r+T_s) = {s_h_max:.2f} m exceed the declared ranges '
                f'({t_max} s / {m_max} m). The standoff will carry LESS than '
                f'S_h — record the shortfall in SAFETY.md.')
        P.enable_velocity_standoff = True
        P.velocity_standoff_time_s = min(t_stand, t_max)
        P.velocity_standoff_max = min(s_h_max, m_max)

        # ── 3. PFL / reduced-speed ceilings (Step 9) ────────────────────
        if P.link_speed_max > P.iso_v_pfl:
            msg = (
                f'ISO 10218-2:2025 Annex M: link_speed_max='
                f'{P.link_speed_max:.3f} m/s exceeds iso_v_pfl='
                f'{P.iso_v_pfl:.3f} m/s, the power-and-force-limited speed. The '
                f'ceiling is not clamped silently — relaunch with '
                f'link_speed_max:={P.iso_v_pfl:.2f} '
                f'retreat_cap_max_speed:={0.9 * P.iso_v_pfl:.2f}, or lower both '
                f'in {P.config_path}. Keep retreat_cap_max_speed strictly below '
                f'link_speed_max.')
            self.get_logger().error(msg)
            raise ValueError(msg)
        if P.retreat_cap_max_speed >= P.link_speed_max:
            msg = (
                f'retreat_cap_max_speed={P.retreat_cap_max_speed:.3f} must stay '
                f'strictly below link_speed_max={P.link_speed_max:.3f}: the '
                f'retreat cap is the inner bound, the speed row the outer one, '
                f'and inverting them makes the outer row unreachable.')
            self.get_logger().error(msg)
            raise ValueError(msg)

        # ── 4. Report what the flag actually changed ────────────────────
        self.get_logger().warn(
            f'ISO 10218 layer ACTIVE (mode={P.iso_mode}) — this is an '
            f'ISO-ALIGNED research configuration, NOT a certified safety '
            f'function. See franka_experiments/SAFETY.md.\n'
            f'  d_safe={P.d_safe:.3f} m >= C+Z_d+Z_r={d_floor:.3f} m '
            f'(C={P.iso_c_intrusion:.3f} Z_d={P.iso_z_depth:.3f} '
            f'Z_r={P.iso_z_robot:.3f})\n'
            f'  S_h via velocity standoff: time_s={P.velocity_standoff_time_s:.3f} s '
            f'max={P.velocity_standoff_max:.3f} m '
            f'(T_r={P.iso_t_reaction:.3f} s, v_h={P.iso_v_human:.2f} m/s, '
            f'a_s={P.iso_a_stop:.2f} m/s^2)\n'
            f'  S_r+S_s via SSM speed rows: '
            f'{"ON" if P.iso_ssm_speed_rows else "off"}\n'
            f'  independent monitor: '
            f'{"ON" if P.iso_monitor_enabled else "off"}  '
            f'link_speed_max={P.link_speed_max:.3f} <= v_PFL={P.iso_v_pfl:.3f} m/s')

    def _warmup(self) -> None:
        """Run every per-tick code path once with dummy data, results discarded.

        Measured in the logs: the first real _qp_tick / _update_constraints after
        startup stalled ~400-800 ms (the tick=442 solve=391 ms event, then an
        830 ms gap), all during pentagon's WARMUP phase with the robot stationary.
        Cause is one-shot lazy cost paid on first use: first Pinocchio FK/Jacobian
        on a never-resolved frame, first _frame_id() cache miss, first OSQP setup
        + lazy library init, cold memory pages. Doing it here moves that cost into
        node construction — where the robot is not moving and a delay has no
        safety/motion consequence — instead of onto the first live control tick.

        Nothing is published (no _pub / _status_pub); the node's real-tick OSQP
        state (_osqp_prob / _prev_nc) is left untouched by using throwaway OSQP
        instances. Any sub-step that fails only logs a warning and is skipped, so
        warm-up can never block node startup.
        """
        t0      = time.perf_counter()
        q0      = np.zeros(NV)
        qdot0   = np.zeros(NV)
        p_dummy = np.array([0.3, 0.0, 0.5])      # plausible workspace point

        # (a) Pinocchio FK + Jacobian internal structures. with_jdot=True so the
        #     Jacobian-time-variation pass (used per real tick now) is also warmed.
        try:
            self._rows._kin.update(q0, qdot0, with_jdot=True)
        except Exception as exc:
            self.get_logger().warn(f'warmup: kin.update failed: {exc}')

        # (c) Populate _fid_cache for every link that can appear as
        #     robot_link_name in MultiLinkDistance (fr3_link0..link8), and
        # (b) warm the point-Jacobian path on the first link that resolves.
        first_fid = None
        for link in (f'fr3_link{i}' for i in range(9)):
            try:
                fid = self._rows._frame_id(link)
                if fid is not None and first_fid is None:
                    first_fid = fid
            except Exception as exc:
                self.get_logger().warn(f"warmup: _frame_id('{link}') failed: {exc}")
        if first_fid is not None:
            try:
                self._rows._kin.point_jacobian(first_fid, p_dummy)   # warms J and J̇ paths
            except Exception as exc:
                self.get_logger().warn(f'warmup: point_jacobian failed: {exc}')
        else:
            self.get_logger().warn('warmup: no robot link resolved — Jacobian path not warmed')

        # (d) Exercise the native-OSQP paths used in _qp_tick for both n_c=0 and
        #     n_c=1 (setup + update(Ax) + solve), on throwaway problems. Also pays
        #     osqp's lazy library-init cost here rather than on the first tick.
        try:
            l0, u0 = build_osqp_bounds(None, None, self._box_lb, self._box_ub)
            prob0 = osqp.OSQP()
            prob0.setup(P=self._P_csc, q=self._qvec, A=build_osqp_A(None, NV, N_SLACK),
                        l=l0, u=u0, warm_start=True,
                        max_iter=self.P.osqp_max_iter, verbose=False)
            prob0.solve()

            G1 = np.zeros((1, NX)); G1[0, NV + G_OBS] = -1.0   # dummy row
            h1 = np.array([1.0])
            l1, u1 = build_osqp_bounds(G1, h1, self._box_lb, self._box_ub)
            prob1 = osqp.OSQP()
            prob1.setup(P=self._P_csc, q=self._qvec, A=build_osqp_A(G1, NV, N_SLACK),
                        l=l1, u=u1, warm_start=True,
                        max_iter=self.P.osqp_max_iter, verbose=False)
            prob1.update(q=self._qvec, l=l1, u=u1, Ax=build_osqp_A(G1, NV, N_SLACK).data)
            prob1.solve()
        except Exception as exc:
            self.get_logger().warn(f'warmup: OSQP path failed: {exc}')

        dt_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(
            f'warmup complete in {dt_ms:.1f} ms — one-shot lazy costs paid in '
            f'__init__ (robot stationary), not on the first control tick')

    def _on_joint_state(self, msg: JointState) -> None:
        n2p = dict(zip(msg.name, msg.position))
        n2v = dict(zip(msg.name, msg.velocity))
        try:
            q    = np.array([n2p[n] for n in FR3_JOINTS])
            qdot = np.array([n2v[n] for n in FR3_JOINTS])
        except KeyError:
            return
        prev = self._js
        if (prev is not None
                and np.array_equal(q, prev.q)
                and np.array_equal(qdot, prev.qdot)):
            # Identical to the previous message. Remember when the run STARTED
            # (the previous message's stamp), so the QP thread can age it.
            if self._js_frozen_since is None:
                self._js_frozen_since = prev.stamp
        else:
            self._js_frozen_since = None
        self._js = JointSnap(q, qdot, self._now())

        # ── DIAGNOSTIC ONLY — realized q̈ estimate (see __init__; NOT control) ─
        # Finite difference of MEASURED q̇ → q̈_real, EMA-smoothed. Δt uses the
        # ROS header stamp (sensor/controller time), NOT receipt wall-time:
        # callback-scheduling jitter would corrupt the derivative (a late
        # callback inflates Δt and deflates the estimate). Guarded to a sane
        # joint_states interval so a dropped/duplicated stamp can't blow it up.
        t_hdr = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._diag_qdot_prev is not None and self._diag_t_prev is not None:
            dt = t_hdr - self._diag_t_prev
            if 1e-4 < dt < 0.1:
                raw = (qdot - self._diag_qdot_prev) / dt
                a   = self.P.diag_qddot_alpha
                self._diag_qddot_real = a * self._diag_qddot_real + (1.0 - a) * raw
        self._diag_qdot_prev = qdot
        self._diag_t_prev    = t_hdr

    def _on_qddot_nom(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float64)
        if data.shape == (NV,):
            self._nom = NomSnap(data, self._now())

    def _on_distances(self, msg: MultiLinkDistance) -> None:
        P = self.P
        # The four track fields are read UNCONDITIONALLY, even in 'residual'
        # mode. They are cheap (one Vector3 and one 9-vector already in the
        # message) and reading them here rather than behind the mode switch
        # means the diagnostic line can report what the tracker is saying while
        # the barrier is still driven by the residual — which is exactly how the
        # two get compared on hardware before the source is flipped.
        # A publisher that knows nothing about tracking sends zeros, and zeros
        # are the documented "no track" state that contributes nothing.
        #
        # The control-point label comes from perception_msgs.labelled_links,
        # the ONE definition of the convention — the publisher annotates
        # skip_keys through the same function. It is positional over every
        # entry, invalid ones included, and the label is carried on
        # Obstacle.cp_label rather than rebuilt inside the ConstraintBuilder,
        # which is what used to make it depend on which rows survived the
        # builder's filters. See that function for what went wrong.
        parsed = []
        for cp_label, ld in labelled_links(msg):
            if not ld.valid:
                continue
            parsed.append(Obstacle(
                link=ld.robot_link_name,
                d=float(ld.distance),
                pr=np.array([ld.closest_point_robot.x,
                             ld.closest_point_robot.y,
                             ld.closest_point_robot.z]),
                ph=np.array([ld.closest_point_human.x,
                             ld.closest_point_human.y,
                             ld.closest_point_human.z]),
                conf=float(ld.confidence),
                v_vec=np.array([ld.obstacle_velocity.x,
                                ld.obstacle_velocity.y,
                                ld.obstacle_velocity.z]),
                frames_seen=int(ld.frames_seen),
                vel_cov=np.asarray(ld.velocity_covariance,
                                   dtype=np.float64).reshape(3, 3),
                track_id=int(ld.track_id),
                cp_label=cp_label,
                **self._latency_fields(ld),
            ))
        items = tuple(parsed)
        now = self._now()
        # Capture time, with a plausibility guard: an unset header stamp reads
        # as 0.0 and would make Δt ≈ 1.8e9 s, silently zeroing every velocity
        # estimate; a clock skew the other way would make Δt negative. Accept it
        # only when it sits in a sane window behind `now`, else fall back to the
        # receipt time (which reproduces the pre-fix behaviour exactly).
        t_cap = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now - t_cap
        if not (-P.distance_capture_skew_tol < age < P.distance_capture_age_max):
            if t_cap != 0.0 or self._cap_warned is False:
                self.get_logger().warn(
                    f'per_link_distances header stamp implausible '
                    f'(age={age:.3f} s) → falling back to receipt time; '
                    f'v_obs keeps the pre-fix jitter',
                    throttle_duration_sec=5.0)
                self._cap_warned = True
            t_cap = now
        self._diag_cap_age = age
        self._obs = ObstacleSnap(items, now, t_cap)

        # ── Empty-frame run (roadmap Step 7) ────────────────────────────
        # Perception keeps publishing (the heartbeat is deliberate — it is how a
        # consumer tells "nothing near" from "perception dead"), so an empty
        # frame does NOT age `obs` and does NOT trip the staleness check. That
        # is correct when nothing is nearby and wrong when something was: the
        # arm then runs with zero obstacle rows against an obstacle that was
        # inside the active zone one frame ago, and nothing anywhere says so.
        #
        # The memory is what makes this cheap: remember whether the last
        # NON-EMPTY frame was close, and fault only on a run of empty frames
        # that follows one. An empty run after a far-away frame is just an empty
        # scene and costs nothing.
        if items:
            self._empty_since = None
            self._empty_faulted = False
            d_min = min((ob.d for ob in items), default=float('inf'))
            self._empty_close = d_min < P.zone_r_active * P.d_safe
        elif self._empty_since is None:
            self._empty_since = now

    @staticmethod
    def _latency_fields(ld) -> dict:
        """The three latency-compensation fields of a LinkDistance, or an empty
        dict when the message package predates them (the Obstacle defaults are
        then None and the term evaluates to its velocity-only floor)."""
        if not hasattr(ld, 'obstacle_acceleration'):
            return {}
        return dict(
            a_vec=np.array([ld.obstacle_acceleration.x,
                            ld.obstacle_acceleration.y,
                            ld.obstacle_acceleration.z]),
            pos_cov=np.asarray(ld.position_covariance, dtype=np.float64).reshape(3, 3),
            pv_cov=np.asarray(ld.position_velocity_covariance,
                              dtype=np.float64).reshape(3, 3))

    # ═════════════════════════════════════════════════════════════════════
    #  Perception rate (50 Hz) — geometry only; Pinocchio lives here
    # ═════════════════════════════════════════════════════════════════════

    def _update_constraints(self) -> None:
        js, obs = self._js, self._obs
        if js is not None and obs is not None:
            self._con = self._rows.build(js, obs, self._now())

    def _elevate_thread_priority(self) -> None:
        """Put *only* the calling (QP) thread on SCHED_FIFO at moderate priority.

        On Linux, scheduling policy/priority are per-thread and sched_setscheduler
        targets a kernel TID; passing threading.get_native_id() (this thread's
        TID) elevates ONLY this thread — the grp_io / grp_perc executor threads
        keep default scheduling. Must be called from inside the QP callback, not
        __init__: the executor spawns the per-group thread only after spin().

        Priority 50 (mid SCHED_FIFO range) deliberately leaves headroom above it
        for the real 1 kHz robot controller, which must outrank this Python node.
        Degrades gracefully: if the process may not raise its RT priority the
        node keeps running at default scheduling (logs a warning, no crash).

        Privilege comes from an RTPRIO rlimit, NOT a capability: this is the same
        real-time setup libfranka already requires. With the user in a `realtime`
        group and /etc/security/limits.conf granting
            @realtime  -  rtprio  99
            @realtime  -  memlock unlimited
        an unprivileged process may call sched_setscheduler(SCHED_FIFO, prio) for
        any prio ≤ the soft RTPRIO limit — no CAP_SYS_NICE, no root. Prefer this
        over `setcap cap_sys_nice+ep` on the interpreter: file caps mark python3
        as AT_SECURE, which strips LD_LIBRARY_PATH/PYTHONPATH and breaks ROS 2's
        env-based library/package discovery. Verify the limit with `ulimit -r`
        (already confirmed 99 on this machine). Do NOT request root here.
        """
        try:
            tid = threading.get_native_id()          # this QP thread's kernel TID
            os.sched_setscheduler(tid, os.SCHED_FIFO, os.sched_param(50))
            self.get_logger().info(
                f'_qp_tick thread (tid={tid}) elevated to SCHED_FIFO prio 50')
        except PermissionError:
            self.get_logger().warn(
                'Cannot set SCHED_FIFO (RTPRIO rlimit too low) — _qp_tick runs '
                'at default scheduling. Add the user to the `realtime` group and '
                'set `rtprio 99` in /etc/security/limits.conf (same setup as '
                'libfranka); check with `ulimit -r`.')
        except Exception as exc:
            self.get_logger().warn(f'_elevate_thread_priority failed: {exc}')

    # ═════════════════════════════════════════════════════════════════════
    #  Control rate (100 Hz) — read snapshots, solve, publish
    # ═════════════════════════════════════════════════════════════════════

    def _qp_tick(self) -> None:
        P = self.P
        if not self._priority_set:                      # one-shot, needs the
            self._priority_set = True                   # executor's own thread
            self._elevate_thread_priority()
        self._watch_tick_gap()

        js = self._js
        if js is None:
            return
        now = self._now()

        # ── STEP 0: the ISO layer's stop, above every other branch ──────
        # Ahead of the staleness policy on purpose: when the independent monitor
        # says the separation-distance bound was crossed, there is nothing for
        # the rows to contribute — they are the mechanism that just failed to
        # keep the arm on the right side of it. So the rows are skipped entirely
        # and the arm brakes on q̇ with the stop profile's own time constant,
        # through the SAME box_only_solve the QP fallback ladder uses, so the
        # state box still clips it to ±decel_max.
        #
        # NOT zeros. q̈ = 0 reads as "hold this velocity" all the way down the
        # chain, and with gravity added by the firmware a zero TORQUE is
        # coasting: the arm would sail through the stop it was just told to make.
        # This is the same lesson the joint-state-stale branch below records,
        # and it has been measured twice on this hardware.
        if self._iso_stop_active(now):
            if not self._iso_was_latched:
                self._iso_was_latched = True
                self.get_logger().error(
                    'ISO SSM stop active → braking on q̈ = −q̇/iso_stop_tau '
                    f'(tau={P.iso_stop_tau:.3f} s), CBF rows skipped. '
                    'NON-SAFETY-RATED: this is not a protective stop.')
            qddot_stop = np.clip(-js.qdot / P.iso_stop_tau, self._lb, self._ub)
            self._qvec[:NV] = -qddot_stop
            self._box_lb[:NV], self._box_ub[:NV] = self._lb, self._ub
            x = box_only_solve(self._P_csc, self._qvec, self._box_lb,
                               self._box_ub, max_iter=P.osqp_max_iter)
            if x is None:
                x = np.concatenate([qddot_stop, np.zeros(N_SLACK)])
            self._fallback_count[OSQP_LEVEL_BRAKE] += 1
            self._last_level = OSQP_LEVEL_BRAKE
            self._publish(x[:NV])
            self._publish_status(0, 0.0, fault=1.0, n_act=0)
            return
        if self._iso_was_latched:
            self._iso_was_latched = False
            self.get_logger().warn('ISO SSM stop cleared → filtering resumed')

        # ── STEP 1: is the state usable at all? ─────────────────────────
        # Both failures degrade to braking on the last known q̇. Zeros would
        # NOT: q̈ = 0 reads as "hold this velocity" all the way down the chain,
        # so the arm coasts through the outage — measured, twice, ending in a
        # joint_velocity_violation reflex.
        if now - js.stamp > P.joint_state_timeout:
            self.get_logger().error('joint state stale → braking on last known q̇',
                                    throttle_duration_sec=0.5)
            self._publish(np.clip(-P.k_brake * js.qdot, self._lb, self._ub))
            self._publish_status(0, 0.0, fault=1.0, n_act=0)
            return
        qdot = js.qdot
        js_frozen = self._joint_state_frozen(now, qdot)

        # ── STEP 2: the nominal command we are filtering ────────────────
        nom = self._nom
        fresh_nom = nom is not None and now - nom.stamp < P.nom_timeout
        if not fresh_nom:
            self.get_logger().warn('qddot_nom stale → braking',
                                   throttle_duration_sec=2.0)
        qddot_nom = nom.qddot if fresh_nom else -P.k_brake * qdot
        # The task's OWN command, before any bias this filter adds — the
        # livelock test needs to know what the commander wanted, not what the
        # filter has already talked itself into.
        nom_raw = np.array(qddot_nom, copy=True)
        if js_frozen:
            qddot_nom = -P.k_brake * qdot           # overrides even a fresh one

        # ── STEP 3: the CBF rows, and their right-hand side ─────────────
        # The rows themselves were built at 50 Hz; only their RHS is refreshed
        # here, with this tick's q̇.
        con, obs = self._con, self._obs
        G = h_qp = None
        n_c = n_active = 0
        fault = 0.0
        # Checked BEFORE the row branch, not after: with joint-limit rows on, a
        # run of empty distance frames still produces a non-empty, perfectly
        # fresh snapshot — of joint-limit rows only. That snapshot would take
        # the first branch and the filter would sail past the fault with no
        # obstacle row in the QP, which is the exact state this detects.
        empty_fault = self._empty_frame_fault(now)
        if js_frozen:
            fault = 1.0                             # blind: no rows, brake
        elif empty_fault:
            # Same degradation as a stale frame, for the same reason: the
            # barrier has no obstacle geometry and must not pass the nominal
            # through.
            qddot_nom = -P.k_brake * qdot
            fault = 1.0
        elif con is not None and now - con.t_dist < P.distance_timeout:
            n_c, G = con.A.shape[0], con.G
            n_active = int(np.count_nonzero(con.h_bar < 0.0))
            self._smooth_qdot(qdot)
            # k0/k1 stay the scalars here; the snapshot's per-row gains take
            # over inside build_row_rhs when the zone ladder put them there.
            h_qp, self._diag_caps = build_row_rhs(
                con, qdot, self._qdot_cbf, k0=P.k0_cbf, k1=P.k1_cbf,
                retreat_horizon=P.retreat_cap_horizon_s,
                speed_horizon=P.link_speed_horizon_s)
        elif obs is not None and now - obs.stamp > P.distance_timeout:
            # Perception was received once and has since gone stale — a failure
            # of the channel that feeds the barrier must degrade toward MORE
            # conservative, never toward silent passthrough. (Obstacles merely
            # out of range keep obs fresh, so they do not land here.)
            qddot_nom = -P.k_brake * qdot
            fault = 1.0
            self.get_logger().warn('distance stale → braking fallback (CBF inactive)',
                                   throttle_duration_sec=2.0)

        # ── STEP 3a: the zone ladder's task switch ──────────────────────
        # Below zone_r_priority*d_safe the trajectory starts giving way, and
        # below zone_r_hold*d_safe it gives way entirely: the nominal is faded into a
        # braking command so the arm sheds its speed and holds station instead
        # of pursuing a path that leads into something.
        #
        # Faded into BRAKING, not into zero. A zero nominal makes the QP
        # minimise ‖q̈‖², whose unconstrained solution is q̈ = 0 — "hold this
        # velocity", i.e. the arm coasts into the obstacle at whatever speed it
        # already had. That is the exact failure the joint-state-stale branch
        # above documents, and it is why "stop" here is a command and not an
        # absence of one.
        #
        # This is NOT a freeze. The CBF rows are untouched and keep their full
        # authority, so an obstacle that goes on closing is still pushed away
        # from; only the TASK is switched off. That distinction is load-bearing
        # for a 5 cm boundary read by a depth sensor whose calibration residual
        # measured 4.2 cm with 4.2 cm of spread: a latch there would fire on
        # noise, and a frozen arm is the wrong answer to a limb still moving.
        #
        # Placed BEFORE the three evasion biases on purpose. Those biases are
        # avoidance, not task, and attenuating them in the zone where avoidance
        # matters most would be precisely backwards.
        w_task = 1.0
        if self._zones is not None:
            # Global gap, not per row: there is one trajectory to switch off.
            # inf on every braking path — n_c is 0 there while `con` still
            # holds the last snapshot, and the nominal is ALREADY the braking
            # command, so a second opinion off stale geometry would only fight
            # it. The ramp still advances, so the task fades back in normally
            # once rows return.
            d_zone = (float(con.d_obs_min) if (n_c > 0 and con is not None)
                      else float('inf'))
            w_task = self._zones.task_weight(d_zone, self._dt_qp)
            if w_task < 1.0:
                qddot_nom = (w_task * qddot_nom
                             + (1.0 - w_task) * (-P.k_brake * qdot))
                if w_task <= 0.0 and not self._zone_held:
                    self.get_logger().warn(
                        f'ZONE hold: obstacle at {d_zone:.3f} m < '
                        f'{self._zones.bounds[0]:.3f} m → trajectory suspended, barrier '
                        f'rows keep full authority (resumes over '
                        f'{P.zone_resume_s:.1f} s once it clears)')
                    self._zone_held = True
            if w_task >= 1.0 and self._zone_held:
                self.get_logger().info('ZONE hold released → trajectory resumed')
                self._zone_held = False

        # ── STEP 3a+: the state governor's task switch ──────────────────
        # Same lever as 3a, different reason. The ladder asks "is something
        # coming at me"; this asks "can I still do what I am being told". A
        # joint about to cross its firmware velocity envelope, a collapsing
        # sigma_min, a closing self-collision gap: each makes the nominal
        # infeasible on its own, with no obstacle involved, and the answer to an
        # infeasible demand is to stop making it — not to push harder against
        # the constraint that is refusing it.
        #
        # Multiplied into w_task rather than min()'d with it: the two switches
        # are independent and a product is the only combination where each
        # keeps its full effect regardless of the other's state.
        #
        # Placed with 3a and BEFORE the evasion biases, for the reason 3a gives:
        # those are avoidance, not task, and attenuating avoidance is backwards.
        # Note what this does NOT do: it does not touch the rows, the slack
        # prices, or /cbf_status. See utils/state_governor.py.
        if self._gov is not None:
            # sigma and d_sc come from the 50 Hz rebuild; the velocity margin is
            # computed here from THIS tick's state. Gated on n_c > 0, the same
            # test the ladder uses for d_zone and for the same reason: on every
            # braking path `con` still holds the last snapshot, and governing off
            # geometry the QP itself has already refused to use would only add a
            # second opinion formed on stale data. The velocity term is ungated —
            # it reads js, which STEP 1 has already established is fresh.
            #
            # Note the fade is arithmetically inert on those paths anyway: the
            # nominal IS -k_brake*qdot there, and fading a braking command into a
            # braking command is the identity. The gate is about the ramp state
            # and the diagnostic, not about the command.
            snap = con if n_c > 0 and con is not None else None
            gov = self._gov.weight(
                q=js.q, qdot=qdot,
                sigma=(getattr(self._rows, 'diag_sigma', None)
                       if snap is not None else None),
                d_sc=(snap.d_sc_min if snap is not None else None),
                dt=self._dt_qp)
            self._diag_gov = gov
            if gov.w < 1.0:
                qddot_nom = (gov.w * qddot_nom
                             + (1.0 - gov.w) * (-P.k_brake * qdot))
                w_task *= gov.w
                if gov.w <= 0.0 and not self._gov_held:
                    self.get_logger().warn(
                        f'GOVERNOR hold: {gov.binding} margin exhausted '
                        f'(vel={gov.margins[0]:+.3f} rad/s, '
                        f'sigma={gov.margins[1]:+.3f}, '
                        f'd_sc={gov.margins[2]:+.3f} m) → trajectory suspended, '
                        f'rows keep full authority '
                        f'(resumes over {P.governor_resume_s:.1f} s)')
                    self._gov_held = True
            if gov.w >= 1.0 and self._gov_held:
                self.get_logger().info('GOVERNOR hold released → trajectory resumed')
                self._gov_held = False
        self._diag_w_task = w_task

        # ── STEP 3b: steer around, not just away from, a close obstacle ─
        # Bias only, on the QP's TARGET — every G/h row above is untouched, so
        # this cannot loosen the safety guarantee, only shift which feasible
        # q̈ the solver prefers. See cbf_qp_assembly.tangential_bias.
        #
        # Two smoothing layers, deliberately at two different rates:
        # tangential_bias() itself already blends its two direction sources
        # continuously (no per-tick hard switch); THIS EMA additionally
        # smooths the resulting vector ACROSS ticks, because even a smoothly
        # blended bias still rotates with âᵢ as the arm moves and with which
        # rows are engaged — unfiltered, that showed up on hardware as visible
        # oscillation.
        #
        # Gated on n_c > 0, which is set ONLY on the branch that actually built
        # rows from a fresh snapshot. That gate is load-bearing: on every
        # braking path above (state frozen, perception stale) n_c stays 0 while
        # `con` still holds the LAST snapshot, so an ungated call would steer
        # sideways off stale geometry at exactly the moment the filter has
        # decided it is blind. Braking stays pure. The EMA still runs on those
        # ticks, so the bias FADES OUT instead of freezing at whatever it was
        # when the feed died.
        a_tan = P.cbf_tangential_filter_alpha
        if n_c > 0:
            raw_bias = tangential_bias(
                qddot_nom, self._qdot_cbf, con, gain=P.cbf_tangential_gain,
                engage_margin=P.cbf_tangential_engage_margin,
                max_bias=P.cbf_tangential_max_bias)
            self._tan_bias *= a_tan
            self._tan_bias += (1.0 - a_tan) * raw_bias
            qddot_nom = qddot_nom + self._tan_bias
        else:
            self._tan_bias *= a_tan

        # ── STEP 3c: get OUT OF THE WAY when backing off cannot work ────
        # The tangential bias above uses the directions the barrier leaves
        # free, from the arm's own intent. This one answers a different
        # question: the acceleration box says the robot CANNOT null this
        # closing rate before the gap reaches zero, so backing off along n̂ is
        # not a solution however hard it is pushed — and the tracked obstacle
        # VELOCITY says which way to step aside instead. See utils.cbf_evasion.
        #
        # Also a bias on q̈_nom, so every row above stays exactly as binding as
        # it was and the QP cannot be made infeasible by it.
        #
        # Same n_c > 0 gate and same EMA structure as the tangential bias, for
        # the same two reasons: never steer off stale geometry on a braking
        # path, and fade out rather than freeze when the feed dies. Its own
        # accumulator, because the two engage on different triggers and must be
        # able to decay independently.
        if n_c > 0 and con is not None and con.esc_bias is not None:
            self._esc_bias *= a_tan
            self._esc_bias += (1.0 - a_tan) * con.esc_bias
            qddot_nom = qddot_nom + self._esc_bias
        else:
            self._esc_bias *= a_tan

        # ── STEP 3d: step aside when the obstacle cannot be OUTRUN ──────
        # The closed-form retreat authority (velocity box along n̂) says the
        # point cannot separate as fast as the obstacle closes, so the bias
        # rotates the target toward the fastest direction ⟂ v_obs. Its own,
        # FASTER EMA: the situation it answers is over in a few hundred ms,
        # and the 100 ms constant of the tangential filter would spend most
        # of that ramping up. Same gate, same fade-out as the two above.
        a_o = P.outrun_evasion_filter_alpha
        if n_c > 0 and con is not None and con.outrun_bias is not None:
            self._outrun_bias *= a_o
            self._outrun_bias += (1.0 - a_o) * con.outrun_bias
            qddot_nom = qddot_nom + self._outrun_bias
        else:
            self._outrun_bias *= a_o

        # ── STEP 3e: livelock escape ────────────────────────────────────
        # The detector is fed from the PREVIOUS tick's outcome (how much the
        # QP bent the nominal) and this tick's joint speed; the direction is
        # the snapshot's nullspace-projected lateral. Bias on the objective.
        if P.enable_livelock_escape:
            # "Blocked" means the task WANTS to move and the QP is taking that
            # away. Both halves are needed: an arm parked at its goal beside a
            # violated barrier also has a large ‖q̈_safe − q̈_nom‖ (the barrier
            # is pushing it off the goal), and nudging THAT sideways is
            # unexplained motion, not an escape — measured in the noisy-static
            # scenario, where it fired with the arm holding still on target.
            blocked = (n_c > 0 and con is not None and con.livelock_dir is not None
                       and float(np.linalg.norm(nom_raw)) > P.livelock_nominal_min
                       and float(np.linalg.norm(self._qddot_prev - self._nom_prev))
                       > P.livelock_dnorm_thr)
            moving = self._progress.push(now, js.q) > P.livelock_progress_thr
            self._lock_mag = self._livelock.update(now, blocked=blocked, moving=moving)
            if self._livelock.escaping and not self._lock_was:
                self.get_logger().warn(
                    f'LIVELOCK: QP bending the nominal for {P.livelock_stall_s:.1f} s '
                    f'with the arm still → tangential escape #{self._livelock.n_escapes} '
                    f'(gain {P.livelock_gain:.2f} rad/s², max {P.livelock_max_s:.1f} s)')
            elif self._lock_was and not self._livelock.escaping:
                self.get_logger().info(
                    f'LIVELOCK: escape ended ({self._livelock.last_reason})')
            self._lock_was = self._livelock.escaping
            # EMA'd across ticks exactly like the tangential and evasion
            # biases. Without it this was the one bias added raw: the escape
            # direction is rebuilt at 50 Hz and can rotate when the closest
            # row changes, so a raw add put a step of the full gain into
            # q̈_nom — the arm jerks sideways instead of leaning into it.
            raw_lock = np.zeros(NV)
            if self._lock_mag > 0.0 and con is not None and con.livelock_dir is not None:
                raw_lock = P.livelock_gain * self._lock_mag * con.livelock_dir
            self._lock_bias *= a_tan
            self._lock_bias += (1.0 - a_tan) * raw_lock
            # Added only with rows in hand, the same gate the other two biases
            # use: on a braking path (state frozen, perception stale) n_c is 0
            # while `con` still holds the last snapshot, and steering off that
            # geometry is the one thing braking must not do. The EMA keeps
            # running, so the bias FADES OUT instead of freezing.
            if n_c > 0:
                qddot_nom = qddot_nom + self._lock_bias
        else:
            self._lock_bias *= self.P.cbf_tangential_filter_alpha
        self._nom_prev = qddot_nom

        # ── STEP 4: the hard state box ──────────────────────────────────
        # Underneath every row, and NOT relaxable: one integration step must not
        # push |q̇| past the limit, and the position braking curve must keep the
        # joint able to stop. Then the slew box, so the arm can track what comes
        # out. Both mutate _box_lb/_box_ub in place, before the bounds are read.
        self._qvec[:NV] = -qddot_nom
        self._diag_vel_ratio, self._diag_vel_bite = position_velocity_accel_box(
            js.q, qdot, acc_lb=self._lb, acc_ub=self._ub,
            qdot_max=self._qdot_max, v_margin=P.velocity_box_margin,
            q_min=self._q_min, q_max=self._q_max,
            q_margin=P.position_margin_rad, brake_eta=P.position_brake_eta,
            dt=self._dt_qp, relax_dt=P.state_box_relax_s,
            out_lb=self._box_lb[:NV], out_ub=self._box_ub[:NV],
            clip_to_limits=P.accel_box_clip_to_limits)
        if P.slew_box_enabled:
            self._box_lb[:NV], self._box_ub[:NV] = apply_slew_limit(
                self._box_lb[:NV], self._box_ub[:NV],
                self._qddot_prev, P.max_qddot_delta)
        if float(np.max(self._diag_vel_ratio)) > P.diag_vel_ratio_thr:
            self.get_logger().info(
                f'VELHI t={now:.3f} n_c={n_c} '
                + format_velocity_summary(qdot, self._diag_vel_ratio,
                                          self._diag_vel_bite))

        # ── STEP 5: solve ───────────────────────────────────────────────
        qddot_safe, slack, solve_ms, res, level = self._solve(G, h_qp, n_c, qdot)
        if level != OSQP_LEVEL_FULL:
            fault = 1.0                     # the barrier was off for this tick

        # ── STEP 6: publish, then report ────────────────────────────────
        if P.slew_box_enabled:
            np.subtract(qddot_safe, self._qddot_prev, out=self._diag_slew_step)
            np.greater(np.abs(self._diag_slew_step), P.max_qddot_delta - 1e-6,
                       out=self._diag_slew_bite)
        if self._brake_authority_fault(qddot_safe, n_active):
            fault = 1.0
        self._publish(qddot_safe)
        self._publish_status(n_c, slack, fault, n_active,
                             con.d_obs_min if con is not None else float('inf'),
                             qdot=qdot, con=con)
        self._report(now, con, h_qp, n_c, n_active, qdot, qddot_safe,
                     qddot_nom, slack, solve_ms, res)

    # ── QP tick helpers ──────────────────────────────────────────────────

    def _watch_tick_gap(self) -> None:
        """Warn when the executor did not come back on time."""
        t = time.perf_counter()
        if self._last_tick_t is not None:
            gap_ms = (t - self._last_tick_t) * 1e3
            if gap_ms > self._gap_warn_thr_ms:
                self.get_logger().warn(f'_qp_tick gap: {gap_ms:.1f} ms')
        self._last_tick_t = t

    def _joint_state_frozen(self, now: float, qdot) -> bool:
        """A re-stamped but UNCHANGED state, which the timeout cannot see.

        Strictly worse than a dead topic: the velocity box reads a q̇ that never
        grows, so it never tightens, while the real joint accelerates. This is
        what a 30 Hz republisher does when it stalls — it keeps emitting its
        cached values with fresh header stamps.
        """
        P = self.P
        if (self._js_frozen_since is None
                or (now - self._js_frozen_since) <= P.joint_state_freeze_timeout
                or float(np.max(np.abs(qdot))) <= P.joint_state_freeze_min_speed):
            return False
        self.get_logger().error(
            f'joint state FROZEN for {now - self._js_frozen_since:.3f} s '
            f'(identical q/q̇, fresh stamps) while |q̇|max='
            f'{float(np.max(np.abs(qdot))):.2f} rad/s → braking, CBF rows '
            f'dropped. The publisher is re-stamping cached values.',
            throttle_duration_sec=0.5)
        return True

    def _smooth_qdot(self, qdot) -> None:
        """EMA on the q̇ used by the k1 anticipation term and by
        ``tangential_bias``'s qdot-fallback direction — nothing else.

        ḣ is a DERIVATIVE of a measured signal and k1 multiplies it straight
        into the bound: unfiltered it was 159 % of h_qp's whole swing on
        hardware and the command flipped sign nine times in 28 intervals.
        ``tangential_bias`` NORMALISES q̇'s orthogonal component to get a
        direction, which amplifies raw-signal noise even more than a linear
        term does — the same failure mode, one derivative worse — so it reuses
        this filtered copy rather than opening a second one. The barrier
        value, the row direction, the accel box, the braking fallback and
        every diagnostic keep the raw q̇.
        """
        a = self.P.cbf_hdot_filter_alpha
        if a > 0.0:
            self._qdot_cbf *= a
            self._qdot_cbf += (1.0 - a) * qdot
        else:
            np.copyto(self._qdot_cbf, qdot)

    def _solve(self, G, h_qp, n_c, qdot):
        """Push the moving parts into OSQP and solve. Returns (q̈, slack, ms, res).

        ``setup()`` only when the row count changes the sparsity pattern;
        otherwise ``update()`` the vectors that move. NEVER returns ``None``:
        a failed solve walks the fallback ladder in ``cbf_qp_assembly`` (box
        only, then closed-form braking) and reports which rung answered — the
        warm-start iterate is discarded so a bad solve cannot seed the next.

        The row count is QUANTISED to ``qp_row_block`` first (see
        ``pad_rows_to_block``): the real n_c changes almost every rebuild, and
        re-``setup()``ing on each change was a per-tick allocate-and-factorize
        spike in a 100 Hz Python node. The caller's ``n_c`` is still the real
        one — padding is invisible to every diagnostic.
        """
        G, h_qp = pad_rows_to_block(G, h_qp, self.P.qp_row_block)
        n_rows = 0 if G is None else G.shape[0]
        l, u = build_osqp_bounds(G, h_qp, self._box_lb, self._box_ub)
        if n_rows != self._prev_nc or self._osqp_prob is None:
            self._prev_nc = n_rows
            self._osqp_prob = osqp.OSQP()
            self._osqp_prob.setup(
                P=self._P_csc, q=self._qvec, A=build_osqp_A(G, NV, N_SLACK),
                l=l, u=u, warm_start=True,
                max_iter=self.P.osqp_max_iter, verbose=False)
        elif n_rows > 0:
            self._osqp_prob.update(q=self._qvec, l=l, u=u,
                                   Ax=build_osqp_A(G, NV, N_SLACK).data)
        else:
            self._osqp_prob.update(q=self._qvec, l=l, u=u)

        t0 = time.perf_counter()
        res = self._osqp_prob.solve()
        solve_ms = (time.perf_counter() - t0) * 1e3

        self._diag_slack[:] = 0.0
        # ── The fallback ladder (cbf_qp_assembly): never "no solution" ──
        x = accept_iterate(
            res.x, res.info.status_val, self._box_lb, self._box_ub, NV,
            solved=osqp.constant('OSQP_SOLVED'),
            inaccurate=(osqp.constant('OSQP_SOLVED_INACCURATE')
                        if self.P.accept_inaccurate_qp else -999))
        level = OSQP_LEVEL_FULL
        if x is None:
            self._qp_fail_count += 1
            # Level 1: same objective, box only. A projection onto a box; a
            # fresh instance so the failed warm start is not reused.
            x = box_only_solve(self._P_csc, self._qvec, self._box_lb, self._box_ub,
                               max_iter=self.P.osqp_max_iter)
            level = OSQP_LEVEL_BOX
            if x is None:
                # Level 2: closed-form braking inside the tightened box.
                x = np.concatenate([braking_command(qdot, self._box_lb[:NV],
                                                    self._box_ub[:NV],
                                                    k_brake=self.P.k_brake),
                                    np.zeros(N_SLACK)])
                level = OSQP_LEVEL_BRAKE
            self.get_logger().error(
                f'QP not solved ({res.info.status}) → fallback level {level} '
                f'({LEVEL_NAMES[level]}) [qp_fail_count={self._qp_fail_count} '
                f'levels={self._fallback_count}]',
                throttle_duration_sec=0.5)
            self._osqp_prob, self._prev_nc = None, -1
        self._fallback_count[level] += 1
        self._last_level = level
        if n_c > 0 and level == OSQP_LEVEL_FULL:
            np.copyto(self._diag_slack, x[NV:])
        return x[:NV], float(self._diag_slack.max()), solve_ms, res, level

    def _report(self, now, con, h_qp, n_c, n_active, qdot, qddot_safe,
                qddot_nom, slack, solve_ms, res) -> None:
        """Solve-time line every 100 ticks, CBFDIAG line on its own throttle."""
        self._tick_count += 1
        if self._tick_count % 100 == 0 or solve_ms > 5.0:
            tail = (f'iter={res.info.iter} status={res.info.status} '
                    f'h_norm={float(np.linalg.norm(h_qp)):.2f}'
                    if n_c > 0 else 'iter=- status=- h_norm=-')
            self.get_logger().info(
                f'tick={self._tick_count} n_c={n_c} solve={solve_ms:.2f}ms '
                f'qddot_nom_norm={float(np.linalg.norm(qddot_nom)):.2f} '
                f'qp_fails={self._qp_fail_count} fb={self._fallback_count} '
                f'lock={self._lock_mag:.2f} ' + tail)
        if n_c > 0 and (now - self._last_diag_t) >= self.P.diag_period_s:
            self._last_diag_t = now
            self.get_logger().info(format_cbf_diag(
                now=now, con=con, rows=self._rows, caps=self._diag_caps,
                h_qp=h_qp, qdot=qdot, qdot_cbf=self._qdot_cbf,
                qddot_safe=qddot_safe, qddot_nom=qddot_nom,
                qddot_real=self._diag_qddot_real, slack=self._diag_slack,
                n_active_cps=n_active, vel_ratio=self._diag_vel_ratio,
                vel_bite=self._diag_vel_bite, slew_bite=self._diag_slew_bite,
                cap_age=self._diag_cap_age, w_task=self._diag_w_task,
                gov=self._diag_gov,
                iso_v_closing=float(self._status_msg.data[7])
                if len(self._status_msg.data) > 7 else 0.0,
                iso_stop=float(self._status_msg.data[8])
                if len(self._status_msg.data) > 8 else 0.0))

    # ═════════════════════════════════════════════════════════════════════
    #  Output
    # ═════════════════════════════════════════════════════════════════════

    def _brake_authority_fault(self, qddot_cmd, n_active: int) -> bool:
        """Is the arm actually DOING what the barrier told it to? [E]

        ``S_p`` assumes ``a_s`` is delivered. The q̈ → τ chain here is pure
        feed-forward (``M q̈ + C q̇``, no PD, no friction model), so "commanded"
        and "realized" are two different numbers and the gap between them is
        already visible on hardware as ``qdd_cmd_rad`` vs ``qdd_real_rad`` in the
        CBFDIAG line. If the realized acceleration is consistently a fraction of
        the commanded one while a barrier row is pushing, then every stopping
        distance computed from ``a_s`` is optimistic by exactly that fraction.

        ``frac = (q̈_real · q̈_cmd) / ‖q̈_cmd‖²`` — the PROJECTION, not the norm
        ratio: what matters is how much of the commanded direction is being
        delivered, and a large realized acceleration in some other direction is
        not braking authority.

        DIAGNOSTIC ONLY, and deliberately so until the fault rate is known on
        hardware: it raises ``fault_braking`` on cbf_status and logs, and does
        not stop. A stop on an unvalidated threshold in the q̈ → τ chain would
        fire on model error, not on danger.
        """
        P = self.P
        if not P.iso_enabled or n_active <= 0:
            self._brake_frac_run = 0
            return False
        n2 = float(qddot_cmd @ qddot_cmd)
        if n2 <= 0.25:                      # ‖q̈_cmd‖ <= 0.5 rad/s²: nothing asked
            self._brake_frac_run = 0
            return False
        frac = float(self._diag_qddot_real @ qddot_cmd) / n2
        self._diag_brake_frac = frac
        if frac >= P.iso_brake_frac_min:
            self._brake_frac_run = 0
            self._brake_frac_warned = False
            return False
        self._brake_frac_run += 1
        if self._brake_frac_run < P.iso_brake_frac_ticks:
            return False
        if not self._brake_frac_warned:
            self._brake_frac_warned = True
            self.get_logger().error(
                f'BRAKING AUTHORITY: realized/commanded acceleration = '
                f'{frac:.2f} < {P.iso_brake_frac_min:.2f} for '
                f'{self._brake_frac_run} ticks with {n_active} obstacle row(s) '
                f'violated. Every stopping distance derived from '
                f'iso_a_stop={P.iso_a_stop:.2f} m/s^2 is optimistic by about '
                f'this factor.\n'
                f'  q̈_cmd  = {np.round(qddot_cmd, 2).tolist()}\n'
                f'  q̈_real = {np.round(self._diag_qddot_real, 2).tolist()}\n'
                f'  Diagnostic only — no stop. Re-measure a_s with '
                f'scripts/iso_constants_measure.py stop.')
        return True

    def _empty_frame_fault(self, now: float) -> bool:
        """A run of empty distance frames that began while something was close.

        Returns True once the run exceeds ``iso_empty_frame_max_s``. Gated on
        ``iso_enabled``: the fault is an ISO-layer addition, and with the flag
        off an empty frame behaves exactly as it did before (no rows, no fault).
        """
        P = self.P
        if not P.iso_enabled or not self._empty_close or self._empty_since is None:
            return False
        run = now - self._empty_since
        if run <= P.iso_empty_frame_max_s:
            return False
        if not self._empty_faulted:
            self._empty_faulted = True
            self.get_logger().error(
                f'EMPTY distance frames for {run:.3f} s > '
                f'{P.iso_empty_frame_max_s:.3f} s, and the last frame that was '
                f'not empty had an obstacle inside '
                f'{P.zone_r_active * P.d_safe:.3f} m → braking, CBF rows '
                f'dropped. Perception is alive (it is still heartbeating) but '
                f'it has stopped SEEING something it was seeing.')
        return True

    def _on_iso_safety(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 5:
            self._iso = [float(v) for v in msg.data]
            self._iso_stamp = self._now()

    def _iso_stop_active(self, now: float) -> bool:
        """Is the ISO layer demanding a stop right now?

        Two ways to answer yes, and the second is the one worth reading twice:

        * the monitor has LATCHED — the separation-distance bound was crossed;
        * the monitor's topic is STALE (or has never arrived). With
          ``iso_monitor_enabled`` the monitor is part of the safety chain, and a
          missing channel in a safety chain is a fault, not a quiet absence. The
          age is measured from node construction when nothing has ever arrived,
          so a monitor that never starts trips this within one
          ``distance_timeout`` rather than never.

        ``scripts/iso_preflight_check.py`` is what turns that into a startup
        failure instead of a braking robot; this is the runtime backstop.
        """
        P = self.P
        if not (P.iso_enabled and P.iso_monitor_enabled):
            return False
        if self._iso is not None and self._iso[0] >= 1.0:
            return True
        age = now - (self._iso_stamp if self._iso is not None else self._iso_start)
        if age > P.distance_timeout:
            self.get_logger().error(
                f'iso_safety stale ({age:.2f} s > {P.distance_timeout:.2f} s) '
                f'with iso_monitor_enabled → braking. The monitor is part of '
                f'the chain; a missing channel is a fault.',
                throttle_duration_sec=2.0)
            return True
        return False

    def _publish(self, qddot_safe) -> None:
        """Send q̈_safe, and remember it: the NEXT tick's slew box is centred on
        what actually went out, not on whatever the QP happened to compute."""
        self._qddot_prev[:] = qddot_safe
        msg = Float64MultiArray()
        msg.data = qddot_safe.tolist()
        self._pub.publish(msg)

    def _publish_status(self, n_c, slack, fault, n_act, d_obs=float('inf'),
                        qdot=None, con=None) -> None:
        """/cbf_status = [n_rows, slack, fault, n_violated, d_min, + ISO tail].

        ``fault_braking`` marks a SAFETY-CHAIN fault (state stale or frozen,
        perception stale, QP failed) — distinct from n_c = 0 during normal
        "nothing nearby" operation. ``d_obs`` is the closest obstacle gap, which
        the commander's phase governor reads: slowing on genuine proximity is
        safe, slowing on tracking error deadlocks (error grows → phase freezes →
        reference parked on the blocked pose → the CBF keeps blocking).

        ISO tail (roadmap Step 10), APPENDED so every existing consumer keeps
        working — both of them (frame_grabber, rl_policy_commander) index
        positionally behind a ``len()`` guard::

            data[5] S_p_min        [m] separation distance the worst CP needs
            data[6] v_cap_min      [m/s] tightest SSM speed cap
            data[7] v_closing_max  [m/s] fastest closing speed of any CP
            data[8] iso_stop_latched
            data[9]  v_obs_cond     [m/s] largest CONDITIONED closing speed any
                                    row used (post median / deadband / clamp) —
                                    the retreat cap's and the evasion's input
            data[10] v_obs_hdot     [m/s] the signed n̂ᵀv_track of largest
                                    magnitude that entered ḣ (enable_vobs_in_hdot)

        The MONITOR's numbers win when its message is fresh: it is the channel
        that decides, and publishing the filter's own opinion next to a monitor
        that disagrees would make the log unreadable at exactly the moment it
        matters. With no monitor running, the fields fall back to what the row
        builder computed this rebuild, and ``v_closing_max`` is measured here
        from the obstacle rows directly.
        """
        iso = self._iso
        fresh = (iso is not None
                 and (self._now() - self._iso_stamp) < self.P.distance_timeout)
        if fresh:
            s_p, v_cap, v_cls, latched = iso[2], iso[3], iso[4], iso[0]
        else:
            rows = self._rows
            s_p = float(getattr(rows, 'diag_ssm_sp', 0.0))
            v_cap = float(getattr(rows, 'diag_ssm_cap', float('inf')))
            v_cls = 0.0
            if con is not None and qdot is not None and con.A.shape[0]:
                sep = con.A @ qdot          # + = separating, so closing is -sep
                obs_rows = con.group == G_OBS
                if obs_rows.any():
                    v_cls = max(float(-np.min(sep[obs_rows])), 0.0)
            latched = 0.0
        # ── data[9..10]: what the barrier was actually FED ──────────────
        # The tracker's raw velocity is on the wire (LinkDistance.obstacle_
        # velocity) and is easy to log, but between it and the barrier sits a
        # 5-frame median, a deadband and a clamp. Logging only the raw input
        # and reasoning about the output is how a filter's own artefacts get
        # attributed to the thing it filters: measured on a hardware run, the
        # raw tracker peaked at 3.84 m/s while 24 of its 34 excursions lasted a
        # single perception frame — exactly what a 5-frame median removes by
        # construction. These two are the CONDITIONED values, so the question
        # "does the artefact reach the barrier" is answerable from a bag.
        #
        # Both are per-REBUILD (50 Hz) maxima, not per-QP-tick: they are the
        # builder's own diagnostics, and they hold whatever the last rebuild
        # saw until the next one replaces them.
        rows = self._rows
        self._status_msg.data = [float(n_c), slack, fault, float(n_act), d_obs,
                                 float(s_p), float(v_cap), float(v_cls),
                                 float(latched),
                                 float(getattr(rows, 'diag_v_obs', 0.0)),
                                 float(getattr(rows, 'diag_vobs_hdot', 0.0))]
        self._status_pub.publish(self._status_msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = CBFSafetyFilter()
    # One thread per callback group: I/O, constraint builder, QP loop. The QP
    # thread must never wait on perception.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
