"""A real ConstraintBuilder driven headless, for the row-level regression tests.

Everything here is a stand-in for the two things the builder needs from outside
— a parameter namespace and a kinematics object — and NOTHING is a stand-in for
the builder itself: the tests that import this run the shipped
``ConstraintBuilder.build`` and compare the arrays it actually produces.

The Jacobian is analytic and configuration-dependent (so ``a = n̂ᵀJ_p`` moves
with q the way it does on the robot) but deliberately simple, so a test can
predict ``aᵀq̈`` by hand when it needs to.

No ROS, no Pinocchio.
"""

import numpy as np

from franka_experiments.utils.cbf_state_rows import (
    ConstraintBuilder, JointSnap, Obstacle, ObstacleSnap, Shadow,
    ShadowRowBuilder)

NV = 7


class _Log:
    """Swallows messages, but RECORDS them, so a test can assert one was said.

    A constraint the builder decided not to emit has to be announced -- the
    `max_rows` cap and the leverage gate both drop candidate safety rows -- and
    "it is announced" is only checkable if the sink keeps what it was told.
    """

    def __init__(self):
        self.msgs = []

    def _rec(self, kind, a):
        self.msgs.append((kind, str(a[0]) if a else ''))

    def info(self, *a, **k):    self._rec('info', a)
    def warn(self, *a, **k):    self._rec('warn', a)
    def warning(self, *a, **k): self._rec('warning', a)
    def error(self, *a, **k):   self._rec('error', a)
    def debug(self, *a, **k):   self._rec('debug', a)

    def said(self, needle, kind=None):
        """Every recorded message containing ``needle``."""
        return [m for k, m in self.msgs
                if needle in m and (kind is None or k == kind)]


class _Kin:
    """Analytic 3×7 point Jacobian; J̇ₚ is a fixed small perturbation."""

    def __init__(self):
        self.q = np.zeros(NV)
        #: label -> world position, for control_point_position(). Only the
        #: obstacle memory needs it; everything else works off ob.pr.
        self.cp_positions = {}
        #: CBFKinematics exposes every CONFIGURED control point here, and the
        #: self-mask test enumerates it rather than the labels that happened to
        #: carry a row -- the link that explains a disappearance is usually not
        #: the link the vanished row was on. Aliased so the harness cannot
        #: drift from cp_positions.
        self._control_points = self.cp_positions

    def update(self, q, qdot, with_jdot=True):
        self.q = np.asarray(q, dtype=np.float64)

    def resolve_frame_id(self, link):
        return 1 + (abs(hash(link)) % 7)

    def control_point_position(self, label):
        """Where a configured control point IS now, as CBFKinematics reports.

        The harness' points do not move with q -- a test that needs motion
        writes the position it wants into ``cp_positions``. Unknown labels
        raise ``KeyError``, exactly as the real accessor does, so a caller that
        mistypes a label gets an error rather than a different point.
        """
        return np.asarray(self.cp_positions[label.partition('.')[0]],
                          dtype=np.float64)

    def point_jacobian(self, fid, p_world):
        # Smooth in q so the row direction is configuration dependent, and
        # full rank in the first three rows so every escape direction has some
        # leverage. Scaled to a metre-ish arm.
        j = np.arange(NV, dtype=np.float64)
        Jp = np.empty((3, NV))
        Jp[0] = 0.40 * np.cos(0.5 * j + self.q)
        Jp[1] = 0.35 * np.sin(0.7 * j + 2.0 * self.q)
        Jp[2] = 0.30 * np.cos(0.3 * j - self.q) + 0.10
        Jpd = 0.05 * np.ones((3, NV))
        return Jp, Jpd


class P:
    """Every attribute ``ConstraintBuilder.build`` reads, at shipped values."""

    # gains / geometry
    d_safe = 0.15
    cbf_obstacle_horizon = 1.2
    min_confidence = 0.5
    cbf_min_leverage = 0.05
    cbf_h_recovery_alpha = 0.6
    distance_timeout = 0.5
    # families that would add unrelated rows — off, so a test sees only the
    # obstacle rows it built and any change to them is unambiguous.
    joint_limit_rows_enabled = False
    joint_limit_row_margin = 0.05
    joint_limit_row_horizon = 0.30
    joint_limit_row_velocity_horizon = False
    joint_limit_row_horizon_lead = 1.5
    retreat_cap_enabled = False
    link_speed_rows_enabled = False
    position_brake_eta = 0.5
    # obstacle velocity
    obstacle_velocity_enabled = True
    obstacle_velocity_alpha = 0.7
    obstacle_velocity_max = 2.0
    obstacle_velocity_source = 'residual'
    obstacle_velocity_min_frames = 3
    obstacle_velocity_residual_floor = False
    obstacle_velocity_median = 1            # off, so existing tests are unchanged
    # Rate-independent forms, all OFF here: every existing test keeps driving
    # the frame-counted path it was written against, and the tests that exercise
    # the durations turn them on explicitly.
    obstacle_velocity_dt_min_s = 0.0
    obstacle_velocity_tau_s = 0.0
    obstacle_velocity_median_s = 0.0
    obstacle_velocity_rot_hold_s = 0.0
    obstacle_velocity_min_span_s = 0.0
    # Fast-track trust: OFF here, so every older test keeps the old gate.
    obstacle_velocity_fast_trust = False
    obstacle_velocity_fast_min_frames = 3
    obstacle_velocity_fast_speed = 1.0
    obstacle_velocity_fast_k_sigma = 2.0
    obstacle_velocity_fast_max = 6.0
    velocity_feedforward_min_span_s = 0.0
    obstacle_input_rate_hz = 30.0
    obstacle_velocity_track_deadband = 0.0
    obstacle_velocity_residual_floor_gap = 0.0  # no gap gate by default
    obstacle_velocity_normal_rot_max = 0.0  # rotation guard off by default
    obstacle_velocity_identity_jump = 0.0   # identity guard off by default
    # feedforward
    enable_velocity_feedforward = False
    obstacle_decel_assumed = 4.0
    velocity_feedforward_gain = 1.0
    velocity_braking_margin_max = 0.25
    velocity_feedforward_min_frames = 3
    # speed-proportional standoff
    enable_velocity_standoff = False
    velocity_standoff_time_s = 0.20
    velocity_standoff_max = 0.20
    velocity_standoff_alpha = 0.8
    # curvature term of ḧ (Task 2) — OFF by default, so every existing test
    # keeps the pre-curvature RHS it was written against.
    enable_curvature_term = False
    enable_vobs_in_hdot = False
    vobs_hdot_max = 2.0
    curvature_obstacle_radius = 0.30
    curvature_kappa_max = 2.0
    # uncertainty
    enable_uncertainty_margin = False
    uncertainty_k_sigma = 2.0
    uncertainty_margin_alpha = 0.0
    link_speed_reaction_s = 0.20
    # sensor range uncertainty (Phase 3)
    enable_sensor_range_uncertainty = False
    sensor_range_f_px = 428.8
    sensor_range_baseline_m = 0.095
    sensor_range_sigma_d_px = 0.20
    sensor_range_k_sigma = 2.0
    sensor_range_margin_max = 0.25
    # slack weighting
    enable_weighted_slack = False
    slack_weight_max = 5.0
    slack_weight_rho = 0.30
    slack_weight_rho_qlim = 0.40
    slack_weight_alpha = 6.0
    # evasion
    enable_lateral_evasion = False
    lateral_evasion_gain = 1.5
    lateral_evasion_max_bias = 3.0
    lateral_evasion_authority = 0.5
    lateral_evasion_engage_ratio = 0.7
    lateral_evasion_v_min = 0.15
    # outrun evasion (Phase 2)
    enable_outrun_evasion = False
    outrun_evasion_margin = 0.8
    outrun_evasion_ramp_start = 0.6
    outrun_evasion_accel = 1.0
    outrun_evasion_gain = 1.0
    outrun_evasion_v_ref = 0.5
    outrun_evasion_v_min = 0.15
    outrun_evasion_max_bias = 3.0
    outrun_evasion_filter_alpha = 0.7
    outrun_evasion_engage_gap = 0.0      # ungated, so existing tests are unchanged
    outrun_evasion_v_avail_floor = 0.0
    velocity_box_margin = 0.9
    # livelock escape (Phase 3c) — only the builder-side flag matters here
    enable_livelock_escape = False
    livelock_engage_gap = 0.0
    # latency compensation (Phase 4)
    enable_latency_compensation = False
    latency_t_blind = 0.085
    latency_k_sigma = 2.0
    latency_margin_max = 0.25
    # caps (unused while their families are off, but read at construction)
    retreat_cap_base_speed = 0.15
    retreat_cap_obstacle_gain = 1.0
    retreat_cap_depth_gain = 4.0
    retreat_cap_depth_speed_ref = 0.3
    retreat_cap_engage_gap = 0.35
    retreat_cap_max_speed = 0.6
    link_speed_max = 0.8
    link_speed_v_at_d_safe = 0.75   # d_safe/0.75 = 0.20 s: same cap as before
    link_speed_activate_frac = 0.5
    # zone ladder — OFF by default so every existing test keeps the scalar
    # gains and the bit-identical rows it was written against.
    enable_zone_ladder = False
    k0_cbf = 25.0
    k1_cbf = 10.5
    # Ratios x d_safe (0.15 here) chosen to land on the 0.30/0.20/0.10/0.05 m
    # ladder and 0.04 m blend the builder tests were written against. The
    # shipped YAML ratios differ; test_cbf_zone_ladder checks those separately.
    zone_r_notice = 0.30 / 0.15
    zone_r_active = 0.20 / 0.15
    zone_r_priority = 0.10 / 0.15
    zone_r_hold = 0.05 / 0.15
    zone_blend_r = 0.04 / 0.15
    zone_k0_notice = 12.0
    zone_k1_notice = 7.0
    zone_k0_priority = 50.0
    zone_k1_priority = 14.0
    zone_slack_m_priority = 0.5
    zone_slack_m_hold = 0.25
    zone_task_priority_cut = 0.5
    zone_resume_s = 0.5
    # ── ISO 10218 layer — OFF, so every existing test keeps the rows it was
    # written against. Only iso_enabled + iso_ssm_speed_rows together change
    # anything; iso_mode == 'reduced' adds the 250 mm/s TCP row.
    iso_enabled = False
    iso_mode = 'automatic'
    iso_ssm_speed_rows = False
    iso_t_reaction = 0.10
    iso_a_stop = 4.0
    iso_v_human = 2.0
    iso_c_intrusion = 0.10
    iso_z_depth = 0.06
    iso_z_robot = 0.01
    iso_tcp_reduced_speed = 0.25
    # ── Evidence-conditioned trust — all three OFF, so every existing test
    # keeps the rows and the RHS it was written against. ───────────────────
    enable_evidence_margin = False
    evidence_margin_fresh_m = 0.01
    evidence_margin_young_m = 0.04
    evidence_margin_coasting_m = 0.08
    evidence_margin_untracked_m = 0.02
    evidence_coast_grace_s = 0.07
    enable_obstacle_memory = False
    memory_hold_s = 0.50
    memory_trigger_gap_m = 0.25
    memory_min_frames = 8
    memory_inflate_speed_mps = 0.20
    memory_require_occlusion = False
    memory_camera_x = 0.0
    memory_camera_y = 0.0
    memory_camera_z = 0.0
    memory_self_radius_m = 0.08
    memory_mask_halo_rad = 0.0278
    memory_mask_depth_tol_m = 0.16
    curvature_radius_from_normal = False
    curvature_normal_window_s = 0.20
    curvature_normal_rate_floor = 0.30
    # ── Self-Shadow Barrier — OFF, so every existing test keeps exactly the
    # rows it was written against. The SSB tests turn it on explicitly.
    ssb_rows_enabled = False
    ssb_d_safe = 0.05
    ssb_row_horizon = 0.20
    ssb_row_release = 1.5
    ssb_max_rows = 4
    ssb_hold_s = 0.10
    ssb_inflate_speed_mps = 0.20
    rho_slack_ssb = 400.0
    ssb_slack_max = 1e6


def make_params(**over):
    p = P()
    for k, v in over.items():
        assert hasattr(P, k), f'unknown parameter {k!r}'
        setattr(p, k, v)
    return p


ACC = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
QD = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])


def make_builder(kin=None, cp_jacobian=False, **over):
    """A real ConstraintBuilder on the harness' kinematics.

    ``kin`` lets a test keep a reference to the kinematics it can then write
    ``cp_positions`` into. ``cp_jacobian=True`` supplies the control-point
    Jacobian callable the torque node supplies on the robot -- the obstacle
    memory needs it, because its row is attached to the control point the
    vanished track was last seen on rather than to a point on the wire.
    """
    kin = _Kin() if kin is None else kin
    cpj = None
    if cp_jacobian:
        def cpj(label):
            return kin.point_jacobian(1, kin.control_point_position(label))
    P_ = make_params(**over)
    # The Self-Shadow Barrier is a policy object the node builds in
    # build_optional_row_builders; this harness constructs the builder
    # directly, so it has to do the same thing or the family would be silently
    # absent whatever the flag said.
    ssb = None
    if getattr(P_, 'ssb_rows_enabled', False):
        ssb = ShadowRowBuilder(
            d_safe=P_.ssb_d_safe, horizon=P_.ssb_row_horizon,
            max_rows=P_.ssb_max_rows, release=P_.ssb_row_release,
            hold_s=P_.ssb_hold_s, inflate_mps=P_.ssb_inflate_speed_mps,
            min_leverage=P_.cbf_min_leverage, with_kappa=False)
    log = _Log()
    b = ConstraintBuilder(
        P_, kin,
        q_min=np.full(NV, -2.8), q_max=np.full(NV, 2.8),
        acc_lb=-ACC, acc_ub=ACC, logger=log, qdot_max=QD,
        control_point_jacobian=cpj, ssb_rows=ssb)
    #: the logger, reachable so a test can assert what the builder ANNOUNCED
    #: (a dropped safety row has to be said out loud, not merely counted)
    b.test_log = log
    return b


def make_obstacle(d=0.25, pr=(0.5, 0.0, 0.5), ph=(0.5, -0.25, 0.5),
                  link='fr3_link5', v=None, frames_seen=0, cov=None,
                  track_id=0, conf=1.0, a=None, pos_cov=None, pv_cov=None,
                  cp_label='', range_m=None):
    arr = lambda x: None if x is None else np.asarray(x, dtype=np.float64)
    return Obstacle(link=link, d=float(d),
                    pr=np.asarray(pr, dtype=np.float64),
                    ph=np.asarray(ph, dtype=np.float64), conf=float(conf),
                    v_vec=arr(v), frames_seen=int(frames_seen), vel_cov=arr(cov),
                    track_id=int(track_id), a_vec=arr(a), pos_cov=arr(pos_cov),
                    pv_cov=arr(pv_cov), cp_label=cp_label,
                    range_m=None if range_m is None else float(range_m))


def make_shadow(cp_label='fr3_link5#0', link='fr3_link5', d=0.10,
                pr=(0.5, 0.0, 0.5), ph=(0.5, -0.10, 0.5), front_m=0.05):
    """One SSB measurement, as LinkDistance's ``shadow_*`` fields carry it.

    ``d`` is the SURFACE gap perception reported, i.e. the centre distance
    ``|pr - ph|`` already less the capsule radius; the builder recovers that
    radius as ``|pr - ph| - d`` so a row rebuilt at the CURRENT control-point
    position speaks the same units.
    """
    return Shadow(cp_label=cp_label, link=link, d=float(d),
                  pr=np.asarray(pr, dtype=np.float64),
                  ph=np.asarray(ph, dtype=np.float64), front_m=float(front_m))


def make_js(q=0.1, qdot=0.0, stamp=0.0):
    return JointSnap(q=np.full(NV, q), qdot=np.full(NV, qdot), stamp=stamp)


def make_obs(items, stamp=0.0, t_cap=None, shadows=()):
    return ObstacleSnap(items=tuple(items), stamp=stamp,
                        t_cap=stamp if t_cap is None else t_cap,
                        shadows=tuple(shadows))


def run(builder, obstacles, *, n_frames=1, dt=1.0 / 30.0, qdot=0.0, q=0.1,
        shadows=()):
    """Drive `n_frames` rebuilds and return the LAST ConstraintSnap.

    ``shadows`` takes the same shape as ``obstacles``: a sequence, or a
    callable of the frame index for a stream that changes between frames.
    """
    con = None
    for k in range(n_frames):
        t = k * dt
        con = builder.build(make_js(q=q, qdot=qdot, stamp=t),
                            make_obs(obstacles(k) if callable(obstacles)
                                     else obstacles, stamp=t,
                                     shadows=(shadows(k) if callable(shadows)
                                              else shadows)), t)
    return con
