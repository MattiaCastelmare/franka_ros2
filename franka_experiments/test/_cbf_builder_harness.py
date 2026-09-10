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
    ConstraintBuilder, JointSnap, Obstacle, ObstacleSnap)

NV = 7


class _Log:
    def info(self, *a, **k):    pass
    def warn(self, *a, **k):    pass
    def warning(self, *a, **k): pass
    def error(self, *a, **k):   pass
    def debug(self, *a, **k):   pass


class _Kin:
    """Analytic 3×7 point Jacobian; J̇ₚ is a fixed small perturbation."""

    def __init__(self):
        self.q = np.zeros(NV)

    def update(self, q, qdot, with_jdot=True):
        self.q = np.asarray(q, dtype=np.float64)

    def resolve_frame_id(self, link):
        return 1 + (abs(hash(link)) % 7)

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
    # uncertainty
    enable_uncertainty_margin = False
    uncertainty_k_sigma = 2.0
    uncertainty_margin_alpha = 0.0
    link_speed_reaction_s = 0.20
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


def make_params(**over):
    p = P()
    for k, v in over.items():
        assert hasattr(P, k), f'unknown parameter {k!r}'
        setattr(p, k, v)
    return p


ACC = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
QD = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])


def make_builder(**over):
    return ConstraintBuilder(
        make_params(**over), _Kin(),
        q_min=np.full(NV, -2.8), q_max=np.full(NV, 2.8),
        acc_lb=-ACC, acc_ub=ACC, logger=_Log(), qdot_max=QD)


def make_obstacle(d=0.25, pr=(0.5, 0.0, 0.5), ph=(0.5, -0.25, 0.5),
                  link='fr3_link5', v=None, frames_seen=0, cov=None,
                  track_id=0, conf=1.0, a=None, pos_cov=None, pv_cov=None,
                  cp_label=''):
    arr = lambda x: None if x is None else np.asarray(x, dtype=np.float64)
    return Obstacle(link=link, d=float(d),
                    pr=np.asarray(pr, dtype=np.float64),
                    ph=np.asarray(ph, dtype=np.float64), conf=float(conf),
                    v_vec=arr(v), frames_seen=int(frames_seen), vel_cov=arr(cov),
                    track_id=int(track_id), a_vec=arr(a), pos_cov=arr(pos_cov),
                    pv_cov=arr(pv_cov), cp_label=cp_label)


def make_js(q=0.1, qdot=0.0, stamp=0.0):
    return JointSnap(q=np.full(NV, q), qdot=np.full(NV, qdot), stamp=stamp)


def make_obs(items, stamp=0.0, t_cap=None):
    return ObstacleSnap(items=tuple(items), stamp=stamp,
                        t_cap=stamp if t_cap is None else t_cap)


def run(builder, obstacles, *, n_frames=1, dt=1.0 / 30.0, qdot=0.0, q=0.1):
    """Drive `n_frames` rebuilds and return the LAST ConstraintSnap."""
    con = None
    for k in range(n_frames):
        t = k * dt
        con = builder.build(make_js(q=q, qdot=qdot, stamp=t),
                            make_obs(obstacles(k) if callable(obstacles)
                                     else obstacles, stamp=t), t)
    return con
