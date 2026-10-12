"""enable_human_prediction: a SOFT forecast row next to each human row.

The hard row of a human limb stays a barrier on where it is; the soft row is the same row
moved to where its Kalman velocity puts it after human_prediction_horizon_s, with the
forecast's uncertainty as a margin, in its own slack family. These tests pin:

- off (the default), or on a source-0 obstacle, the snapshot is exactly the old one;
- the soft row is the hard row's a, ċ, v_obs and gains, with the forecast barrier;
- a receding limb, a young track and the status slack behave as documented;
- in the QP the hard row holds and the soft one yields at rho/(1 + rho) (unit leverage).

Drives the real ConstraintBuilder (_cbf_builder_harness) and a real OSQP. No ROS.
"""

import numpy as np
import osqp
import pytest
import scipy.sparse as sparse

from _cbf_builder_harness import NV, make_builder, make_obstacle, run
from franka_experiments.utils.cbf_qp_assembly import (
    build_osqp_A, build_osqp_bounds, build_row_rhs)
from franka_experiments.utils.cbf_state_rows import (
    G_OBS, G_PRED, HARD_GROUPS, N_SLACK, NX, human_prediction_tightening,
    latency_compensation_terms)
from franka_experiments.utils.logging_utils import format_cbf_diag

PR = (0.5, 0.0, 0.5)
PH = (0.5, -0.25, 0.5)
N_HAT = np.array([0.0, 1.0, 0.0])            # obstacle -> control point: +y closes
POS_COV = np.diag([0.02, 0.03, 0.02]) ** 2
PV_COV = np.diag([1e-4, 2e-4, 1e-4])
VEL_COV = np.diag([0.15, 0.2, 0.15]) ** 2


def _limb(k, *, v=(0.0, 1.0, 0.0), frames_seen=20, src=1, d0=0.40):
    ob = make_obstacle(d=d0 - 0.01 * k, pr=PR, ph=PH, v=v, frames_seen=frames_seen,
                       cov=VEL_COV, pos_cov=POS_COV, pv_cov=PV_COV, cp_label='1:fr3_link5#0')
    return [ob._replace(src=src)]


def _build(on, obstacles=_limb, *, depth_floor=0.0, **over):
    b = make_builder(obstacle_velocity_source='tracker', enable_human_prediction=on, **over)
    b._P.retreat_excess_speed_max = depth_floor     # read with getattr, not in the harness
    return b, run(b, obstacles, n_frames=6, qdot=0.05)


def _assert_identical(a, b):
    for field in ('A', 'h_bar', 'jdot_qdot', 'G', 'v_obs', 'group'):
        np.testing.assert_array_equal(getattr(a, field), getattr(b, field))


# ── The pure term ────────────────────────────────────────────────────────────

def test_shift_is_signed_and_clamped_margin_is_the_propagated_sigma():
    kw = dict(horizon_s=0.3, k_sigma=1.0, margin_max=0.15, v_max=2.0)
    shift, margin = human_prediction_tightening(N_HAT, (0.0, 1.0, 0.0), POS_COV, PV_COV,
                                                VEL_COV, **kw)
    assert shift == pytest.approx(0.3)
    P = POS_COV + 0.3 * (PV_COV + PV_COV.T) + 0.09 * VEL_COV
    assert margin == pytest.approx(np.sqrt(N_HAT @ P @ N_HAT))
    _, same = latency_compensation_terms(N_HAT, None, None, POS_COV, PV_COV, VEL_COV,
                                         t_blind=0.3, k_sigma=1.0, margin_max=0.15)
    assert margin == same
    assert human_prediction_tightening(N_HAT, (0.0, -0.8, 0.0), None, None, None,
                                       **kw) == pytest.approx((-0.24, 0.0))
    assert human_prediction_tightening(N_HAT, (0.0, 9.0, 0.0), None, None, None,
                                       **kw)[0] == pytest.approx(0.6)       # 2 m/s clamp
    assert human_prediction_tightening(N_HAT, None, None, None, VEL_COV * 100,
                                       **kw) == pytest.approx((0.0, 0.15))  # margin clamp


# ── The default path ─────────────────────────────────────────────────────────

def test_off_adds_no_row_and_leaves_the_snapshot_alone():
    _, off = _build(False)
    assert not np.any(off.group == G_PRED)
    assert not np.any(off.G[:, NV + G_PRED])
    # On, but nothing to predict: a source-0 obstacle, a young track
    for obstacles in (lambda k: _limb(k, src=0), lambda k: _limb(k, frames_seen=1)):
        _assert_identical(_build(True, obstacles)[1], _build(False, obstacles)[1])


# ── The soft row ─────────────────────────────────────────────────────────────

def test_the_soft_row_is_the_hard_row_moved_to_the_forecast():
    b, con = _build(True)
    hard = int(np.flatnonzero(con.group == G_OBS)[0])
    soft = int(np.flatnonzero(con.group == G_PRED)[0])
    np.testing.assert_array_equal(con.A[soft], con.A[hard])
    assert con.jdot_qdot[soft] == con.jdot_qdot[hard]
    assert con.v_obs[soft] == con.v_obs[hard]
    shift, margin = human_prediction_tightening(
        N_HAT, (0.0, 1.0, 0.0), POS_COV, PV_COV, VEL_COV, horizon_s=0.3, k_sigma=1.0,
        margin_max=0.15, v_max=2.0)
    assert con.h_bar[soft] == pytest.approx(con.h_bar[hard] - shift - margin)
    assert b.diag_hpred == pytest.approx(shift + margin)
    # Its own slack column, and only that one
    assert con.G[soft, NV + G_PRED] == -1.0 and con.G[soft, NV + G_OBS] == 0.0
    assert con.G[hard, NV + G_PRED] == 0.0
    assert con.links[soft] == 'pred:' + con.links[hard]
    # The hard row itself is exactly the prediction-off row
    _, off = _build(False)
    assert con.h_bar[hard] == off.h_bar[hard]


def test_a_receding_limb_forecast_never_binds_before_the_hard_row():
    _, con = _build(True, lambda k: _limb(k, v=(0.0, -0.8, 0.0)))
    hard, soft = (int(np.flatnonzero(con.group == g)[0]) for g in (G_OBS, G_PRED))
    assert con.h_bar[soft] > con.h_bar[hard]


def test_the_soft_row_takes_its_hard_row_s_gains_and_depth_floor():
    _, con = _build(True, extra_k0_cbf=9.0, extra_k1_cbf=6.0, depth_floor=0.2)
    hard, soft = (int(np.flatnonzero(con.group == g)[0]) for g in (G_OBS, G_PRED))
    assert con.k0_row[soft] == con.k0_row[hard] == 9.0
    assert con.k1_row[soft] == con.k1_row[hard] == 6.0
    assert con.h_pen_row[soft] == con.h_pen_row[hard] == pytest.approx(-0.2 * 6.0 / 9.0)
    _, zoned = _build(True, enable_zone_ladder=True)
    hard, soft = (int(np.flatnonzero(zoned.group == g)[0]) for g in (G_OBS, G_PRED))
    assert zoned.k0_row[soft] == zoned.k0_row[hard]
    assert zoned.k1_row[soft] == zoned.k1_row[hard]


def test_rate_cap_rows_stay_the_trailing_block():
    caps = dict(retreat_cap_enabled=True, link_speed_rows_enabled=True)
    _, on = _build(True, **caps)
    _, off = _build(False, **caps)
    assert on.n_cap == off.n_cap > 0
    np.testing.assert_array_equal(on.A[-on.n_cap:], off.A[-off.n_cap:])
    assert on.A.shape[0] == off.A.shape[0] + 1


# ── In the QP ────────────────────────────────────────────────────────────────

def _solve(con, qddot_nom, rho_pred, qdot):
    P = np.eye(NX)
    P[NV + G_OBS, NV + G_OBS] = 1000.0
    for g in range(N_SLACK):
        if g not in (G_OBS, G_PRED):
            P[NV + g, NV + g] = 1000.0
    P[NV + G_PRED, NV + G_PRED] = rho_pred
    q = np.zeros(NX)
    q[:NV] = -qddot_nom
    h_qp, _ = build_row_rhs(con, qdot, qdot, k0=25.0, k1=10.5, retreat_horizon=0.2,
                            speed_horizon=0.1)
    lb = np.concatenate([np.full(NV, -50.0), np.zeros(N_SLACK)])
    ub = np.concatenate([np.full(NV, 50.0), np.full(N_SLACK, 1e6)])
    l, u = build_osqp_bounds(con.G, h_qp, lb, ub)
    prob = osqp.OSQP()
    prob.setup(P=sparse.csc_matrix(P), q=q, A=build_osqp_A(con.G, NV, N_SLACK), l=l, u=u,
               eps_abs=1e-10, eps_rel=1e-10, max_iter=400000, verbose=False)
    res = prob.solve()
    assert res.info.status_val == osqp.constant('OSQP_SOLVED')
    return res.x, -h_qp       # x, and the row bounds b (aᵀq̈ + m·s ≥ b)


@pytest.mark.parametrize('rho', [0.5, 2.0, 20.0, 1000.0])
def test_the_hard_row_holds_and_the_forecast_yields_at_rho_over_one_plus_rho(rho):
    """Along a, the QP minimises ½(x - c)²/‖a‖² + ½ρ_obs(b_h - x)₊² + ½ρ(b_p - x)₊²
    (x = aᵀq̈, c = aᵀq̈_nom). With the hard row slack, the soft row is met for
    ρ‖a‖²/(1 + ρ‖a‖²) of its gap; once that falls short of the hard bound, the hard
    row takes over, as stiff as any obstacle row (ρ_obs = 1000)."""
    qdot = np.full(NV, 0.05)
    _, con = _build(True)
    hard, soft = (int(np.flatnonzero(con.group == g)[0]) for g in (G_OBS, G_PRED))
    a = con.A[hard]
    nominal = -0.5 * a / (a @ a)        # pushes toward the limb: aᵀq̈_nom = -0.5
    x, b = _solve(con, nominal, rho, qdot)
    qdd, s = x[:NV], x[NV:]
    c, aa = float(a @ nominal), float(a @ a)
    b_h, b_p = b[hard], b[soft]
    assert b_p > b_h > c                                           # both rows want more
    x_soft = (c + rho * aa * b_p) / (1.0 + rho * aa)
    if x_soft >= b_h:     # the forecast alone sets it, at rho/(1 + rho) of its gap
        expected = x_soft
        assert (a @ qdd - c) / (b_p - c) == pytest.approx(rho * aa / (1.0 + rho * aa), rel=1e-5)
    else:                 # the hard row binds, priced like every obstacle row
        expected = (c + aa * (1000.0 * b_h + rho * b_p)) / (1.0 + aa * (1000.0 + rho))
    assert a @ qdd == pytest.approx(expected, rel=1e-6)
    # The hard row gives at most what any obstacle row gives at rho_obs = 1000
    assert b_h - a @ qdd <= (b_h - c) / (1.0 + 1000.0 * aa) + 1e-6
    assert s[G_PRED] == pytest.approx(max(b_p - a @ qdd, 0.0), abs=1e-6)


# ── What the node reports ────────────────────────────────────────────────────

def test_status_and_diag_line_report_hard_rows_and_the_forecast_apart():
    assert G_PRED not in HARD_GROUPS and len(HARD_GROUPS) == N_SLACK - 1
    b, con = _build(True)
    slack = np.zeros(N_SLACK)
    slack[G_PRED] = 0.7
    n = con.A.shape[0]
    line = format_cbf_diag(
        now=1.0, con=con, rows=b, caps=(0.0, 0.0, 0.0, 0.0), h_qp=np.zeros(n),
        qdot=np.zeros(NV), qdot_cbf=np.zeros(NV), qddot_safe=np.zeros(NV),
        qddot_nom=np.zeros(NV), qddot_real=np.zeros(NV), slack=slack, n_active_cps=0,
        vel_ratio=np.zeros(NV), vel_bite=np.zeros(NV, bool), slew_bite=np.zeros(NV, bool),
        cap_age=0.0)
    fields = dict(t.split('=', 1) for t in line.split() if '=' in t)
    assert not fields['link'].startswith('pred:')      # the line is about the hard row
    assert float(fields['s_pred']) == pytest.approx(0.7)
    assert float(fields['hpred']) == pytest.approx(b.diag_hpred, abs=1e-4)
    assert float(np.max(slack[list(HARD_GROUPS)])) == 0.0   # the status slack ignores it
