"""retreat_excess_speed_max: the k0 term of a violated obstacle row sees h̄ no deeper than
-dv·k1/k0, so the row's rest speed v_obs + (k0/k1)·|h̄| exceeds the obstacle's own speed by
at most dv. Rows without a floor (−inf) and an unviolated row are untouched.
"""
from types import SimpleNamespace

import numpy as np

from franka_experiments.utils.cbf_qp_assembly import build_row_rhs

K0, K1, DV = 25.0, 10.5, 0.25


def _con(h_bar, h_pen):
    n = len(h_bar)
    return SimpleNamespace(
        A=np.zeros((n, 7)), v_obs=np.zeros(n), h_bar=np.asarray(h_bar, float),
        jdot_qdot=np.zeros(n), b_ff=None, n_cap=0, n_rtr=0,
        k0_row=None, k1_row=None, h_pen_row=h_pen)


def _rest_speed(h_qp):
    # A = 0, v_obs = 0: h_qp = k0·h_k0, and the row rests where k1·v = -h_qp.
    return -h_qp / K1


def test_deep_violation_is_capped_at_dv():
    floor = -DV * K1 / K0
    con = _con([-0.30, -0.01, 0.10], np.array([floor, floor, floor]))
    h_qp, _ = build_row_rhs(con, np.zeros(7), np.zeros(7), k0=K0, k1=K1, retreat_horizon=0.15, speed_horizon=0.2)
    v = _rest_speed(h_qp)
    assert np.isclose(v[0], DV)                       # 0.30 m deep would have rested at 0.71 m/s
    assert np.isclose(v[1], 0.01 * K0 / K1)           # shallow: below the cap, unchanged
    assert np.isclose(h_qp[2], K0 * 0.10)             # not violated: unchanged


def test_no_floor_is_the_previous_behaviour():
    con = _con([-0.30, -0.30], np.array([-np.inf, -np.inf]))
    ref = _con([-0.30, -0.30], None)
    h1, _ = build_row_rhs(con, np.zeros(7), np.zeros(7), k0=K0, k1=K1, retreat_horizon=0.15, speed_horizon=0.2)
    h2, _ = build_row_rhs(ref, np.zeros(7), np.zeros(7), k0=K0, k1=K1, retreat_horizon=0.15, speed_horizon=0.2)
    assert np.array_equal(h1, h2)
