"""Per-row l1 obstacle slack (obstacle_slack_mode: row_l1) vs the shared quadratic
family slack, through the PRODUCTION objective builder and solver path.

CBF_QP_SIMPLIFICATION_ANALYSIS.md, section 6 R2: with one quadratic slack shared by
every obstacle row,
  (a) a binding row ALWAYS gives in against a nominal that pushes into it,
      s* = r / (1 + rho*||a||^2) > 0, i.e. a static penetration h_ss = -s/k0;
  (b) one unsatisfiable row relaxes every other obstacle row by the same amount.
The l1 per-row slack is an exact penalty: zero slack whenever the row is feasible,
and each row pays only for itself.
"""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter
from franka_experiments.utils.cbf_qp_assembly import build_qp_objective
from franka_experiments.utils.cbf_state_rows import (
    G_OBS, G_SC, N_SLACK, NV, NX, row_slack_slots)

RHO_OBS = 1000.0     # fr3_control.yaml rho_slack
RHO_L1 = 1.0e3       # fr3_control.yaml rho_slack_l1
EPS = 1.0e3          # fr3_control.yaml obstacle_row_slack_eps
EPS_REL = 1.0e-5     # fr3_control.yaml obstacle_row_osqp_eps_rel
SLOTS = 32


def _node(mode: str, qddot_nom: np.ndarray, box: float = 10.0):
    n_slot = SLOTS if mode == 'row_l1' else 0
    P_csc, qvec = build_qp_objective(NV, N_SLACK, {G_OBS: RHO_OBS, G_SC: 2e4},
                                     n_slot=n_slot, rho_l1=RHO_L1, eps=EPS)
    qvec[:NV] = -qddot_nom
    n_slack = N_SLACK + n_slot
    return NS(P=NS(qp_row_block=1, osqp_max_iter=20000,
                   accept_inaccurate_qp=True, k_brake=3.0,
                   obstacle_row_osqp_eps_rel=EPS_REL),
              _P_csc=P_csc, _qvec=qvec,
              _box_lb=np.r_[-box * np.ones(NV), np.zeros(n_slack)],
              _box_ub=np.r_[box * np.ones(NV), np.full(n_slack, 1e6)],
              _prev_nc=-1, _osqp_prob=None, _diag_slack=np.zeros(N_SLACK),
              _qp_fail_count=0, _fallback_count=[0, 0, 0],
              _n_slack=n_slack, _n_slot=n_slot,
              get_logger=lambda: NS(error=lambda *a, **k: None))


def _rows(mode: str, a_list, r_list):
    """G x <= h_qp for rows  a_i^T qddot + s_(col i) >= r_i, laid out as the
    row builder does: shared family column in 'family', own slot in 'row_l1'."""
    n_slot = SLOTS if mode == 'row_l1' else 0
    G = np.zeros((len(a_list), NX + n_slot))
    for i, a in enumerate(a_list):
        G[i, :NV] = -np.asarray(a, dtype=float)
        G[i, NX + i if n_slot else NV + G_OBS] = -1.0
    return G, -np.asarray(r_list, dtype=float)


def _solve(mode, a_list, r_list, qddot_nom):
    node = _node(mode, qddot_nom)
    G, h = _rows(mode, a_list, r_list)
    qdd, slack, _, _, level = CBFSafetyFilter._solve(node, G, h, len(a_list),
                                                     np.zeros(NV))
    assert level == 0, 'the full QP must solve, not fall back'
    return qdd, slack


def test_family_mode_objective_is_the_historical_one():
    P_csc, q = build_qp_objective(NV, N_SLACK, {G_OBS: RHO_OBS, G_SC: 2e4})
    expect = np.eye(NX)
    expect[NV + G_OBS, NV + G_OBS] = RHO_OBS
    expect[NV + G_SC, NV + G_SC] = 2e4
    np.testing.assert_array_equal(P_csc.toarray(), expect)
    np.testing.assert_array_equal(q, np.zeros(NX))


def test_row_slack_slots_follows_the_mode():
    assert row_slack_slots(NS()) == 0
    assert row_slack_slots(NS(obstacle_slack_mode='family',
                              obstacle_row_slack_slots=32)) == 0
    assert row_slack_slots(NS(obstacle_slack_mode='row_l1',
                              obstacle_row_slack_slots=32)) == 32


def test_quadratic_family_slack_leaks_l1_row_slack_does_not():
    """One feasible row, nominal pushing 5 rad/s^2 into it (r = 5 above a^T qdd_nom)."""
    a = np.zeros(NV); a[1] = 0.5                      # ||a|| = 0.5
    qdd_nom = np.zeros(NV)
    r = 5.0 * 0.5                                     # needs qdd_2 >= 5
    qdd_f, s_f = _solve('family', [a], [r], qdd_nom)
    qdd_l, s_l = _solve('row_l1', [a], [r], qdd_nom)
    s_pred = r / (1.0 + RHO_OBS * a @ a)              # closed form, section 6 R2
    assert s_f == pytest.approx(s_pred, rel=0.05)     # the leak exists ...
    assert a @ qdd_f < r - 0.5 * s_pred               # ... and the row is violated
    assert s_l < 1e-3                                 # exact penalty: no leak
    assert a @ qdd_l >= r - 1e-3                      # row satisfied


def test_an_unsatisfiable_row_no_longer_relaxes_the_others():
    """Row A needs qdd_1 >= 20 against a +-10 box (deficit 10); row B needs qdd_2 >= 1."""
    a_A = np.zeros(NV); a_A[0] = 1.0
    a_B = np.zeros(NV); a_B[1] = 1.0
    qdd_nom = np.zeros(NV)
    qdd_f, _ = _solve('family', [a_A, a_B], [20.0, 1.0], qdd_nom)
    qdd_l, s_l = _solve('row_l1', [a_A, a_B], [20.0, 1.0], qdd_nom)
    # family: the shared slack (~10) covers row B as well, so B is not enforced
    assert qdd_f[1] < 0.5
    # row_l1: B is enforced on its own; A pays exactly its box deficit
    assert qdd_l[1] >= 1.0 - 1e-3
    assert qdd_l[0] == pytest.approx(10.0, abs=1e-3)
    assert s_l == pytest.approx(10.0, rel=1e-3)       # reported, not hidden


@pytest.mark.parametrize('mode', ['family', 'row_l1'])
def test_with_no_active_row_the_command_is_the_nominal(mode):
    """22 rows, every one satisfied with margin by the nominal itself.

    Regression for the first row_l1 run: with OSQP's default eps_rel the linear
    slack prices (in q) loosened the dual tolerance and the command drifted
    ~3 rad/s^2 off the nominal while no row was active (CBFDIAG dnorm 4-7 with
    n_act=0). The filter must be the identity when nothing binds.
    """
    rng = np.random.default_rng(0)
    for _ in range(20):
        qdd_nom = rng.normal(0.0, 2.0, NV)
        A = rng.normal(0.0, 0.4, (22, NV))
        r = A @ qdd_nom - rng.uniform(1.0, 50.0, 22)      # all inactive
        qdd, slack = _solve(mode, list(A), list(r), qdd_nom)
        assert np.linalg.norm(qdd - qdd_nom) < 0.02
        assert slack < 1e-3
