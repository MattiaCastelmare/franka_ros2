import numpy as np
import pytest

FR3_QDDOT_MAX = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
FR3_QDOT_MAX  = np.array([2.62]*7)
FR3_Q_MAX     = np.array([2.9007, 1.8361, 2.9007, 0.0, 2.8763, 4.6216, 3.0508])
FR3_Q_MIN     = np.array([-2.9007,-1.8361,-2.9007,-3.0770,-2.8763, 0.4398,-3.0508])

COMMON = dict(
    qdot      = np.zeros(7),
    q         = np.zeros(7),
    dt        = 0.005,
    qddot_min = -FR3_QDDOT_MAX,
    qddot_max =  FR3_QDDOT_MAX,
    qdot_min  = -FR3_QDOT_MAX,
    qdot_max  =  FR3_QDOT_MAX,
    q_min     =  FR3_Q_MIN,
    q_max     =  FR3_Q_MAX,
)

def make_qp(solver='osqp'):
    try:
        from franka_experiments.utils.cbf_qp import CBFQP
        return CBFQP(nv=7, rho_slack=1000.0, solver=solver)
    except Exception:
        pytest.skip("CBFQP not available")

def test_unconstrained():
    qp = make_qp()
    qddot_nom = np.array([1.0, 0, 0, 0, 0, 0, 0])
    sol, slack = qp.solve(
        qddot_nom=qddot_nom,
        A_cbf=np.zeros((0,7)), b_cbf=np.zeros(0),
        **COMMON)
    assert sol is not None, "QP returned None"
    assert np.allclose(sol, qddot_nom, atol=1e-3), f"Expected qddot_nom, got {sol}"
    assert slack < 1e-4, f"Slack should be ~0, got {slack}"

def test_active_cbf():
    qp = make_qp()
    qddot_nom = np.array([-2.0, 0, 0, 0, 0, 0, 0])
    A_cbf = np.eye(1, 7)       # first joint only
    b_cbf = np.array([0.5])
    sol, slack = qp.solve(
        qddot_nom=qddot_nom,
        A_cbf=A_cbf, b_cbf=b_cbf,
        **COMMON)
    assert sol is not None
    assert sol[0] >= 0.5 - 1e-4, f"CBF violated: qddot[0]={sol[0]:.4f} < 0.5"
    assert slack < 1e-3

def test_infeasible_uses_slack():
    qp = make_qp()
    # Conflicting: qddot[0] >= 10 AND qddot[0] <= -10
    A_cbf = np.array([[ 1,0,0,0,0,0,0],
                       [-1,0,0,0,0,0,0]], dtype=float)
    b_cbf = np.array([10.0, 10.0])
    sol, slack = qp.solve(
        qddot_nom=np.zeros(7),
        A_cbf=A_cbf, b_cbf=b_cbf,
        **COMMON)
    assert sol is not None, "Should not return None thanks to slack"
    assert slack > 0.1, f"Slack should be positive, got {slack}"
