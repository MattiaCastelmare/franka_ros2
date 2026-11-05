"""Exercise the production solver path under forced failures, not a mirror."""
from types import SimpleNamespace as NS

import numpy as np
import pytest
import scipy.sparse as sp

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter
from franka_experiments.utils.cbf_state_rows import NV, NX, N_SLACK
from franka_experiments.utils.cbf_hard_limits import apply_slew_limit, hard_accel_box
from franka_experiments.utils.cbf_diagnostics import family_violations, STATUS_FIELDS


@pytest.mark.parametrize('failure', ['max_iter', 'exception', 'nan'])
def test_failed_qp_brakes_even_when_nominal_and_slew_demand_acceleration(failure):
    class Solver:
        def update(self, **kw):
            if failure == 'exception':
                raise RuntimeError('injected solver failure')

        def solve(self):
            x = np.full(NX, np.nan) if failure == 'nan' else np.zeros(NX)
            return NS(x=x, info=NS(status_val=7, status=failure, iter=1))

    v = np.array([1., -1., .5, -.5, .2, -.2, .1])
    node = NS(P=NS(qp_row_block=1, osqp_max_iter=1, accept_inaccurate_qp=False,
                   k_brake=3.),
              _P_csc=sp.eye(NX, format='csc'), _qvec=-np.ones(NX)*8,
              _box_lb=np.r_[np.ones(NV)*3., np.zeros(N_SLACK)],
              _box_ub=np.r_[np.ones(NV)*4., np.full(N_SLACK, 1e6)],
              _state_lb=-np.ones(NV)*5., _state_ub=np.ones(NV)*5.,
              _prev_nc=0, _osqp_prob=Solver(), _diag_slack=np.ones(N_SLACK),
              _qp_fail_count=0, _fallback_count=[0, 0, 0],
              get_logger=lambda: NS(error=lambda *a, **k: None))
    command, slack, _, _, level = CBFSafetyFilter._solve(node, None, None, 0, v)
    np.testing.assert_allclose(command, -3.*v)
    assert np.all(v*command <= 0.)
    assert level == 2 and slack == 0.
    assert node._fallback_count == [0, 0, 1]
    assert node._osqp_prob is None and node._prev_nc == -1


def test_slew_cannot_override_a_required_velocity_recovery():
    q = np.array([0., -.7, 0., -2.35, 0., 1.57, .7])
    v = np.zeros(7); v[0] = 1.005
    lo, hi = hard_accel_box(q, v, acc_lb=-np.ones(7)*6., acc_ub=np.ones(7)*6.,
                           qdot_max=np.ones(7), v_margin=1., q_min=q-1., q_max=q+1.,
                           q_margin=0., brake_eta=1., dt=.01,
                           firmware_envelope=False, clip_to_limits=True)
    lo, hi = apply_slew_limit(lo, hi, np.ones(7)*4., .5)
    assert hi[0] == pytest.approx(-.5)
    assert v[0] + hi[0]*.01 <= 1. + 1e-12
    assert lo[0] <= hi[0]


def test_family_residuals_use_the_emitted_command_and_its_actual_slack():
    G = np.zeros((3, NX)); G[:, 0] = [-1., 1., -2.]
    G[0, NV] = G[2, NV] = -1.; G[1, NV+1] = -1.
    rhs = np.array([-2., -.5, -5.])
    cmd = np.zeros(NV); cmd[0] = 1.
    sl = np.zeros(N_SLACK); sl[:2] = [2., .25]
    raw, relaxed = family_violations(G, rhs, cmd, sl, np.array([0, 1, 0]))
    np.testing.assert_allclose(raw[:2], [3., 1.5])
    np.testing.assert_allclose(relaxed[:2], [1., 1.25])
    assert len(set(STATUS_FIELDS)) == len(STATUS_FIELDS)


def test_status_age_tracks_the_geometry_actually_used_not_a_newer_queued_frame():
    from test_iso_status_contract import _StatusStub, _status
    node = _StatusStub()
    node._obs = NS(t_cap=99.95, stamp=99.96)
    node._js = NS(stamp=99.999)
    con = NS(capture_stamp=99.8, built_stamp=99.99, build_wall_ms=4.3)
    out = dict(zip(STATUS_FIELDS, _status(node, con=con)))
    assert out['distance_capture_age_s'] == pytest.approx(.2)
    assert out['distance_receipt_age_s'] == pytest.approx(.04)
    assert out['constraint_snapshot_age_s'] == pytest.approx(.01)
    assert out['constraint_build_wall_ms'] == pytest.approx(4.3)
