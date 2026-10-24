"""The scenario commander's laws ARE cbf_scenarios.run_scenario's (spec Phase 4).

run_scenario computes its nominal inline, so there is nothing to import. Instead
this extracts those exact source lines with ``inspect`` (the way
approach_envelope.py patches run_scenario) and executes them on random states.
The results must be bit-equal to simexp.scenario_laws. A change on either side
fails here.
"""
import importlib
import inspect
import os
import sys
import textwrap

import numpy as np
import pytest

from simexp import scenario_laws as L

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
JOINT_LINE = 'qddot_nom = -12.0 * (q - goal) - 2.0 * np.sqrt(12.0) * qdot'
CART_LINES = [
    'a_task = 25.0 * e + 10.0 * (cart_ref - v_ee)',
    'JJt = Jp_ee @ Jp_ee.T + (0.05 ** 2) * np.eye(3)',
    'J_pinv = Jp_ee.T @ np.linalg.inv(JJt)',
    'qddot_nom = J_pinv @ (a_task - Jpd_ee @ qdot)',
    'qddot_nom -= 6.0 * (np.eye(NV) - J_pinv @ Jp_ee) @ qdot',
]


@pytest.fixture(scope='module')
def source():
    sys.path.insert(0, os.path.join(REPO, 'franka_experiments', 'scripts'))
    C = importlib.import_module('cbf_scenarios')
    lines = [ln.strip() for ln in inspect.getsource(C.run_scenario).splitlines()]
    return C, lines


def test_the_laws_are_still_written_as_restated(source):
    _, lines = source
    for ln in [JOINT_LINE] + CART_LINES:
        assert ln in lines, f'cbf_scenarios.run_scenario no longer contains: {ln}'


def test_joint_pd_is_bit_equal(source):
    C, _ = source
    rng = np.random.default_rng(0)
    for _ in range(50):
        q, qdot, goal = (rng.normal(size=7) for _ in range(3))
        ns = {'np': np, 'q': q, 'qdot': qdot, 'goal': goal}
        exec(JOINT_LINE, ns)
        assert np.array_equal(L.joint_pd(q, qdot, goal), ns['qddot_nom'])


def test_cartesian_is_bit_equal(source):
    C, _ = source
    rng = np.random.default_rng(1)
    for _ in range(50):
        Jp, Jpd = rng.normal(size=(3, 7)), rng.normal(size=(3, 7))
        qdot, e, v_ref, v_ee = rng.normal(size=7), rng.normal(size=3), rng.normal(size=3), \
            rng.normal(size=3)
        ns = {'np': np, 'NV': C.NV, 'Jp_ee': Jp, 'Jpd_ee': Jpd, 'qdot': qdot, 'e': e,
              'cart_ref': v_ref, 'v_ee': v_ee}
        exec(textwrap.dedent('\n'.join(CART_LINES)), ns)
        assert np.array_equal(L.cartesian(Jp, Jpd, qdot, e, v_ref, v_ee), ns['qddot_nom'])


def test_flange_kinematics_is_cbf_scenarios_filter(source):
    C, _ = source
    F = C.Filter(C.shipped_params())
    k = L.FlangeKinematics()
    rng = np.random.default_rng(2)
    for _ in range(5):
        q, qd = C.Q0 + rng.uniform(-0.4, 0.4, 7), rng.normal(size=7) * 0.3
        flange, Jp, Jpd = k.update(q, qd)
        F.control_points(q, qd)
        fl = F.kin.data.oMf[F.fids['fr3_link8']].translation
        Jp2, Jpd2 = F.kin.point_jacobian(F.fids['fr3_link8'], fl)
        assert np.array_equal(flange, fl) and np.array_equal(Jp, Jp2) and np.array_equal(Jpd, Jpd2)


def test_nominal_law_holds_q0_before_and_follows_the_reference_after():
    nl = L.NominalLaw({'law': 'cartesian', 'cart_ref': [0.12, 0.0, 0.0]},
                      q0=[0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70])
    q0 = np.array([0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70])
    np.testing.assert_array_equal(nl.hold(q0, np.zeros(7)), np.zeros(7))
    _, p0, fl = nl(0.0, q0, np.zeros(7))
    np.testing.assert_allclose(p0, fl)
    _, p1, _ = nl(1.0, q0, np.zeros(7))
    np.testing.assert_allclose(p1 - p0, [0.12, 0.0, 0.0])
    with pytest.raises(ValueError):
        L.NominalLaw({'law': 'pentagon'}, q0=q0)
