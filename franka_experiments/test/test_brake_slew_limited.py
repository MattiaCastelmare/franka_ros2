"""The level-2 brake obeys the jerk bound where the state box lets it.

``qp_failure_policy: brake`` answers a failed QP with a closed form, which by
construction knows nothing about ``max_qddot_delta``. When the QP fails on only
SOME ticks that produced chatter: the command alternated between a slewed QP
solution and an unslewed brake, tick after tick, and the arm thrashed (measured
with the solver starved to 100 iterations, 32 % of ticks failing, in
sim_runs/task1_safe_fallback_20261104).

``brake_slew_limited`` intersects the braking box with the slew box through the
same ``apply_slew_limit`` and the same yield rule. These tests pin the two
halves of that rule: it SMOOTHS where the boxes overlap, and it GETS OUT OF THE
WAY where they do not — because the state box is what bounds |q̇| after one
integration step, and a brake that respects the jerk bound instead of the state
box is the bug this whole task is about.
"""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter
from franka_experiments.utils.cbf_state_rows import NV


def _node(*, brake_slew, prev=None, delta=None, lb=-5.0, ub=5.0):
    n = NS(P=NS(brake_slew_limited=brake_slew, slew_box_enabled=True),
           _state_lb=np.full(NV, lb), _state_ub=np.full(NV, ub))
    if prev is not None:
        n._qddot_prev = np.asarray(prev, dtype=float)
    if delta is not None:
        n._diag_slew_delta_eff = delta
    return n


def test_off_is_the_plain_state_box():
    lo, hi = CBFSafetyFilter._brake_box(
        _node(brake_slew=False, prev=np.zeros(NV), delta=0.5))
    assert np.allclose(lo, -5.0) and np.allclose(hi, 5.0)


def test_on_it_is_the_state_box_intersected_with_the_slew_window():
    """q̈_prev = 2, Δ = 0.5 → the brake may move to [1.5, 2.5] this tick, which
    is inside the state box, so that window is what comes back."""
    lo, hi = CBFSafetyFilter._brake_box(
        _node(brake_slew=True, prev=np.full(NV, 2.0), delta=0.5))
    assert np.allclose(lo, 1.5) and np.allclose(hi, 2.5)


def test_a_brake_the_state_box_demands_is_not_deferred_by_the_jerk_bound():
    """The yield rule, which is the whole safety argument.

    The state box says q̈ ≤ −4 (the joint is over its velocity limit), the slew
    window is [1.5, 2.5]. They are disjoint, so the state box is handed back
    WHOLE and the brake is free to jump — the jerk bound is never the reason
    the arm does not stop.
    """
    n = _node(brake_slew=True, prev=np.full(NV, 2.0), delta=0.5)
    n._state_ub = np.full(NV, -4.0)
    lo, hi = CBFSafetyFilter._brake_box(n)
    assert np.allclose(hi, -4.0), 'the state box wins where the two are disjoint'
    assert np.all(lo <= hi + 1e-12)


def test_the_box_is_always_inside_the_state_box():
    """Whatever the slew window is, the result cannot leave the state box."""
    rng = np.random.default_rng(5)
    for _ in range(200):
        lb = rng.uniform(-8, 2, NV)
        ub = lb + rng.uniform(0, 8, NV)
        n = _node(brake_slew=True, prev=rng.uniform(-6, 6, NV),
                  delta=float(rng.uniform(0.05, 3.0)))
        n._state_lb, n._state_ub = lb, ub
        lo, hi = CBFSafetyFilter._brake_box(n)
        assert np.all(lo >= lb - 1e-12) and np.all(hi <= ub + 1e-12)
        assert np.all(lo <= hi + 1e-12)


@pytest.mark.parametrize('missing', ['prev', 'delta'])
def test_a_node_with_no_previous_command_brakes_on_the_plain_state_box(missing):
    """Nothing to be continuous WITH, so there is nothing to smooth."""
    kw = dict(brake_slew=True, prev=np.zeros(NV), delta=0.5)
    kw['prev' if missing == 'prev' else 'delta'] = None
    lo, hi = CBFSafetyFilter._brake_box(_node(**kw))
    assert np.allclose(lo, -5.0) and np.allclose(hi, 5.0)


def test_successive_ticks_walk_to_the_braking_value_instead_of_jumping():
    """The property the chatter complaint is about: from q̈ = 3, a brake that
    wants −3 arrives over several ticks, each step bounded by Δ."""
    want = -3.0
    prev, delta, seen = 3.0, 0.5, []
    for _ in range(20):
        n = _node(brake_slew=True, prev=np.full(NV, prev), delta=delta)
        lo, hi = CBFSafetyFilter._brake_box(n)
        prev = float(np.clip(want, lo, hi)[0])
        seen.append(prev)
    steps = np.abs(np.diff([3.0] + seen))
    assert np.all(steps <= delta + 1e-12), 'every step inside the jerk bound'
    assert seen[-1] == pytest.approx(want), 'and it does arrive'
