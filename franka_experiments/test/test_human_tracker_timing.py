"""filter_step_dt: the time step of the human tracker's Kalman filters.

The filters predict over the real elapsed time (a cap would predict too little motion and too
little uncertainty across a dropped stretch of frames) and restart when their state is stale.
"""

import pytest

from franka_experiments.utils.human_utils import filter_step_dt

DT = 1.0 / 30.0
STALE = 0.5   # reset_after_s


def test_first_step_takes_the_default():
    assert filter_step_dt(None, 12.0, DT, STALE) == (DT, False)


@pytest.mark.parametrize("gap", [DT, 0.1, 0.2, 0.3, 0.45, STALE])
def test_the_real_gap_is_used_up_to_the_stale_limit(gap):
    dt, restart = filter_step_dt(10.0, 10.0 + gap, DT, STALE)
    assert not restart and dt == pytest.approx(gap)


@pytest.mark.parametrize("now", [10.6, 13.0, 9.9, 10.0])
def test_a_stale_state_or_time_going_back_restarts(now):
    assert filter_step_dt(10.0, now, DT, STALE) == (DT, True)


def test_a_tiny_step_is_floored():
    assert filter_step_dt(10.0, 10.0 + 1e-6, DT, STALE) == (1e-3, False)
