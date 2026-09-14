"""Obstacle link-speed cap scaled with d_safe.

With a fixed 0.20 s blind time the cap at 5 cm was 0.25 m/s whatever d_safe
said, so lowering d_safe could not bring the arm closer. The obstacle term is
now v_at_d_safe * gap / d_safe; the self-collision term keeps the fixed time.
"""

import numpy as np
import pytest

from franka_experiments.utils.cbf_state_rows import (
    link_speed_cap, obstacle_link_speed_cap)


def test_reproduces_the_fixed_blind_time_at_d_safe_020():
    for d in np.linspace(0.0, 0.5, 51):
        assert (obstacle_link_speed_cap(d, v_max=1.3, d_safe=0.20, v_at_d_safe=1.0)
                == pytest.approx(link_speed_cap(d, v_max=1.3, reaction_s=0.20)))


def test_the_cap_scales_with_d_safe():
    for ds in (0.03, 0.10, 0.20):
        kw = dict(v_max=5.0, d_safe=ds, v_at_d_safe=1.0)
        assert obstacle_link_speed_cap(ds, **kw) == pytest.approx(1.0)
        assert obstacle_link_speed_cap(0.5 * ds, **kw) == pytest.approx(0.5)
        assert obstacle_link_speed_cap(0.0, **kw) == 0.0


def test_a_smaller_d_safe_allows_a_faster_final_approach():
    # 5 cm from the obstacle: 0.25 m/s at d_safe=0.20, only v_max at 0.03.
    assert obstacle_link_speed_cap(0.05, v_max=1.3, d_safe=0.20,
                                   v_at_d_safe=1.0) == pytest.approx(0.25)
    assert obstacle_link_speed_cap(0.05, v_max=1.3, d_safe=0.03,
                                   v_at_d_safe=1.0) == pytest.approx(1.3)


def test_a_zero_d_safe_does_not_divide_by_zero():
    assert np.isfinite(obstacle_link_speed_cap(0.05, v_max=1.3, d_safe=0.0,
                                               v_at_d_safe=1.0))
