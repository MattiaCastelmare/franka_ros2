"""cbf.enable_velocity_standoff — barrier moved out in proportion to v_app.

d_safe_eff = d_safe + min(time_s·v_app, max). Pure numpy, no ROS.
"""
import numpy as np
import pytest

from _cbf_builder_harness import make_builder, make_obstacle, run
from franka_experiments.utils.cbf_state_rows import velocity_standoff

# Default harness geometry: pr - ph = (0, 0.25, 0), so n̂ = +y (obstacle → CP).
N_HAT = np.array([0.0, 1.0, 0.0])
# Tracker source with no conditioning, so v_o = n̂ᵀv exactly.
TRACK = dict(obstacle_velocity_source='tracker', obstacle_velocity_track_deadband=0.0)
T_S, MAX = 0.20, 0.20


def _tracked(v_vec, frames=5):
    return [make_obstacle(v=v_vec, frames_seen=frames, track_id=7,
                          cp_label='fr3_link5#0')]


def _snap(on, obstacles, n_frames=1, **over):
    b = make_builder(**TRACK, enable_velocity_standoff=on,
                     velocity_standoff_time_s=T_S, velocity_standoff_max=MAX,
                     **over)
    return run(b, obstacles, n_frames=n_frames), b


def test_function_is_linear_one_sided_and_clamped():
    assert velocity_standoff(0.0, time_s=0.2, max_m=0.2) == 0.0
    assert velocity_standoff(-0.8, time_s=0.2, max_m=0.2) == 0.0
    assert velocity_standoff(0.3, time_s=0.2, max_m=0.2) == pytest.approx(0.06)
    assert velocity_standoff(0.6, time_s=0.2, max_m=0.2) == pytest.approx(0.12)
    assert velocity_standoff(5.0, time_s=0.2, max_m=0.2) == 0.2


def test_static_obstacle_is_identical_to_flag_off():
    off, _ = _snap(False, _tracked(0.0 * N_HAT))
    on, b = _snap(True, _tracked(0.0 * N_HAT))
    assert np.array_equal(on.h_bar, off.h_bar)
    assert np.array_equal(on.A, off.A)
    assert b.diag_hstand == 0.0


@pytest.mark.parametrize('v', [0.1, 0.3, 0.6, 1.0])
def test_barrier_moves_out_proportionally_to_closing_speed(v):
    off, _ = _snap(False, _tracked(v * N_HAT))
    on, b = _snap(True, _tracked(v * N_HAT))
    assert np.array_equal(on.A, off.A)
    assert (on.h_bar - off.h_bar)[0] == pytest.approx(-T_S * v, rel=1e-12)
    assert b.diag_hstand == pytest.approx(T_S * v)


def test_receding_obstacle_adds_nothing():
    off, _ = _snap(False, _tracked(-0.5 * N_HAT))
    on, _ = _snap(True, _tracked(-0.5 * N_HAT))
    assert np.array_equal(on.h_bar, off.h_bar)


def test_clamped_at_max():
    on, b = _snap(True, _tracked(1.8 * N_HAT))
    off, _ = _snap(False, _tracked(1.8 * N_HAT))
    assert (on.h_bar - off.h_bar)[0] == pytest.approx(-MAX, rel=1e-12)
    assert b.diag_hstand == MAX


def test_unconfirmed_track_adds_nothing():
    off, _ = _snap(False, _tracked(0.5 * N_HAT, frames=2))
    on, _ = _snap(True, _tracked(0.5 * N_HAT, frames=2))
    assert np.array_equal(on.h_bar, off.h_bar)


def test_rise_is_instant_decay_is_smoothed():
    alpha = 0.8
    seq = lambda k: _tracked((0.5 if k == 0 else 0.0) * N_HAT)
    off1, _ = _snap(False, seq, n_frames=2)
    on1, _ = _snap(True, seq, n_frames=2, velocity_standoff_alpha=alpha)
    # Frame 0 put 0.1 m in instantly; at frame 1 the obstacle has stopped and
    # the standoff only decays by one EMA step.
    assert (on1.h_bar - off1.h_bar)[0] == pytest.approx(-alpha * T_S * 0.5, rel=1e-12)
