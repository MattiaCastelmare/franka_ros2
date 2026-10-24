"""obstacle_velocity_fast_trust: a young track is trusted only when its closing
speed clears k sigma of its own covariance.

The gate exists for a thrown object (rosbag/ball_throws_2): the span gate holds
every track's velocity at 0 for ~265 ms and the ball arrived first. What must
hold, and what these tests pin:

* a young, FAST, CONFIDENT track reaches the QP (v_obs > 0), and gets the fast
  ceiling rather than the 2 m/s one;
* a young track whose speed is inside its own uncertainty — the prior-dominated
  two-frame track the span gate was written against — does NOT;
* a receding track never does (the gate may only tighten);
* with the flag off, the rows are bit-identical to the old behaviour.

Drives the real ConstraintBuilder through _cbf_builder_harness. No ROS.
"""

import numpy as np

from _cbf_builder_harness import make_builder, make_obstacle, run

# n̂ = (pr − ph)/‖·‖ = +y for this geometry, so a velocity along +y is CLOSING.
PR = (0.5, 0.0, 0.5)
PH = (0.5, -0.25, 0.5)


def _v_obs(fast, v, cov, frames_seen=4, vobs=False, **over):
    """Largest per-row v_obs. vobs=False exercises the conditioned v_o path
    (_obstacle_speed_tracked + median), vobs=True the n̂ᵀv term inside ḣ."""
    over.setdefault('obstacle_velocity_min_frames', 8)   # the robot's gate
    b = make_builder(obstacle_velocity_source='tracker',
                     obstacle_velocity_fast_trust=fast, **over)
    b._P.enable_vobs_in_hdot = vobs
    con = run(b, [make_obstacle(pr=PR, ph=PH, v=v, frames_seen=frames_seen,
                                cov=cov, track_id=7)], n_frames=3, qdot=0.0)
    return float(np.max(con.v_obs)) if len(con.v_obs) else 0.0


def test_young_fast_confident_track_reaches_the_qp():
    for vobs in (False, True):
        assert _v_obs(False, (0.0, 3.0, 0.0), np.eye(3) * 0.01, vobs=vobs) == 0.0
        assert _v_obs(True, (0.0, 3.0, 0.0), np.eye(3) * 0.01, vobs=vobs) > 2.5


def test_fast_track_gets_the_fast_ceiling_not_the_2_mps_one():
    for vobs in (False, True):
        v = _v_obs(True, (0.0, 5.0, 0.0), np.eye(3) * 0.01, vobs=vobs,
                   obstacle_velocity_max=2.0, obstacle_velocity_fast_max=6.0)
        assert v > 4.5


def test_prior_dominated_young_track_is_not_trusted():
    # sigma 1 m/s (the sigma_v0 prior): 1.5 - 2*1 < 1.0
    assert _v_obs(True, (0.0, 1.5, 0.0), np.eye(3) * 1.0) == 0.0


def test_slow_young_track_is_not_trusted():
    assert _v_obs(True, (0.0, 0.8, 0.0), np.eye(3) * 1e-4) == 0.0


def test_receding_fast_track_is_never_trusted():
    assert _v_obs(True, (0.0, -3.0, 0.0), np.eye(3) * 0.01) <= 0.0
    # Not even inside ḣ, where a TRUSTED track's signed value would be used:
    # a young receding track must not be trusted at all.
    assert _v_obs(True, (0.0, -3.0, 0.0), np.eye(3) * 0.01, vobs=True) == 0.0


def test_too_few_frames_is_not_trusted():
    assert _v_obs(True, (0.0, 3.0, 0.0), np.eye(3) * 0.01, frames_seen=2,
                  obstacle_velocity_fast_min_frames=3) == 0.0


def test_flag_off_is_bit_identical_to_a_builder_without_the_block():
    kw = dict(pr=PR, ph=PH, v=(0.0, 3.0, 0.0), frames_seen=4, cov=np.eye(3) * 0.01, track_id=7)
    a = run(make_builder(obstacle_velocity_source='tracker', obstacle_velocity_fast_trust=False),
            [make_obstacle(**kw)], n_frames=5, qdot=0.05)
    b = run(make_builder(obstacle_velocity_source='tracker', obstacle_velocity_fast_trust=False,
                         obstacle_velocity_fast_speed=0.1, obstacle_velocity_fast_k_sigma=0.0),
            [make_obstacle(**kw)], n_frames=5, qdot=0.05)
    for x, y in ((a.A, b.A), (a.h_bar, b.h_bar), (a.v_obs, b.v_obs)):
        np.testing.assert_array_equal(x, y)
