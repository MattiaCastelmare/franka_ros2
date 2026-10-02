"""utils.ballistic_prediction: predicted impact points as extra obstacle hits.

Pins: a fast, confident track heading at a control point produces one hit at
the predicted position with the predicted gap; a slow, uncertain, receding or
missing track produces none; the input results are never mutated; disabled is
a no-op.
"""

from types import SimpleNamespace

import numpy as np

from franka_experiments.utils.ballistic_prediction import (
    PREDICTED_CLUSTER_ID, PredictionConfig, add_predicted_hits, predict_trajectory)
from franka_experiments.utils.distance_engine import ControlPointResult

CFG = PredictionConfig(enabled=True, horizon_s=0.5, step_s=0.005, min_speed=1.0,
                       k_sigma=2.0, min_frames=3, object_radius_m=0.035,
                       publish_gap_m=0.30, min_time_s=0.03, assume_ballistic=False)


def _cp(point=(0.5, 0.0, 0.5), radius=0.06):
    return ControlPointResult(point=np.array(point, float), seg_idx=0, cp_idx=0,
                              radius=radius, start_link='fr3_link4', end_link='fr3_link5')


def _track(p, v, sigma=0.05, frames=5, mode=None):
    t = SimpleNamespace(position=np.array(p, float), velocity=np.array(v, float),
                        velocity_cov=np.eye(3) * sigma ** 2, frames_seen=frames)
    if mode is not None:
        t.mode_prob = np.array(mode, float)
    return t


def test_ball_heading_at_the_cp_gives_one_hit_at_the_predicted_point():
    cp = _cp()
    ball = _track(p=(1.5, 0.0, 0.5), v=(-3.0, 0.0, 0.0))    # straight at the CP
    out, n = add_predicted_hits([cp], [ball], CFG)
    assert n == 1
    (hit,) = out[0].extras
    assert hit.cluster_id == PREDICTED_CLUSTER_ID
    assert hit.distance == 0.0                              # predicted contact
    # the predicted point is on the ball's line, near the CP, not where the ball is now
    assert abs(hit.point[0] - 0.5) < 0.2 and abs(hit.point[1]) < 1e-9
    assert hit.direction @ (cp.point - hit.point) > 0       # obstacle -> CP
    assert cp.extras == []                                  # input untouched


def test_gap_of_a_near_miss_is_the_predicted_miss_distance():
    ball = _track(p=(1.5, 0.25, 0.5), v=(-3.0, 0.0, 0.0))   # passes 0.25 m off the CP centre
    out, n = add_predicted_hits([_cp()], [ball], CFG)
    assert n == 1
    assert abs(out[0].extras[0].distance - (0.25 - 0.06 - 0.035)) < 0.01


def test_far_miss_slow_uncertain_receding_and_young_give_nothing():
    cases = [
        _track(p=(1.5, 0.8, 0.5), v=(-3.0, 0.0, 0.0)),                 # far miss
        _track(p=(1.5, 0.0, 0.5), v=(-0.8, 0.0, 0.0)),                 # slow
        _track(p=(1.5, 0.0, 0.5), v=(-1.5, 0.0, 0.0), sigma=0.5),      # 1.5 - 2*0.5 < 1
        _track(p=(1.0, 0.0, 0.5), v=(3.0, 0.0, 0.0)),                  # receding
        _track(p=(1.5, 0.0, 0.5), v=(-3.0, 0.0, 0.0), frames=2),       # too young
    ]
    for t in cases:
        assert add_predicted_hits([_cp()], [t], CFG)[1] == 0


def test_gravity_follows_the_imm_mode():
    t_bal = _track(p=(0, 0, 1), v=(1, 0, 0), mode=(0.3, 0.7))
    t_gen = _track(p=(0, 0, 1), v=(1, 0, 0), mode=(0.8, 0.2))
    taus, Pb = predict_trajectory(t_bal, CFG)
    _, Pg = predict_trajectory(t_gen, CFG)
    assert abs(Pb[-1, 2] - (1 - 0.5 * 9.81 * taus[-1] ** 2)) < 1e-9
    assert Pg[-1, 2] == 1.0


def test_disabled_is_a_no_op():
    cps = [_cp()]
    out, n = add_predicted_hits(cps, [_track(p=(1.5, 0, 0.5), v=(-3, 0, 0))],
                                PredictionConfig(enabled=False))
    assert n == 0 and out is cps


def test_replace_mode_substitutes_the_hit_of_the_same_track():
    from dataclasses import replace
    cfg = PredictionConfig(**{**CFG.__dict__, 'replace_current': True})
    cp = replace(_cp(), distance=0.4, direction=np.array([1.0, 0, 0]),
                 closest_obstacle_point=np.array([1.0, 0.0, 0.5]), cluster_id=3)
    ball = _track(p=(1.0, 0.25, 0.5), v=(-3.0, 0.0, 0.0))
    ball.track_id = 7
    out, n = add_predicted_hits([cp], [ball], cfg, track_id_of=lambda cid, p: 7 if cid == 3 else 0)
    assert n == 1
    r = out[0]
    assert r.cluster_id == PREDICTED_CLUSTER_ID and r.extras == []     # replaced, not added
    assert r.distance <= 0.4                                           # never less conservative
    assert abs(r.direction[0]) < 0.2                                   # ⟂ to the flight, not along it


def test_replace_mode_leaves_other_obstacles_alone():
    from dataclasses import replace
    cfg = PredictionConfig(**{**CFG.__dict__, 'replace_current': True})
    cp = replace(_cp(), distance=0.2, direction=np.array([0, 1.0, 0]),
                 closest_obstacle_point=np.array([0.5, -0.3, 0.5]), cluster_id=1)   # a person
    ball = _track(p=(1.5, 0.0, 0.5), v=(-3.0, 0.0, 0.0))
    ball.track_id = 7
    out, n = add_predicted_hits([cp], [ball], cfg, track_id_of=lambda cid, p: 2 if cid == 1 else 0)
    assert out[0].cluster_id == 1 and out[0].distance == 0.2           # person row intact
    assert n == 1 and out[0].extras[0].cluster_id == PREDICTED_CLUSTER_ID
