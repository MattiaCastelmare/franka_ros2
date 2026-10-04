"""human_distance → cbf_safety_filter: closest-point parameter and the LinkDistance track fields.

The geometry tests are pure numpy. The message tests need the built franka_msgs
(with the HumanArmState covariance fields) and are skipped otherwise.
"""

import numpy as np
import pytest

from franka_experiments.utils.capsule_geometry import (
    HumanArmGeometry, RobotGeometry, closest_point_on_segment, point_to_capsule_distance,
)


# ── Geometry ────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('point, alpha', [
    ([0.5, 1.0, 0.0], 0.5),     # interior
    ([-2.0, 0.3, 0.0], 0.0),    # clamped at the start
    ([3.0, -0.2, 0.0], 1.0),    # clamped at the end
])
def test_closest_point_parameter(point, alpha):
    a, b = np.zeros(3), np.array([1.0, 0.0, 0.0])
    got_alpha, closest = closest_point_on_segment(np.array(point, float), a, b)
    assert got_alpha == pytest.approx(alpha)
    assert np.allclose(closest, a + alpha * (b - a))


def test_degenerate_segment_returns_its_start():
    a = np.array([0.1, 0.2, 0.3])
    alpha, closest = closest_point_on_segment(np.ones(3), a, a.copy())
    assert alpha == 0.0 and np.allclose(closest, a)


def test_capsule_distance_subtracts_both_radii():
    capsule = {'p0': np.zeros(3), 'p1': np.array([0.0, 0.0, 1.0]), 'radius': 0.1}
    d, closest, alpha = point_to_capsule_distance(np.array([0.5, 0.0, 0.25]), capsule, point_radius=0.05)
    assert d == pytest.approx(0.5 - 0.05 - 0.1)
    assert alpha == pytest.approx(0.25) and np.allclose(closest, [0.0, 0.0, 0.25])


def test_human_capsules_carry_keypoint_indices_and_skip_invalid_ends():
    kps = np.array([[0.0, 0.0, 0.0], [0.3, 0.0, 0.0], [0.6, 0.0, 0.0], [0.7, 0.0, 0.0]])
    caps = HumanArmGeometry().build_capsules(kps, valid=np.array([False, True, True, True]))
    assert [c['name'] for c in caps] == ['human_forearm', 'human_hand']
    assert [c['indices'] for c in caps] == [(1, 2), (2, 3)]


def test_minimum_distance_reports_alpha_and_capsule():
    caps = HumanArmGeometry(safety_margin=0.0).build_capsules(
        np.array([[0.0, 0.0, 0.0], [0.4, 0.0, 0.0], [0.8, 0.0, 0.0], [0.9, 0.0, 0.0]]))
    cp = {'name': 'cp', 'position': np.array([0.1, 0.5, 0.0]), 'radius': 0.05}
    info = RobotGeometry(definitions=[]).minimum_distance_to_human([cp], caps)
    assert info['capsule']['name'] == 'human_upper_arm'
    assert info['alpha'] == pytest.approx(0.25)


# ── LinkDistance track fields ───────────────────────────────────────────────

franka_msgs = pytest.importorskip('franka_msgs.msg')
needs_new_msgs = pytest.mark.skipif(
    not hasattr(franka_msgs.HumanArmState(), 'velocity_covariance'),
    reason='franka_msgs built without the HumanArmState covariance fields',
)


def _capsule_with_tracks():
    vel = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 0.0]])
    P = np.stack([np.eye(3) * (k + 1) for k in range(4)])
    return {
        'indices': (1, 2),
        'velocities': vel,
        'covariances': (P, 10.0 * P, 0.1 * P),
        'frames_seen': np.array([0, 12, 5, 0]),
        'track_id': 5,
    }


def test_track_fields_interpolate_velocity_and_covariance():
    from franka_experiments.nodes.human_distance import HumanDistance
    ld = franka_msgs.LinkDistance()
    alpha = 0.25
    HumanDistance.fill_track_fields(ld, _capsule_with_tracks(), alpha)

    v = ld.obstacle_velocity
    assert (v.x, v.y, v.z) == pytest.approx((0.75, 0.5, 0.0))
    # (1 - a)^2 * 2 I + a^2 * 3 I, independent filters
    expected = (0.75**2 * 2.0 + 0.25**2 * 3.0)
    P_pp = np.asarray(ld.position_covariance).reshape(3, 3)
    P_vv = np.asarray(ld.velocity_covariance).reshape(3, 3)
    assert np.allclose(P_pp, expected * np.eye(3))
    assert np.allclose(P_vv, 10.0 * expected * np.eye(3))
    assert ld.frames_seen == 5           # the weaker endpoint
    assert ld.track_id == 5
    a = ld.obstacle_acceleration
    assert (a.x, a.y, a.z) == (0.0, 0.0, 0.0)


@needs_new_msgs
def test_arm_state_round_trip_keeps_covariances_of_valid_keypoints_only():
    from std_msgs.msg import Header
    from franka_experiments.utils.human_utils import build_arm_state_msg, extract_human_covariances
    P = np.stack([np.eye(3) * (k + 1) for k in range(4)])
    valid = np.array([True, True, False, True])
    msg = build_arm_state_msg(
        positions=np.zeros((4, 3)), velocities=np.zeros((4, 3)), visibilities=np.ones(4),
        measured=valid, keypoint_valid=valid, age=np.zeros(4), header=Header(), base_frame='fr3_link0',
        covariances=(P, 2.0 * P, 3.0 * P), frames_seen=np.array([4, 7, 9, 1]),
    )
    P_pp, P_vv, P_pv, frames_seen = extract_human_covariances(msg)
    assert np.allclose(P_pp[[0, 1, 3]], P[[0, 1, 3]])
    assert np.allclose(P_vv[3], 2.0 * P[3]) and np.allclose(P_pv[1], 3.0 * P[1])
    assert np.allclose(P_pp[2], 0.0)       # invalid keypoint: zeros, like its position
    assert frames_seen.tolist() == [4, 7, 0, 1]