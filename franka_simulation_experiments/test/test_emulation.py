"""simexp.emulation: cbf_scenarios' perception emulation, through the real message constructor.

The constants are pinned to the LINES of cbf_scenarios.run_scenario they copy,
so an edit there fails here instead of silently splitting columns 1 and 2.
"""
import inspect
import os

import numpy as np
import pytest
import yaml

from simexp import emulation as E

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))


@pytest.fixture(scope='module')
def C():
    return E._cbf_scenarios()


@pytest.fixture(scope='module')
def cfg():
    with open(os.path.join(REPO, 'franka_experiments', 'config', 'fr3_complete.yaml')) as f:
        return yaml.safe_load(f)


def test_constants_are_cbf_scenarios(C):
    src = inspect.getsource(C.run_scenario)
    assert inspect.signature(C.run_scenario).parameters['t_cam'].default == E.T_CAM
    assert 'rng.normal(0, 0.02, 3)' in src and E.V_SIGMA == 0.02
    assert 'vel_cov=(0.05 ** 2) * np.eye(3)' in src and E.VEL_COV == 0.05 ** 2
    assert 'pos_cov=(0.01 ** 2) * np.eye(3)' in src and E.POS_COV == 0.01 ** 2
    assert 'track_id=1' in src
    assert 'frames_seen = int((t_cap - s.first_seen) / frame_period) + 1' in src
    assert 'ph = p_s + s.r * (p_cp - p_s) / max(np.linalg.norm(p_cp - p_s), 1e-9)' in src


def test_nearest_is_by_surface_gap():
    cp = np.zeros(3)
    centres = np.array([[0.5, 0.0, 0.0], [0.0, 0.45, 0.0]])
    radii = np.array([0.20, 0.05])           # the first is farther but bigger
    k, gap, n, ph = E.nearest(cp, centres, radii, 0.06)
    assert k == 0
    assert gap == pytest.approx(0.5 - 0.20 - 0.06)
    np.testing.assert_allclose(n, [-1.0, 0.0, 0.0])          # obstacle -> CP
    np.testing.assert_allclose(ph, [0.30, 0.0, 0.0])          # on the sphere's surface


def test_frames_seen_counts_per_obstacle_from_first_sight():
    tr = E.TrackEmulator(2, 1 / 90.0, cam_noise=False, seed=0)
    assert tr.frames_seen(0, 1.0) == 1
    assert tr.frames_seen(0, 1.0 + 10 / 90.0 + 1e-9) == 11
    assert tr.frames_seen(1, 1.5) == 1                        # its own clock


def test_velocity_noise_is_cbf_scenarios(C):
    tr = E.TrackEmulator(1, 1 / 90.0, cam_noise=False, seed=3)
    rng = np.random.default_rng(3)
    np.testing.assert_allclose(tr.velocity([0.0, -0.3, 0.0]),
                               np.array([0.0, -0.3, 0.0]) + rng.normal(0, 0.02, 3))
    trn = E.TrackEmulator(1, 1 / 90.0, cam_noise=True, seed=4)
    rng = np.random.default_rng(4)
    np.testing.assert_allclose(trn.velocity(np.zeros(3)), C.camera_velocity_noise(rng)[0])


def test_messages_are_built_by_the_real_constructor(cfg):
    from franka_experiments.utils.perception_msgs import build_cp_messages, labelled_links
    cps = [dict(point=np.array([0.3, 0.0, 0.5]), seg_idx=4, cp_idx=0, radius=0.06,
                start_link='fr3_link7', end_link='fr3_link8'),
           dict(point=np.array([0.3, 0.0, 0.3]), seg_idx=3, cp_idx=0, radius=0.06,
                start_link='fr3_link6', end_link='fr3_link7')]
    centres = np.array([[0.3, 0.4, 0.5]])
    radii = np.array([0.05])
    res, ks, gaps = E.frame_results(cps, centres, radii)
    np.testing.assert_allclose(gaps, [0.4 - 0.11, np.hypot(0.4, 0.2) - 0.11])
    _, mld = build_cp_messages(cp_results=res, n_pts=500, stamp=_stamp(),
                               frame_id='fr3_link0', segment_links=cfg['robot']['segment_links'],
                               thresholds=cfg['distance']['thresholds'], fallback=2.0, zones={})
    tr = E.TrackEmulator(1, 1 / 90.0, cam_noise=False, seed=0)
    E.fill_track_fields(mld, res, ks, np.array([[0.0, -0.3, 0.0]]), tr, 2.0)
    assert len(mld.links) == 2 and all(ld.valid for ld in mld.links)
    by_link = {ld.robot_link_name: ld for _, ld in labelled_links(mld)}
    assert by_link['fr3_link8'].distance == pytest.approx(0.29)
    for ld in mld.links:
        assert ld.track_id == 1 and ld.frames_seen == 1
        np.testing.assert_allclose(np.reshape(ld.velocity_covariance, (3, 3)), E.VEL_COV * np.eye(3))
        np.testing.assert_allclose(np.reshape(ld.position_covariance, (3, 3)), E.POS_COV * np.eye(3))
        assert abs(ld.obstacle_velocity.y + 0.3) < 0.1


def test_a_cp_inside_the_sphere_reports_zero_not_negative():
    cps = [dict(point=np.array([0.0, 0.0, 0.0]), seg_idx=0, cp_idx=0, radius=0.06,
                start_link='a', end_link='b')]
    res, _, gaps = E.frame_results(cps, np.array([[0.05, 0.0, 0.0]]), np.array([0.05]))
    assert gaps[0] < 0 and res[0].distance == 0.0


def _stamp():
    from builtin_interfaces.msg import Time
    return Time(sec=2, nanosec=0)
