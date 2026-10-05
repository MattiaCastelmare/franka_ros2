"""human_validation: workspace box, segment lengths, learned lengths, torso check, hysteresis."""

import os

import numpy as np
import pytest
import yaml

from franka_experiments.utils.human_validation import (
    HumanValidator, in_box, segment_lengths, torso_plausible,
)

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'human_params.yaml')
DIRECT = np.ones(4, dtype=bool)

# shoulder, elbow, wrist, index: upper arm 0.30, forearm 0.25, hand 0.08
ARM = np.array([[0.5, 0.0, 0.6], [0.5, 0.0, 0.3], [0.5, 0.25, 0.3], [0.5, 0.33, 0.3]])


@pytest.fixture
def cfg():
    with open(CONFIG) as f:
        tracker = yaml.safe_load(f)['human_tracker']
    on = dict(enabled=True, check_workspace=True, check_segments=True, check_torso=True)
    return dict(tracker['validation'], **on), float(tracker['visibility_threshold'])


def _arm(forearm=0.25):
    arm = ARM.copy()
    arm[2:, 1] += forearm - 0.25        # move wrist and hand along y
    return arm


def test_in_box_and_segment_lengths():
    assert in_box(ARM, np.full(3, -1.0), np.full(3, 1.0)).all()
    assert not in_box([[2.0, 0.0, 0.0], [np.nan, 0, 0]], np.full(3, -1.0), np.full(3, 1.0)).any()
    np.testing.assert_allclose(segment_lengths(ARM, DIRECT), [0.30, 0.25, 0.08])
    # a borrowed depth (not direct) removes both segments it belongs to
    lengths = segment_lengths(ARM, np.array([True, True, False, True]))
    assert lengths[0] == pytest.approx(0.30) and np.isnan(lengths[1:]).all()


def test_disabled_passes_everything_through(cfg):
    c, vis = cfg
    v = HumanValidator(dict(c, enabled=False), vis)
    far = ARM + 10.0
    np.testing.assert_array_equal(v.filter_arm('left', far, DIRECT), far)
    assert v.torso_ok(np.full((2, 3), np.nan), None)
    assert v.engage_visibility_threshold == vis


def test_hysteresis_threshold(cfg):
    c, vis = cfg
    assert HumanValidator(c, vis).engage_visibility_threshold > vis


def test_workspace_drops_only_the_outside_keypoint(cfg):
    v = HumanValidator(*cfg)
    arm = ARM.copy()
    arm[0] = [5.0, 0.0, 0.6]            # shoulder far away
    out = v.filter_arm('left', arm, DIRECT)
    assert np.isnan(out[0]).all() and np.isfinite(out[1:]).all()
    assert v.rejects['workspace'] == 1


def test_implausible_segment_rejects_the_whole_arm(cfg):
    v = HumanValidator(*cfg)
    out = v.filter_arm('left', _arm(forearm=0.60), DIRECT)
    assert np.isnan(out).all() and v.rejects['segment_band'] == 1


def test_borrowed_depth_is_not_length_checked(cfg):
    v = HumanValidator(*cfg)
    direct = np.array([True, True, False, True])
    out = v.filter_arm('left', _arm(forearm=0.60), direct)
    assert np.isfinite(out).all()


def test_learned_lengths_reject_a_different_arm(cfg):
    c, vis = cfg
    v = HumanValidator(c, vis)
    for _ in range(c['learn_frames']):
        assert np.isfinite(v.filter_arm('left', _arm(0.25), DIRECT)).all()
    assert np.isfinite(v.filter_arm('left', _arm(0.27), DIRECT)).all()   # +8 %
    assert np.isnan(v.filter_arm('left', _arm(0.32), DIRECT)).all()      # +28 %, in band
    assert v.rejects['learned_length'] == 1
    # the other arm and a new person are learned from scratch
    assert np.isfinite(v.filter_arm('right', _arm(0.32), DIRECT)).all()
    v.reset()
    assert np.isfinite(v.filter_arm('left', _arm(0.32), DIRECT)).all()


def test_torso_check(cfg):
    c, _ = cfg
    w, l = c['shoulder_width_band'], c['torso_length_band']
    shoulders = np.array([[0.8, -0.2, 0.6], [0.8, 0.2, 0.6]])
    hips = np.array([[0.8, -0.15, 0.1], [0.8, 0.15, 0.1]])
    assert torso_plausible(shoulders, hips, w, l)
    assert torso_plausible(shoulders, np.full((2, 3), np.nan), w, l)       # hips out of frame
    assert not torso_plausible(shoulders * [1, 3, 1], hips, w, l)         # 1.2 m wide
    assert not torso_plausible(shoulders, hips + [0, 0, 1.0], w, l)       # hips above shoulders
    assert not torso_plausible([[0.8, -0.2, 0.6], [np.nan] * 3], hips, w, l)
    v = HumanValidator(*cfg)
    assert not v.torso_ok(shoulders * [1, 3, 1], hips) and v.rejects['torso'] == 1


def test_each_check_has_its_own_switch(cfg):
    c, vis = cfg
    v = HumanValidator(dict(c, check_segments=False, check_torso=False), vis)
    assert np.isfinite(v.filter_arm('left', _arm(forearm=0.60), DIRECT)).all()
    assert v.torso_ok(np.full((2, 3), np.nan), None)
    far = ARM.copy()
    far[0] = [5.0, 0.0, 0.6]
    assert np.isnan(v.filter_arm('left', far, DIRECT)[0]).all()   # workspace still on
    v = HumanValidator(dict(c, check_workspace=False), vis)
    assert np.isfinite(v.filter_arm('left', far, np.array([False, True, True, True]))).all()
