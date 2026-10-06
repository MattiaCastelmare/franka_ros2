"""human_validation: workspace box, segment lengths, learned lengths, torso check, hysteresis,
identity of the arm (static background, FR3 body) and the depth background model."""

import os

import numpy as np
import pytest
import yaml

from franka_experiments.utils.human_validation import (
    DepthBackground, HumanValidator, in_box, robot_distance, segment_lengths, torso_plausible,
)

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'human_params.yaml')
DIRECT = np.ones(4, dtype=bool)

# shoulder, elbow, wrist, index: upper arm 0.30, forearm 0.25, hand 0.08
ARM = np.array([[0.5, 0.0, 0.6], [0.5, 0.0, 0.3], [0.5, 0.25, 0.3], [0.5, 0.33, 0.3]])


@pytest.fixture
def cfg():
    with open(CONFIG) as f:
        tracker = yaml.safe_load(f)['human_tracker']
    on = dict(enabled=True, check_workspace=True, check_segments=True, check_torso=True,
              check_background=True, check_robot=True)
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


# FR3 standing upright at the origin: link origins up to 1 m
ROBOT = np.array([[0.0, 0.0, z] for z in (0.0, 0.333, 0.649, 1.0)])


def test_shoulder_on_the_static_scene_rejects_the_arm(cfg):
    v = HumanValidator(*cfg)
    background = [False, True, True, True]          # shoulder is part of the scene
    assert np.isnan(v.filter_arm('left', ARM, DIRECT, foreground=background)).all()
    assert v.rejects['background'] == 1
    # in front of the scene, or scene not known yet: kept
    assert np.isfinite(v.filter_arm('left', ARM, DIRECT, foreground=[True] * 4)).all()
    assert np.isfinite(v.filter_arm('left', ARM, DIRECT, foreground=[None] * 4)).all()


def test_a_hand_on_the_table_does_not_reject_the_arm(cfg):
    v = HumanValidator(*cfg)
    assert np.isfinite(v.filter_arm('left', ARM, DIRECT, foreground=[True, True, False, False])).all()


def test_without_shoulder_depth_the_elbow_decides(cfg):
    v = HumanValidator(*cfg)
    direct = np.array([False, True, True, True])     # shoulder depth borrowed
    assert np.isfinite(v.filter_arm('left', ARM, direct, foreground=[False, True, True, True])).all()
    assert np.isnan(v.filter_arm('left', ARM, direct, foreground=[True, False, True, True])).all()


def test_shoulder_inside_the_robot_rejects_the_arm(cfg):
    v = HumanValidator(*cfg)
    assert robot_distance(np.array([0.5, 0.0, 0.6]), ROBOT) == pytest.approx(0.5)
    assert np.isfinite(v.filter_arm('left', ARM, DIRECT, robot_nodes=ROBOT)).all()
    on_robot = ARM - [0.45, 0.0, 0.0]                # shoulder 5 cm from the links
    assert np.isnan(v.filter_arm('left', on_robot, DIRECT, robot_nodes=ROBOT)).all()
    assert v.rejects['on_robot'] == 1
    # a hand touching the robot is a person: only the shoulder decides
    touching = np.array([[0.5, 0.0, 0.6], [0.3, 0.0, 0.45], [0.07, 0.0, 0.5], [0.0, 0.0, 0.5]])
    assert np.isfinite(v.filter_arm('left', touching, DIRECT, robot_nodes=ROBOT)).all()


def test_engageable_needs_shoulder_and_elbow_in_3d():
    assert HumanValidator.engageable(ARM, DIRECT)
    assert not HumanValidator.engageable(ARM, np.array([True, False, True, True]))
    no_elbow = ARM.copy()
    no_elbow[1] = np.nan
    assert not HumanValidator.engageable(no_elbow, DIRECT)


@pytest.fixture
def background():
    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)['human_tracker']['validation']['background']
    return DepthBackground(dict(cfg, warmup_frames=2, absorb_tau_s=1.0, max_depth_m=4.0), 0.1)


def _scene(depth=3.0, box=None, box_depth=1.5):
    """64x64 depth image in mm: a wall, optionally a box (r0, r1, c0, c1) in front of it."""
    img = np.full((64, 64), depth * 1000.0, dtype=np.uint16)
    if box is not None:
        r0, r1, c0, c1 = box
        img[r0:r1, c0:c1] = box_depth * 1000.0
    return img


def test_background_static_object_vs_person(background):
    robot = (8, 40, 8, 24)                          # a static object since the start
    for k in range(5):
        background.update(_scene(box=robot), 0.03 * k)
    assert background.foreground(16, 24, 1.5) is False      # on the static object
    assert background.foreground(48, 24, 2.0) is True       # a person in front of the wall
    assert background.foreground(48, 24, 2.95) is False     # the wall itself


def test_background_warmup_and_reveal(background):
    background.update(_scene(box=(8, 40, 40, 56)), 0.0)
    assert background.foreground(48, 24, 1.5) is None       # still warming up
    background.update(_scene(box=(8, 40, 40, 56)), 0.03)
    assert background.foreground(48, 24, 1.5) is False      # a person present since startup...
    for k in range(10):
        background.update(_scene(), 0.06 + 0.03 * k)        # ...walks away: the wall is revealed
    assert background.foreground(48, 24, 1.5) is True       # and is a person when back


def test_background_absorbs_a_new_object_but_not_the_tracked_person(background):
    for k in range(3):
        background.update(_scene(), 0.03 * k)
    box = (8, 40, 40, 56)
    protect = [(48, 24, 16)]                        # (u, v, radius) of the tracked keypoints
    for k in range(200):                            # 6 s, absorb_tau_s = 1 s
        background.update(_scene(box=box), 0.1 + 0.03 * k, protect)
    assert background.foreground(48, 24, 1.5) is True       # protected: still a person
    for k in range(200):
        background.update(_scene(box=box), 6.2 + 0.03 * k)
    assert background.foreground(48, 24, 1.5) is False      # left alone: part of the scene


def test_background_out_of_range_is_not_taken_from_the_first_person(background):
    for k in range(3):
        background.update(_scene(depth=6.0), 0.03 * k)      # wall beyond the sensor range
    for k in range(10):                                     # a person walks in front of it
        background.update(_scene(depth=6.0, box=(8, 40, 40, 56), box_depth=2.0), 0.1 + 0.03 * k)
    assert background.foreground(48, 24, 2.0) is True
    for k in range(60):                                     # ...and stays there, untracked
        background.update(_scene(depth=6.0, box=(8, 40, 40, 56), box_depth=2.0), 0.5 + 0.03 * k)
    assert background.foreground(48, 24, 2.0) is False      # absorbed after absorb_tau_s


def test_not_a_person_needs_every_decided_arm_rejected(cfg):
    v = HumanValidator(*cfg)
    on_robot = ARM - [0.45, 0.0, 0.0]
    v.filter_arm('left', on_robot, DIRECT, robot_nodes=ROBOT)
    v.filter_arm('right', np.full((4, 3), np.nan), DIRECT, robot_nodes=ROBOT)   # not seen
    assert v.not_a_person(['left', 'right']) == 'on_robot'
    v.filter_arm('right', ARM, DIRECT, robot_nodes=ROBOT)                       # a person
    assert v.not_a_person(['left', 'right']) is None
    v.filter_arm('left', np.full((4, 3), np.nan), DIRECT)
    v.filter_arm('right', np.full((4, 3), np.nan), DIRECT)
    assert v.not_a_person(['left', 'right']) is None                            # nothing seen


def test_new_rejects_reports_only_the_interval(cfg):
    v = HumanValidator(*cfg)
    far = ARM.copy()
    far[0] = [5.0, 0.0, 0.6]
    v.filter_arm('left', far, DIRECT)
    assert v.new_rejects() == 'workspace=1'
    assert v.new_rejects() == ''
    v.filter_arm('left', far, DIRECT)
    assert v.new_rejects() == 'workspace=1' and v.summary() == 'workspace=2'


def test_hand_on_the_robot_out_of_reach_is_dropped(cfg):
    c, vis = cfg
    v = HumanValidator(dict(c, check_segments=False), vis)
    # person 1.2 m behind the robot, MediaPipe puts the wrist and index on the links
    behind = np.array([[-1.2, 0.0, 0.6], [-1.1, 0.0, 0.35], [0.05, 0.0, 0.5], [0.03, 0.0, 0.5]])
    out = v.filter_arm('left', behind, DIRECT, robot_nodes=ROBOT)
    assert np.isfinite(out[:2]).all() and np.isnan(out[2:]).all()
    assert v.rejects['out_of_reach'] == 2


def test_hand_touching_the_robot_within_reach_is_kept(cfg):
    v = HumanValidator(*cfg)
    touching = np.array([[0.5, 0.0, 0.6], [0.3, 0.0, 0.45], [0.07, 0.0, 0.5], [0.0, 0.0, 0.5]])
    assert np.isfinite(v.filter_arm('left', touching, DIRECT, robot_nodes=ROBOT)).all()


def test_hand_alone_on_the_robot_needs_a_recent_shoulder(cfg):
    c, vis = cfg
    v = HumanValidator(dict(c, check_segments=False), vis)
    hand_only = np.full((4, 3), np.nan)
    hand_only[2:] = [[0.07, 0.0, 0.5], [0.0, 0.0, 0.5]]
    assert np.isnan(v.filter_arm('left', hand_only, DIRECT, robot_nodes=ROBOT, t=0.0)).all()
    # shoulder seen in reach a moment ago: the hand on the robot is a real touch
    v.filter_arm('left', np.array([[0.5, 0.0, 0.6]] + [[np.nan] * 3] * 3), DIRECT, t=1.0)
    assert np.isfinite(v.filter_arm('left', hand_only, DIRECT, robot_nodes=ROBOT, t=1.2)[2:]).all()
    # ...too long ago, or out of reach: dropped
    assert np.isnan(v.filter_arm('left', hand_only, DIRECT, robot_nodes=ROBOT, t=1.6)[2:]).all()
    v.filter_arm('left', np.array([[-1.2, 0.0, 0.6]] + [[np.nan] * 3] * 3), DIRECT, t=2.0)
    assert np.isnan(v.filter_arm('left', hand_only, DIRECT, robot_nodes=ROBOT, t=2.1)[2:]).all()
    # the other arm and a new person start without memory
    v.filter_arm('left', np.array([[0.5, 0.0, 0.6]] + [[np.nan] * 3] * 3), DIRECT, t=3.0)
    assert np.isnan(v.filter_arm('right', hand_only, DIRECT, robot_nodes=ROBOT, t=3.1)[2:]).all()
    v.reset()
    assert np.isnan(v.filter_arm('left', hand_only, DIRECT, robot_nodes=ROBOT, t=3.1)[2:]).all()


def test_hand_on_the_table_within_reach_is_kept(cfg):
    v = HumanValidator(*cfg)
    on_table = [True, True, False, False]             # wrist and index on the static scene
    assert np.isfinite(v.filter_arm('left', ARM, DIRECT, foreground=on_table)).all()


def test_elbow_on_the_robot_with_the_shoulder_just_hidden(cfg):
    c, vis = cfg
    v = HumanValidator(dict(c, check_segments=False), vis)
    nan = [np.nan] * 3
    arm = np.array([[0.4, 0.0, 0.6], [0.12, 0.0, 0.5], [0.3, 0.0, 0.3], [0.32, 0.0, 0.25]])
    no_shoulder = arm.copy()
    no_shoulder[0] = nan
    # no shoulder ever seen: the elbow on the robot carries the identity, arm rejected
    assert np.isnan(v.filter_arm('left', no_shoulder, DIRECT, robot_nodes=ROBOT, t=0.0)).all()
    assert v.rejects['on_robot'] == 1
    # shoulder seen a moment ago: the elbow resting on the robot is in reach, arm kept
    v.filter_arm('left', arm, DIRECT, robot_nodes=ROBOT, t=1.0)
    out = v.filter_arm('left', no_shoulder, DIRECT, robot_nodes=ROBOT, t=1.2)
    assert np.isfinite(out[1:]).all() and v.not_a_person(['left']) is None
    # ...but an "elbow" on the robot far from that shoulder is the robot itself: only it goes
    v.filter_arm('left', np.array([[-1.0, 0.0, 0.6], nan, nan, nan]), DIRECT, t=2.0)
    far = np.array([nan, [0.1, 0.0, 0.5], [-0.8, 0.0, 0.3], [-0.85, 0.0, 0.3]])
    out = v.filter_arm('left', far, DIRECT, robot_nodes=ROBOT, t=2.1)
    assert np.isnan(out[1]).all() and np.isfinite(out[2:]).all()


def test_occluded_keypoint_is_the_nearer_one_on_the_robot(cfg):
    v = HumanValidator(*cfg)
    # arm behind the robot: the elbow pixel falls on the links, its depth 0.6 m nearer
    arm = np.array([[-0.3, 0.0, 0.6], [0.05, 0.0, 0.5], [-0.3, 0.2, 0.3], [-0.32, 0.25, 0.3]])
    depths = np.array([2.1, 1.5, 2.1, 2.1])
    assert v.occluded(arm, depths, DIRECT, robot_nodes=ROBOT) == [1]
    assert v.rejects['occluded_depth'] == 1
    # same jump, but the nearer point is in free space: left alone
    free = arm.copy()
    free[1] = [-0.3, 0.6, 0.5]
    assert v.occluded(free, depths, DIRECT, robot_nodes=ROBOT) == []
    # on the static scene instead of the robot: occluded too
    assert v.occluded(free, depths, DIRECT, foreground=[True, False, True, True]) == [1]
    # a jump the segment can span is no occlusion
    assert v.occluded(arm, np.array([2.1, 1.8, 2.1, 2.1]), DIRECT, robot_nodes=ROBOT) == []
    # a borrowed depth is never an occluder reading
    assert v.occluded(arm, depths, np.array([True, False, True, True]), robot_nodes=ROBOT) == []


def test_occlusion_switch(cfg):
    c, vis = cfg
    v = HumanValidator(dict(c, check_occlusion=False), vis)
    arm = np.array([[-0.3, 0.0, 0.6], [0.05, 0.0, 0.5], [-0.3, 0.2, 0.3], [-0.32, 0.25, 0.3]])
    assert v.occluded(arm, np.array([2.1, 1.5, 2.1, 2.1]), DIRECT, robot_nodes=ROBOT) == []


def test_occlusion_reaches_the_robot_surface_off_the_link_line(cfg):
    c, vis = cfg
    v = HumanValidator(dict(c, check_segments=False), vis)
    # nearer point 0.20 m from the link line: outside robot_clearance, inside the occlusion one
    arm = np.array([[-0.3, 0.0, 0.6], [0.20, 0.0, 0.5], [-0.3, 0.2, 0.3], [-0.32, 0.25, 0.3]])
    assert v.occluded(arm, np.array([2.1, 1.5, 2.1, 2.1]), DIRECT, robot_nodes=ROBOT) == [1]
    # ...while the identity check keeps its own, tighter clearance
    assert np.isfinite(v.filter_arm('left', arm, DIRECT, robot_nodes=ROBOT)).all()


def test_occlusion_seen_across_a_borrowed_neighbour(cfg):
    v = HumanValidator(*cfg)
    # the wrist has a borrowed depth: the index on the robot is compared with the elbow
    arm = np.array([[-0.3, 0.0, 0.6], [-0.3, 0.2, 0.35], [-0.3, 0.3, 0.35], [0.05, 0.0, 0.5]])
    direct = np.array([True, True, False, True])
    depths = np.array([1.95, 2.13, 2.04, 1.35])          # 0.78 m: more than forearm + hand
    assert v.occluded(arm, depths, direct, robot_nodes=ROBOT) == [3]
    # within what forearm + hand can span: no occlusion
    assert v.occluded(arm, np.array([1.95, 2.13, 2.04, 1.75]), direct, robot_nodes=ROBOT) == []
