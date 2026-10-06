"""2D overlay of the tracked arm: only valid keypoints, measured vs predicted."""

import numpy as np
import pytest

from franka_experiments.utils.human_utils import (
    draw_predicted_arm, draw_tracked_arm, project_to_pixel, tracked_arm_pixels,
)

LANDMARKS = np.array([[100.0, 50.0], [110.0, 80.0], [120.0, 110.0], [125.0, 120.0]])
PREDICTED = [(101.0, 51.0), (111.0, 81.0), (130.0, 115.0), None]


def test_project_to_pixel():
    identity = np.eye(3), np.zeros(3)
    assert project_to_pixel([0.0, 0.0, 2.0], *identity, 500.0, 500.0, 320.0, 240.0) == (320.0, 240.0)
    assert project_to_pixel([0.2, -0.1, 2.0], *identity, 500.0, 500.0, 320.0, 240.0) == \
        pytest.approx((370.0, 215.0))
    assert project_to_pixel([0.0, 0.0, -1.0], *identity, 500.0, 500.0, 320.0, 240.0) is None
    assert project_to_pixel([np.nan, 0.0, 1.0], *identity, 500.0, 500.0, 320.0, 240.0) is None


def test_only_valid_keypoints_are_drawn_measured_at_mediapipe_predicted_at_estimate():
    measured = [True, True, False, False]
    valid = [True, True, True, False]
    keypoints = tracked_arm_pixels(LANDMARKS, measured, valid, PREDICTED)
    assert keypoints[0] == ((100.0, 50.0), True)          # measured: MediaPipe pixel
    assert keypoints[1] == ((110.0, 80.0), True)
    assert keypoints[2] == ((130.0, 115.0), False)        # predicted: projected estimate
    assert keypoints[3] is None                           # invalid: not drawn


def test_predicted_keypoint_without_projection_is_not_drawn():
    keypoints = tracked_arm_pixels(None, [False] * 4, [True, False, False, True], PREDICTED)
    assert keypoints == [((101.0, 51.0), False), None, None, None]


def test_draw_tracked_arm_skips_segments_to_missing_keypoints():
    image = np.zeros((200, 200, 3), dtype=np.uint8)
    keypoints = [((20.0, 20.0), True), None, ((100.0, 100.0), False), ((150.0, 150.0), True)]
    draw_tracked_arm(image, keypoints, ('shoulder', 'elbow', 'wrist', 'index'), 1.0, False)
    assert image[20, 20].any() and image[150, 150].any()
    assert not image[60, 60].any()                        # no segment through the missing elbow
    assert image[125, 125].any()                          # wrist-index segment drawn


def test_predicted_keypoint_is_hollow_measured_is_filled():
    image = np.zeros((200, 200, 3), dtype=np.uint8)
    keypoints = [((40.0, 40.0), True), None, ((140.0, 140.0), False), None]
    draw_tracked_arm(image, keypoints, ('shoulder', 'elbow', 'wrist', 'index'), 1.0, False)
    assert image[40, 40].any()                            # filled centre
    assert not image[140, 140].any() and image[140, 146].any()   # ring only


def test_prediction_ghosts_fade_with_the_horizon_and_skip_invalid_keypoints():
    image = np.zeros((200, 200, 3), dtype=np.uint8)
    steps = [
        [(40.0, 40.0), (60.0, 40.0), None, None],     # +1 step: strongest
        [(40.0, 120.0), (60.0, 120.0), None, None],   # +3 steps: faint
        [(40.0, 120.0), (60.0, 120.0), None, None],
        [None, None, None, None],                     # nothing valid: skipped
        [(500.0, 500.0), None, None, None],           # outside the frame: skipped
    ]
    draw_predicted_arm(image, steps, 1.0)
    near, far = image[40, 50], image[120, 50]         # on each ghost's segment
    assert near[2] > far[2] > 0                       # red, fading
    assert near[0] == near[1] == 0                    # pure red over black
    assert image[160:, :].sum() == 0 and image[:, 100:].sum() == 0
