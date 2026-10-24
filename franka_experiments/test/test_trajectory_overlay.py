"""The EE overlay on the camera image: what lands where, and what is refused.

The image overlay answers the same question as the RViz one — is the end
effector where the commander asked — but through a projection, which adds its
own ways of lying: a point BEHIND the camera has a perfectly good pixel (the
mirrored one), a trail can fade on the wrong end, and a run clipped at the frame
edge can be drawn along a path the arm never took. Those are the tests here.
"""

from __future__ import annotations

import numpy as np
import pytest

from franka_experiments.utils.trajectory_overlay import (
    PixelBounds,
    composite,
    draw_fading_polyline,
    fading_runs,
    project_base_to_pixels,
    trail_alphas,
)

# Camera one metre along +x of the base, optical axes aligned with the base
# axes, so that p_cam = p_base − t and the arithmetic in a test is readable.
R_ID = np.eye(3)
T_CAM = np.array([1.0, 0.0, 0.0])
K = np.array([[600.0, 0.0, 320.0],
              [0.0, 600.0, 240.0],
              [0.0, 0.0, 1.0]])


# ── Projection ───────────────────────────────────────────────────────────────

def test_point_on_the_optical_axis_lands_on_the_principal_point():
    uv, valid = project_base_to_pixels([[1.0, 0.0, 2.0]], R_ID, T_CAM, K)
    assert bool(valid[0])
    assert uv[0] == pytest.approx([320.0, 240.0])


def test_pixel_scales_with_the_focal_length_and_the_depth():
    # 0.1 m off axis at 2 m → 600 · 0.1 / 2 = 30 px right of the centre.
    uv, valid = project_base_to_pixels([[1.1, 0.0, 2.0]], R_ID, T_CAM, K)
    assert bool(valid[0])
    assert uv[0] == pytest.approx([350.0, 240.0])


def test_a_point_behind_the_camera_is_refused_not_mirrored():
    """The pinhole equations answer for z<0 too, with the mirrored pixel.

    Drawn, that is a trace on the far side of the image from where the arm is —
    and it looks exactly like a real excursion, which is why it is refused here
    rather than clipped later.
    """
    uv, valid = project_base_to_pixels([[1.1, 0.0, -2.0]], R_ID, T_CAM, K)
    assert not bool(valid[0])
    assert np.all(np.isnan(uv[0]))


def test_a_point_on_the_image_plane_is_refused():
    _, valid = project_base_to_pixels([[1.0, 0.0, 0.0]], R_ID, T_CAM, K)
    assert not bool(valid[0])


def test_non_finite_input_is_refused_rather_than_propagated():
    uv, valid = project_base_to_pixels(
        [[1.0, 0.0, 2.0], [np.nan, 0.0, 2.0], [1.0, np.inf, 2.0]],
        R_ID, T_CAM, K)
    assert list(valid) == [True, False, False]
    assert np.all(np.isnan(uv[1:]))


def test_empty_input_is_an_empty_answer_not_a_crash():
    uv, valid = project_base_to_pixels([], R_ID, T_CAM, K)
    assert uv.shape == (0, 2)
    assert valid.shape == (0,)


def test_rotation_is_applied_as_the_inverse_of_the_stored_extrinsic():
    """``camera_extrinsics.yaml`` stores camera→base; projection needs base→camera.

    Getting this backwards is the classic overlay bug: the traces land on the
    image, move when the arm moves, and sit in entirely the wrong place.
    """
    # Camera yawed 90° about base z: p_base = R·p_cam + t.
    R = np.array([[0.0, -1.0, 0.0],
                  [1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0]])
    p_cam_expected = np.array([0.0, 0.0, 2.0])
    p_base = R @ p_cam_expected + T_CAM
    uv, valid = project_base_to_pixels([p_base], R, T_CAM, K)
    assert bool(valid[0])
    assert uv[0] == pytest.approx([320.0, 240.0])


def test_zero_distortion_is_the_pinhole_answer():
    a, _ = project_base_to_pixels([[1.1, 0.05, 2.0]], R_ID, T_CAM, K)
    b, _ = project_base_to_pixels([[1.1, 0.05, 2.0]], R_ID, T_CAM, K,
                                  D=[0.0, 0.0, 0.0, 0.0, 0.0])
    assert a[0] == pytest.approx(b[0])


def test_positive_k1_pushes_an_off_axis_point_outward():
    straight, _ = project_base_to_pixels([[1.2, 0.0, 2.0]], R_ID, T_CAM, K)
    curved, _ = project_base_to_pixels([[1.2, 0.0, 2.0]], R_ID, T_CAM, K,
                                       D=[0.2, 0.0, 0.0, 0.0, 0.0])
    assert curved[0, 0] > straight[0, 0] > 320.0
    assert curved[0, 1] == pytest.approx(240.0)


# ── Fade ─────────────────────────────────────────────────────────────────────

def test_the_head_is_solid_and_the_tail_is_gone():
    t = np.array([0.0, 1.0, 2.0])
    a = trail_alphas(t, now=2.0, ttl=2.0, head_hold=0.25)
    assert a[-1] == pytest.approx(1.0)      # just published
    assert a[0] == pytest.approx(0.0)       # exactly ttl old
    assert 0.0 < a[1] < 1.0


def test_head_hold_keeps_the_newest_fraction_at_full_opacity():
    t = np.array([0.0, 1.0, 1.6, 2.0])      # ages 2.0, 1.0, 0.4, 0.0
    a = trail_alphas(t, now=2.0, ttl=2.0, head_hold=0.25)   # hold = 0.5 s
    assert a[2] == pytest.approx(1.0)
    assert a[3] == pytest.approx(1.0)


def test_alphas_are_clipped_for_points_older_than_the_trail():
    a = trail_alphas([-10.0, 0.0], now=0.0, ttl=1.0)
    assert a[0] == 0.0
    assert a[1] == pytest.approx(1.0)


def test_a_disabled_trail_does_not_fade_to_invisible():
    """``ttl <= 0`` means "no expiry", and must not mean "draw nothing"."""
    a = trail_alphas([0.0, 100.0], now=1e6, ttl=0.0)
    assert np.all(a == 1.0)


def test_alphas_of_an_empty_trace():
    assert trail_alphas([], now=1.0, ttl=2.0).shape == (0,)


# ── Runs ─────────────────────────────────────────────────────────────────────

def _uv(n, x0=0.0):
    return np.stack([np.arange(n, dtype=float) * 10.0 + x0,
                     np.full(n, 100.0)], axis=1)


def test_a_uniform_trace_is_a_single_run():
    runs = fading_runs(_uv(5), np.ones(5, dtype=bool), np.ones(5), levels=8)
    assert len(runs) == 1
    pts, alpha = runs[0]
    assert len(pts) == 5
    assert alpha == pytest.approx((7 + 0.5) / 8)


def test_an_invalid_vertex_breaks_the_line_instead_of_being_skipped():
    """Joining across a hole draws the arm through space it never occupied."""
    valid = np.array([True, True, False, True, True])
    runs = fading_runs(_uv(5), valid, np.ones(5), levels=8)
    assert len(runs) == 2
    assert [len(p) for p, _ in runs] == [2, 2]
    # Nothing spans the gap: no run holds both sides of index 2.
    for pts, _ in runs:
        xs = set(pts[:, 0].tolist())
        assert not ({0.0, 30.0} <= xs)


def test_opacity_steps_share_a_vertex_so_the_curve_stays_continuous():
    alphas = np.array([0.1, 0.4, 0.9])      # three different buckets
    runs = fading_runs(_uv(3), np.ones(3, dtype=bool), alphas, levels=8)
    assert len(runs) == 2
    first_pts, _ = runs[0]
    second_pts, _ = runs[1]
    assert tuple(first_pts[-1]) == tuple(second_pts[0])


def test_a_single_vertex_draws_nothing():
    assert fading_runs(_uv(1), np.ones(1, dtype=bool), np.ones(1)) == []
    assert fading_runs(_uv(2), np.array([True, False]), np.ones(2)) == []


def test_runs_are_ordered_oldest_first_so_the_newest_pass_wins():
    alphas = np.array([0.1, 0.1, 0.9, 0.9])
    runs = fading_runs(_uv(4), np.ones(4, dtype=bool), alphas, levels=8)
    assert [a for _, a in runs] == sorted(a for _, a in runs)


# ── Bounds and compositing ───────────────────────────────────────────────────

def test_bounds_start_empty_and_grow():
    b = PixelBounds()
    assert b.empty
    assert b.roi(2, 640, 480) is None
    b.add(100, 50)
    b.add(10, 200)
    assert b.roi(0, 640, 480) == (10, 50, 100, 200)


def test_the_roi_is_padded_but_never_leaves_the_frame():
    b = PixelBounds()
    b.add(0, 0)
    b.add(639, 479)
    assert b.roi(5, 640, 480) == (0, 0, 639, 479)


def test_compositing_with_zero_alpha_leaves_the_frame_untouched():
    img = np.full((8, 8, 3), 100, dtype=np.uint8)
    layer = np.full((8, 8, 3), 255, dtype=np.uint8)
    alpha = np.zeros((8, 8), dtype=np.uint8)
    composite(img, layer, alpha, (0, 0, 7, 7))
    assert np.all(img == 100)


def test_compositing_with_full_alpha_replaces_only_inside_the_roi():
    img = np.full((8, 8, 3), 100, dtype=np.uint8)
    layer = np.full((8, 8, 3), 255, dtype=np.uint8)
    alpha = np.full((8, 8), 255, dtype=np.uint8)
    composite(img, layer, alpha, (2, 2, 4, 4))
    assert np.all(img[2:5, 2:5] == 255)
    assert np.all(img[0:2, :] == 100)
    assert np.all(img[5:, :] == 100)


def test_half_alpha_is_a_blend():
    img = np.zeros((4, 4, 3), dtype=np.uint8)
    layer = np.full((4, 4, 3), 200, dtype=np.uint8)
    alpha = np.full((4, 4), 128, dtype=np.uint8)
    composite(img, layer, alpha, (0, 0, 3, 3))
    assert np.all(np.abs(img.astype(int) - 100) <= 1)


def test_compositing_without_a_roi_is_a_no_op():
    img = np.zeros((4, 4, 3), dtype=np.uint8)
    composite(img, np.full((4, 4, 3), 255, np.uint8),
              np.full((4, 4), 255, np.uint8), None)
    assert np.all(img == 0)


# ── Drawing: the layers, the box, and what never reaches the frame ───────────

def _layers(h=60, w=80):
    return np.zeros((h, w, 3), np.uint8), np.zeros((h, w), np.uint8)


def test_drawing_paints_both_layers_and_reports_the_box():
    layer, alpha = _layers()
    uv = np.array([[10.0, 30.0], [50.0, 30.0]])
    bounds = PixelBounds()
    n = draw_fading_polyline(layer, alpha, uv, np.ones(2, bool), np.ones(2),
                             (0, 0, 255), 2, bounds)
    assert n == 1
    assert alpha[30, 30] > 0, 'the opacity mask must carry the line'
    assert layer[30, 30, 2] > 0, 'and the colour layer the colour'
    assert bounds.roi(0, 80, 60) == (10, 30, 50, 30)


def test_a_trail_off_the_frame_draws_nothing_and_grows_no_box():
    """The arm leaves the field of view; the overlay must not pay for it.

    An off-screen run costs a full-frame composite if it reaches the bounding
    box, so it is rejected before either.
    """
    layer, alpha = _layers()
    uv = np.array([[-500.0, 30.0], [-400.0, 30.0]])
    bounds = PixelBounds()
    assert draw_fading_polyline(layer, alpha, uv, np.ones(2, bool), np.ones(2),
                                (0, 0, 255), 2, bounds) == 0
    assert bounds.empty
    assert not np.any(alpha)


def test_a_trail_crossing_the_edge_is_still_drawn():
    layer, alpha = _layers()
    uv = np.array([[-500.0, 30.0], [40.0, 30.0]])
    bounds = PixelBounds()
    assert draw_fading_polyline(layer, alpha, uv, np.ones(2, bool), np.ones(2),
                                (0, 0, 255), 2, bounds) == 1
    assert alpha[30, 0] > 0 and alpha[30, 40] > 0


def test_the_faded_tail_is_dimmer_than_the_head():
    layer, alpha = _layers()
    uv = np.array([[10.0, 30.0], [40.0, 30.0], [70.0, 30.0]])
    draw_fading_polyline(layer, alpha, uv, np.ones(3, bool),
                         np.array([0.1, 0.5, 1.0]), (0, 0, 255), 2,
                         PixelBounds())
    assert int(alpha[30, 15]) < int(alpha[30, 65])
