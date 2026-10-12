"""human_measurement: the pinhole measurement model of a deprojected RGB-D keypoint.

pinhole_covariance is the first-order covariance of deproject(u, v, depth); ArmMeasurer
hands it to the filter, rotated into the base frame, under kf_measurement_model: pinhole.
"""

import numpy as np
import pytest

from franka_experiments.utils.human_measurement import (
    ArmMeasurer, deproject, depth_std, pinhole_covariance, ray_covariance,
)

INTRINSICS = (644.0, 642.5, 647.9, 359.1)   # the D455 colour stream, 1280x720
NAMES = ("shoulder", "elbow", "wrist", "index")


def test_pinhole_covariance_matches_the_spread_of_deprojected_points():
    """Bilinear deprojection: first order is exact but for the sigma_px * sigma_d term."""
    rng = np.random.default_rng(0)
    u, v, d, sigma_px, sigma_d = 1000.0, 150.0, 2.5, 3.0, 0.04
    n = 400_000
    uu, vv, dd = rng.normal(u, sigma_px, n), rng.normal(v, sigma_px, n), rng.normal(d, sigma_d, n)
    fx, fy, cx, cy = INTRINSICS
    samples = np.stack([(uu - cx) * dd / fx, (vv - cy) * dd / fy, dd], axis=1)
    np.testing.assert_allclose(samples[0], deproject(uu[0], vv[0], dd[0], *INTRINSICS))
    expected = pinhole_covariance(u, v, d, INTRINSICS, sigma_px, sigma_d)
    np.testing.assert_allclose(np.cov(samples.T), expected, rtol=0.03, atol=1e-7)


def test_pinhole_covariance_at_the_principal_point():
    fx, fy, cx, cy = INTRINSICS
    R = pinhole_covariance(cx, cy, 2.0, INTRINSICS, 4.0, 0.03)
    np.testing.assert_allclose(R, np.diag([(4.0 * 2.0 / fx) ** 2, (4.0 * 2.0 / fy) ** 2, 0.03**2]))


def test_a_depth_error_lies_along_the_viewing_ray():
    u, v, d = 1100.0, 600.0, 3.0
    R = pinhole_covariance(u, v, d, INTRINSICS, 0.01, 0.10)
    ray = deproject(u, v, d, *INTRINSICS)
    principal = np.linalg.eigh(R)[1][:, -1]
    assert abs(principal @ ray) / np.linalg.norm(ray) == pytest.approx(1.0, abs=1e-6)


def test_depth_std_is_a_floor_plus_the_stereo_term():
    assert depth_std(0.0, 0.03, 0.0015) == pytest.approx(0.03)
    assert depth_std(3.0, 0.0, 0.0015) == pytest.approx(0.0015 * 9.0)
    assert depth_std(2.0, 0.03, 0.0015) == pytest.approx(np.hypot(0.03, 0.006))


def _measurer(model):
    m = ArmMeasurer(NAMES, 0.4, patch_radius=2, min_depth_m=0.1, max_depth_m=4.0,
                    measurement_std=0.03, fallback_depth_std_m=0.12, measurement_model=model,
                    pixel_std_px=4.5, depth_std_m=0.03, depth_std_quad=0.0015)
    m.set_intrinsics(*INTRINSICS)
    return m


# Camera looking along base +x: optical z -> base x, optical x -> base -y, optical y -> base -z
ROTATION = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
TRANSLATION = np.array([1.75, 0.15, 1.06])
PIXELS = {0: (600, 300), 1: (650, 420), 2: (700, 500)}   # the index has no pixel
OWN_DEPTHS = np.array([2.0, 2.1, 0.0, 0.0])               # the wrist borrows the arm's depth


def test_pinhole_model_gives_every_point_its_base_frame_covariance():
    positions, depths, covs, borrowed = _measurer("pinhole").deproject_arm(
        PIXELS, OWN_DEPTHS, (ROTATION, TRANSLATION))
    assert borrowed.tolist() == [False, False, True, False]
    assert np.isfinite(positions[:3]).all() and np.isnan(positions[3]).all()
    assert np.isnan(covs[3]).all()
    for i, sigma_d in ((0, depth_std(2.0, 0.03, 0.0015)), (1, depth_std(2.1, 0.03, 0.0015)),
                       (2, 0.12)):
        camera = pinhole_covariance(*PIXELS[i], depths[i], INTRINSICS, 4.5, sigma_d)
        np.testing.assert_allclose(covs[i], ROTATION @ camera @ ROTATION.T)
        # Loose along the viewing ray (base frame), tight across it
        ray = positions[i] - TRANSLATION
        ray /= np.linalg.norm(ray)
        assert ray @ covs[i] @ ray > 4.0 * np.max(np.linalg.eigvalsh(covs[i])[:2])


def test_isotropic_model_leaves_own_depths_to_the_default_r():
    positions, _, covs, borrowed = _measurer("isotropic").deproject_arm(
        PIXELS, OWN_DEPTHS, (ROTATION, TRANSLATION))
    assert borrowed.tolist() == [False, False, True, False]
    assert np.isnan(covs[[0, 1, 3]]).all()
    np.testing.assert_allclose(covs[2], ray_covariance(positions[2] - TRANSLATION, 0.03, 0.12))


def test_no_camera_tf_measures_nothing():
    positions, depths, covs, borrowed = _measurer("pinhole").deproject_arm(
        PIXELS, OWN_DEPTHS, None)
    assert np.isnan(positions).all() and np.isnan(covs).all() and not borrowed.any()
    np.testing.assert_array_equal(depths, OWN_DEPTHS)


def test_unknown_model_is_refused():
    with pytest.raises(ValueError):
        ArmMeasurer(NAMES, 0.4, 2, 0.1, 4.0, 0.03, 0.12, measurement_model="ekf")
