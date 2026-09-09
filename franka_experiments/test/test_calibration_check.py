"""Calibration drift: does the projected robot model land on the measured arm?

This is the one comparison the distance pipeline never makes. "Obstacle" is
defined as "not robot", so if the mask is subtracted from the wrong place the
arm is not removed and becomes its own obstacle — with complete confidence and
no error reported anywhere.

The tests below are built on a synthetic scene where the answer is known
exactly: a plane at a known depth, a model made of points ON that plane, and a
calibration error introduced deliberately. They also pin the honesty
requirement — that no verdict is issued without a baseline, because the absolute
residual carries several centimetres of mesh-sampling bias.

Pure numpy.
"""

import numpy as np

from franka_experiments.utils.calibration_check import (
    CalibrationResidual, calibration_residual)

K = np.array([[400.0, 0.0, 320.0], [0.0, 400.0, 240.0], [0.0, 0.0, 1.0]])
H, W = 480, 640
R = np.eye(3)
T = np.zeros(3)


def _plane_depth(z):
    return np.full((H, W), float(z))


def _model_on_plane(z, n=4000, half=0.25, seed=0):
    """Model points lying exactly on the fronto-parallel plane at depth z."""
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-half, half, size=(n, 2))
    return {'link': np.column_stack([xy, np.full(n, float(z))])}


# ── A correct calibration reads ~zero ───────────────────────────────────────

def test_a_perfect_match_reads_near_zero():
    r = calibration_residual(_model_on_plane(1.5), R, T, K, _plane_depth(1.5),
                             step=1)
    assert r.n_pixels > 200
    assert abs(r.median_m) < 0.005, r.median_m
    assert r.coverage > 0.9


# ── An error shows up with the right size and sign ──────────────────────────

def test_a_model_in_front_of_the_arm_gives_a_positive_residual():
    """POSITIVE means the measured surface is FARTHER than the model, i.e. the
    model sits in front of the real arm. Getting this sign backwards would send
    an operator to look in the opposite direction."""
    r = calibration_residual(_model_on_plane(1.4), R, T, K, _plane_depth(1.5),
                             step=1)
    assert np.isclose(r.median_m, 0.10, atol=0.005)


def test_a_model_behind_the_arm_gives_a_negative_residual():
    r = calibration_residual(_model_on_plane(1.6), R, T, K, _plane_depth(1.5),
                             step=1)
    assert np.isclose(r.median_m, -0.10, atol=0.005)


def test_a_translational_extrinsic_error_is_recovered():
    """The realistic failure: the extrinsic is off by a few centimetres along
    the optical axis, so every unprojected point is displaced by it."""
    for dz in (-0.08, -0.02, 0.03, 0.09):
        r = calibration_residual(_model_on_plane(1.5), R, np.array([0.0, 0.0, dz]),
                                 K, _plane_depth(1.5), step=1)
        # p_cam = R^T (p_base - t), so a +dz extrinsic pulls the model's depth
        # DOWN by dz; the measurement is unchanged, so measured - model = +dz.
        assert np.isclose(r.median_m, dz, atol=0.006), (dz, r.median_m)


def test_the_spread_is_small_for_a_rigid_offset():
    """A rigid offset moves everything equally, so the SPREAD stays small. That
    is what separates 'the camera moved' from 'the model is the wrong shape'."""
    r = calibration_residual(_model_on_plane(1.5), R, np.array([0.0, 0.0, 0.05]),
                             K, _plane_depth(1.5), step=1)
    assert r.mad_m < 0.01


# ── The model's front surface, not its back ─────────────────────────────────

def test_back_facing_model_points_do_not_drag_the_residual():
    """A mesh is sampled on both sides, but a depth camera only ever sees the
    front. Taking the MINIMUM model depth per pixel is what keeps the back of
    the link out of the statistic — without it every calibration would look
    badly wrong by roughly the link's own thickness."""
    front = _model_on_plane(1.5, n=3000, seed=1)['link']
    back = _model_on_plane(1.65, n=3000, seed=2)['link']     # far side of the link
    r = calibration_residual({'link': np.vstack([front, back])}, R, T, K,
                             _plane_depth(1.5), step=1)
    assert abs(r.median_m) < 0.01, r.median_m


def test_an_occluder_in_front_of_the_arm_does_not_dominate():
    """A real obstacle in front of part of the arm makes those pixels read much
    closer. The MEDIAN survives it; a mean would not."""
    depth = _plane_depth(1.5)
    depth[:, :200] = 0.8                       # something in front of a third
    r = calibration_residual(_model_on_plane(1.5, n=6000), R, T, K, depth, step=1)
    assert abs(r.median_m) < 0.01, r.median_m


# ── The honesty requirement ─────────────────────────────────────────────────

def test_no_verdict_without_a_baseline():
    """THE requirement. The absolute residual carries several centimetres of
    mesh-sampling bias (measured: 7 cm of swing from the sample count alone), so
    judging it against zero would fire on a perfectly good calibration — and a
    warning that fires when nothing is wrong is a warning nobody reads."""
    big = CalibrationResidual(median_m=0.25, mad_m=0.02, n_pixels=500,
                              coverage=0.9)
    assert big.is_suspicious() is False
    assert 'no baseline set' in big.describe()
    assert 'verdict' in big.describe()


def test_drift_from_a_baseline_is_flagged():
    r = CalibrationResidual(median_m=0.06, mad_m=0.02, n_pixels=500, coverage=0.9)
    assert r.is_suspicious(baseline_m=0.01, tol_m=0.03)
    txt = r.describe(baseline_m=0.01, tol_m=0.03)
    assert 'DRIFTED' in txt and 'own obstacle' in txt


def test_no_drift_from_the_baseline_is_not_flagged():
    r = CalibrationResidual(median_m=0.012, mad_m=0.02, n_pixels=500, coverage=0.9)
    assert not r.is_suspicious(baseline_m=0.01, tol_m=0.03)
    assert 'within tolerance' in r.describe(baseline_m=0.01, tol_m=0.03)


def test_too_few_pixels_is_inconclusive_not_a_pass():
    r = CalibrationResidual(median_m=0.5, mad_m=0.02, n_pixels=3, coverage=0.01)
    assert not r.is_suspicious(baseline_m=0.0)
    assert 'inconclusive' in r.describe()


# ── Degenerate inputs ───────────────────────────────────────────────────────

def test_an_empty_model_is_handled():
    r = calibration_residual({}, R, T, K, _plane_depth(1.5))
    assert r.n_pixels == 0 and np.isnan(r.median_m)


def test_a_model_entirely_behind_the_camera_is_handled():
    r = calibration_residual({'l': np.array([[0.0, 0.0, -1.0]])}, R, T, K,
                             _plane_depth(1.5))
    assert r.n_pixels == 0


def test_a_model_projecting_outside_the_image_is_handled():
    r = calibration_residual({'l': np.array([[9.0, 9.0, 1.0]])}, R, T, K,
                             _plane_depth(1.5))
    assert r.n_pixels == 0


def test_depth_outside_the_validity_band_is_not_counted():
    """Zero means "no return" on these sensors, and counting it as a measurement
    of 0 m would report a metre of drift on every dropout."""
    r = calibration_residual(_model_on_plane(1.5), R, T, K,
                             np.zeros((H, W)), step=1)
    assert r.n_pixels == 0
