"""The offline half of scripts/human_kf_consistency.py: the noise estimator and the ray frame.

`replay` needs ROS, MediaPipe and a bag and is not covered here; it imports them lazily so
that this module loads with numpy alone.
"""

import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import human_kf_consistency as hkc  # noqa: E402

DT = 1.0 / 30.0


def _noisy_motion(n, sigma, rng):
    t = np.arange(n) * DT
    return t, 0.4 + 0.3 * t - 0.8 * t**2 + rng.normal(0.0, sigma, n)   # quadratic motion


@pytest.mark.parametrize("order", [2, 3])
def test_differences_recover_white_noise_under_motion(order):
    """D2 and D3 cancel the motion (a constant D2 does not move a median-based spread)."""
    rng = np.random.default_rng(1)
    t, x = _noisy_motion(20_000, 0.004, rng)
    diff, _ = hkc.differences(x, np.ones_like(x, dtype=bool), t, order)
    assert hkc.robust_std(diff, hkc.DIFFERENCES[order][1]) == pytest.approx(0.004, rel=0.05)


def test_low_passed_noise_is_seen_by_its_differences_as_theory_says():
    """First-order smoothing y_k = phi y_k-1 + (1 - phi) w_k (correlation phi^l): the
    differences see sigma_y sqrt(1 - 4/3 phi + 1/3 phi^2) (D2) and
    sigma_y sqrt(1 - 1.5 phi + 0.6 phi^2 - 0.1 phi^3) (D3), below sigma_y, D3 a little
    below D2. Smoothing shows in the level of D2 far more than in D3 / D2."""
    rng = np.random.default_rng(2)
    phi = 0.5
    t = np.arange(40_000) * DT
    white = rng.normal(0.0, 1.0, t.size)
    smooth = np.empty_like(white)
    smooth[0] = white[0]
    for k in range(1, t.size):
        smooth[k] = (1.0 - phi) * white[k] + phi * smooth[k - 1]
    sigma_y = (1.0 - phi) / np.sqrt(1.0 - phi**2)
    valid = np.ones(t.size, dtype=bool)
    d2 = hkc.robust_std(hkc.differences(smooth, valid, t, 2)[0], 6.0)
    d3 = hkc.robust_std(hkc.differences(smooth, valid, t, 3)[0], 20.0)
    assert d2 == pytest.approx(sigma_y * np.sqrt(1 - 4 / 3 * phi + phi**2 / 3), rel=0.03)
    assert d3 == pytest.approx(sigma_y * np.sqrt(1 - 1.5 * phi + 0.6 * phi**2 - 0.1 * phi**3),
                               rel=0.03)


def test_differences_skip_gaps_and_uneven_frames():
    t = np.arange(10) * DT
    t[6:] += 0.5 * DT                       # a late frame between 5 and 6
    valid = np.ones(10, dtype=bool)
    valid[2] = False
    _, centre = hkc.differences(np.zeros(10), valid, t, 2)
    # Windows (k-1, k, k+1): none touches frame 2 or spans the 5 -> 6 gap
    assert centre.tolist() == [4, 7, 8]


def test_ray_basis_is_orthonormal_and_starts_along_the_ray():
    run = SimpleNamespace(t_cb=np.array([1.75, 0.15, 1.06]),
                          R_cb=np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]))
    points = np.array([[0.3, 0.2, 0.8], [0.5, -0.4, 1.2]])
    basis = hkc.ray_basis(run, points)
    for b, p in zip(basis, points):
        np.testing.assert_allclose(b @ b.T, np.eye(3), atol=1e-12)
        ray = (p - run.t_cb) / np.linalg.norm(p - run.t_cb)
        np.testing.assert_allclose(b[0], ray)
