"""Phase 2: the offline half of scripts/range_noise_calibration.py.

`fit_sigma_d` is pure regression (no ROS) and is exercised directly, plus an
end-to-end smoke test of `fit` against a synthetic capture file — the same
JSON shape `capture` writes, fabricated here instead of requiring a live
camera. The live `capture` subcommand itself needs a streaming depth camera
and is NOT covered by this suite; it imports rclpy/cv_bridge lazily inside
the function body for exactly that reason (importing this module standalone
must not require ROS).
"""

import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import range_noise_calibration as rnc  # noqa: E402

F_PX = 428.8
BASELINE_M = 0.095
TRUE_SIGMA_D = 0.18


def _sigma_z(z, sigma_d=TRUE_SIGMA_D, f_px=F_PX, baseline_m=BASELINE_M):
    return (np.asarray(z) ** 2) / (f_px * baseline_m) * sigma_d


# ── fit_sigma_d: pure regression ────────────────────────────────────────────

def test_recovers_the_true_constant_on_noiseless_data():
    z = np.linspace(0.15, 1.0, 8)
    sigma_z = _sigma_z(z)
    sigma_d, r2 = rnc.fit_sigma_d(z, sigma_z, F_PX, BASELINE_M)
    assert np.isclose(sigma_d, TRUE_SIGMA_D, rtol=1e-9)
    assert r2 > 0.999999


def test_recovers_the_true_constant_under_realistic_noise():
    rng = np.random.default_rng(0)
    z = np.linspace(0.15, 1.0, 12)
    sigma_z = _sigma_z(z) * (1.0 + rng.normal(0.0, 0.05, size=z.shape))
    sigma_d, r2 = rnc.fit_sigma_d(z, sigma_z, F_PX, BASELINE_M)
    assert abs(sigma_d - TRUE_SIGMA_D) / TRUE_SIGMA_D < 0.1
    assert r2 > 0.9


def test_a_linear_law_fits_far_worse_than_the_quadratic_one():
    """If the true error were LINEAR in z rather than quadratic, fitting the
    quadratic model should show it via a visibly degraded R^2 — the whole
    point of Phase 4 is not to assume the quadratic law without checking."""
    z = np.linspace(0.15, 1.0, 10)
    sigma_z_linear = 0.02 * z          # wrong functional form on purpose
    _, r2 = rnc.fit_sigma_d(z, sigma_z_linear, F_PX, BASELINE_M)
    assert r2 < 0.98


def test_all_zero_distance_returns_zero_not_a_division_error():
    z = np.zeros(5)
    sigma_d, r2 = rnc.fit_sigma_d(z, np.ones(5) * 0.01, F_PX, BASELINE_M)
    assert sigma_d == 0.0 and r2 == 0.0


def test_a_single_repeated_distance_does_not_crash_on_zero_variance():
    z = np.full(5, 0.5)
    sigma_d, r2 = rnc.fit_sigma_d(z, np.full(5, 0.01), F_PX, BASELINE_M)
    assert np.isfinite(sigma_d) and np.isfinite(r2)


def test_per_sample_f_px_is_accepted_as_an_array():
    z = np.linspace(0.15, 1.0, 6)
    f = np.full(z.shape, F_PX) + np.linspace(-0.5, 0.5, z.shape[0])
    sigma_z = (z ** 2) / (f * BASELINE_M) * TRUE_SIGMA_D
    sigma_d, r2 = rnc.fit_sigma_d(z, sigma_z, f, BASELINE_M)
    assert np.isclose(sigma_d, TRUE_SIGMA_D, rtol=1e-6)
    assert r2 > 0.999


# ── cmd_fit: end-to-end against a synthetic capture file ───────────────────

class _Args:
    def __init__(self, inp, baseline_m):
        self.inp = inp
        self.baseline_m = baseline_m


def test_cmd_fit_reports_the_fitted_constant_end_to_end(tmp_path, capsys):
    z = np.linspace(0.15, 1.0, 6)
    sigma_z = _sigma_z(z)
    records = [dict(distance_truth_m=float(zi), mean_z_m=float(zi),
                    std_z_m=float(si), n_samples=500, f_px=F_PX)
              for zi, si in zip(z, sigma_z)]
    p = tmp_path / 'capture.json'
    p.write_text(json.dumps(records))

    rnc.cmd_fit(_Args(str(p), BASELINE_M))
    out = capsys.readouterr().out
    assert 'fitted sigma_d_px' in out
    assert f'{TRUE_SIGMA_D:.4f}' in out
    assert 'sensor_range_sigma_d_px' in out


def test_cmd_fit_refuses_fewer_than_two_distances(tmp_path):
    records = [dict(distance_truth_m=0.3, mean_z_m=0.3, std_z_m=0.001,
                    n_samples=100, f_px=F_PX)]
    p = tmp_path / 'capture.json'
    p.write_text(json.dumps(records))
    with pytest.raises(SystemExit):
        rnc.cmd_fit(_Args(str(p), BASELINE_M))


def test_cmd_fit_refuses_a_missing_file(tmp_path):
    with pytest.raises(SystemExit):
        rnc.cmd_fit(_Args(str(tmp_path / 'nope.json'), BASELINE_M))


def test_module_imports_without_ros():
    """capture's rclpy/cv_bridge imports must be lazy (inside the function),
    not module-level — this test only proves the import above succeeded
    without rclpy on the path, which the whole suite already needs true."""
    assert hasattr(rnc, 'cmd_capture') and hasattr(rnc, 'cmd_fit')
