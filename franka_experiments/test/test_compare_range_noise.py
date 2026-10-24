"""Phase 4: scripts/compare_range_noise.py's `synthetic` mode.

Pure numpy, no ROS, no bag — drives the REAL DistanceEngine with a single
noisy pixel at a known distance and checks that the whole chain (injected
disparity noise -> DistanceEngine.compute -> ControlPointResult.range_m ->
range_noise_calibration.fit_sigma_d) is internally consistent with
`sensor_range_uncertainty`'s own formula.

This does NOT validate a real camera's actual noise (see the module
docstring in compare_range_noise.py) — it validates that the CODE agrees
with itself on a KNOWN, deliberately injected law. An earlier version of
`_noisy_single_pixel_depth` fixed baseline_m at 1.0 "because it cancels",
which was simply wrong (off by exactly a factor of baseline_m) and was
caught by test_recovering_the_injected_constant_holds_at_two_different_
baselines below — kept as a named regression test for that class of bug.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import compare_range_noise as crn  # noqa: E402
import range_noise_calibration as rnc  # noqa: E402

from franka_experiments.utils.cbf_state_rows import sensor_range_uncertainty


F_PX = 428.8
BASELINE_M = 0.095


def test_zero_injected_noise_reads_back_the_true_distance():
    rng = np.random.default_rng(0)
    depth = crn._noisy_single_pixel_depth(0.5, F_PX, BASELINE_M, 0.0, rng)
    z = int(depth[int(crn._CY), int(crn._CX)]) / 1000.0
    assert abs(z - 0.5) < 0.001, z          # within one mm-quantisation step


def test_only_one_pixel_is_painted():
    """The whole point of the single-pixel design: no argmin-over-many-
    candidates order-statistic bias — see the docstring in
    compare_range_noise.py for why a full-frame wall would be wrong here."""
    rng = np.random.default_rng(0)
    depth = crn._noisy_single_pixel_depth(0.5, F_PX, BASELINE_M, 1.0, rng)
    assert np.count_nonzero(depth) == 1


def test_the_empirical_std_matches_the_documented_formula():
    """At a noise level well above the 1 mm depth-quantisation floor, the
    empirical std of range_m across frames must track sensor_range_
    uncertainty's own sigma_z(z) formula, not just something proportional
    to it."""
    sigma_d_true = 3.0
    records = crn.synthetic_noise_sweep(
        [0.3, 0.6, 1.0], f_px=F_PX, baseline_m=BASELINE_M,
        sigma_d_px_true=sigma_d_true, n_frames=400, seed=1)
    for r in records:
        z = r['distance_truth_m']
        predicted = (z ** 2) / (F_PX * BASELINE_M) * sigma_d_true
        # 400 samples: std-of-std is roughly predicted/sqrt(2*399) ~ 3.5%;
        # 25% tolerance is generous but still catches a wrong formula/units.
        assert abs(r['std_z_m'] - predicted) / predicted < 0.25, (
            z, r['std_z_m'], predicted)


def test_recovering_the_injected_constant_holds_at_two_different_baselines():
    """Regression test for the baseline_m unit-mismatch bug: injecting noise
    at baseline_m=B and fitting against the SAME B must recover the
    injected sigma_d_px, for more than one value of B — a fixed/hardcoded
    baseline inside the injector would pass at exactly one B and fail (by a
    factor of B_wrong/B_true) at any other."""
    sigma_d_true = 2.5
    for baseline_m in (0.05, 0.095, 0.20):
        records = crn.synthetic_noise_sweep(
            np.linspace(0.2, 1.0, 6), f_px=F_PX, baseline_m=baseline_m,
            sigma_d_px_true=sigma_d_true, n_frames=300, seed=2)
        z = np.array([r['distance_truth_m'] for r in records])
        sigma_z = np.array([r['std_z_m'] for r in records])
        fitted, r2 = rnc.fit_sigma_d(z, sigma_z, F_PX, baseline_m)
        assert r2 > 0.9, (baseline_m, r2)
        assert abs(fitted - sigma_d_true) / sigma_d_true < 0.2, (
            baseline_m, fitted, sigma_d_true)


def test_sensor_range_uncertainty_at_k_sigma_one_tracks_the_fitted_margin():
    """sensor_range_uncertainty(k_sigma=1) is a 1-std margin by definition;
    fed the constant this test fits from noisy synthetic data, it must land
    close to the actual empirical std it was fit from — connecting the
    calibration script's output back to the production function."""
    sigma_d_true = 2.0
    records = crn.synthetic_noise_sweep(
        [0.4, 0.8], f_px=F_PX, baseline_m=BASELINE_M,
        sigma_d_px_true=sigma_d_true, n_frames=400, seed=3)
    z = np.array([r['distance_truth_m'] for r in records])
    sigma_z = np.array([r['std_z_m'] for r in records])
    fitted, r2 = rnc.fit_sigma_d(z, sigma_z, F_PX, BASELINE_M)
    assert r2 > 0.9
    for r in records:
        margin = sensor_range_uncertainty(
            r['distance_truth_m'], f_px=F_PX, baseline_m=BASELINE_M,
            sigma_d_px=fitted, k_sigma=1.0, margin_max=10.0)
        assert abs(margin - r['std_z_m']) / r['std_z_m'] < 0.3


def test_cmd_synthetic_runs_end_to_end(capsys):
    class _Args:
        sigma_d_px = 3.0
        f_px = F_PX
        baseline_m = BASELINE_M
        z_min = 0.2
        z_max = 0.8
        n_distances = 4
        n_frames = 100
        seed = 0

    crn.cmd_synthetic(_Args())
    out = capsys.readouterr().out
    assert 'fitted   sigma_d_px' in out
    assert 'does NOT measure the real sensor' in out
