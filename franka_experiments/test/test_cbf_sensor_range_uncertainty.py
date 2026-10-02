"""The sensor-range tightening: off is exactly off, never negative, and grows
with range the way a structured-light / stereo depth sensor's own noise does.

    sigma_z(z) = z^2 / (f_px * baseline_m) * sigma_d_px
    margin     = min(k_sigma * sigma_z, margin_max)

This is a NEW, independent term from `uncertainty_margin` (test_cbf_
uncertainty_margin.py) — that one prices the TRACKER's velocity covariance,
this one prices the RAW DEPTH READING. Phase 1 covers only the pure function;
its effect on a real ConstraintSnap, composition with the other two
tightenings, and the flag-off bit-identical regression test are added in
Phase 3 once the field is threaded onto Obstacle.
"""

import numpy as np

from franka_experiments.utils.cbf_state_rows import sensor_range_uncertainty
from _cbf_builder_harness import make_builder, make_obstacle, run

KW = dict(f_px=428.8, baseline_m=0.095, sigma_d_px=0.2, k_sigma=2.0,
         margin_max=0.25)
PR, PH = (0.5, 0.0, 0.5), (0.5, -0.25, 0.5)
N = np.array([0.0, 1.0, 0.0])


# ── The exact-zero paths ────────────────────────────────────────────────────

def test_zero_z_is_exactly_zero():
    assert sensor_range_uncertainty(0.0, **KW) == 0.0


def test_negative_z_is_exactly_zero():
    """A range can never be negative on a real reading; a malformed one must
    not be trusted enough to produce a margin."""
    assert sensor_range_uncertainty(-1.0, **KW) == 0.0


def test_nonfinite_z_is_exactly_zero():
    assert sensor_range_uncertainty(float('nan'), **KW) == 0.0
    assert sensor_range_uncertainty(float('inf'), **KW) == 0.0


def test_zero_k_sigma_is_exactly_zero():
    kw = dict(KW); kw['k_sigma'] = 0.0
    assert sensor_range_uncertainty(1.0, **kw) == 0.0


def test_zero_sigma_d_is_exactly_zero():
    """No admitted disparity error -> no margin, exactly, not approximately."""
    kw = dict(KW); kw['sigma_d_px'] = 0.0
    assert sensor_range_uncertainty(1.0, **kw) == 0.0


def test_subtracting_it_at_zero_is_a_bitwise_no_op():
    rng = np.random.default_rng(0)
    h = rng.normal(size=32)
    assert np.array_equal(h - sensor_range_uncertainty(0.0, **KW), h)


# ── Malformed / degenerate configuration is ignored, not trusted ───────────

def test_a_malformed_focal_length_is_ignored_rather_than_trusted():
    for bad in (0.0, -100.0, float('nan'), float('inf')):
        kw = dict(KW); kw['f_px'] = bad
        assert sensor_range_uncertainty(1.0, **kw) == 0.0


def test_a_malformed_baseline_is_ignored_rather_than_trusted():
    for bad in (0.0, -0.1, float('nan'), float('inf')):
        kw = dict(KW); kw['baseline_m'] = bad
        assert sensor_range_uncertainty(1.0, **kw) == 0.0


def test_a_negative_sigma_d_is_ignored_rather_than_trusted():
    kw = dict(KW); kw['sigma_d_px'] = -0.2
    assert sensor_range_uncertainty(1.0, **kw) == 0.0


def test_a_malformed_margin_max_is_ignored_rather_than_trusted():
    for bad in (0.0, -0.25, float('nan')):
        kw = dict(KW); kw['margin_max'] = bad
        assert sensor_range_uncertainty(1.0, **kw) == 0.0


def test_a_nonfinite_sigma_d_does_not_produce_a_nan():
    kw = dict(KW); kw['sigma_d_px'] = float('nan')
    m = sensor_range_uncertainty(1.0, **kw)
    assert np.isfinite(m) and m == 0.0


# ── Monotonicity, sign and the documented formula ───────────────────────────

def test_the_margin_is_never_negative():
    rng = np.random.default_rng(1)
    for _ in range(200):
        z = rng.uniform(-1.0, 5.0)
        m = sensor_range_uncertainty(z, **KW)
        assert m >= 0.0 and np.isfinite(m)


def test_the_margin_grows_monotonically_with_z():
    prev = -1.0
    for z in np.linspace(0.05, 3.0, 40):
        m = sensor_range_uncertainty(float(z), **KW)
        assert m > prev or m == KW['margin_max']
        prev = m


def test_the_margin_is_the_documented_formula():
    z = 0.5
    sigma_z = (z * z) / (KW['f_px'] * KW['baseline_m']) * KW['sigma_d_px']
    expect = min(KW['k_sigma'] * sigma_z, KW['margin_max'])
    assert np.isclose(sensor_range_uncertainty(z, **KW), expect)


def test_the_margin_is_quadratic_in_z_before_the_clamp():
    """sigma_z ~ z^2, so doubling z (while staying under the clamp) should
    quadruple the margin."""
    kw = dict(KW); kw['margin_max'] = 10.0     # clamp out of the way
    z = 0.2
    m1 = sensor_range_uncertainty(z, **kw)
    m2 = sensor_range_uncertainty(2.0 * z, **kw)
    assert np.isclose(m2, 4.0 * m1)


def test_the_clamp_binds_at_large_z():
    assert sensor_range_uncertainty(10.0, **KW) == KW['margin_max']


def test_a_smaller_disparity_error_gives_a_smaller_or_equal_margin():
    kw_small = dict(KW); kw_small['sigma_d_px'] = 0.1
    kw_large = dict(KW); kw_large['sigma_d_px'] = 0.4
    z = 0.5
    assert sensor_range_uncertainty(z, **kw_small) < sensor_range_uncertainty(z, **kw_large)


# ── Effect on the real snapshot ─────────────────────────────────────────────

RANGE_PARAMS = dict(sensor_range_f_px=428.8, sensor_range_baseline_m=0.095,
                    sensor_range_sigma_d_px=0.2, sensor_range_k_sigma=2.0,
                    sensor_range_margin_max=0.25)


def _con(enable, range_m, **over):
    b = make_builder(enable_sensor_range_uncertainty=enable, **RANGE_PARAMS, **over)
    return run(b, [make_obstacle(pr=PR, ph=PH, range_m=range_m)], n_frames=5)


def test_flag_off_is_bit_identical_with_a_full_range_on_the_wire():
    a = _con(False, 0.6)
    b = _con(False, None)
    np.testing.assert_array_equal(a.h_bar, b.h_bar)
    np.testing.assert_array_equal(a.A, b.A)
    np.testing.assert_array_equal(a.G, b.G)


def test_flag_on_with_no_range_is_bit_identical():
    np.testing.assert_array_equal(_con(True, None).h_bar, _con(False, None).h_bar)


def test_flag_on_tightens_by_exactly_the_documented_formula():
    z = 0.6
    off = _con(False, z).h_bar[0]
    on = _con(True, z).h_bar[0]
    expect = sensor_range_uncertainty(
        z, f_px=RANGE_PARAMS['sensor_range_f_px'],
        baseline_m=RANGE_PARAMS['sensor_range_baseline_m'],
        sigma_d_px=RANGE_PARAMS['sensor_range_sigma_d_px'],
        k_sigma=RANGE_PARAMS['sensor_range_k_sigma'],
        margin_max=RANGE_PARAMS['sensor_range_margin_max'])
    assert expect > 0.0
    assert np.isclose(off - on, expect)


def test_it_tightens_on_the_very_first_frame_unlike_the_tracker_terms():
    """Unlike uncertainty_margin/latency_compensation, this term is NOT
    gated on obstacle_velocity_min_frames: a brand-new detection (frame 1,
    frames_seen=0, no track at all) must still get the tightening."""
    high_gate = dict(obstacle_velocity_min_frames=1000)   # would starve Phase 3/4
    on = run(make_builder(enable_sensor_range_uncertainty=True, **RANGE_PARAMS,
                          **high_gate),
             [make_obstacle(pr=PR, ph=PH, range_m=0.6)], n_frames=1)
    off = run(make_builder(enable_sensor_range_uncertainty=False, **RANGE_PARAMS,
                           **high_gate),
              [make_obstacle(pr=PR, ph=PH, range_m=0.6)], n_frames=1)
    assert on.h_bar[0] < off.h_bar[0]


def test_the_tightening_does_not_compound_across_rebuilds():
    b = make_builder(enable_sensor_range_uncertainty=True, **RANGE_PARAMS)
    ob = [make_obstacle(pr=PR, ph=PH, range_m=0.6)]
    vals = [run(b, ob, n_frames=1).h_bar[0] for _ in range(20)]
    assert max(vals) - min(vals) < 1e-9, 'the tightening is compounding'


# ── Composition with uncertainty_margin and latency_compensation ───────────

def test_all_three_tightenings_compose_linearly():
    """uncertainty_margin (tracker velocity covariance), latency_compensation
    (predicted displacement + propagated position covariance) and
    sensor_range_uncertainty (raw depth noise) are three independent `h -=`
    statements with no shared clamp between them: the drop with all three on
    must equal the sum of what each contributes alone."""
    kw = dict(v=(0.0, 0.5, 0.0), a=(0.0, 0.2, 0.0), frames_seen=20,
             cov=np.eye(3) * 0.04, pos_cov=np.eye(3) * 1e-4,
             pv_cov=np.eye(3) * 1e-3, range_m=0.6)

    def h_with(**flags):
        b = make_builder(obstacle_velocity_source='tracker', **RANGE_PARAMS, **flags)
        return run(b, [make_obstacle(pr=PR, ph=PH, **kw)], n_frames=5).h_bar[0]

    off = h_with(enable_uncertainty_margin=False, enable_latency_compensation=False,
                enable_sensor_range_uncertainty=False)
    unc_only = h_with(enable_uncertainty_margin=True, enable_latency_compensation=False,
                      enable_sensor_range_uncertainty=False)
    lat_only = h_with(enable_uncertainty_margin=False, enable_latency_compensation=True,
                      enable_sensor_range_uncertainty=False)
    rng_only = h_with(enable_uncertainty_margin=False, enable_latency_compensation=False,
                      enable_sensor_range_uncertainty=True)
    all_three = h_with(enable_uncertainty_margin=True, enable_latency_compensation=True,
                       enable_sensor_range_uncertainty=True)

    d_unc, d_lat, d_rng = off - unc_only, off - lat_only, off - rng_only
    assert d_unc > 0.0 and d_lat > 0.0 and d_rng > 0.0

    assert np.isclose(off - all_three, d_unc + d_lat + d_rng, atol=1e-9), (
        off - all_three, d_unc + d_lat + d_rng)
