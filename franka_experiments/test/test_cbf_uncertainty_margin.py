"""The uncertainty-derived tightening: off is exactly off, and more doubt is
always more margin — never less.

    h_eff = h − k_sigma · sqrt(n̂ᵀ P_vv n̂) · t_latency

This is the one term in the filter whose size is set by the ESTIMATOR rather
than by a constant chosen offline, so the properties that have to hold are
about monotonicity and sign rather than about a particular number: a filter
that is less sure must never end up with a looser barrier than one that is
sure, and no covariance may ever produce a loosening.

Covers both the pure function and its effect on a real ConstraintSnap.
"""

import numpy as np

from franka_experiments.utils.cbf_state_rows import uncertainty_margin
from _cbf_builder_harness import make_builder, make_obstacle, run

KW = dict(k_sigma=2.0, t_latency=0.20, margin_max=0.25)
N = np.array([0.0, 1.0, 0.0])
PR, PH = (0.5, 0.0, 0.5), (0.5, -0.25, 0.5)


# ── The exact-zero paths ────────────────────────────────────────────────────

def test_no_covariance_is_exactly_zero():
    """The "no track" state. Not approximately zero — `h - 0.0` has to be
    bit-identical to not evaluating the term."""
    assert uncertainty_margin(N, None, **KW) == 0.0


def test_zero_covariance_is_exactly_zero():
    assert uncertainty_margin(N, np.zeros((3, 3)), **KW) == 0.0


def test_zero_k_sigma_is_exactly_zero():
    assert uncertainty_margin(N, np.eye(3), k_sigma=0.0, t_latency=0.2,
                              margin_max=0.25) == 0.0


def test_subtracting_it_at_zero_is_a_bitwise_no_op():
    rng = np.random.default_rng(0)
    h = rng.normal(size=32)
    assert np.array_equal(h - uncertainty_margin(N, None, **KW), h)


# ── Monotonicity and sign ───────────────────────────────────────────────────

def test_the_margin_is_never_negative():
    """A negative margin would LOOSEN the barrier — the one thing no estimate in
    this filter is allowed to do. Swept over covariances that are not even
    valid, because the wire is float64 from another process."""
    rng = np.random.default_rng(1)
    for _ in range(200):
        M = rng.normal(size=(3, 3))
        for P in (M @ M.T, M, -np.eye(3), np.zeros((3, 3))):
            assert uncertainty_margin(rng.normal(size=3), P, **KW) >= 0.0


def test_a_larger_covariance_gives_a_strictly_larger_tightening():
    prev = -1.0
    for s in np.linspace(0.0, 0.05, 25):
        m = uncertainty_margin(N, np.eye(3) * s, **KW)
        assert m > prev or m == KW['margin_max']
        prev = m


def test_the_margin_is_the_documented_formula():
    P = np.diag([0.01, 0.04, 0.09])
    expect = 2.0 * np.sqrt(float(N @ P @ N)) * 0.20
    assert np.isclose(uncertainty_margin(N, P, **KW), expect)


def test_only_the_component_along_n_hat_matters():
    """The barrier consumes one scalar; uncertainty in the two directions it
    cannot see must cost nothing."""
    a = uncertainty_margin(N, np.diag([0.0, 0.04, 0.0]), **KW)
    b = uncertainty_margin(N, np.diag([9.0, 0.04, 9.0]), **KW)
    assert np.isclose(a, b)


def test_the_clamp_binds():
    assert uncertainty_margin(N, np.eye(3) * 100.0, **KW) == KW['margin_max']


def test_a_negative_quadratic_form_does_not_produce_a_nan():
    """P arrives over the wire after a Joseph update, a frame rotation and a
    float trip; its quadratic form can land at −1e−20 and sqrt of that is NaN,
    which would propagate into h and out through the whole QP."""
    m = uncertainty_margin(N, -np.eye(3) * 1e-20, **KW)
    assert np.isfinite(m) and m == 0.0


def test_a_malformed_covariance_is_ignored_rather_than_trusted():
    for bad in (np.zeros((2, 2)), np.full((3, 3), np.nan),
                np.full((3, 3), np.inf)):
        assert uncertainty_margin(N, bad, **KW) == 0.0


# ── Effect on the real snapshot ─────────────────────────────────────────────

def _con(enable, cov, frames_seen=20, **over):
    b = make_builder(obstacle_velocity_source='tracker',
                     enable_uncertainty_margin=enable, **over)
    return run(b, [make_obstacle(pr=PR, ph=PH, v=(0.0, 0.5, 0.0),
                                 frames_seen=frames_seen, cov=cov)],
               n_frames=5)


def test_flag_off_leaves_the_barrier_bit_identical():
    a = _con(False, np.eye(3) * 0.04)
    b = _con(False, None)
    np.testing.assert_array_equal(a.h_bar, b.h_bar)
    np.testing.assert_array_equal(a.A, b.A)
    np.testing.assert_array_equal(a.G, b.G)


def test_flag_on_with_no_covariance_is_still_bit_identical():
    np.testing.assert_array_equal(_con(True, None).h_bar, _con(False, None).h_bar)


def test_flag_on_tightens_and_only_tightens():
    off = _con(False, np.eye(3) * 0.04).h_bar[0]
    on = _con(True, np.eye(3) * 0.04).h_bar[0]
    assert on < off
    assert np.isclose(off - on, 2.0 * 0.2 * 0.2)     # k*sqrt(0.04)*0.20


def test_a_young_track_does_not_slam_the_margin_to_its_clamp():
    """A track one measurement old starts at sigma_v0 = 1 m/s by construction.
    Ungated, that would take the full clamp every time an obstacle appears —
    shrinking the workspace on the ARRIVAL of an obstacle rather than on any
    property of it."""
    young = _con(True, np.eye(3) * 1.0, frames_seen=1).h_bar[0]
    none = _con(False, None).h_bar[0]
    assert young == none


def test_the_margin_does_not_compound_across_rebuilds():
    """Ordering guard. The term is applied AFTER the recovery EMA stores h, so
    the EMA keeps tracking the MEASURED barrier. Were the tightened value
    stored, each frame would start from an already-tightened h and the term
    would drift without bound."""
    b = make_builder(obstacle_velocity_source='tracker',
                     enable_uncertainty_margin=True)
    ob = [make_obstacle(pr=PR, ph=PH, v=(0.0, 0.5, 0.0), frames_seen=20,
                        cov=np.eye(3) * 0.04)]
    vals = [run(b, ob, n_frames=1).h_bar[0] for _ in range(20)]
    assert max(vals) - min(vals) < 1e-9, 'the tightening is compounding'


# ── The margin must not step (hardware jitter, Sep 2026) ────────────────────

def test_the_margin_is_smoothed_across_rebuilds():
    """On hardware the margin sat at its floor of 0.032 m and jumped to 0.106
    whenever a control point matched a different track: through k0 = 25 that
    is a 1.85 rad/s² step in the row's right-hand side, from a term that is
    supposed to be a slowly-varying standoff. It is an admission of ignorance,
    not a measurement, and nothing about it is urgent to one tick."""
    import numpy as np
    from _cbf_builder_harness import make_builder, make_obstacle, run

    def barrier(alpha, cov_seq):
        b = make_builder(obstacle_velocity_source='tracker',
                         enable_uncertainty_margin=True,
                         uncertainty_margin_alpha=alpha)
        return [float(c) for c in [run(b, lambda k: [make_obstacle(
            pr=(0.5, 0.0, 0.5), ph=(0.5, -0.25, 0.5), v=(0.0, 0.3, 0.0),
            frames_seen=20, cov=np.eye(3) * cov_seq[min(k, len(cov_seq) - 1)])],
            n_frames=i + 1).h_bar[0] for i in range(len(cov_seq))]]

    seq = [0.0064] * 4 + [0.0700] * 6          # sigma_v 0.08 -> 0.26 m/s
    raw = np.abs(np.diff(barrier(0.0, seq)))
    ema = np.abs(np.diff(barrier(0.8, seq)))
    assert raw.max() > 4 * ema.max(), (raw.max(), ema.max())
