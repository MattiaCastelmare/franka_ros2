"""Unit tests for utils/avoidance.py (pure numpy — no ROS environment needed).

Covers the avoidance-first primitives used by pentagon_qddot_commander:
influence weighting, tangential redirection ("steer, don't brake"), the
feasibility-driven phase-governor target and the asymmetric rate limiter.

Run with pytest, or directly:  python3 test_avoidance.py
"""

import numpy as np

from franka_experiments.utils.avoidance import (
    influence_weight,
    tangential_redirect,
    feasibility_beta_target,
    rate_limited,
)


# ── influence_weight ─────────────────────────────────────────────────────────

def test_influence_far_is_zero():
    assert influence_weight(1.0, 0.25, 0.40) == 0.0
    assert influence_weight(0.40, 0.25, 0.40) == 0.0        # boundary: off
    assert influence_weight(float('inf'), 0.25, 0.40) == 0.0
    assert influence_weight(float('nan'), 0.25, 0.40) == 0.0


def test_influence_close_is_one():
    assert influence_weight(0.25, 0.25, 0.40) == 1.0
    assert influence_weight(0.05, 0.25, 0.40) == 1.0


def test_influence_monotone_and_bounded():
    ds = np.linspace(0.25, 0.40, 50)
    gs = [influence_weight(float(d), 0.25, 0.40) for d in ds]
    assert all(0.0 <= g <= 1.0 for g in gs)
    assert all(a >= b - 1e-12 for a, b in zip(gs, gs[1:]))   # non-increasing in d


def test_influence_degenerate_band_is_step():
    # d_close == d_infl → hard step at d_infl (d ≥ d_infl is off), never NaN
    assert influence_weight(0.29, 0.30, 0.30) == 1.0
    assert influence_weight(0.30, 0.30, 0.30) == 0.0
    assert influence_weight(0.31, 0.30, 0.30) == 0.0


# ── tangential_redirect ──────────────────────────────────────────────────────

def _unit(v):
    return v / np.linalg.norm(v)


def test_redirect_inactive_when_gamma_zero():
    a = np.array([1.0, 0.0, 0.0])
    n = np.array([-1.0, 0.0, 0.0])
    out, t = tangential_redirect(a, n, 0.0, np.array([0.0, 1.0, 0.0]))
    assert t is None and np.allclose(out, a)


def test_redirect_inactive_when_moving_away():
    # a along +n (away from obstacle) → untouched
    n = _unit(np.array([0.0, 0.0, 1.0]))
    a = 2.0 * n
    out, t = tangential_redirect(a, n, 1.0, np.array([0.0, 1.0, 0.0]))
    assert t is None and np.allclose(out, a)


def test_redirect_full_gamma_removes_approach_preserves_magnitude():
    # Head-on approach with a usable tangent preference.
    n = np.array([0.0, 0.0, 1.0])                 # obstacle below → n̂ points up
    a = np.array([0.0, 3.0, -4.0])                # approach component = 4 (into)
    t_pref = np.array([1.0, 0.5, 0.0])
    out, t_hat = tangential_redirect(a, n, 1.0, t_pref)
    assert abs(out @ n) < 1e-12                   # approach fully removed
    assert t_hat is not None and abs(t_hat @ n) < 1e-12
    # magnitude re-injected sideways: ‖out‖ ≥ tangential part of a
    assert np.linalg.norm(out) >= np.linalg.norm(a - (a @ n) * n) - 1e-12


def test_redirect_partial_gamma_scales():
    n = np.array([0.0, 0.0, 1.0])
    a = np.array([0.0, 0.0, -2.0])
    out, _ = tangential_redirect(a, n, 0.5, np.array([1.0, 0.0, 0.0]))
    assert np.isclose(out @ n, -1.0)              # half the approach removed


def test_redirect_headon_degenerate_pref_uses_fallback():
    # t_pref ∥ n̂ → tangential projection vanishes → deterministic fallback ⊥ n̂
    n = np.array([0.0, 0.0, 1.0])
    a = np.array([0.0, 0.0, -1.0])
    out, t_hat = tangential_redirect(a, n, 1.0, t_pref=-n)
    assert t_hat is not None
    assert abs(t_hat @ n) < 1e-12
    assert np.isclose(np.linalg.norm(t_hat), 1.0)
    assert np.linalg.norm(out) > 0.5              # motion continues, no dead stop


def test_redirect_hysteresis_prefers_previous_tangent():
    n = np.array([0.0, 0.0, 1.0])
    a = np.array([0.0, 0.0, -1.0])
    t_prev = np.array([0.0, 1.0, 0.0])
    out, t_hat = tangential_redirect(a, n, 1.0, t_pref=-n, t_prev=t_prev)
    assert np.allclose(t_hat, t_prev)             # degenerate pref → keep side


def test_redirect_cap_limits_reinjection():
    n = np.array([0.0, 0.0, 1.0])
    a = np.array([0.0, 0.0, -10.0])
    out, t_hat = tangential_redirect(a, n, 1.0, np.array([1.0, 0.0, 0.0]),
                                     redirect_max=2.0)
    assert np.isclose(out @ t_hat, 2.0)           # tangential part capped
    assert abs(out @ n) < 1e-12                   # approach still fully removed


# ── feasibility_beta_target ──────────────────────────────────────────────────

_KW = dict(slack_engage=0.02, slack_full=0.5,
           err_lo=0.02, err_hi=0.05, manip_thr=0.05)


def test_beta_healthy_is_one():
    assert feasibility_beta_target(0.0, False, 0.0, 0.2, **_KW) == 1.0


def test_beta_fault_is_zero():
    assert feasibility_beta_target(0.0, True, 0.0, 0.2, **_KW) == 0.0


def test_beta_slack_ramp():
    b_mid = feasibility_beta_target(0.26, False, 0.0, 0.2, **_KW)
    assert 0.0 < b_mid < 1.0
    assert feasibility_beta_target(0.5, False, 0.0, 0.2, **_KW) == 0.0
    assert feasibility_beta_target(0.019, False, 0.0, 0.2, **_KW) == 1.0


def test_beta_error_ramp():
    assert feasibility_beta_target(0.0, False, 0.05, 0.2, **_KW) == 0.0
    assert feasibility_beta_target(0.0, False, 0.01, 0.2, **_KW) == 1.0
    b = feasibility_beta_target(0.0, False, 0.035, 0.2, **_KW)
    assert 0.0 < b < 1.0


def test_beta_manipulability_collapse():
    assert feasibility_beta_target(0.0, False, 0.0, 0.0, **_KW) == 0.0
    b = feasibility_beta_target(0.0, False, 0.0, 0.025, **_KW)
    assert np.isclose(b, 0.5)


def test_beta_takes_worst_factor():
    # f_slack = 0.5, f_err ≈ 0.667, f_manip = 0.2 → min is the manip factor
    b = feasibility_beta_target(0.26, False, 0.03, 0.01, **_KW)
    assert np.isclose(b, 0.2)


def test_beta_never_negative_or_above_one():
    for slack in (0.0, 0.3, 5.0):
        for err in (0.0, 0.04, 1.0):
            for w in (0.0, 0.03, 10.0):
                b = feasibility_beta_target(slack, False, err, w, **_KW)
                assert 0.0 <= b <= 1.0


# ── rate_limited ─────────────────────────────────────────────────────────────

def test_rate_limiter_asymmetric():
    # down: fast (4/s), up: slow (1/s)
    b = rate_limited(1.0, 0.0, rate_up=1.0, rate_down=4.0, dt=0.1)
    assert np.isclose(b, 0.6)
    b = rate_limited(0.0, 1.0, rate_up=1.0, rate_down=4.0, dt=0.1)
    assert np.isclose(b, 0.1)


def test_rate_limiter_reaches_target():
    assert rate_limited(0.95, 1.0, 1.0, 4.0, 0.1) == 1.0
    assert rate_limited(0.02, 0.0, 1.0, 4.0, 0.1) == 0.0


if __name__ == '__main__':
    import sys
    mod = sys.modules['__main__']
    fails = 0
    for name in sorted(dir(mod)):
        if name.startswith('test_'):
            try:
                getattr(mod, name)()
                print(f'PASS {name}')
            except AssertionError as exc:
                fails += 1
                print(f'FAIL {name}: {exc}')
    sys.exit(1 if fails else 0)
