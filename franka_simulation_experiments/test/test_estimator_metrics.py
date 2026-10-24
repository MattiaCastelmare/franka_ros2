"""simexp.estimator_metrics on synthetic series with known answers (spec Phase 5.1)."""
import numpy as np
import pytest

from simexp import estimator_metrics as EM

DT = 1.0 / 90.0


def _series(t, v_true, v_est, *, d=None, qdot=None, vnorm=None, est='v_trk', tid=1, fs=None,
            sigma=None, obs=0):
    n = t.size
    s = dict(t=t, d=np.full(n, 0.4) if d is None else d,
             qdot=np.zeros(n) if qdot is None else qdot,
             v_true=v_true, v_true_norm=np.abs(v_true) if vnorm is None else vnorm,
             on_sphere=np.ones(n), obs=np.full(n, obs),
             track_id=np.full(n, tid), frames_seen=np.arange(1, n + 1) if fs is None else fs,
             sigma=np.full(n, 0.05) if sigma is None else sigma)
    s['v_trk'] = v_est if est == 'v_trk' else np.full(n, np.nan)
    s['v_res'] = v_est if est == 'v_res' else np.zeros(n)
    return s


def test_lag_bias_rms_of_a_delayed_biased_estimate():
    t = np.arange(0, 6, DT)
    tru = 0.3 * np.sin(2 * np.pi * 0.5 * t)
    delay = 6                                     # frames: 67 ms
    est = np.r_[np.full(delay, tru[0]), tru[:-delay]] + 0.1
    out = EM.estimator_scores({'fr3_link8#0': _series(t, tru, est, vnorm=np.full(t.size, 0.3))},
                              'v_trk', DT)
    assert out['n_cp'] == 1
    assert out['lag_ms'] == pytest.approx(-1e3 * delay * DT, abs=3.0)   # NEGATIVE = trails
    assert out['bias'] == pytest.approx(0.1, abs=0.01)
    assert out['rms'] > 0.1


def test_noise_floor_uses_static_windows_on_a_truly_static_obstacle():
    rng = np.random.default_rng(0)
    t = np.arange(0, 4, DT)
    est = rng.normal(0, 0.03, t.size)
    s = _series(t, np.zeros(t.size), est, est='v_res', vnorm=np.zeros(t.size))
    out = EM.estimator_scores({'a#0': s}, 'v_res', DT)
    assert out['n_static'] > 0.9 * t.size
    assert out['noise_floor'] == pytest.approx(0.03, rel=0.1)
    # a moving obstacle never counts towards the noise floor
    s2 = dict(s, v_true_norm=np.full(t.size, 0.3))
    assert EM.estimator_scores({'a#0': s2}, 'v_res', DT)['n_static'] == 0


def test_untracked_samples_read_as_the_safe_zero():
    t = np.arange(0, 2, DT)
    est = np.full(t.size, np.nan)
    out = EM.estimator_scores({'a#0': _series(t, np.full(t.size, 0.5), est)}, 'v_trk', DT)
    assert out['bias'] == pytest.approx(-0.5)


def test_too_few_samples_are_skipped_like_compare_vobs():
    t = np.arange(0, (EM.MIN_SAMPLES - 1) * DT, DT)[:EM.MIN_SAMPLES - 1]
    assert EM.estimator_scores({'a#0': _series(t, t * 0, t * 0)}, 'v_trk', DT)['n_cp'] == 0


def test_confirm_trust_identity_and_sigma_calibration():
    rng = np.random.default_rng(1)
    t = 1.0 + np.arange(0, 3, DT)
    n = t.size
    tid = np.where(t < 1.1, 0, np.where(t < 2.0, 5, 7))
    fs = np.where(tid > 0, np.arange(n), 0)
    sigma = np.full(n, 0.1)
    tru = np.full(n, 0.4)
    est = tru + rng.normal(0, 0.1, n)
    s = _series(t, tru, est, tid=0, fs=fs, sigma=sigma)
    s['track_id'] = tid
    out = EM.track_scores({'a#0': s}, trust_n=24)
    o = out['per_obstacle'][0]
    assert o['confirm_ms'] == pytest.approx(100, abs=1e3 * DT + 1e-6)
    first_trusted = t[(tid > 0) & (fs >= 24)][0]
    assert o['trust_ms'] == pytest.approx(1e3 * (first_trusted - 1.0), abs=1e-6)
    assert o['n_ids'] == 2 and o['id_switches'] == 1
    c = out['sigma_calibration']
    assert c['k1'] == pytest.approx(EM.GAUSS[1], abs=0.06)
    assert c['k2'] == pytest.approx(EM.GAUSS[2], abs=0.04)
    assert c['k3'] > 0.98


def test_trust_gate_is_the_cbfs():
    P = EM.control_params()
    n = max(int(P['obstacle_velocity_min_frames']),
            int(np.ceil(float(P['obstacle_velocity_min_span_s']) * 90.0)))
    assert EM.trust_frames(90.0) == n


def test_the_residual_is_the_shipped_estimator_and_recovers_a_closing_speed():
    b = EM.make_residual('residual')
    assert type(b).__name__ == 'ConstraintBuilder'
    n = np.array([0.0, 1.0, 0.0])
    v = 0.0
    for k in range(120):                      # gap closing at 0.3 m/s, arm still
        v = b._obstacle_speed('fr3_link8#0', 0.6 - 0.3 * k * DT, 1.0 + k * DT, 0.0, n)
    assert v == pytest.approx(0.3, abs=0.03)


def test_pooled_scores_cover_control_points_under_the_30_sample_cut():
    t = np.arange(0, 12 * DT, DT)[:12]                 # 12 frames per CP: a fast pass
    series = {f'cp#{k}': _series(t, np.full(12, 4.0), np.full(12, 3.5)) for k in range(3)}
    out = EM.estimator_scores(series, 'v_trk', DT)
    assert out['n_cp'] == 0 and np.isnan(out['rms'])  # compare_vobs' per-CP definition
    assert out['pooled_n'] == 36
    assert out['pooled_bias'] == pytest.approx(-0.5) and out['pooled_rms'] == pytest.approx(0.5)
