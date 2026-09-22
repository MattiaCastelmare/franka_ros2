"""Frame counts derived from durations, and the rate estimate behind them.

The property that matters is not any single conversion: it is that the DURATION
a threshold stands for is the same at 30 Hz and at 90 Hz, because that is what
broke silently when the depth profile changed. Most of what follows asserts
exactly that, at both rates, for the thresholds the perception config states.
"""

import math

import pytest

from franka_experiments.utils.rate_scaling import (
    FrameRateEstimator,
    ema_alpha_for,
    frames_for,
    tau_for_alpha,
)
from franka_experiments.utils.obstacle_tracker import TrackManager
from franka_experiments.utils.self_detection import SelfDetectionMonitor


# ── frames_for ────────────────────────────────────────────────────────────

def test_frames_for_reproduces_the_legacy_tuning_at_30hz():
    """The durations in fr3_complete.yaml must be the old counts at 30 Hz."""
    assert frames_for(0.167, 30.0) == 5      # max_missed
    assert frames_for(0.167, 30.0, minimum=2) == 5   # confirm_window
    assert frames_for(0.667, 30.0, minimum=2) == 20  # self_detection.window
    assert frames_for(0.5, 30.0) == 15               # self_detection.release


def test_frames_for_scales_to_90hz():
    assert frames_for(0.167, 90.0) == 15
    assert frames_for(0.667, 90.0) == 60
    assert frames_for(0.5, 90.0) == 45


@pytest.mark.parametrize('hz', [15.0, 30.0, 60.0, 90.0])
def test_duration_is_preserved_across_rates(hz):
    """A count/rate round trip stays within one frame of the duration."""
    for seconds in (0.167, 0.5, 0.667):
        assert abs(frames_for(seconds, hz) / hz - seconds) <= 1.0 / hz


def test_frames_for_floors_at_one_frame():
    """Zero frames of coast or confirmation is a counter that trips on nothing."""
    assert frames_for(0.001, 30.0) == 1
    assert frames_for(0.0, 90.0) == 1
    assert frames_for(0.001, 30.0, minimum=2) == 2


def test_frames_for_rounds_rather_than_truncates():
    # 0.166 s at 30 Hz is 4.98 frames: truncation would give 4 and make the
    # config's meaning depend on the third decimal.
    assert frames_for(0.166, 30.0) == 5


def test_frames_for_rejects_nonsense():
    with pytest.raises(ValueError):
        frames_for(0.1, 0.0)
    with pytest.raises(ValueError):
        frames_for(0.1, float('nan'))
    with pytest.raises(ValueError):
        frames_for(-0.1, 30.0)


# ── EMA time constant ─────────────────────────────────────────────────────

def test_tau_for_alpha_recovers_the_legacy_lag():
    """lpf_alpha 0.5 at 30 Hz is the 48 ms written in the config."""
    assert tau_for_alpha(0.5, 30.0) == pytest.approx(0.0481, abs=5e-4)


def test_ema_alpha_matches_the_legacy_weight_at_the_tuned_rate():
    """The migration is behaviour-preserving at 30 Hz, to 3 decimals."""
    assert ema_alpha_for(0.0481, 1.0 / 30.0) == pytest.approx(0.5, abs=1e-3)


def test_ema_alpha_holds_the_lag_when_the_rate_triples():
    """Same tau, three times the rate: the per-frame weight must rise."""
    a30 = ema_alpha_for(0.048, 1.0 / 30.0)
    a90 = ema_alpha_for(0.048, 1.0 / 90.0)
    assert a90 > a30
    # Three 90 Hz steps must decay the history as much as one 30 Hz step —
    # that IS the definition of an unchanged time constant.
    assert a90 ** 3 == pytest.approx(a30, rel=1e-9)


def test_ema_alpha_refuses_to_invent_an_interval():
    assert ema_alpha_for(0.048, 0.0) == 0.0
    assert ema_alpha_for(0.048, None) == 0.0
    assert ema_alpha_for(0.0, 1.0 / 30.0) == 0.0


# ── FrameRateEstimator ────────────────────────────────────────────────────

def _feed(est, hz, n):
    for _ in range(n):
        changed = est.add(1.0 / hz)
    return changed


def test_estimator_reports_nominal_until_it_settles():
    est = FrameRateEstimator(nominal_hz=90.0, window=20)
    est.add(1.0 / 30.0)
    assert not est.settled
    assert est.hz == 90.0            # the claim, until there is a measurement
    assert est.measured_hz is None


def test_estimator_flags_a_driver_fallback():
    """The 30 -> 15 fps fallback this rig has actually seen."""
    est = FrameRateEstimator(nominal_hz=30.0, window=20)
    changed = _feed(est, 15.0, 200)
    assert est.settled
    assert est.measured_hz == pytest.approx(15.0, rel=0.05)
    assert changed is False          # already reported before the last frame
    assert est.hz == pytest.approx(15.0, rel=0.05)


def test_estimator_reports_a_material_change_exactly_once():
    est = FrameRateEstimator(nominal_hz=30.0, window=20)
    reports = sum(1 for _ in range(300) if est.add(1.0 / 90.0))
    assert reports == 1


def test_estimator_ignores_ordinary_jitter():
    """+-3 ms at 30 Hz is 9% and must not retune the safety layer."""
    est = FrameRateEstimator(nominal_hz=30.0, window=20)
    dts = [1.0 / 30.0 + (0.003 if i % 2 else -0.003) for i in range(300)]
    assert not any(est.add(dt) for dt in dts)


def test_estimator_ignores_implausible_intervals():
    est = FrameRateEstimator(nominal_hz=30.0, window=5)
    for dt in (0.0, -1.0, 2.0, float('nan'), None):
        assert est.add(dt) is False
    assert est.n_samples == 0


def test_estimator_does_not_chase_a_breathing_processing_rate():
    """The dry-run defect: 49-57 Hz must retune once, not every 100 ms.

    When the compute loop is the bottleneck the rate it processes at wanders,
    and each wander used to cross a 10% band measured from the last report —
    which retuned the tracker's lifecycle several times a second.
    """
    est = FrameRateEstimator(nominal_hz=90.0)
    dts = [1.0 / (53.0 + (4.0 if i % 2 else -4.0)) for i in range(2000)]
    assert sum(1 for dt in dts if est.add(dt)) == 1
    assert est.measured_hz == pytest.approx(53.0, rel=0.05)


def test_estimator_still_catches_a_real_profile_change():
    """The cooldown must not swallow 50 -> 15 after an earlier retune.

    A big step can be reported twice — once part-way down the EMA's ramp, once
    at the bottom — and that is the honest behaviour: the thresholds should
    follow the rate while it is falling rather than wait for it to arrive. What
    must not happen is many reports, or none.
    """
    est = FrameRateEstimator(nominal_hz=90.0, min_interval=300)
    first = sum(1 for _ in range(500) if est.add(1.0 / 50.0))
    assert first == 1
    after = sum(1 for _ in range(1000) if est.add(1.0 / 15.0))
    assert 1 <= after <= 2
    assert est.measured_hz == pytest.approx(15.0, rel=0.05)


def test_estimator_resets_to_the_claim():
    est = FrameRateEstimator(nominal_hz=30.0, window=20)
    _feed(est, 90.0, 100)
    est.reset()
    assert not est.settled
    assert est.hz == 30.0


# ── retune, on the consumers ──────────────────────────────────────────────

def test_tracker_retune_keeps_live_tracks():
    """A rate change must not reap the identities the coast budget protects."""
    trk = TrackManager(confirm_hits=3, confirm_window=5, max_missed=5)
    for _ in range(4):
        trk.step([[1.0, 0.0, 0.0]], 1.0 / 30.0)
    assert len(trk.confirmed_tracks()) == 1
    ids_before = [t.track_id for t in trk.tracks]

    assert trk.retune(confirm_hits=9, confirm_window=15, max_missed=15)
    assert (trk.confirm_hits, trk.confirm_window, trk.max_missed) == (9, 15, 15)
    assert [t.track_id for t in trk.tracks] == ids_before
    # Already confirmed stays confirmed: confirmation is evidence already
    # gathered, not a property of the current window.
    assert len(trk.confirmed_tracks()) == 1


def test_tracker_retune_is_idempotent():
    trk = TrackManager(confirm_hits=3, confirm_window=5, max_missed=5)
    assert not trk.retune(confirm_hits=3, confirm_window=5, max_missed=5)


def test_tracker_retune_trims_an_overlong_history():
    trk = TrackManager(confirm_hits=3, confirm_window=15, max_missed=5)
    for _ in range(15):
        trk.step([[1.0, 0.0, 0.0]], 1.0 / 90.0)
    trk.retune(confirm_hits=3, confirm_window=5, max_missed=5)
    assert all(len(h) <= 5 for h in trk._hits.values())


def test_tracker_retune_never_asks_for_more_hits_than_window():
    trk = TrackManager()
    trk.retune(confirm_hits=9, confirm_window=4, max_missed=15)
    assert trk.confirm_window >= trk.confirm_hits


def test_self_detection_retune_keeps_a_verdict():
    """A verdict is a claim about the calibration; the frame rate is not evidence."""
    mon = SelfDetectionMonitor(window=3, motion_min_m=0.05, offset_tol_m=0.02,
                               confirm=2, release=5)
    for i in range(6):
        x = 0.1 * i
        mon.update('fr3_link5#0', [x, 0.0, 0.0], [x + 0.03, 0.0, 0.0])
    assert mon.is_self('fr3_link5#0')

    assert mon.retune(window=9, confirm=6, release=15)
    assert (mon.window, mon.confirm, mon.release) == (9, 6, 15)
    assert mon.is_self('fr3_link5#0')


def test_self_detection_retune_is_idempotent():
    mon = SelfDetectionMonitor(window=20, confirm=5, release=15)
    assert not mon.retune(window=20, confirm=5, release=15)


def test_self_detection_retune_trims_history():
    mon = SelfDetectionMonitor(window=20, motion_min_m=0.05, confirm=2,
                               release=5)
    for i in range(20):
        x = 0.01 * i
        mon.update('k', [x, 0.0, 0.0], [x + 0.5, 0.0, 0.0])
    mon.retune(window=5, confirm=2, release=5)
    assert all(len(h) <= 5 for h in mon._hist.values())


# ── the end-to-end property the change exists for ─────────────────────────

@pytest.mark.parametrize('hz', [15.0, 30.0, 60.0, 90.0])
def test_lifecycle_durations_are_rate_independent(hz):
    """Coast, birth window and self-detection delays, in SECONDS, at any rate.

    The config states 0.167 s of coast, a 0.167 s birth window at 60% hits and
    0.667/0.167/0.5 s for the guard. Derived at each rate, every one of them
    must come back within a frame of the duration it stands for.
    """
    coast = frames_for(0.167, hz)
    window = frames_for(0.167, hz, minimum=2)
    hits = max(1, min(window, round(window * 0.6)))
    sd = [frames_for(s, hz, minimum=2) for s in (0.667, 0.167, 0.5)]

    tol = 1.0 / hz
    assert abs(coast / hz - 0.167) <= tol
    assert abs(window / hz - 0.167) <= tol
    assert abs(hits / hz - 0.1) <= tol          # 60% of 0.167 s
    for frames, seconds in zip(sd, (0.667, 0.167, 0.5)):
        assert abs(frames / hz - seconds) <= tol


def test_the_90hz_config_reproduces_the_30hz_tuning():
    """Sanity: the numbers this repo was tuned with are the 30 Hz instance."""
    assert frames_for(0.167, 30.0) == 5                      # max_missed: 5
    assert frames_for(0.167, 30.0, minimum=2) == 5           # confirm_window: 5
    assert round(5 * 0.6) == 3                               # confirm_hits: 3
    assert frames_for(0.667, 30.0, minimum=2) == 20          # window: 20
    assert frames_for(0.167, 30.0) == 5                      # confirm: 5
    assert frames_for(0.5, 30.0) == 15                       # release: 15
    assert math.isclose(tau_for_alpha(0.5, 30.0), 0.0481, abs_tol=5e-4)


# ── the node's own derivation ─────────────────────────────────────────────
# Called unbound on a stub: what is under test is the arithmetic that turns the
# config's durations into the counters the tracker and the guard are built with,
# and that needs neither a ROS context nor a camera.

def _stub(trk_timing=None, sd_timing=None, trk_raw=None, sd_raw=None):
    from types import SimpleNamespace
    return SimpleNamespace(_trk_timing=trk_timing, _sd_timing=sd_timing,
                           _trk_cfg_raw=trk_raw or {}, _sd_cfg_raw=sd_raw or {})


def _node_cls():
    from franka_experiments.nodes.real_time_distance import RealTimeDistance
    return RealTimeDistance


@pytest.mark.parametrize('hz,expected_trk,expected_sd', [
    (30.0, (3, 5, 5), (20, 5, 15)),        # the tuning this repo was measured at
    (90.0, (9, 15, 15), (60, 15, 45)),     # the profile it now runs
])
def test_node_derives_the_counts_from_the_config_durations(hz, expected_trk,
                                                           expected_sd):
    R = _node_cls()
    st = _stub(trk_timing={'window_s': 0.167, 'coast_s': 0.167,
                           'hits_frac': 0.6},
               sd_timing={'window_s': 0.667, 'confirm_s': 0.167,
                          'release_s': 0.5})
    assert R._track_counts(st, hz) == expected_trk
    assert R._self_detect_counts(st, hz) == expected_sd


def test_node_falls_back_to_frame_counts_for_an_unmigrated_config():
    """An old config must behave exactly as before, at any measured rate."""
    R = _node_cls()
    st = _stub(trk_raw={'confirm_hits': 3, 'confirm_window': 5,
                        'max_missed': 5},
               sd_raw={'window': 20, 'confirm': 5, 'release': 15})
    assert R._track_counts(st, 90.0) == (3, 5, 5)
    assert R._self_detect_counts(st, 90.0) == (20, 5, 15)


def test_node_refuses_a_half_migrated_config():
    """Seconds for the coast, frames for the confirmation: use the frames."""
    R = _node_cls()
    st = _stub(trk_raw={'confirm_hits': 3, 'confirm_window': 5,
                        'max_missed': 5})
    assert R._read_track_timing(st, {'max_coast_s': 0.167}) is None


def test_the_shipped_config_states_durations():
    """fr3_complete.yaml is migrated, and states the 90 Hz it now runs at."""
    import os
    import yaml
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(here, 'config', 'fr3_complete.yaml')) as fh:
        cfg = yaml.safe_load(fh)
    trk, dist = cfg['tracking'], cfg['distance']
    assert trk['max_coast_s'] == pytest.approx(0.167)
    assert trk['confirm_window_s'] == pytest.approx(0.167)
    assert trk['confirm_hits_frac'] == pytest.approx(0.6)
    sd = trk['self_detection']
    assert (sd['window_s'], sd['confirm_s'], sd['release_s']) == \
        pytest.approx((0.667, 0.167, 0.5))
    assert dist['lpf_tau_s'] == pytest.approx(0.048, abs=1e-3)
    assert dist['depth_rate_hz'] == pytest.approx(90.0)
    # The frame-counted keys are GONE, not shadowed: a reader who changes one
    # and sees nothing happen is the failure this migration exists to prevent.
    for dead in ('max_missed', 'confirm_hits', 'confirm_window'):
        assert dead not in trk, f'{dead} still in tracking:'
    for dead in ('window', 'confirm', 'release'):
        assert dead not in sd, f'{dead} still in self_detection:'


def test_estimator_survives_a_startup_stall():
    """The defect this median replaced an EMA for.

    A stack startup delivered a handful of 100-200 ms hitches among 11 ms
    frames. An EMA of dt read 6.8 Hz off them and rescaled a 0.167 s coast to
    one frame; the median must stay on the frames that are actually arriving.
    """
    est = FrameRateEstimator(nominal_hz=90.0, window=120)
    dts = []
    for i in range(400):
        dts.append(0.147 if i % 9 == 0 else 1.0 / 90.0)   # ~11% hitches
    reports = sum(1 for dt in dts if est.add(dt))
    assert est.measured_hz == pytest.approx(90.0, rel=0.05)
    assert reports == 0


def test_estimator_moves_once_half_the_window_is_slow():
    """It is not blind to a real slowdown, only to a minority of hitches."""
    est = FrameRateEstimator(nominal_hz=90.0, window=120)
    assert sum(1 for _ in range(200) if est.add(1.0 / 90.0)) == 0
    reports = sum(1 for _ in range(200) if est.add(1.0 / 45.0))
    assert reports == 1
    assert est.measured_hz == pytest.approx(45.0, rel=0.05)


def test_counts_never_go_degenerate_at_an_absurd_rate():
    """Even at 7 Hz the tracker keeps a coast and a tentative state."""
    R = _node_cls()
    st = _stub(trk_timing={'window_s': 0.167, 'coast_s': 0.167,
                           'hits_frac': 0.6},
               sd_timing={'window_s': 0.667, 'confirm_s': 0.167,
                          'release_s': 0.5})
    hits, window, coast = R._track_counts(st, 6.8)
    assert coast >= 2 and window >= 3 and 2 <= hits < window
    w, c, r = R._self_detect_counts(st, 6.8)
    assert w >= 4 and c >= 2 and r >= 3
