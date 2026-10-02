"""Singularity recovery: detector, quintic return to q_home, automatic resume.

Pure numpy; no ROS.
"""

import numpy as np
import pytest

from franka_experiments.utils.singularity_recovery import (
    SingularityRecovery, plan_duration, quintic_coeffs, quintic_eval)

QH = np.array([0.0, -0.78, 0.0, -2.36, 0.0, 1.57, 0.78])
Q0 = QH + np.array([0.3, 0.2, -0.4, 0.3, 0.5, -0.2, 0.6])
Z = np.zeros(7)
DT = 0.01


def _rec(**kw):
    d = dict(stall_s=1.0, vmax=0.5, amax=2.0, t_min=1.0, settle_s=0.5,
             cooldown_s=2.0)
    d.update(kw)
    return SingularityRecovery(**d)


def _run(rec, seconds, stuck=True, q=Q0, qdot=Z, t0=0.0):
    """Tick with a stuck (or healthy) signature; returns (refs, events)."""
    refs, events, t = [], [], t0
    for _ in range(int(round(seconds / DT))):
        sig, spd, err = (0.01, 0.0, 0.1) if stuck else (0.5, 0.1, 0.0)
        refs.append(rec.update(t, q, qdot, QH, sig, spd, err))
        e = rec.pop_event()
        if e:
            events.append((round(t, 2), e))
        t += DT
    return refs, events


def _until_resumed(rec, margin=0.2):
    """Tick the stuck signature up to just after the 'resumed' event (the
    fixture does not move the arm, so it would re-trigger after the cooldown)."""
    refs, events, t = [], [], 0.0
    while not any(e == 'resumed' for _, e in events) and t < 30.0:
        r, ev = _run(rec, DT, t0=t)
        refs += r
        events += ev
        t += DT
    r, ev = _run(rec, margin, t0=t)
    return refs + r, events + ev


def test_quintic_boundary_conditions():
    T = 2.0
    c = quintic_coeffs(Q0, Z, QH, T)
    q, dq, ddq = quintic_eval(c, 0.0)
    assert np.allclose(q, Q0) and np.allclose(dq, 0) and np.allclose(ddq, 0)
    q, dq, ddq = quintic_eval(c, T)
    assert np.allclose(q, QH) and np.allclose(dq, 0, atol=1e-9)
    assert np.allclose(ddq, 0, atol=1e-9)


def test_plan_respects_velocity_and_acceleration_limits():
    for v0 in (Z, np.full(7, 0.3), np.full(7, -0.4)):
        T = plan_duration(Q0, v0, QH, 0.5, 2.0)
        c = quintic_coeffs(Q0, v0, QH, T)
        taus = np.linspace(0, T, 400)
        dq = np.array([quintic_eval(c, x)[1] for x in taus])
        ddq = np.array([quintic_eval(c, x)[2] for x in taus])
        assert np.abs(dq).max() <= 0.5 + 1e-9
        assert np.abs(ddq).max() <= 2.0 + 1e-9


def test_no_trigger_while_moving_or_away_from_the_singularity_or_without_error():
    for sig, spd, err in ((0.5, 0.0, 0.1),     # far from a singularity
                          (0.01, 0.1, 0.1),    # near one but moving
                          (0.01, 0.0, 0.0)):   # near one, nothing to track
        rec, t = _rec(), 0.0
        for _ in range(500):
            assert rec.update(t, Q0, Z, QH, sig, spd, err) is None
            t += DT
        assert rec.state == rec.IDLE and rec.n_recoveries == 0


def test_a_brief_stall_does_not_trigger_and_resets_the_timer():
    rec = _rec()
    _run(rec, 0.9)                       # 0.9 s < stall_s
    _run(rec, 0.5, stuck=False, t0=0.9)  # recovers on its own
    _run(rec, 0.9, t0=1.4)               # a new stall restarts from 0
    assert rec.n_recoveries == 0


def test_stuck_triggers_return_arrives_at_home_and_resumes():
    rec = _rec()
    refs, events = _until_resumed(rec)
    names = [e for _, e in events]
    assert names[:3] == ['started', 'arrived', 'resumed']
    t_start = events[0][0]
    assert 1.0 <= t_start <= 1.1                       # after stall_s
    # reference starts at the measured state and ends exactly at home
    first = next(r for r in refs if r is not None)
    assert np.allclose(first[0], Q0, atol=1e-3)
    t_arr = events[1][0]
    at_arrival = refs[int(round(t_arr / DT))]
    assert np.allclose(at_arrival[0], QH, atol=1e-9)
    assert np.allclose(at_arrival[1], 0, atol=1e-9)
    # resume hands control back: reference is None after 'resumed'
    t_res = events[2][0]
    assert refs[int(round(t_res / DT))] is None
    assert not rec.active


def test_reference_is_continuous_and_bounded():
    rec = _rec()
    refs, events = _until_resumed(rec)
    live = [r for r in refs if r is not None]
    q = np.array([r[0] for r in live])
    dq = np.array([r[1] for r in live])
    assert np.abs(dq).max() <= 0.5 + 1e-9
    assert np.abs(np.diff(q, axis=0)).max() <= 0.5 * DT * 1.5   # no jumps


def test_cooldown_blocks_an_immediate_second_recovery_then_rearms():
    rec = _rec()
    _, events = _until_resumed(rec, margin=1.5)     # inside cooldown_s = 2
    assert rec.n_recoveries == 1       # still stuck signature, but cooling down
    t_res = next(t for t, e in events if e == 'resumed')
    # after cooldown_s + stall_s the still-stuck task re-triggers
    _run(rec, 3.5, t0=t_res + 1.7)
    assert rec.n_recoveries == 2


def test_already_home_uses_the_minimum_duration():
    rec = _rec(t_min=1.0)
    rec.update(0.0, QH, Z, QH, 0.01, 0.0, 0.1)
    for k in range(1, 200):
        rec.update(k * DT, QH, Z, QH, 0.01, 0.0, 0.1)
    assert rec.n_recoveries == 1 and rec.duration == pytest.approx(1.0)
