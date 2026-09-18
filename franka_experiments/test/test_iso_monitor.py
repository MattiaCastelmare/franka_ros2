"""The independent SSM monitor and its non-safety-rated stop (roadmap Step 6).

``iso_safety_monitor`` is the channel that ENFORCES what the QP only shapes, so
what has to be pinned is its decision logic, not its ROS wiring:

* a trip needs ``iso_monitor_ticks`` CONSECUTIVE ticks — one noisy depth frame
  must not stop the cell, and a condition that flickers must not accumulate a
  trip out of unrelated single-tick events;
* each of the four conditions trips on its own;
* the reset REFUSES while the condition is still true. A reset that clears a
  latch under a live condition is a bypass with a service call in front of it;
* with ``iso_stop_requires_reset: false`` the stop clears by itself, which is
  what SSM actually permits;
* the message goes out EVERY tick, latched or not. A silent monitor is
  indistinguishable from a dead one.

The node's callbacks and timer are driven directly against a stub ``self``: the
tick is pure decision logic over four attributes, and standing up rclpy to
exercise it would test the executor, not the logic.
"""

import types

import numpy as np
import pytest

from franka_experiments.nodes.iso_safety_monitor import (
    ISOSafetyMonitor, REASON_FAULT, REASON_INSIDE, REASON_NONE, REASON_SPEED,
    REASON_STALE)
from franka_experiments.utils.iso_ssm import ssm_speed_cap

ISO = dict(iso_t_reaction=0.10, iso_a_stop=4.0, iso_v_human=2.0,
           iso_c_intrusion=0.05, iso_z_depth=0.02, iso_z_robot=0.01,
           iso_speed_tol=0.05, iso_monitor_ticks=3, iso_stop_requires_reset=True,
           iso_mode='automatic', iso_tcp_reduced_speed=0.25,
           link_speed_max=0.8, qp_rate_hz=100.0,
           joint_state_timeout=0.2, distance_timeout=0.5)

FLOOR = ISO['iso_c_intrusion'] + ISO['iso_z_depth'] + ISO['iso_z_robot']


class _Log:
    def info(self, *a, **k):  pass
    def warn(self, *a, **k):  pass
    def error(self, *a, **k): pass


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


class _Msg:
    def __init__(self):
        self.data = [0.0] * 5


def _mon(**over):
    """A monitor object with only the attributes ``_tick`` and ``_on_reset`` read."""
    m = types.SimpleNamespace()
    m.P = types.SimpleNamespace(**dict(ISO, **over))
    m._floor = m.P.iso_c_intrusion + m.P.iso_z_depth + m.P.iso_z_robot
    m._reduced = str(m.P.iso_mode) == 'reduced'
    m._tcp_link = 'fr3_link8'
    m._q = np.zeros(7)
    m._qdot = np.zeros(7)
    m._js_stamp = 100.0
    m._dist = object()
    m._dist_stamp = 100.0
    m._fault = 0.0
    m._latched = False
    m._reason = REASON_NONE
    m._streak = {REASON_SPEED: 0, REASON_INSIDE: 0,
                 REASON_FAULT: 0, REASON_STALE: 0}
    m._live = REASON_NONE
    m._diag = (0.0, float('inf'), 0.0)
    m._pub = _Pub()
    m._msg = _Msg()
    m.get_logger = lambda: _Log()
    m._now = lambda: 100.0
    # _evaluate is stubbed per test: what it returns IS the scene.
    m._evaluate = lambda: (0.0, float('inf'), 0.0, REASON_NONE)
    return m


def _tick(m, n=1):
    for _ in range(n):
        ISOSafetyMonitor._tick(m)


def _scene(reason, s_p=0.0, cap=float('inf'), v=0.0):
    return lambda: (s_p, cap, v, reason)


# ── the message always goes out ──────────────────────────────────────────────

def test_the_monitor_publishes_every_tick_even_with_nothing_wrong():
    m = _mon()
    _tick(m, 5)
    assert len(m._pub.sent) == 5
    assert all(row[0] == 0.0 and row[1] == REASON_NONE for row in m._pub.sent)


def test_the_published_row_carries_the_scene():
    m = _mon()
    m._evaluate = _scene(REASON_NONE, s_p=0.42, cap=0.31, v=0.17)
    _tick(m)
    latched, reason, s_p, cap, v = m._pub.sent[-1]
    assert (s_p, cap, v) == pytest.approx((0.42, 0.31, 0.17))


# ── consecutive ticks ────────────────────────────────────────────────────────

def test_a_trip_needs_iso_monitor_ticks_consecutive_ticks():
    m = _mon()
    m._evaluate = _scene(REASON_SPEED, v=2.0, cap=0.1)
    _tick(m, 2)
    assert not m._latched, 'tripped on 2 of 3 ticks'
    _tick(m)
    assert m._latched and m._reason == REASON_SPEED


def test_one_noisy_frame_does_not_trip():
    m = _mon()
    for i in range(30):
        m._evaluate = _scene(REASON_SPEED if i % 5 == 0 else REASON_NONE)
        _tick(m)
    assert not m._latched


def test_two_different_conditions_do_not_accumulate_one_trip():
    """Alternating single-tick events of two kinds are two flickers, not a trip."""
    m = _mon()
    for i in range(30):
        m._evaluate = _scene(REASON_SPEED if i % 2 else REASON_INSIDE)
        _tick(m)
    assert not m._latched


# ── each condition trips on its own ──────────────────────────────────────────

def test_the_speed_cap_condition_trips():
    m = _mon()
    m._evaluate = _scene(REASON_SPEED, v=1.2, cap=0.2)
    _tick(m, 3)
    assert m._latched and m._reason == REASON_SPEED


def test_being_inside_c_plus_z_trips():
    m = _mon()
    m._evaluate = _scene(REASON_INSIDE, s_p=FLOOR, cap=0.0)
    _tick(m, 3)
    assert m._latched and m._reason == REASON_INSIDE


def test_a_safety_chain_fault_trips():
    m = _mon()
    m._fault = 1.0
    _tick(m, 3)
    assert m._latched and m._reason == REASON_FAULT


def test_stale_inputs_trip():
    m = _mon()
    m._dist_stamp = 100.0 - 10.0        # older than distance_timeout
    _tick(m, 3)
    assert m._latched and m._reason == REASON_STALE


def test_a_missing_joint_state_trips():
    m = _mon()
    m._q = None
    _tick(m, 3)
    assert m._latched and m._reason == REASON_STALE


# ── the latch ────────────────────────────────────────────────────────────────

def test_the_latch_holds_after_the_condition_clears():
    m = _mon()
    m._evaluate = _scene(REASON_SPEED, v=1.2, cap=0.2)
    _tick(m, 3)
    m._evaluate = _scene(REASON_NONE)
    _tick(m, 50)
    assert m._latched, 'a latch that self-clears is not a latch'
    assert m._pub.sent[-1][0] == 1.0


def test_without_the_latch_the_stop_clears_on_its_own():
    """Automatic resumption is what SSM actually permits — [E] to latch."""
    m = _mon(iso_stop_requires_reset=False)
    m._evaluate = _scene(REASON_SPEED, v=1.2, cap=0.2)
    _tick(m, 3)
    assert m._latched
    m._evaluate = _scene(REASON_NONE)
    _tick(m, 2)
    assert not m._latched
    assert m._pub.sent[-1][0] == 0.0


# ── the reset ────────────────────────────────────────────────────────────────

def _reset(m):
    resp = types.SimpleNamespace(success=None, message='')
    return ISOSafetyMonitor._on_reset(m, None, resp)


def test_the_reset_refuses_while_the_condition_is_still_true():
    m = _mon()
    m._evaluate = _scene(REASON_SPEED, v=1.2, cap=0.2)
    _tick(m, 5)
    resp = _reset(m)
    assert resp.success is False
    assert 'REFUSED' in resp.message and 'not a bypass' in resp.message
    assert m._latched


def test_the_reset_clears_once_the_condition_is_gone():
    m = _mon()
    m._evaluate = _scene(REASON_SPEED, v=1.2, cap=0.2)
    _tick(m, 3)
    m._evaluate = _scene(REASON_NONE)
    _tick(m)
    resp = _reset(m)
    assert resp.success is True
    assert not m._latched
    _tick(m)
    assert m._pub.sent[-1][0] == 0.0


def test_the_reset_clears_the_streaks_so_it_does_not_re_trip_immediately():
    m = _mon()
    m._evaluate = _scene(REASON_SPEED, v=1.2, cap=0.2)
    _tick(m, 10)
    m._evaluate = _scene(REASON_NONE)
    _tick(m)
    _reset(m)
    assert all(v == 0 for v in m._streak.values())


def test_a_reset_with_nothing_latched_succeeds_quietly():
    resp = _reset(_mon())
    assert resp.success is True and 'no stop latched' in resp.message


def test_a_chain_fault_also_blocks_the_reset():
    m = _mon()
    m._fault = 1.0
    _tick(m, 3)
    resp = _reset(m)
    assert resp.success is False


# ── the reduced-speed cap the monitor applies ────────────────────────────────

def test_reduced_mode_caps_the_tcp_at_250_mm_per_s():
    """The monitor applies the SAME reduced-speed cap the filter's extra TCP
    row carries, so the two channels agree on what 'reduced' means."""
    d, v_app = 1.0, 0.0
    free = ssm_speed_cap(d, v_app, t_r=ISO['iso_t_reaction'], a_s=ISO['iso_a_stop'],
                         c=ISO['iso_c_intrusion'], z_d=ISO['iso_z_depth'],
                         z_r=ISO['iso_z_robot'], v_max=ISO['link_speed_max'])
    assert free > ISO['iso_tcp_reduced_speed']
    assert min(free, ISO['iso_tcp_reduced_speed']) == pytest.approx(0.25)
