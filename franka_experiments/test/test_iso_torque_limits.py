"""Command feasibility: torque limits and braking authority (roadmap Step 8).

``S_p`` assumes ``a_s`` is actually DELIVERED. Two things can make that false,
and both are invisible without these:

* the torque command exceeds what the joint can produce, or steps faster than
  the drive can follow. ``rt_torque_controller`` has always clipped; what is new
  is that the clipping is OBSERVABLE, on ``/NS_1/torque_saturation``;
* the realized acceleration is a fraction of the commanded one, which is the
  ``qdd_cmd_rad`` / ``qdd_real_rad`` gap already visible on hardware. Every
  stopping distance derived from ``a_s`` is then optimistic by that fraction.

All **[E]**. The rated analogues are *stopping time limiting* and *stopping
distance limiting* (ISO 10218-1:2025, 5.5.6 / 5.5.7) and this is neither.
"""

import types

import numpy as np
import pytest

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter
from franka_experiments.nodes.qddot_to_torque import QddotToTorqueNode

NV = 7
EFFORT = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])


class _Log:
    def warn(self, *a, **k):  pass
    def error(self, *a, **k): pass
    def info(self, *a, **k):  pass


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


class _Msg:
    def __init__(self):
        self.data = [0.0] * NV


def _node(iso=True, rate=40.0):
    n = types.SimpleNamespace()
    n._iso_enabled = iso
    n._tau_rate_max = rate
    n._effort_max = EFFORT.copy()
    n._tau_prev = None
    n._sat_pub = _Pub()
    n._sat_msg = _Msg()
    n.get_logger = lambda: _Log()
    return n


def _limit(n, tau):
    return QddotToTorqueNode._limit_and_report(n, np.asarray(tau, dtype=float))


# ── the effort clip ──────────────────────────────────────────────────────────

def test_the_torque_is_clipped_to_the_manufacturer_effort_limits():
    n = _node()
    out = _limit(n, [200.0, 0, 0, 0, 0, 0, 50.0])
    assert out[0] == pytest.approx(87.0)
    assert out[6] == pytest.approx(12.0)


def test_a_negative_overshoot_is_clipped_too():
    n = _node()
    assert _limit(n, [-200.0] + [0.0] * 6)[0] == pytest.approx(-87.0)


def test_saturation_is_flagged_per_joint():
    n = _node()
    _limit(n, [200.0, 0, 0, 0, 0, 0, 50.0])
    assert n._sat_pub.sent[-1] == [1.0, 0, 0, 0, 0, 0, 1.0]


def test_a_torque_inside_the_limits_flags_nothing():
    n = _node()
    _limit(n, [10.0] * NV)
    assert n._sat_pub.sent[-1] == [0.0] * NV


# ── the rate limit ───────────────────────────────────────────────────────────

def test_the_rate_limit_bounds_the_step_between_ticks():
    n = _node(rate=40.0)
    _limit(n, [0.0] * NV)
    out = _limit(n, [80.0] + [0.0] * 6)
    assert out[0] == pytest.approx(40.0), 'one tick, one rate limit'
    out = _limit(n, [80.0] + [0.0] * 6)
    assert out[0] == pytest.approx(80.0), 'the second tick gets there'


def test_the_first_command_is_not_rate_limited():
    """There is no previous torque to step from; rate-limiting against a
    fictional zero would put a ramp in front of every start."""
    n = _node(rate=1.0)
    assert _limit(n, [50.0] + [0.0] * 6)[0] == pytest.approx(50.0)


def test_a_rate_hit_is_flagged():
    n = _node(rate=10.0)
    _limit(n, [0.0] * NV)
    _limit(n, [50.0] + [0.0] * 6)
    assert n._sat_pub.sent[-1][0] == 1.0


def test_tau_prev_tracks_what_actually_went_out():
    """Not what was computed: a rate limit centred on a command that was never
    sent walks away from the real torque."""
    n = _node(rate=40.0)
    _limit(n, [0.0] * NV)
    out = _limit(n, [500.0] + [0.0] * 6)
    assert out[0] == pytest.approx(40.0)
    assert n._tau_prev[0] == pytest.approx(40.0)


# ── the flag-off path ────────────────────────────────────────────────────────

def test_with_the_flag_off_the_torque_is_untouched():
    n = _node(iso=False, rate=1.0)
    _limit(n, [0.0] * NV)
    out = _limit(n, [500.0, -500.0, 0, 0, 0, 0, 0])
    assert out[0] == pytest.approx(500.0)
    assert out[1] == pytest.approx(-500.0)


def test_with_the_flag_off_the_saturation_topic_still_reports():
    """Observability changes no number, so it is not behind the flag."""
    n = _node(iso=False)
    _limit(n, [200.0] + [0.0] * 6)
    assert n._sat_pub.sent[-1][0] == 1.0


# ── braking authority ────────────────────────────────────────────────────────

class _FilterStub:
    def __init__(self, real, **over):
        defaults = dict(iso_enabled=True, iso_brake_frac_min=0.5,
                        iso_brake_frac_ticks=10, iso_a_stop=1.0)
        defaults.update(over)
        self.P = types.SimpleNamespace(**defaults)
        self._diag_qddot_real = np.asarray(real, dtype=float)
        self._brake_frac_run = 0
        self._brake_frac_warned = False
        self._diag_brake_frac = 1.0

    def get_logger(self):
        return _Log()


def _brake(f, cmd, n_active=1, n=1):
    out = False
    for _ in range(n):
        out = CBFSafetyFilter._brake_authority_fault(
            f, np.asarray(cmd, dtype=float), n_active)
    return out


def test_a_delivered_command_raises_nothing():
    cmd = [2.0, 0, 0, 0, 0, 0, 0]
    assert _brake(_FilterStub(cmd), cmd, n=50) is False


def test_a_half_delivered_command_faults_after_the_tick_window():
    cmd = np.array([4.0, 0, 0, 0, 0, 0, 0])
    f = _FilterStub(0.2 * cmd)
    assert _brake(f, cmd, n=9) is False, 'must need the full window'
    assert _brake(f, cmd, n=1) is True


def test_the_projection_and_not_the_norm_ratio_is_what_counts():
    """A large realized acceleration in another direction is not braking."""
    cmd = np.array([4.0, 0, 0, 0, 0, 0, 0])
    f = _FilterStub(np.array([0.0, 40.0, 0, 0, 0, 0, 0]))
    assert _brake(f, cmd, n=20) is True


def test_a_small_command_is_not_judged():
    cmd = [0.1] + [0.0] * 6            # ‖q̈_cmd‖ well under 0.5 rad/s²
    assert _brake(_FilterStub([0.0] * NV), cmd, n=50) is False


def test_no_active_obstacle_row_no_judgement():
    cmd = [4.0] + [0.0] * 6
    assert _brake(_FilterStub([0.0] * NV), cmd, n_active=0, n=50) is False


def test_the_run_resets_when_the_command_is_delivered_again():
    cmd = np.array([4.0, 0, 0, 0, 0, 0, 0])
    f = _FilterStub(0.1 * cmd)
    _brake(f, cmd, n=9)
    f._diag_qddot_real = cmd.copy()     # authority is back
    _brake(f, cmd, n=1)
    assert f._brake_frac_run == 0
    f._diag_qddot_real = 0.1 * cmd
    assert _brake(f, cmd, n=9) is False, 'the run must restart, not resume'


def test_the_fault_is_inert_with_the_flag_off():
    cmd = np.array([4.0, 0, 0, 0, 0, 0, 0])
    f = _FilterStub(np.zeros(NV), iso_enabled=False)
    assert _brake(f, cmd, n=100) is False
