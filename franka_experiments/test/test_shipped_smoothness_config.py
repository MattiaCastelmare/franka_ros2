"""Guards on the shipped avoidance-smoothness settings (2026-09-30, docs/ball_throw_closed_loop.md).

These are not physics tests; they pin the numbers that the closed-loop evaluation justified, so that a
later edit of fr3_control.yaml that undoes them has to say so here, in a test, and not by accident.
"""
import os

import numpy as np
import yaml

CFG = os.path.join(os.path.dirname(__file__), '..', 'config', 'fr3_control.yaml')


def _params():
    with open(CFG) as fh:
        return yaml.safe_load(fh)['params']


def test_command_jerk_is_bounded():
    P = _params()
    assert P['slew_box_enabled'] is True
    jerk = P['max_qddot_delta'] * P['qp_rate_hz']          # rad/s^3
    # 500 (the old 5.0 rad/s^2 per tick) was a 0 -> 10 rad/s^2 step in 20 ms; the measured plateau where
    # clearance stops paying for jerk is 100-200.
    assert jerk <= 300.0, f'command jerk bound {jerk:.0f} rad/s^3: see docs/ball_throw_closed_loop.md'


def test_wrist_authority_is_capped_and_the_box_is_complete():
    P = _params()
    acc = P['qddot_accel_limits']
    assert len(acc) == 7 and all(a > 0 for a in acc)
    assert max(acc) <= 6.0 + 1e-9, 'a wider box was measured jerky-on-paper only; see the doc before widening'
    # never wider than libfranka's rated acceleration
    assert max(acc) <= P['qddot_max_abs'] <= 10.0


def test_joint_velocity_cap_leaves_room_below_the_firmware_limit():
    P = _params()
    # watchman_profile.md: SLS-J sits 20 % above the software cap, which needs margin <= 1/1.2
    assert P['velocity_box_margin'] <= 1.0 / 1.2


def test_slew_and_box_are_consistent_with_the_task_needs():
    """The tracked path must still be reachable: the task needs ~1 rad/s and a few rad/s^2 at most."""
    P = _params()
    qd_cap = P['velocity_box_margin'] * 2.62
    assert qd_cap >= 1.5
    assert np.min(P['qddot_accel_limits']) >= 2.5


def test_the_state_governor_fades_before_the_velocity_box_clamps():
    P = _params()
    assert P['governor_envelope_margin'] <= P['velocity_box_margin'], (
        'state_governor.py: the governor must start fading BEFORE the hard box clamps')
