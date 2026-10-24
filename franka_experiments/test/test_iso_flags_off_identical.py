"""With every ISO flag off, nothing moved. The ground rule, as assertions.

The whole ISO layer is behind flags that ship ``false``, and the promise that
comes with that is stronger than "it probably still works": with the flags off
the filter's numerical output must be what it was before the layer existed. That
promise is easy to break by accident — a new term added unconditionally, a
default that shifted, a row that appears one tick earlier — and impossible to
check by reading.

So this pins the SHIPPED configuration rather than a constructed one: it reads
``config/fr3_control.yaml``, asserts every ``iso_*`` switch is off, and then
drives the real ``ConstraintBuilder`` through scenes that exercise each of the
paths the ISO layer touches, comparing against the same builder with the ISO
parameters set to values that WOULD change the answer if the flag were read.
"""

import os

import numpy as np
import pytest
import yaml

from franka_experiments.utils.cbf_qp_assembly import build_row_rhs

from _cbf_builder_harness import make_builder, make_obstacle, ObstacleSnap, JointSnap

NV = 7
CONFIG = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                      '..', 'config', 'fr3_control.yaml')

#: ISO constants that WOULD bite hard if any flag were read: a huge intrusion
#: distance, a feeble deceleration, a fast human. If a single ISO code path runs
#: with the flags off, one of the comparisons below moves.
LOUD = dict(iso_t_reaction=0.9, iso_a_stop=0.2, iso_v_human=4.0,
            iso_c_intrusion=1.9, iso_z_depth=0.9, iso_z_robot=0.9,
            iso_tcp_reduced_speed=0.001)

QDOT = np.array([0.6, -0.4, 0.5, 0.3, -0.5, 0.4, 0.2])


@pytest.fixture(scope='module')
def params():
    with open(CONFIG) as fh:
        return yaml.safe_load(fh)['params']


def test_every_iso_switch_ships_off(params):
    assert params['iso_enabled'] is False
    assert params['iso_ssm_speed_rows'] is False
    assert params['iso_monitor_enabled'] is False
    assert params['iso_mode'] == 'automatic'


def test_the_launch_defaults_agree_with_the_yaml():
    ld = os.path.join(os.path.dirname(CONFIG), 'launch_defaults.yaml')
    with open(ld) as fh:
        d = yaml.safe_load(fh)
    assert d['iso_enabled'] is False
    assert d['iso_ssm_speed_rows'] is False
    assert d['iso_monitor_enabled'] is False
    assert d['iso_mode'] == 'automatic'
    # Empty string = "leave the YAML ceiling alone".
    assert d['link_speed_max'] == ''
    assert d['retreat_cap_max_speed'] == ''


def _snapshot(d, qdot=QDOT, **over):
    over.setdefault('link_speed_rows_enabled', True)
    over.setdefault('retreat_cap_enabled', True)
    over.setdefault('joint_limit_rows_enabled', True)
    over.setdefault('link_speed_activate_frac', 0.0)
    b = make_builder(**over)
    con = b.build(JointSnap(np.zeros(NV), np.asarray(qdot, float), 0.0),
                  ObstacleSnap((make_obstacle(d=d, cp_label='fr3_link5#0'),),
                               0.0, 0.0), 0.0)
    assert con is not None
    return b, con


def _assert_identical(a, b):
    assert np.array_equal(a.A, b.A)
    assert np.array_equal(a.h_bar, b.h_bar)
    assert np.array_equal(a.cap_v, b.cap_v)
    assert np.array_equal(a.group, b.group)
    assert np.array_equal(a.v_obs, b.v_obs)
    assert list(a.links) == list(b.links)


@pytest.mark.parametrize('d', [0.05, 0.10, 0.18, 0.35, 0.9])
def test_the_rows_are_identical_at_every_distance(d):
    _, base = _snapshot(d)
    _, loud = _snapshot(d, **LOUD)
    _assert_identical(base, loud)


def test_the_rows_are_identical_in_reduced_mode_with_the_master_flag_off():
    _, base = _snapshot(0.25)
    _, loud = _snapshot(0.25, iso_mode='reduced', **LOUD)
    _assert_identical(base, loud)
    assert not any(l.startswith('red:') for l in loud.links)


def test_the_rows_are_identical_with_ssm_rows_asked_for_but_no_master_flag():
    _, base = _snapshot(0.25)
    _, loud = _snapshot(0.25, iso_ssm_speed_rows=True, **LOUD)
    _assert_identical(base, loud)


def test_the_right_hand_side_is_identical_too():
    """The rows could match while the bound they carry did not."""
    for d in (0.08, 0.25, 0.6):
        _, base = _snapshot(d)
        _, loud = _snapshot(d, iso_ssm_speed_rows=True, iso_mode='reduced', **LOUD)
        kw = dict(k0=25.0, k1=10.5, retreat_horizon=0.2, speed_horizon=0.2)
        h_b, caps_b = build_row_rhs(base, QDOT, QDOT, **kw)
        h_l, caps_l = build_row_rhs(loud, QDOT, QDOT, **kw)
        assert np.array_equal(h_b, h_l)
        assert caps_b == caps_l


def test_the_rows_are_identical_with_the_velocity_standoff_on():
    """The standoff is the mechanism the ISO layer re-parameterises for S_h, so
    it is the one most likely to move without the flag."""
    kw = dict(enable_velocity_standoff=True, obstacle_velocity_enabled=True,
              obstacle_velocity_source='tracker', obstacle_velocity_min_frames=1)
    b1 = make_builder(**kw, link_speed_rows_enabled=True,
                      link_speed_activate_frac=0.0)
    b2 = make_builder(**kw, **LOUD, link_speed_rows_enabled=True,
                      link_speed_activate_frac=0.0)
    for builder in (b1, b2):
        pass
    ob = make_obstacle(d=0.30, cp_label='fr3_link5#0', v=(0.0, 0.8, 0.0),
                       frames_seen=5)
    js = JointSnap(np.zeros(NV), QDOT, 0.0)
    obs = ObstacleSnap((ob,), 0.0, 0.0)
    _assert_identical(b1.build(js, obs, 0.0), b2.build(js, obs, 0.0))


def test_the_ssm_diagnostics_read_their_off_state():
    b, _ = _snapshot(0.25, **LOUD)
    assert b.diag_ssm_sp == 0.0
    assert b.diag_ssm_cap == float('inf')
