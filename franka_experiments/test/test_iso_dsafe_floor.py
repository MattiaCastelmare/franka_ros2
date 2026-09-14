"""The ISO d_safe floor, the S_h parameterisation and the PFL ceiling.

``cbf_safety_filter._iso_configure`` is the ONE place the ``iso_enabled`` flag
changes anything, so it is the one place worth pinning:

* with the flag off it must not touch a single value — that is the whole
  "behaviour is bit-identical with the flags off" ground rule, in one assertion;
* with the flag on it must RAISE, not clamp, when ``d_safe`` is below
  ``C + Z_d + Z_r`` or when ``link_speed_max`` is above ``iso_v_pfl``. A
  ceiling that silently moves when it cannot be met reads like a guarantee and
  is not one;
* with the SHIPPED configuration (C = 0.85 m, d_safe = 0.10 m) the raise MUST
  fire. That is not a bug to be fixed later: it is the finding that conformant
  SSM is unachievable in this cell, and a test that ever goes green here
  without d_safe having moved means the check was weakened.

The method is called unbound against a stub ``self``: it reads nothing off the
node but the logger, so there is no reason to stand up a ROS node for it.
"""

import os
import types

import pytest
import yaml

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter

CONFIG = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                      '..', 'config', 'fr3_control.yaml')


class _Log:
    def info(self, *a, **k):  pass
    def warn(self, *a, **k):  pass
    def error(self, *a, **k): pass


class _Stub:
    def get_logger(self):
        return _Log()


def _P(**over):
    """A parameter namespace that PASSES every check, so each test breaks one."""
    p = types.SimpleNamespace(
        iso_enabled=True, iso_mode='automatic',
        iso_c_intrusion=0.10, iso_z_depth=0.06, iso_z_robot=0.01,
        iso_t_reaction=0.10, iso_v_human=2.0, iso_a_stop=4.0,
        iso_v_pfl=0.68, iso_ssm_speed_rows=False, iso_monitor_enabled=False,
        d_safe=0.20, link_speed_max=0.60, retreat_cap_max_speed=0.50,
        enable_velocity_standoff=False,
        velocity_standoff_time_s=0.20, velocity_standoff_max=0.20,
        config_path='<test>')
    for k, v in over.items():
        setattr(p, k, v)
    return p


def _run(P):
    CBFSafetyFilter._iso_configure(_Stub(), P)


# ── flag off: nothing moves ──────────────────────────────────────────────────

def test_with_the_flag_off_not_one_value_is_touched():
    P = _P(iso_enabled=False, d_safe=0.001, link_speed_max=99.0,
           retreat_cap_max_speed=99.0)
    before = dict(vars(P))
    _run(P)
    assert dict(vars(P)) == before


# ── the d_safe floor ─────────────────────────────────────────────────────────

def test_d_safe_below_c_plus_z_raises_and_names_all_three_terms():
    P = _P(d_safe=0.10, iso_c_intrusion=0.85, iso_z_depth=0.06, iso_z_robot=0.01)
    with pytest.raises(ValueError) as exc:
        _run(P)
    msg = str(exc.value)
    assert '0.100' in msg and '0.850' in msg and '0.060' in msg and '0.010' in msg
    assert '0.920' in msg                       # the floor itself
    for resolution in ('(a)', '(b)', '(c)'):
        assert resolution in msg
    assert 'no bypass flag' in msg.lower()


def test_d_safe_exactly_at_the_floor_is_accepted():
    _run(_P(d_safe=0.17, iso_c_intrusion=0.10, iso_z_depth=0.06, iso_z_robot=0.01))


def test_the_floor_moves_with_c():
    # Lowering C by demonstrating a detection capability is resolution (a), and
    # it is the only knob that makes d_safe = 0.10 admissible at all.
    _run(_P(d_safe=0.10, iso_c_intrusion=0.03, iso_z_depth=0.06, iso_z_robot=0.01))
    with pytest.raises(ValueError):
        _run(_P(d_safe=0.10, iso_c_intrusion=0.04, iso_z_depth=0.06, iso_z_robot=0.01))


def test_the_shipped_configuration_refuses_to_run_with_iso_enabled():
    """The finding, as an assertion: this cell cannot do conformant SSM.

    If this test starts failing, either d_safe was raised to >= C+Z_d+Z_r (and
    SAFETY.md's deviation register needs updating), or C was lowered without a
    detection-capability measurement (which is not allowed — see Step 1).
    """
    with open(CONFIG) as fh:
        cfg = yaml.safe_load(fh)['params']
    P = _P(**{k: cfg[k] for k in (
        'd_safe', 'iso_c_intrusion', 'iso_z_depth', 'iso_z_robot',
        'iso_t_reaction', 'iso_v_human', 'iso_a_stop', 'iso_v_pfl',
        'link_speed_max', 'retreat_cap_max_speed')})
    assert cfg['iso_enabled'] is False, 'iso_enabled must ship FALSE'
    with pytest.raises(ValueError, match='Annex L'):
        _run(P)


# ── S_h through the velocity standoff ────────────────────────────────────────

def test_s_h_is_carried_by_the_velocity_standoff():
    P = _P(iso_t_reaction=0.10, iso_v_human=2.0, iso_a_stop=4.0)
    _run(P)
    assert P.enable_velocity_standoff is True
    # T_r + v_h/a_s = 0.10 + 0.5 = 0.60 s ; v_h * that = 1.20 m
    assert P.velocity_standoff_time_s == pytest.approx(0.60)
    assert P.velocity_standoff_max == pytest.approx(1.20)


def test_s_h_is_clamped_to_the_declared_ranges_not_silently_widened():
    # T_r + v_h/a_s = 0.1 + 2.0/1.0 = 2.1 s, v_h*that = 4.2 m: both past the
    # CBF_PARAM_SPEC ranges (5.0 s / 2.0 m), so only the metres clamp bites.
    P = _P(iso_t_reaction=0.10, iso_v_human=2.0, iso_a_stop=1.0)
    _run(P)
    assert P.velocity_standoff_time_s == pytest.approx(2.1)
    assert P.velocity_standoff_max == pytest.approx(2.0)


# ── the PFL ceiling ──────────────────────────────────────────────────────────

def test_link_speed_above_v_pfl_raises_rather_than_clamping():
    P = _P(link_speed_max=1.3, iso_v_pfl=0.68)
    with pytest.raises(ValueError, match='Annex M'):
        _run(P)
    assert P.link_speed_max == 1.3, 'it must RAISE, not clamp'


def test_link_speed_exactly_at_v_pfl_is_accepted():
    _run(_P(link_speed_max=0.68, iso_v_pfl=0.68, retreat_cap_max_speed=0.60))


def test_the_retreat_cap_must_stay_strictly_below_the_speed_row():
    with pytest.raises(ValueError, match='retreat_cap_max_speed'):
        _run(_P(link_speed_max=0.60, retreat_cap_max_speed=0.60))
    with pytest.raises(ValueError, match='retreat_cap_max_speed'):
        _run(_P(link_speed_max=0.60, retreat_cap_max_speed=0.65))
