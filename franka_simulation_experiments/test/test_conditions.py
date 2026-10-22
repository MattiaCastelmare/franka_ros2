"""Every experiment toggle is explicit per run (ground rule 3, plan H12)."""
import os

import pytest
import yaml

from simexp import conditions as C

REPO = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..')
LAUNCH_DEFAULTS = os.path.join(REPO, 'franka_experiments', 'config', 'launch_defaults.yaml')


def test_the_four_spec_conditions_exist_and_say_what_the_spec_says():
    r = {n: C.resolve(n)['toggles'] for n in ('residual', 'kf', 'imm', 'imm+pred')}
    assert (r['residual']['obstacle_tracking'], r['residual']['obstacle_velocity_source']) == (False, 'residual')
    for n in ('kf', 'imm', 'imm+pred'):
        assert (r[n]['obstacle_tracking'], r[n]['obstacle_velocity_source']) == (True, 'tracker')
    assert [r[n]['imm_enabled'] for n in ('kf', 'imm', 'imm+pred')] == [False, True, True]
    assert [r[n]['prediction_enabled'] for n in ('kf', 'imm', 'imm+pred')] == [False, False, True]
    assert r['residual']['tracking_enabled'] is False and r['kf']['tracking_enabled'] is True


def test_every_cbf_flag_and_both_tracking_switches_are_passed_to_the_stack():
    a = C.resolve('kf')['stack_args']
    for k in C.CBF_ARGS + C.TRACK_ARGS:
        assert k in a, k
        assert isinstance(a[k], str)


def test_the_cbf_block_mirrors_the_hardware_launch_defaults():
    """The Gazebo default is what a bare hardware launch runs; a change on
    either side must be a deliberate decision, not drift."""
    cfg = C.load()
    with open(LAUNCH_DEFAULTS) as f:
        ld = yaml.safe_load(f)
    for k, v in cfg['cbf'].items():
        assert ld[k] == v, f'{k}: conditions.yaml {v!r} vs launch_defaults.yaml {ld[k]!r}'


def test_overrides_reach_flags_and_perception_and_unknowns_are_refused():
    r = C.resolve('imm', {'vobs_in_hdot': False, 'prediction': True})
    assert r['stack_args']['vobs_in_hdot'] == 'false'
    assert r['perception']['tracking']['prediction']['enabled'] is True
    with pytest.raises(KeyError):
        C.resolve('kf', {'not_a_flag': 1})
    with pytest.raises(KeyError):
        C.resolve('kalman')
    with pytest.raises(ValueError, match='required'):
        C.resolve('')


def test_tracker_source_without_tracking_is_refused():
    with pytest.raises(ValueError):
        C.resolve('residual', {'obstacle_velocity_source': 'tracker'})
