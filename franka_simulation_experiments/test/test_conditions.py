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


def test_the_ssb_arms_differ_in_exactly_one_key():
    """minimal_l1 -> _ssb_obs -> _ssb is a two-step ladder, one knob per step.

    Ground rule 3 (nothing implicit) applied to an ablation: if the observe-only
    arm differed from the acting arm in anything but ``ssb_rows_enabled``, then
    "acting on the shadow changed the motion" would not be attributable, and the
    whole point of carrying three arms would be lost.
    """
    base = C.resolve('minimal_l1')
    obs = C.resolve('minimal_l1_ssb_obs')
    act = C.resolve('minimal_l1_ssb')

    # step 1: the PERCEPTION half, and nothing in the QP
    assert obs['stack_args'] == {**base['stack_args'],
                                 'cbf_param_overrides':
                                     obs['stack_args']['cbf_param_overrides']}
    assert (obs['perception']['distance'] or {}).get('ssb_enabled') is True
    assert 'distance' not in base['perception']
    ssb_keys = {k for k in obs['cbf_params'] if k.startswith('ssb_')
                or k == 'rho_slack_ssb'}
    assert ssb_keys and all(k not in base['cbf_params'] for k in ssb_keys)
    assert obs['cbf_params']['ssb_rows_enabled'] is False
    # every NON-ssb parameter is minimal_l1's, unchanged
    assert {k: v for k, v in obs['cbf_params'].items() if k not in ssb_keys} \
        == base['cbf_params']

    # step 2: the CONSTRAINT half, and nothing else at all
    assert act['perception'] == obs['perception']
    diff = {k for k in set(act['cbf_params']) | set(obs['cbf_params'])
            if act['cbf_params'].get(k) != obs['cbf_params'].get(k)}
    assert diff == {'ssb_rows_enabled'}
    assert act['cbf_params']['ssb_rows_enabled'] is True


def test_the_ssb_arms_log_fast_enough_to_resolve_the_band_crossing():
    """ssb_log_throttle_s must be short against the ssb_poke crossing.

    The ball is inside the deleted band for 0.75 s; the shipped 0.5 s throttle
    would sample two frames of it, which is not enough to claim anything
    time-resolved. This is the arm's measurement instrument, so it is pinned.
    """
    for name in ('minimal_l1_ssb_obs', 'minimal_l1_ssb'):
        d = C.resolve(name)['perception']['distance']
        assert d['ssb_log_throttle_s'] <= 0.10, name


def _flat(d, pre=''):
    """Resolved condition flattened to ``dotted.key -> value``, for diffing."""
    out = {}
    for k, v in (d or {}).items():
        if isinstance(v, dict):
            out.update(_flat(v, pre + k + '.'))
        else:
            out[pre + k] = v
    return out


def test_each_ssb_sensitivity_arm_moves_exactly_one_parameter():
    """The sensitivity arms have to stay attributable, like the ablation ladder.

    Each of them clones ``minimal_l1_ssb`` and changes ONE perception key. If a
    second key came along for the ride, the sweep would measure the pair and the
    sensitivity number would be meaningless. ``description`` and the ``toggles``
    echo of the condition name are excluded -- they are labels, not behaviour.
    """
    base = _flat(C.resolve('minimal_l1_ssb'))
    expect = {
        'minimal_l1_ssb_guard12': ('perception.distance.ssb_guard_m', 0.12),
        'minimal_l1_ssb_tol08': ('perception.mask.depth_gate_tol_m', 0.08),
        'minimal_l1_ssb_tol20': ('perception.mask.depth_gate_tol_m', 0.20),
        'minimal_l1_ssb_dil8': ('perception.mask.ee_mask_dilate_px', 8),
    }
    for name, (key, value) in expect.items():
        arm = _flat(C.resolve(name))
        diff = {k for k in set(base) | set(arm)
                if base.get(k, '<none>') != arm.get(k, '<none>')
                and k != 'description' and not k.startswith('toggles.condition')}
        assert diff == {key}, (name, sorted(diff))
        assert arm[key] == value, name


def test_the_empty_band_arm_really_has_an_empty_band():
    """``minimal_l1_ssb_guard12`` is the falsification arm, so check the premise.

    A candidate needs ``bias + guard < z_model - z_near < bias + tol``, so the
    band is non-empty exactly when ``guard < tol`` -- the bias cancels. The arm
    is only a falsification test if that inequality really fails in the config
    it resolves to, so it is asserted here instead of being asserted about in
    prose. (Getting this inequality wrong is how an earlier draft mislabelled
    the tol-0.08 arm, whose band is 4 cm deep, as empty.)
    """
    import pathlib

    import yaml
    pc = yaml.safe_load((pathlib.Path(__file__).resolve().parents[2]
                         / 'franka_experiments' / 'config'
                         / 'fr3_complete.yaml').read_text())

    def band(name):
        a = C.resolve(name)
        g = float((a['perception'].get('distance') or {}).get(
            'ssb_guard_m', pc['distance']['ssb_guard_m']))
        t = float((a['perception'].get('mask') or {}).get(
            'depth_gate_tol_m', pc['mask']['depth_gate_tol_m']))
        return g, t

    g, t = band('minimal_l1_ssb_guard12')
    assert g >= t, f'band is not empty: guard {g} < tol {t}'
    # the arm it is compared against must have a NON-empty band, or the
    # comparison is between two inert configurations
    g, t = band('minimal_l1_ssb')
    assert g < t, f'reference band is empty: guard {g} >= tol {t}'
    # ... and so must each depth-tolerance arm, which is why neither is the
    # falsification arm
    for name in ('minimal_l1_ssb_tol08', 'minimal_l1_ssb_tol20'):
        g, t = band(name)
        assert g < t, (name, g, t)
