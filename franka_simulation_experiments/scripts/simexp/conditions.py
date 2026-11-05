"""Resolve a condition (+ per-run overrides) into explicit launch arguments (ground rule 3).

``resolve('kf', {'vobs_in_hdot': False})`` returns everything a run needs:

* ``stack_args``: launch arguments for torque_control_stack.launch.py, every
  CBF flag and both perception switches spelled out as strings;
* ``perception``: the nested keys merged into the run's fr3_complete copy
  (tracking.enabled / imm_enabled / prediction.enabled);
* ``toggles``: the flat record of all of them, for run_manifest.json.

Nothing is left to a default: a key missing from conditions.yaml is an error.

TWO PER-CONDITION BLOCKS, both optional, for the ablation arms
--------------------------------------------------------------
The global ``cbf:`` block is pinned to launch_defaults.yaml (test_conditions.py),
because the Gazebo default must be what a bare hardware launch runs. An arm that
is DEFINED by running different flags therefore cannot express itself there:

* ``cbf:`` inside a condition overrides the global block key by key. Only keys
  the global block already has — a typo must not silently invent a flag.
* ``cbf_params:`` inside a condition carries fr3_control.yaml keys that have no
  launch argument at all (``enable_go_around``, ``retreat_cap_enabled``,
  ``state_governor_enabled``, the v_obs conditioning). They are emitted as the
  single ``cbf_param_overrides`` launch argument, which the stack hands to
  cbf_safety_filter as ordinary ROS parameters: the installed YAML is never
  edited, so the next run is unaffected. cbf_safety_filter's own
  ``declare_from_spec`` type-checks and bounds-checks every key and refuses to
  start on one it does not know.

Precedence, lowest to highest: global ``cbf:`` < condition ``cbf:`` < ``--set``.
Both blocks land in ``toggles``, so run_manifest.json records the whole arm.
"""
from __future__ import annotations

import copy
import os

import yaml

#: torque_control_stack launch arguments that a condition must set.
CBF_ARGS = ('lateral_evasion', 'outrun_evasion', 'livelock_escape', 'latency_compensation',
            'zone_ladder', 'uncertainty_margin', 'vobs_in_hdot', 'velocity_standoff',
            'obstacle_velocity_normal_guard', 'obstacle_identity_guard', 'multi_obstacle_k',
            'iso_enabled', 'iso_mode', 'iso_ssm_speed_rows', 'iso_monitor_enabled')
TRACK_ARGS = ('obstacle_tracking', 'obstacle_velocity_source')


def default_path() -> str:
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.join(here, '..', '..', 'config', 'conditions.yaml'),):
        if os.path.isfile(cand):
            return os.path.normpath(cand)
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('franka_simulation_experiments'),
                        'config', 'conditions.yaml')


def load(path: str | None = None) -> dict:
    with open(path or default_path()) as f:
        return yaml.safe_load(f)


def _s(v) -> str:
    if isinstance(v, bool):
        return 'true' if v else 'false'
    return str(v)


def params_arg(params: dict) -> str:
    """``{'enable_go_around': False}`` -> ``'enable_go_around=false'``.

    The wire format of the ``cbf_param_overrides`` launch argument, which the
    stack parses back with ``yaml.safe_load`` per value. Keys are sorted so the
    string is stable, and therefore so is the cell name and the manifest.
    """
    return ';'.join(f'{k}={_s(params[k])}' for k in sorted(params))


def resolve(name: str, overrides: dict | None = None, path: str | None = None) -> dict:
    cfg = load(path)
    if not name:
        raise ValueError('condition:= is required (ground rule 3): one of '
                         + ', '.join(cfg['conditions']))
    if name not in cfg['conditions']:
        raise KeyError(f'unknown condition {name!r}; known: {sorted(cfg["conditions"])}')
    cond = cfg['conditions'][name]
    cbf = copy.deepcopy(cfg['cbf'])
    missing = [k for k in CBF_ARGS if k not in cbf] + [k for k in TRACK_ARGS if k not in cond]
    if missing:
        raise KeyError(f'conditions.yaml does not set {missing}')
    # The condition's own flag block, before --set, and only over keys the
    # global block defines: a misspelt flag here would otherwise be a silently
    # dead override that reads exactly like a live one.
    cond_cbf = copy.deepcopy(cond.get('cbf') or {})
    unknown_flags = sorted(k for k in cond_cbf if k not in cbf)
    if unknown_flags:
        raise KeyError(f'condition {name!r} overrides unknown cbf flags {unknown_flags}')
    cbf.update(cond_cbf)
    # YAML-only parameters: global block first, then the condition's.
    cbf_params = copy.deepcopy(cfg.get('cbf_params') or {})
    cbf_params.update(copy.deepcopy(cond.get('cbf_params') or {}))
    over = dict(overrides or {})
    for k in list(over):
        if k in cbf:
            cbf[k] = over.pop(k)
    # --set wins over the condition's cbf_params too, for a key already there.
    for k in list(over):
        if k in cbf_params:
            cbf_params[k] = over.pop(k)
    track = {k: over.pop(k, cond[k]) for k in TRACK_ARGS}
    perception = copy.deepcopy(cond['perception'])
    for key, path_ in (('imm_enabled', ('imm_enabled',)), ('prediction', ('prediction', 'enabled')),
                       ('tracking_enabled', ('enabled',))):
        if key in over:
            d = perception['tracking']
            for p in path_[:-1]:
                d = d.setdefault(p, {})
            d[path_[-1]] = bool(over.pop(key))
    if over:
        raise KeyError(f'unknown overrides {sorted(over)}')
    if track['obstacle_velocity_source'] == 'tracker' and not track['obstacle_tracking']:
        raise ValueError('obstacle_velocity_source:=tracker needs obstacle_tracking:=true')
    if bool(perception['tracking']['enabled']) != bool(track['obstacle_tracking']):
        raise ValueError('perception tracking.enabled must equal obstacle_tracking')
    stack_args = {k: _s(v) for k, v in {**cbf, **track}.items()}
    if cbf_params:
        stack_args['cbf_param_overrides'] = params_arg(cbf_params)
    toggles = dict(condition=name, **{k: v for k, v in {**cbf, **track}.items()},
                   tracking_enabled=perception['tracking']['enabled'],
                   imm_enabled=perception['tracking']['imm_enabled'],
                   prediction_enabled=perception['tracking']['prediction']['enabled'],
                   cbf_params=cbf_params,
                   cbf_yaml_only=list(cfg.get('cbf_yaml_only', [])))
    return dict(stack_args=stack_args, perception=perception, toggles=toggles,
                cbf_params=cbf_params, description=cond.get('description', ''))
