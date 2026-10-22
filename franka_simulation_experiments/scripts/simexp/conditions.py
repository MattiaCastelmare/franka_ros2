"""Resolve a condition (+ per-run overrides) into explicit launch arguments (ground rule 3).

``resolve('kf', {'vobs_in_hdot': False})`` returns everything a run needs:

* ``stack_args``: launch arguments for torque_control_stack.launch.py, every
  CBF flag and both perception switches spelled out as strings;
* ``perception``: the nested keys merged into the run's fr3_complete copy
  (tracking.enabled / imm_enabled / prediction.enabled);
* ``toggles``: the flat record of all of them, for run_manifest.json.

Nothing is left to a default: a key missing from conditions.yaml is an error.
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
    over = dict(overrides or {})
    for k in list(over):
        if k in cbf:
            cbf[k] = over.pop(k)
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
    toggles = dict(condition=name, **{k: v for k, v in {**cbf, **track}.items()},
                   tracking_enabled=perception['tracking']['enabled'],
                   imm_enabled=perception['tracking']['imm_enabled'],
                   prediction_enabled=perception['tracking']['prediction']['enabled'],
                   cbf_yaml_only=list(cfg.get('cbf_yaml_only', [])))
    return dict(stack_args=stack_args, perception=perception, toggles=toggles,
                description=cond.get('description', ''))
