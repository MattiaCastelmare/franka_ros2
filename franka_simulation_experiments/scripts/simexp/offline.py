"""Column 1 of the three-column comparison: cbf_scenarios.py offline (spec Phase 5.3).

The double integrator with emulated perception, run by cbf_scenarios' own
``scenarios()`` and ``summarize()``. Only the flags are set here, so the
offline filter runs with the same toggles as the Gazebo runs:

* ``obstacle_velocity_source`` from the condition (residual / tracker). The
  offline emulation has no tracker, so kf, imm and imm+pred are the same
  offline run;
* every CBF flag of conditions.yaml ``cbf:`` that exists as a filter
  parameter (``enable_<flag>`` or ``<flag>``), and the two guards the stack
  maps onto thresholds (normal 0.15 rad, identity 0.10 m). cbf_scenarios' shipped_params
  takes only five of them from launch_defaults and the rest from
  fr3_control.yaml, which would leave e.g. vobs_in_hdot and velocity_standoff
  different from the Gazebo stack.

``fps`` is cbf_scenarios' own argument; 30 is what it ships with (and what
config/scenarios.yaml records), 90 is the rate the simulated chain runs at.
"""
from __future__ import annotations

import contextlib
import io

from .emulation import _cbf_scenarios


#: torque_control_stack maps these launch bools onto thresholds (its _cbf_params):
#: on = the value derived in fr3_control.yaml, off = 0.
GUARD_THRESHOLDS = {'obstacle_velocity_normal_guard': ('obstacle_velocity_normal_rot_max', 0.15),
                    'obstacle_identity_guard': ('obstacle_velocity_identity_jump', 0.10)}
#: perception-side (real_time_distance) or not a filter parameter: no offline meaning
NOT_OFFLINE = ('obstacle_tracking', 'obstacle_velocity_source', 'multi_obstacle_k')


def overrides(condition_resolved: dict, P0) -> dict:
    from .conditions import CBF_ARGS
    tg = condition_resolved['toggles']
    over = {'obstacle_velocity_source': tg['obstacle_velocity_source']}
    for k in CBF_ARGS:
        v = tg[k]
        if k.startswith('iso_') or k in NOT_OFFLINE:
            continue
        if k in GUARD_THRESHOLDS:
            name, val = GUARD_THRESHOLDS[k]
            over[name] = val if v else 0.0
            continue
        for name in (f'enable_{k}', k):
            if hasattr(P0, name):
                over[name] = v if not isinstance(getattr(P0, name), bool) else bool(v)
                break
        else:
            raise KeyError(f'CBF flag {k} has no offline parameter (enable_{k} / {k})')
    return over


def run(names: list[str], condition: str, *, fps: float = 30.0, sets: dict | None = None) -> dict:
    """{scenario: summarize() dict, plus 'flags'} for the cbf_scenarios catalog names."""
    from . import conditions
    C = _cbf_scenarios()
    cond = conditions.resolve(condition, sets)
    over = overrides(cond, C.shipped_params())
    P = C.shipped_params(**over)
    with contextlib.redirect_stdout(io.StringIO()):
        res = list(C.scenarios(P, only=list(names), fps=fps))
    return {r['name']: dict(r, flags=over, fps=fps) for r in res}
