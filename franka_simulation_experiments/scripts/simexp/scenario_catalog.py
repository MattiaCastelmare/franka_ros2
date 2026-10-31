"""The scenario catalog (config/scenarios.yaml), resolved into what a Gazebo run needs. No rclpy.

``resolve(name, seed)`` returns:

* ``obstacles``: absolute specs for ScenarioObstacles / gz_world (name, radius,
  law, p0, v0, speed, gravity, park);
* ``nominal``: the scenario commander's law (joint_pd / cartesian) or
  ``pentagon`` (the shipped commander runs instead);
* ``T``, ``scene``, ``motion``;
* ``perception`` / ``depth_adapter``: the Gazebo counterparts of
  cbf_scenarios' perception emulation (cam_noise -> range noise, max_thresh);
* ``start_offset_s``: the seeded start phase within one camera frame period.
  It is 0 for seed 0, so seed 0 is the 1:1 port. It is the only randomness
  unless a scenario declares ``jitter_m``.

Episode discipline (franka_rl _sample_episode): ``check_start`` refuses a start
that is inside the barrier, measured on TRUE control-point gaps at Q0.
"""
from __future__ import annotations

import copy
import os

import numpy as np
import yaml

GRAVITY = (0.0, 0.0, -9.81)


def default_path() -> str:
    here = os.path.dirname(os.path.realpath(__file__))
    cand = os.path.normpath(os.path.join(here, '..', '..', 'config', 'scenarios.yaml'))
    if os.path.isfile(cand):
        return cand
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('franka_simulation_experiments'),
                        'config', 'scenarios.yaml')


def load(path: str | None = None) -> dict:
    with open(path or default_path()) as f:
        return yaml.safe_load(f)


def _fmt_v(v: float) -> str:
    return f'{v:g}'


def throw_names(cfg: dict) -> list[str]:
    ts = cfg['throw_sweep']
    out = []
    for v in ts['speeds']:
        out.append(f'throw_a_v{_fmt_v(v)}')
        out.append(f'throw_b_v{_fmt_v(v)}')
        for L in ts['kinds']['c']['L']:
            out.append(f'throw_c{_fmt_v(L)}_v{_fmt_v(v)}')
    out += [f'throw_ballistic_v{_fmt_v(v)}' for v in cfg['ballistic_throws']['speeds']]
    return out


def all_names(cfg: dict | None = None) -> list[str]:
    cfg = cfg or load()
    return (list(cfg['catalog']) + list(cfg['pentagon']['scenarios']) + throw_names(cfg)
            + list(cfg.get('tests', {})))


def _throw(cfg: dict, name: str) -> dict:
    flange = np.asarray(cfg['flange_q0'], float)
    y = np.array([0.0, 1.0, 0.0])
    if name.startswith('throw_ballistic_v'):
        bt = cfg['ballistic_throws']
        v = float(name[len('throw_ballistic_v'):])
        d = float(bt['release_distance'])
        u = np.asarray(bt['direction'], float)
        g = np.asarray(bt['gravity'], float)
        Tf = d / v
        p0 = flange + d * u
        v0 = (flange - p0) / Tf - 0.5 * g * Tf
        return dict(T=min(Tf + 0.5, 6.0), nominal={'law': 'joint_pd'},
                    obstacles=[dict(p0=p0.tolist(), radius=bt['radius'], law='ballistic',
                                    v0=v0.tolist(), gravity=g.tolist())],
                    source='ballistic_throws')
    ts = cfg['throw_sweep']
    body = name[len('throw_'):]
    kind, vs = body.split('_v')
    v = float(vs)
    R = float(ts['radius'])
    if kind == 'a':
        d0, tsep = float(ts['kinds']['a']['d0']), 0.0
    elif kind == 'b':
        d0, tsep = float(ts['gate']) + R + float(ts['r_cap']), 0.0
    else:
        d0, tsep = float(kind[1:]), float(ts['t_sep'])
    d0 = max(d0 - v * tsep, 0.3)
    T = min(d0 / v + 0.5, 6.0)
    return dict(T=T, nominal={'law': 'joint_pd'},
                obstacles=[dict(p0=(flange + d0 * y).tolist(), radius=R, law='linear',
                                v0=(-v * y).tolist())],
                source='throw_sweep')


def resolve(name: str, seed: int = 0, cfg: dict | None = None, depth_rate_hz: float = 90.0) -> dict:
    cfg = cfg or load()
    flange = np.asarray(cfg['flange_q0'], float)
    q0 = list(cfg['q0'])
    motion = 'scenario'
    if name in cfg['catalog']:
        sc = copy.deepcopy(cfg['catalog'][name])
        src = 'cbf_scenarios'
    elif name in cfg['pentagon']['scenarios']:
        sc = copy.deepcopy(cfg['pentagon']['scenarios'][name])
        sc.setdefault('nominal', {'law': 'pentagon'})
        sc.setdefault('scene', 'rig')        # the task as run on the rig, table included
        motion, src = 'pentagon', 'pentagon'
    elif name in cfg.get('tests', {}):
        sc = copy.deepcopy(cfg['tests'][name])
        src = 'tests'
    elif name.startswith('throw_'):
        sc = _throw(cfg, name)
        src = sc.pop('source')
    else:
        raise KeyError(f'unknown scenario {name!r}')
    rng = np.random.default_rng(seed)
    jitter = float(sc.get('jitter_m', 0.0))
    obstacles = []
    for k, o in enumerate(sc.get('obstacles', [])):
        p0 = (flange + np.asarray(o['offset'], float)) if 'offset' in o else np.asarray(o['p0'], float)
        if jitter > 0 and seed != 0:
            p0 = p0 + rng.uniform(-jitter, jitter, 3)
        spec = dict(name=f'obstacle_{k}', radius=float(o['radius']), law=o.get('law', 'static'),
                    p0=p0.tolist(), v0=list(o.get('v0', (0.0, 0.0, 0.0))),
                    speed=float(o.get('speed', 0.0)), gravity=list(o.get('gravity', GRAVITY)),
                    # oscillate's parameters, carried with the same defaults
                    # gz_world.scenario_obstacles_plugin writes into the SDF.
                    # Dropping them here would silently turn an oscillating
                    # obstacle into one sitting still at p0.
                    axis=list(o.get('axis', (1.0, 0.0, 0.0))),
                    amplitude=float(o.get('amplitude', 0.0)),
                    period=float(o.get('period', 0.0)),
                    park=bool(o.get('park', True)))
        obstacles.append(spec)
    nominal = dict(sc.get('nominal', {'law': 'joint_pd'}))
    if nominal.get('law') == 'joint_pd':
        nominal.setdefault('goal_q', q0)
    gz = sc.get('gazebo', {})
    start_offset = 0.0 if seed == 0 else float(rng.uniform(0.0, 1.0 / depth_rate_hz))
    return dict(name=name, source=src, motion=motion, T=float(sc['T']), q0=q0,
                scene=sc.get('scene', 'bare'), obstacles=obstacles, nominal=nominal,
                emulation=sc.get('emulation', {}), perception=gz.get('perception', {}),
                depth_adapter=gz.get('depth_adapter', {}), seed=int(seed),
                start_offset_s=start_offset, jitter_m=jitter)


def check_start(resolved: dict, cps: list, d_safe: float) -> dict:
    """True gap of every control point to every obstacle at p0; refuse h < 0 at start."""
    from .truth_geometry import surface_gaps
    obs = resolved['obstacles']
    if not obs:
        return {'ok': True, 'min_gap': None}
    g = surface_gaps(cps, [o['p0'] for o in obs], [o['radius'] for o in obs])
    return {'ok': bool(g.min() >= d_safe), 'min_gap': float(g.min()), 'd_safe': d_safe}


def enforce_start_clearance(resolved: dict, cps: list, d_safe: float,
                            margin: float = 0.001) -> dict:
    """Push any obstacle that starts inside the barrier just outside it.

    The catalog positions are cbf_scenarios', whose emulated control points
    (CP_SEGS, R_CAP) are not the real perception set
    (``define_control_points``: the fingertip CP, per-segment radii). With the
    real set, ``wedge`` starts 9 mm inside d_safe, on fr3_link8#0. franka_rl's
    rule is to never start inside the barrier (_sample_episode), so the
    offending obstacle moves radially away from its nearest control point by
    ``d_safe - gap + margin``. The move is recorded in ``start_shift``; every
    other obstacle is untouched.
    """
    from .truth_geometry import surface_gaps
    out = copy.deepcopy(resolved)
    out['start_shift'] = []
    P = np.array([c['point'] for c in cps])
    for o in out['obstacles']:
        g = surface_gaps(cps, [o['p0']], [o['radius']])[:, 0]
        i = int(np.argmin(g))
        if g[i] >= d_safe:
            continue
        n = np.asarray(o['p0'], float) - P[i]
        n /= np.linalg.norm(n)
        delta = (d_safe - g[i] + margin) * n
        o['p0'] = (np.asarray(o['p0'], float) + delta).tolist()
        out['start_shift'].append(dict(obstacle=o['name'], cp=f"{cps[i]['end_link']}#{cps[i]['cp_idx']}",
                                       gap_before=float(g[i]), shift_m=float(np.linalg.norm(delta)),
                                       direction=n.tolist()))
    return out
