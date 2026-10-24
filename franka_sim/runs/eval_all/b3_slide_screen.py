"""Slide video (2026-10-06): pick "obstacle in the way" episodes of increasing difficulty, OUTCOME-BLIND.
Geometry as runs/videos/presentation/screen_blocking.py (hand_clearance: how deep the hand would cut into the
obstacle on the straight EE->target route; negative = the direct route collides). Pool 6000-6999, blocking_fraction 1,
10 s episodes. Prints the distribution and 5 seeds per mode at evenly spaced clearance (shallow -> deepest).
    python3 b3_slide_screen.py OUT.json
"""
import copy, json, sys
import numpy as np, yaml
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
POOL, R_HAND, MIN_LEN, EPISODE_S = range(6000, 7000), 0.13, 0.30, 10.0
CFGS = {'ft': 'franka_sim/models/sac_b3_ft_v10c/config.yaml', 'ens': 'franka_sim/models/sac_v10c/config.yaml'}
def make_env(cfgp, mode):
    c = copy.deepcopy(yaml.safe_load(open(cfgp)))
    c['env']['max_episode_steps'] = int(round(EPISODE_S * float(c['env'].get('control_rate_hz', 100.0))))
    c['env']['cbf_obstacle_enabled'] = False
    c['reward']['terminate_on_success'] = False; c['reward']['terminate_on_collision'] = True
    c['task']['blocking_fraction'] = 1.0; c['obstacle']['mode'] = mode; c['obstacle']['static_fraction'] = 0
    return FrankaCBFEnv(config=c)
def line_dist(a, b, p, lo, hi):
    ab = b - a; t = np.clip((p - a) @ ab / (ab @ ab), lo, hi); return float(np.linalg.norm(p - (a + t * ab)))
def hand_clearance(env):
    a, b = env._ee_pos().copy(), env._target.copy()
    if env.obs_mode == 'static':
        d = line_dist(a, b, env._obstacle_at(env._obs_phase), 0.2, 0.9)
    else:
        dphi = 2 * np.pi * env.obs_speed * env.dt
        d = min(line_dist(a, b, env._obstacle_at(env._obs_phase + k * dphi), 0.3, 1.0) for k in range(0, int(EPISODE_S / env.dt), 5))
    return d - env.r_obs - R_HAND, float(np.linalg.norm(b - a))
out = {}
for mode in ('static', 'sinusoidal'):
    E = {k: make_env(p, mode) for k, p in CFGS.items()}
    geo = []
    for s in POOL:
        o = {}
        for k, env in E.items():
            env.reset(seed=s); o[k] = (env._ee_pos().copy(), env._target.copy(), env._obstacle_at(env._obs_phase).copy())
        assert all(np.allclose(x, y) for x, y in zip(o['ft'], o['ens'])), ('configs differ', mode, s)
        c, L = hand_clearance(E['ft'])
        if L >= MIN_LEN and c < 0: geo.append((round(c, 4), s, round(L, 3)))
    geo.sort(reverse=True)   # shallow -> deep
    cs = np.array([g[0] for g in geo])
    goals = np.linspace(cs[0] - 0.01 if cs[0] > -0.01 else cs[0], cs[-1], 5)
    pick = []
    for g in goals:
        i = int(np.argmin([abs(x - g) if geo[j][1] not in [p[1] for p in pick] else 9 for j, x in enumerate(cs)]))
        pick.append(geo[i])
    print(mode, 'blocking seeds', len(geo), 'clearance range', cs[0], cs[-1], 'quartiles', np.percentile(cs, [25, 50, 75]).round(3))
    print('  pick (clearance m, seed, route length m):', pick)
    out[mode] = dict(n=len(geo), pick=pick)
json.dump(out, open(sys.argv[1], 'w'), indent=1)
