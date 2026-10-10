"""Is the target reachable with every control point at d >= d_safe (static obstacle)?  Constrained IK, multi-start SLSQP.
   SEEDS='3001,3015' python3 feasibility.py MODE   (MODE static|sinusoidal; static: obstacle fixed, so this is exact up to IK)
Prints per seed: min reachable EE-target error under the hard constraints (obstacle d >= d_safe, floor, joint limits)."""
import os, sys, json, yaml, numpy as np, mujoco
from scipy.optimize import minimize
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
c = yaml.safe_load(open('franka_sim/models/sac_v10c/config.yaml')); c.setdefault('task', {})['blocking_fraction'] = 0.6
c['obstacle']['mode'] = sys.argv[1]; c['obstacle']['static_fraction'] = 0
env = FrankaCBFEnv(config=c); m, d = env.model, env.data
ds = float(c['cbf']['d_safe']); fm = 0.03
lo, hi = env.q_min, env.q_max
def kin(q):
    d.qpos[env._qadr] = q; mujoco.mj_kinematics(m, d)
    return d.site_xpos[env._ee_site].copy(), np.array([d.xpos[b] for b in env._cp_body])
rng = np.random.default_rng(0)
for s in map(int, os.environ['SEEDS'].split(',')):
    env.reset(seed=s); tg = env._target.copy(); ob = d.mocap_pos[env._obs_mocap].copy(); q0 = d.qpos[env._qadr].copy()
    def f(q): e, _ = kin(q); return float(np.sum((e - tg) ** 2))
    def g(q):
        _, P = kin(q)
        return np.concatenate([np.linalg.norm(P - ob, axis=1) - env.r_obs - env._cp_radius - ds,   # obstacle
                               P[:, 2] - env._cp_radius - fm])                                     # floor
    best = (9, None)
    starts = [q0] + [rng.uniform(lo, hi) for _ in range(40)]
    for qs in starts:
        r = minimize(f, qs, method='SLSQP', bounds=list(zip(lo + 0.05, hi - 0.05)),
                     constraints=[{'type': 'ineq', 'fun': g}], options=dict(maxiter=300, ftol=1e-10))
        if r.success or g(r.x).min() > -1e-4:
            err = np.sqrt(f(r.x))
            if g(r.x).min() > -1e-4 and err < best[0]: best = (err, r.x)
    e, P = kin(best[1]) if best[1] is not None else (None, None)
    # which control point binds at the best solution
    bind = '' if P is None else env._cp_name[int(np.argmin(np.linalg.norm(P - ob, axis=1) - env.r_obs - env._cp_radius))]
    ee_hand = np.linalg.norm(d.site_xpos[env._ee_site] - d.xpos[env._cp_body[-1]])
    print(json.dumps(dict(seed=s, otd=round(float(np.linalg.norm(tg - ob)), 3), tgt=np.round(tg, 2).tolist(),
                          min_err=round(float(best[0]), 3), feasible=bool(best[0] < env.target_tol), bind=bind,
                          ee_to_handcp=round(float(ee_hand), 3))), flush=True)
