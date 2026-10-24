import yaml, numpy as np, mujoco
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _load_policy
c = yaml.safe_load(open('franka_sim/runs/eval_all/sac_v4_static.yaml')); c['reward']['terminate_on_success'] = False
c['cbf']['floor_enable'] = True; c['cbf']['soft_fallback'] = True
env = FrankaCBFEnv(config=c); pred, _ = _load_policy('franka_sim/models/sac_v4/final_model.zip'); m = env.model
print('floor_enable', env.cbf.floor_enable)
orig = env.cbf.filter; last = {}
def spy(*a, **k):
    r = orig(*a, **k); last['info'] = r[1]; return r
env.cbf.filter = spy
for s in range(1000, 1040):
    o, _ = env.reset(seed=s); n = 0; hits = []; fails = 0
    while True:
        o, _, te, tr, info = env.step(pred(o)); n += 1
        fails += not last['info'].solved
        if info['floor_contact']:
            cons = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) for i in range(env.data.ncon)
                    for g in (env.data.contact[i].geom1, env.data.contact[i].geom2) if g != env._floor_geom
                    and env._floor_geom in (env.data.contact[i].geom1, env.data.contact[i].geom2)}
            hits.append((n, last['info'].solved, tuple(cons), round(min(z - r for z, r, *_ in env._floor_pts), 3)))
        if te or tr: break
    if hits or fails: print('seed', s, 'qp fails', fails, 'floor hits', hits[:3], len(hits))
