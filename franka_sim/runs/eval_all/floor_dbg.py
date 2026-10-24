import yaml, numpy as np, mujoco
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _load_policy
c = yaml.safe_load(open('franka_sim/runs/eval_all/sac_v4_hard.yaml')); c['reward']['terminate_on_success'] = False
c['env']['max_episode_steps'] = 1000; c['cbf']['floor_enable'] = True
env = FrankaCBFEnv(config=c); pred, _ = _load_policy('franka_sim/models/sac_v4/final_model.zip'); m = env.model
orig = env.cbf.filter; last = {}
def spy(*a, **k):
    r = orig(*a, **k); last['info'] = r[1]; return r
env.cbf.filter = spy
o, _ = env.reset(seed=12352); n = 0
while True:
    o, _, te, tr, info = env.step(pred(o)); n += 1
    fl = [(round(z - r, 3)) for z, r, *_ in env._floor_pts]
    if info['floor_contact'] or (min(fl) < 0.02):
        cons = [(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, env.data.contact[i].geom1), mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, env.data.contact[i].geom2), round(env.data.contact[i].dist, 4)) for i in range(env.data.ncon)]
        I = last['info']
        print(n, 'floor h (z-r) per pt', fl, 'solved', I.solved, 'braking', I.braking, 'n_c', I.n_c, 'contacts', cons)
    if te or tr: break
