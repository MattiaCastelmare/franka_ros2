"""Count robot-floor contacts per episode (seeds 12345..12354, as in the video)."""
import sys, yaml, numpy as np, mujoco
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _load_policy
cfgp, model = sys.argv[1], sys.argv[2]
c = yaml.safe_load(open(cfgp)); c['reward']['terminate_on_success'] = False
c['env']['max_episode_steps'] = 1000
for kv in sys.argv[5:]:
    k, v = kv.split('='); sec, key = k.split('.'); c.setdefault(sec, {})[key] = yaml.safe_load(v)
env = FrankaCBFEnv(config=c); pred, _ = _load_policy(model)
m = env.model
floor = [g for g in range(m.ngeom) if m.geom_bodyid[g] == 0 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE]
print('floor geoms', [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g in floor])
robot_bodies = [b for b in range(m.nbody) if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) and
                mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b).startswith(('fr3', 'left', 'right', 'hand'))]
tot = 0
for s in range(int(sys.argv[3]), int(sys.argv[3]) + int(sys.argv[4])):
    o, _ = env.reset(seed=s); hits = {}; zmin = 9; steps = 0
    while True:
        o, _, te, tr, info = env.step(pred(o)); steps += 1
        for i in range(env.data.ncon):
            con = env.data.contact[i]
            for a, b in ((con.geom1, con.geom2), (con.geom2, con.geom1)):
                if a in floor:
                    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[b])
                    hits[name] = hits.get(name, 0) + 1
        zmin = min(zmin, min(env.data.xpos[b][2] for b in robot_bodies if b != robot_bodies[0]))
        if te or tr: break
    tot += bool(hits)
    print(f'seed {s}: floor contact steps {sum(hits.values()):4d} {hits}  min body-origin z {zmin:.3f}  success {info["success"]} err {info["dist"]:.3f}', flush=True)
print('episodes touching the floor:', tot)
