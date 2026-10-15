"""e1 batch (2026-10-15): scene-agnostic SimBa policies, several obstacles, shield in the loop.

Why a new lineage instead of another d-batch fine-tune
------------------------------------------------------
The d2 12-net ensemble is 795/800 clean alone and 750 behind the shield, but every
member sees the world as ONE sphere (24-D obs: obstacle centre + d_min). On the robot
that slot is the nearest reported point of whatever the camera sees; a second person,
a table edge, or the obstacle wrapping around a link are invisible to the policy and
left to the shield. The goal of this batch is a policy that avoids EVERYTHING in the
scene, static or moving, proactively — and the shield stays as the certificate.

Three changes, each with a reason it was not done before:

1. Observation = 51-D (obs.obstacle_velocity + obs.control_point_geometry): per link the
   surface distance and the obstacle→link normal, i.e. the quantities the CBF rows are
   built from and that the robot's MultiLinkDistance already reports (rl_policy.py
   control_point_geometry). Count- and shape-agnostic. v10_obs51 / b3_obs51 "failed to
   learn" on raw, unnormalised inputs with a 256x256 MLP (HANDOFF §5).
2. Network = SimBa (Lee et al., ICLR 2025, arXiv 2410.09754): running-statistics obs
   normalisation + residual pre-LayerNorm blocks + AdamW weight decay. Fixes the input
   scaling problem of (1) at the root and is the published way to make SAC benefit from
   a bigger critic. Our ensembles beat every single net by 150-350/800 — a symptom of
   per-net approximation error, exactly what a regularised, wider critic attacks.
3. Scene = 1-3 spheres per episode (obstacle.count_range), each static or moving
   independently; the primary one keeps blocking_fraction 0.6 + the IK check, extras
   never block the target (franka_cbf_env._sample_extras).

Training follows CBF-RL (Yang, Werner, de Sa, Ames, ICRA 2026): shield in the loop,
bounded correction cost reward.w_correction (d3 lever), constraints as terminations,
d1 randomisation (latency 60±20 ms, noise, dynamics), from scratch (no 51-D parent
exists). From-scratch WITH the shield is the recipe that produced sac_v10, the parent
of everything; from scratch without it never learned to reach (b3).

Arms (seed = run id; 3 envs per run, UTD 1, 8M steps):
  a1 a2  SimBa (critic 2x1024, actor 1x256) + w_correction 0.5
  a3     SimBa, w_correction 0 (shield only — does the correction cost help or hurt from scratch?)
  m1     MLP 256x256 control on the SAME obs/scene/reward (isolates the network change)

    cd franka_sim && python3 runs/eval_all/e1/mk_e1.py      → config_e1_*.yaml
Eval: runs/eval_all/e1/eval_e1.sh (single-sphere held-out protocol = comparable with d1/d2/d3,
plus the 2-3-sphere scenes).
"""
import copy
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
base = yaml.safe_load(open(os.path.join(SIM, 'config_d3_d_b2.yaml')))

SIMBA = dict(actor_hidden=256, actor_blocks=1, critic_hidden=1024, critic_blocks=2,
             expansion=4, weight_decay=1e-2)

# name: (seed, policy, w_correction, note)
RUNS = {
    'a1': (401, 'simba', 0.5, 'SimBa 2x1024 critic, 51-D obs, 1-3 spheres, shield + w_correction 0.5'),
    'a2': (402, 'simba', 0.5, 'SimBa 2x1024 critic, 51-D obs, 1-3 spheres, shield + w_correction 0.5'),
    'a3': (403, 'simba', 0.0, 'SimBa 2x1024 critic, 51-D obs, 1-3 spheres, shield only'),
    'm1': (411, 'mlp', 0.5, 'MLP 256x256 control, 51-D obs, 1-3 spheres, shield + w_correction 0.5'),
}

N_ENVS = 3
# Replay ratio. Measured 2026-10-15 (3 runs sharing the GPU through MPS, 3 envs each, UTD 2):
# SimBa 2x1024 56 fps, SimBa 2x512 63 fps, MLP 100 fps — launch-bound (4x the FLOPs cost 12 %),
# so the gradient steps, not the simulation, set the wall-clock. UTD 1 → ~2x the env throughput.
UTD = 1


def make(name, seed, policy, w_corr, total=8_000_000, n_envs=N_ENVS):
    c = copy.deepcopy(base)
    c['env']['shield'] = True
    c['env']['cbf_obstacle_enabled'] = True
    c['env'].pop('cbf_obstacle_on_prob', None)
    c['obs'] = {'obstacle_velocity': True, 'control_point_geometry': True, 'clip_distance': 1.2}
    c['obstacle']['count_range'] = [1, 3]
    c['task']['target_ik_check'] = True
    c['reward']['w_correction'] = float(w_corr)
    c['reward']['correction_sigma'] = 5.0
    rl = c['rl']
    rl.update(dict(
        algo='SAC', total_timesteps=int(total), n_envs=int(n_envs), seed=int(seed),
        learning_rate=1.0e-4, buffer_size=1_000_000, batch_size=512, gamma=0.99, tau=0.005,
        train_freq=1, gradient_steps=int(UTD * n_envs), learning_starts=10_000,
        ent_coef='auto_0.01', target_entropy=-3.5,        # SimBa: alpha0 1e-2, |A|/2
        device='cuda', tensorboard_log='runs/', save_path='models/',
        checkpoint_freq=500_000, checkpoint_freq_episodes=2000, eval_freq=100_000, tf32=True,
    ))
    if policy == 'simba':
        rl['policy'] = 'SimbaPolicy'
        rl['simba'] = dict(SIMBA)
        rl.pop('net_arch', None)
    else:
        rl['policy'] = 'MlpPolicy'
        rl['net_arch'] = [256, 256]
        rl.pop('simba', None)
    return c


if __name__ == '__main__':
    for name, (seed, policy, w_corr, note) in RUNS.items():
        c = make(name, seed, policy, w_corr)
        p = os.path.join(SIM, f'config_e1_{name}.yaml')
        with open(p, 'w') as f:
            f.write(f'# e1_{name} — generated by runs/eval_all/e1/mk_e1.py from config_d3_d_b2.yaml (2026-10-15).\n'
                    f'# {note}, seed {seed}, from scratch\n')
            yaml.safe_dump(c, f, sort_keys=False)
        print(p)
