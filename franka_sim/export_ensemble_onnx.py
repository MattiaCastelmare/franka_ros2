"""Export an ACTION ENSEMBLE of SAC policies to one ONNX graph.

    observation (N, obs_dim)  →  action (N, 7) = mean_k tanh(μ_k(obs))   ∈ [−1, 1]

Same input/output names and shapes as export_onnx.py, so rl_policy_commander
loads it unchanged. Averaging the members' deterministic actions removes most of
the per-policy inconsistency: on 400 held-out seeds the 3-run ensemble held
383/352 (moving/static) vs 335/302 for its best single member (2026-10-01).

Members must share the observation layout and every deploy-relevant config
section (obs, cbf, joint_limits, env, obstacle radius); they may differ in
reward/rl/seed. The first member's config.yaml is frozen next to the .onnx with
a header listing all members — that file is what the commander reads back.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src python3 -m franka_sim.export_ensemble_onnx \\
        --models franka_sim/models/A/checkpoints/x.zip franka_sim/models/B/... \\
        --out franka_sim/models/sac_v11_ens/ens_soup3.onnx
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import torch as th
import yaml

from stable_baselines3 import SAC

# Sections that change what the policy sees or what the shield does on the robot.
_DEPLOY_SECTIONS = ('obs', 'cbf', 'joint_limits', 'env', 'actuation')


class OnnxableEnsemble(th.nn.Module):
    def __init__(self, policies):
        super().__init__()
        self.actors = th.nn.ModuleList([p.actor for p in policies])

    def forward(self, observation: th.Tensor) -> th.Tensor:
        acts = [a(observation, deterministic=True) for a in self.actors]
        return th.stack(acts, 0).mean(0)


def _frozen_config(zip_path: str) -> str:
    d = os.path.dirname(os.path.abspath(zip_path))
    for cand in (d, os.path.dirname(d)):
        p = os.path.join(cand, 'config.yaml')
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(f'no config.yaml next to {zip_path}')


def export_ensemble(model_paths, out: str, opset: int = 17, verbose: bool = True) -> str:
    cfg_paths = [_frozen_config(m) for m in model_paths]
    cfgs = [yaml.safe_load(open(p)) for p in cfg_paths]
    for p, c in zip(cfg_paths[1:], cfgs[1:]):
        for sec in _DEPLOY_SECTIONS:
            if c.get(sec) != cfgs[0].get(sec):
                raise ValueError(f'config section "{sec}" of {p} differs from {cfg_paths[0]}')
        if c.get('obstacle', {}).get('radius') != cfgs[0].get('obstacle', {}).get('radius'):
            raise ValueError(f'obstacle.radius of {p} differs from {cfg_paths[0]}')

    models = [SAC.load(m, device='cpu') for m in model_paths]
    obs_dim = int(np.prod(models[0].observation_space.shape))
    act_dim = int(np.prod(models[0].action_space.shape))
    for m, path in zip(models, model_paths):
        assert int(np.prod(m.observation_space.shape)) == obs_dim, f'{path}: obs dim mismatch'
        assert int(np.prod(m.action_space.shape)) == act_dim, f'{path}: act dim mismatch'
    if verbose:
        print(f'{len(models)} members, obs_dim={obs_dim}, act_dim={act_dim}')

    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    net = OnnxableEnsemble([m.policy for m in models]).eval()
    kw = dict(input_names=['observation'], output_names=['action'],
              dynamic_axes={'observation': {0: 'batch'}, 'action': {0: 'batch'}},
              opset_version=opset)
    try:
        th.onnx.export(net, th.zeros(1, obs_dim), out, dynamo=False, **kw)
    except TypeError:
        th.onnx.export(net, th.zeros(1, obs_dim), out, **kw)

    # Freeze the shared config next to the graph (what rl_policy_commander reads).
    cfg_out = os.path.join(os.path.dirname(os.path.abspath(out)), 'config.yaml')
    with open(cfg_out, 'w') as f:
        f.write('# Action ensemble exported by franka_sim/export_ensemble_onnx.py\n'
                f'# graph: {os.path.basename(out)} = mean of the deterministic actions of:\n')
        for m in model_paths:
            f.write(f'#   {m}\n')
        f.write(f'# Config below = first member ({cfg_paths[0]}); deploy sections identical across members.\n')
        yaml.safe_dump(cfgs[0], f, sort_keys=False)

    # Validate: onnxruntime vs the mean of the SB3 deterministic predictions.
    import onnxruntime as ort
    sess = ort.InferenceSession(out, providers=['CPUExecutionProvider'])
    max_err = 0.0
    for _ in range(200):
        obs = models[0].observation_space.sample().astype(np.float32)
        a_onnx = sess.run(None, {'observation': obs[None]})[0][0]
        a_ref = np.mean([m.predict(obs, deterministic=True)[0] for m in models], axis=0)
        max_err = max(max_err, float(np.max(np.abs(a_onnx - a_ref))))
    if verbose:
        print(f'exported → {out}\nconfig   → {cfg_out}\n'
              f'validation: max |onnx − mean(sb3)| over 200 obs = {max_err:.2e}')
    assert max_err < 1e-4, 'ensemble ONNX diverges from the SB3 members!'
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--models', nargs='+', required=True, help='SAC .zip members')
    ap.add_argument('--out', required=True, help='.onnx path (config.yaml is written next to it)')
    ap.add_argument('--opset', type=int, default=17)
    a = ap.parse_args()
    export_ensemble(a.models, a.out, a.opset)


if __name__ == '__main__':
    main()
