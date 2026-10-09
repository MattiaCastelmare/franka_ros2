"""PPO trainer for the policy-alone setting (d1, 2026-10-09).

Many parallel envs (SubprocVecEnv), constraints as terminations from the env
config (constraints.terminate), reward normalised by VecNormalize (rewards
only: the observation the policy sees stays the raw one the robot builds, so
the checkpoint exports and deploys like a SAC one).

    python3 -u -m franka_sim.train_ppo --config franka_sim/config_d1_ppo.yaml --exp-name ppo_d1
"""
from __future__ import annotations

import argparse
import os
import shutil

import torch
import yaml
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor, VecNormalize

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.train import SafetyMetricsCallback

_HERE = os.path.dirname(os.path.abspath(__file__))


def make_env(cfg, seed):
    def _init():
        env = FrankaCBFEnv(config=cfg)
        env.reset(seed=seed)
        return env
    return _init


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--exp-name', required=True)
    ap.add_argument('--total-timesteps', type=int, default=None)
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    rl = cfg['rl']
    seed, n = int(rl.get('seed', 0)), int(rl.get('n_envs', 12))
    model_dir = os.path.join(_HERE, rl.get('save_path', 'models/'), a.exp_name)
    os.makedirs(os.path.join(model_dir, 'checkpoints'), exist_ok=True)
    shutil.copy(a.config, os.path.join(model_dir, 'config.yaml'))
    torch.set_num_threads(2)

    env = SubprocVecEnv([make_env(cfg, seed + i) for i in range(n)], start_method='spawn')
    env = VecMonitor(env, info_keywords=('is_success', 'collision'))
    env = VecNormalize(env, norm_obs=False, norm_reward=True, gamma=float(rl.get('gamma', 0.99)))

    model = PPO(
        'MlpPolicy', env,
        learning_rate=float(rl.get('learning_rate', 3e-4)), n_steps=int(rl.get('n_steps', 256)),
        batch_size=int(rl.get('batch_size', 3072)), n_epochs=int(rl.get('n_epochs', 5)),
        gamma=float(rl.get('gamma', 0.99)), gae_lambda=float(rl.get('gae_lambda', 0.95)),
        clip_range=float(rl.get('clip_range', 0.2)), ent_coef=float(rl.get('ent_coef', 0.0)),
        vf_coef=float(rl.get('vf_coef', 0.5)), max_grad_norm=float(rl.get('max_grad_norm', 0.5)),
        policy_kwargs=dict(net_arch=dict(pi=list(rl.get('net_arch', [256, 256])),
                                         vf=list(rl.get('net_arch', [256, 256]))),
                           log_std_init=float(rl.get('log_std_init', -0.5))),
        tensorboard_log=os.path.join(_HERE, rl.get('tensorboard_log', 'runs/')),
        device=rl.get('device', 'cuda'), seed=seed, verbose=1)

    every = int(rl.get('checkpoint_every_steps', 5_000_000))
    ckpt = CheckpointCallback(save_freq=max(1, every // n), save_path=os.path.join(model_dir, 'checkpoints'),
                              name_prefix='ppo', save_vecnormalize=True)
    total = a.total_timesteps or int(rl.get('total_timesteps', 100_000_000))
    print(f'PPO {a.exp_name}: n_envs={n} total={total} device={model.device}', flush=True)
    model.learn(total_timesteps=total, callback=[ckpt, SafetyMetricsCallback(log_freq=20000)],
                tb_log_name=a.exp_name, progress_bar=False)
    model.save(os.path.join(model_dir, 'final_model'))
    env.save(os.path.join(model_dir, 'vecnormalize.pkl'))


if __name__ == '__main__':
    main()
