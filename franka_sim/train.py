"""Train a Safe-RL policy (SAC) on FrankaCBF with the CBF shield in the loop.

Stable-Baselines3 SAC on CUDA (RTX 4070), TensorBoard convergence + safety
curves, periodic checkpoints and a best-model eval callback. The policy learns
the TASK; the CBF filter inside the env certifies SAFETY every step, so this is
safe exploration by construction.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
        python3 -m franka_sim.train                 # full run (config.yaml)
        python3 -m franka_sim.train --total-timesteps 5000 --exp-name smoke

Outputs (under franka_sim/):
    runs/<exp>/           TensorBoard logs
    models/<exp>/best_model.zip     best eval model  (→ export_onnx.py)
    models/<exp>/checkpoints/       periodic snapshots
    models/<exp>/final_model.zip    end-of-run model
    models/<exp>/config.yaml        frozen config for reproducibility
"""

from __future__ import annotations

import argparse
import os
import shutil
from datetime import datetime

import numpy as np
import yaml

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    BaseCallback, CheckpointCallback, EvalCallback,
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CONFIG = os.path.join(_HERE, 'config.yaml')


def make_env(config: dict, seed: int, render_mode=None):
    def _init():
        env = FrankaCBFEnv(config=config, render_mode=render_mode)
        env = Monitor(env, info_keywords=('is_success', 'collision'))
        env.reset(seed=seed)
        return env
    return _init


class SafetyMetricsCallback(BaseCallback):
    """Log CBF/safety scalars to TensorBoard: the paper's safe-exploration curves.

    Accumulates per-step signals from the env `info` dict and writes rolling
    means every `log_freq` steps: collision rate, min surface distance, CBF
    intervention magnitude, slack, and fraction of steps with the shield active.
    """

    def __init__(self, log_freq: int = 1000, verbose: int = 0):
        super().__init__(verbose)
        self.log_freq = log_freq
        self._reset_buffers()

    def _reset_buffers(self):
        self._d_min, self._interv, self._slack = [], [], []
        self._active, self._collisions, self._n = 0, 0, 0

    def _on_step(self) -> bool:
        for info in self.locals.get('infos', []):
            if 'd_min' not in info:
                continue
            self._n += 1
            self._d_min.append(info['d_min'])
            self._interv.append(info['cbf_intervention'])
            self._slack.append(info['cbf_slack'])
            self._active += int(info['cbf_n_c'] > 0)
            self._collisions += int(info.get('collision', False))
        if self._n >= self.log_freq:
            self.logger.record('safety/collision_rate', self._collisions / self._n)
            self.logger.record('safety/min_surface_dist', float(np.min(self._d_min)))
            self.logger.record('safety/mean_surface_dist', float(np.mean(self._d_min)))
            self.logger.record('safety/cbf_active_frac', self._active / self._n)
            self.logger.record('safety/mean_intervention', float(np.mean(self._interv)))
            self.logger.record('safety/mean_slack', float(np.mean(self._slack)))
            self._reset_buffers()
        return True


class EpisodeCheckpointCallback(BaseCallback):
    """Snapshot the model every `every` COMPLETED EPISODES.

    SB3's own CheckpointCallback counts environment STEPS, which is the wrong
    axis for watching a policy come up: episodes end early on success and on
    collision, so the number of steps per episode changes by 5x over a run and
    step-spaced snapshots land at wildly different amounts of task experience.
    Episode-spaced ones are directly comparable, and they are what
    `compare_checkpoints.py` plots against.

    Episodes are counted from the Monitor wrapper's `episode` key rather than
    from `dones`, so a truncated episode counts exactly once and a vectorised
    env with several sub-envs still totals correctly.

    Each snapshot is also exported to ONNX when `export` is set, because the
    ONNX graph — not the .zip — is what the robot runs; a checkpoint you cannot
    hand to `rl_policy_commander` is a checkpoint you cannot actually test.
    """

    def __init__(self, every: int, save_path: str, export: bool = True,
                 name_prefix: str = 'sac_ep', verbose: int = 1):
        super().__init__(verbose)
        self.every = int(every)
        self.save_path = save_path
        self.export = export
        self.name_prefix = name_prefix
        self.n_episodes = 0
        self._next_at = self.every

    def _on_step(self) -> bool:
        for info in self.locals.get('infos', []):
            if 'episode' in info:
                self.n_episodes += 1
        self.logger.record('time/episodes_completed', self.n_episodes)

        if self.every > 0 and self.n_episodes >= self._next_at:
            self._next_at += self.every
            base = os.path.join(self.save_path,
                                f'{self.name_prefix}{self.n_episodes:06d}')
            self.model.save(base)
            if self.verbose:
                print(f'\n[episode-ckpt] {self.n_episodes} episodes '
                      f'({self.num_timesteps} steps) -> {base}.zip')
            if self.export:
                self._export_onnx(base + '.zip')
        return True

    def _export_onnx(self, zip_path: str):
        """Best-effort ONNX export; a failure must never kill a training run."""
        try:
            from franka_sim.export_onnx import export
            export(zip_path, zip_path.replace('.zip', '.onnx'), verbose=False)
        except Exception as exc:                      # noqa: BLE001
            if self.verbose:
                print(f'[episode-ckpt] ONNX export skipped: {exc}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=_DEFAULT_CONFIG)
    ap.add_argument('--exp-name', default=None, help='run name (default: timestamp)')
    ap.add_argument('--total-timesteps', type=int, default=None)
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--device', default=None, help='cuda | cpu (default: config)')
    ap.add_argument('--resume', default=None, help='path to a .zip to continue')
    ap.add_argument('--checkpoint-every-episodes', type=int, default=None,
                    help='snapshot every N completed episodes (0 disables); '
                         'default: rl.checkpoint_freq_episodes in the config')
    ap.add_argument('--no-episode-onnx', action='store_true',
                    help='skip the ONNX export of each episode checkpoint')
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    rl = cfg['rl']

    seed   = args.seed if args.seed is not None else int(rl.get('seed', 0))
    device = args.device or rl.get('device', 'cuda')
    total  = args.total_timesteps or int(rl.get('total_timesteps', 2_000_000))
    exp    = args.exp_name or datetime.now().strftime('sac_%Y%m%d_%H%M%S')

    tb_dir    = os.path.join(_HERE, rl.get('tensorboard_log', 'runs'))
    model_dir = os.path.join(_HERE, rl.get('save_path', 'models'), exp)
    ckpt_dir  = os.path.join(model_dir, 'checkpoints')
    os.makedirs(ckpt_dir, exist_ok=True)
    shutil.copy(args.config, os.path.join(model_dir, 'config.yaml'))

    # ── Envs (train + eval share the config; eval env is deterministic-ish) ──
    train_env = DummyVecEnv([make_env(cfg, seed)])
    eval_env  = DummyVecEnv([make_env(cfg, seed + 1000)])

    policy_kwargs = dict(net_arch=list(rl.get('net_arch', [256, 256])))
    ent_coef = rl.get('ent_coef', 'auto')

    if args.resume:
        print(f'Resuming from {args.resume}')
        model = SAC.load(args.resume, env=train_env, device=device,
                         tensorboard_log=tb_dir)
    else:
        model = SAC(
            rl.get('policy', 'MlpPolicy'), train_env,
            learning_rate=float(rl.get('learning_rate', 3e-4)),
            buffer_size=int(rl.get('buffer_size', 1_000_000)),
            batch_size=int(rl.get('batch_size', 512)),
            gamma=float(rl.get('gamma', 0.99)),
            tau=float(rl.get('tau', 0.005)),
            train_freq=int(rl.get('train_freq', 1)),
            gradient_steps=int(rl.get('gradient_steps', 1)),
            learning_starts=int(rl.get('learning_starts', 10_000)),
            ent_coef=ent_coef,
            policy_kwargs=policy_kwargs,
            device=device, seed=seed, verbose=1, tensorboard_log=tb_dir,
        )

    print(f'device={model.device}  total_timesteps={total}  exp={exp}')

    ep_every = (args.checkpoint_every_episodes
                if args.checkpoint_every_episodes is not None
                else int(rl.get('checkpoint_freq_episodes', 0)))

    callbacks = [
        CheckpointCallback(
            save_freq=int(rl.get('checkpoint_freq', 50_000)),
            save_path=ckpt_dir, name_prefix='sac'),
        EvalCallback(
            eval_env, best_model_save_path=model_dir,
            log_path=model_dir, eval_freq=int(rl.get('eval_freq', 25_000)),
            n_eval_episodes=10, deterministic=True, render=False),
        SafetyMetricsCallback(log_freq=2000),
    ]
    if ep_every > 0:
        callbacks.append(EpisodeCheckpointCallback(
            every=ep_every, save_path=ckpt_dir,
            export=not args.no_episode_onnx))
        print(f'episode checkpoints: every {ep_every} episodes → {ckpt_dir}')

    model.learn(total_timesteps=total, callback=callbacks, tb_log_name=exp,
                progress_bar=True, reset_num_timesteps=not bool(args.resume))
    model.save(os.path.join(model_dir, 'final_model'))
    print(f'\nDone. Models in {model_dir}\n'
          f'  export:  python3 -m franka_sim.export_onnx '
          f'--model {os.path.join(model_dir, "best_model.zip")}')


if __name__ == '__main__':
    main()
