"""SAC throughput probe: env steps + 1 gradient step each, like train.py (UTD 1).

    python3 -m franka_rl.scripts.bench_sac_throughput --config franka_rl/config_d1_a_s1.yaml --steps 3000

Prints one line `fps <steps/s>` measured after learning_starts. Run several copies in
parallel (with or without an MPS daemon) to see how many SAC runs the GPU can carry.
"""
import argparse
import time

import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback

from franka_rl.envs.franka_cbf_env import FrankaCBFEnv, _load_config


class _Clock(BaseCallback):
    def __init__(self, start_at):
        super().__init__()
        self.start_at, self.t0, self.n0 = start_at, None, None

    def _on_step(self):
        if self.t0 is None and self.num_timesteps >= self.start_at:
            torch.cuda.synchronize()
            self.t0, self.n0 = time.perf_counter(), self.num_timesteps
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--steps', type=int, default=3000)
    ap.add_argument('--batch-size', type=int, default=None)
    ap.add_argument('--device', default='cuda')
    a = ap.parse_args()
    cfg = _load_config(a.config)
    rl = cfg['rl']
    warm = 1000
    model = SAC('MlpPolicy', FrankaCBFEnv(cfg), learning_rate=rl['learning_rate'],
                buffer_size=50_000, batch_size=a.batch_size or rl['batch_size'],
                learning_starts=warm, gamma=rl['gamma'], tau=rl['tau'],
                policy_kwargs=dict(net_arch=rl['net_arch']), device=a.device, verbose=0)
    clk = _Clock(warm + 200)
    model.learn(total_timesteps=warm + 200 + a.steps, callback=clk)
    torch.cuda.synchronize()
    dt = time.perf_counter() - clk.t0
    print(f'fps {(model.num_timesteps - clk.n0) / dt:.1f}', flush=True)


if __name__ == '__main__':
    main()
