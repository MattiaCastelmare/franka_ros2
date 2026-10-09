"""Parallel-training capacity probe (2026-10-05): SAC with the b3 config's rl section, device from argv.
    bench_b3.py CONFIG DEVICE TAG  -> prints fps over steps 1500..4500 and peak RSS (MB)."""
import sys, time, resource, yaml, torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
cfgp, dev, tag = sys.argv[1:4]
torch.set_num_threads(1)
c = yaml.safe_load(open(cfgp)); rl = c['rl']
env = FrankaCBFEnv(config=c)
m = SAC('MlpPolicy', env, batch_size=int(rl['batch_size']), learning_starts=1000, buffer_size=int(rl['buffer_size']),
        gamma=rl['gamma'], device=dev, policy_kwargs=dict(net_arch=rl['net_arch']), verbose=0)
class T(BaseCallback):
    def _on_step(self):
        if self.num_timesteps == 1500: self.t0 = time.time()
        return True
cb = T(); m.learn(4500, callback=cb)
print(tag, dev, 'fps %.1f' % (3000 / (time.time() - cb.t0)),
      'rss_MB %d' % (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss // 1024), flush=True)
