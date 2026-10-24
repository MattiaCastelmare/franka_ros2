"""Training throughput probe: SAC on the v10 config, device/threads from argv. Prints steps/s after warm-up."""
import sys, time, yaml, torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
dev = sys.argv[1]; nth = int(sys.argv[2]); arch = [int(x) for x in sys.argv[3].split(',')] if len(sys.argv) > 3 else [256, 256]
torch.set_num_threads(nth)
c = yaml.safe_load(open('franka_sim/config_v10.yaml'))
env = FrankaCBFEnv(config=c)
m = SAC('MlpPolicy', env, batch_size=512, learning_starts=1000, device=dev, policy_kwargs=dict(net_arch=arch), verbose=0)
class T(BaseCallback):
    def _on_step(self):
        if self.num_timesteps == 1500: self.t0 = time.time()
        return True
cb = T(); m.learn(4500, callback=cb)
print(dev, nth, arch, 'fps %.1f' % (3000 / (time.time() - cb.t0)), flush=True)
