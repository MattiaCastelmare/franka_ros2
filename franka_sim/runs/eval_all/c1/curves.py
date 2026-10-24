"""Training curves (rollout/success_rate = last-step held over the last 100 stochastic episodes, ent_coef) in 100k bins."""
import glob, sys, numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
runs = sys.argv[1:]
for r in runs:
    ds = sorted(glob.glob(f'franka_sim/runs/{r}_*'))
    vals = {}
    for d in ds:
        ea = EventAccumulator(d, size_guidance={'scalars': 0}); ea.Reload()
        for tag in ('rollout/success_rate', 'train/ent_coef', 'rollout/ep_rew_mean'):
            if tag in ea.Tags()['scalars']:
                vals.setdefault(tag, []).extend((e.step, e.value) for e in ea.Scalars(tag))
    out = []
    for tag in ('rollout/success_rate', 'train/ent_coef'):
        v = np.array(sorted(vals.get(tag, [])))
        if not len(v): continue
        bins = np.arange(2_500_000, 4_000_001, 100_000)
        row = [np.mean(v[(v[:, 0] >= a) & (v[:, 0] < a + 100_000), 1]) if ((v[:, 0] >= a) & (v[:, 0] < a + 100_000)).any() else np.nan for a in bins[:-1]]
        out.append(f'{tag.split("/")[1][:7]:7s} ' + ' '.join(f'{x:5.3f}' if np.isfinite(x) else '  -  ' for x in row))
    print(f'{r:22s}', ' | '.join(out))
