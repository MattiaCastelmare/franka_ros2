"""Print rollout/eval curves of runs from their tfevents, binned per 500k steps.
    python3 runs/eval_all/e1/tb_curves.py runs/sac_e1_a1_1 runs/sac_e1_m1_1 ..."""
import sys
import numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

TAGS = ['rollout/success_rate', 'eval/success_rate', 'train/ent_coef', 'train/critic_loss',
        'rollout/ep_len_mean', 'safety/intervention_rate', 'safety/collision_rate']
BIN = float(sys.argv[-1]) if sys.argv[-1].replace('.', '').isdigit() else 500_000
for d in [a for a in sys.argv[1:] if not a.replace('.', '').isdigit()]:
    ea = EventAccumulator(d, size_guidance={'scalars': 0}); ea.Reload()
    have = ea.Tags()['scalars']
    print(f'== {d}  ({len(have)} tags)')
    cols = [t for t in TAGS if t in have]
    extra = [t for t in have if t.startswith('safety/')][:6]
    cols += [t for t in extra if t not in cols]
    data = {t: np.array([(e.step, e.value) for e in ea.Scalars(t)]) for t in cols}
    smax = max(v[:, 0].max() for v in data.values())
    print('step_M  ' + ' '.join(f'{t.split("/")[-1][:12]:>12}' for t in cols))
    for b in np.arange(0, smax + 1, BIN):
        row = []
        for t in cols:
            v = data[t]; m = (v[:, 0] >= b) & (v[:, 0] < b + BIN)
            row.append(f'{v[m, 1].mean():12.3f}' if m.any() else ' ' * 12)
        print(f'{b / 1e6:6.2f}  ' + ' '.join(row))
