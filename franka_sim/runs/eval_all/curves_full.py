"""All tfevents scalars of the three sac_v10 runs (+ v10c) → curves_full.json, downsampled to <=400 points per tag."""
import json, glob
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
R = '/ros2_ws/src/franka_sim/runs/'
out = {}
for run, d in [('sac_v10', 'sac_v10_1'), ('sac_v10_obs51', 'sac_v10_obs51_1'), ('sac_v10_g997', 'sac_v10_g997_1'), ('sac_v10c', 'sac_v10c_0')]:
    ea = EventAccumulator(glob.glob(R + d + '/events*')[0], size_guidance={'scalars': 0}); ea.Reload()
    out[run] = {}
    for tag in ea.Tags()['scalars']:
        ev = ea.Scalars(tag); k = max(1, len(ev) // 400)
        out[run][tag] = [[e.step, round(e.value, 5)] for e in ev[::k]]
json.dump(out, open(R + 'eval_all/curves_full.json', 'w'), separators=(',', ':'))
print({r: sorted(v) for r, v in out.items()}['sac_v10'])
