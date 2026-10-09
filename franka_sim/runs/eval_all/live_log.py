"""Live training log read from tfevents + evaluations.npz (for runs whose stdout log went stale).

    docker exec -it franka_ros2 python3 /ros2_ws/src/franka_sim/runs/eval_all/live_log.py [run] [total_steps]
Defaults: run sac_v10c (tfevents runs/sac_v10c_0, evals models/sac_v10c), total 4_000_000. Refresh 20 s, Ctrl-C to quit.
"""
import glob
import os
import sys
import time

import numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

RUN = sys.argv[1] if len(sys.argv) > 1 else 'sac_v10c'
TOTAL = int(sys.argv[2]) if len(sys.argv) > 2 else 4_000_000
ROOT = '/ros2_ws/src/franka_sim/'
EV_DIR = sorted(p for p in glob.glob(f'{ROOT}runs/{RUN}_*') if os.path.isdir(p))[-1]
NPZ = f'{ROOT}models/{RUN}/evaluations.npz'
TXT = f'{ROOT}runs/{RUN}_train.log'
GROUPS = ['rollout', 'safety', 'time', 'train']
B, D, G, Y, R0 = '\033[1m', '\033[2m', '\033[32m', '\033[33m', '\033[0m'


def table(ea):
    rows, step = [], 0
    for grp in GROUPS:
        tags = sorted(t for t in ea.Tags()['scalars'] if t.startswith(grp + '/'))
        if not tags:
            continue
        rows.append((f'{grp}/', None))
        for t in tags:
            ev = ea.Scalars(t)[-1]
            step = max(step, ev.step)
            rows.append(('    ' + t.split('/', 1)[1], ev.value))
    return rows, step


def fmt(v):
    if v is None:
        return ''
    return f'{v:.3g}' if abs(v) < 1e4 else f'{v:.4g}'


def main():
    ea = EventAccumulator(glob.glob(EV_DIR + '/events*')[0], size_guidance={'scalars': 0})
    while True:
        ea.Reload()
        rows, step = table(ea)
        fps_ev = ea.Scalars('time/fps') if 'time/fps' in ea.Tags()['scalars'] else []
        fps = fps_ev[-1].value if fps_ev else float('nan')
        if sys.stdout.isatty():
            print('\033[2J\033[H', end='')
        txt_age = (time.time() - os.path.getmtime(TXT)) / 60 if os.path.exists(TXT) else float('nan')
        print(f'{B}{RUN}{R0}  {time.strftime("%H:%M:%S")}   source: {os.path.basename(EV_DIR)}/tfevents'
              f'  {D}(text log {os.path.basename(TXT)} last written {txt_age:.0f} min ago){R0}')
        frac = min(1.0, step / TOTAL)
        eta_h = (TOTAL - step) / fps / 3600 if fps == fps and fps > 0 else float('nan')
        bar = '█' * int(40 * frac) + '░' * (40 - int(40 * frac))
        print(f'{bar} {step:,}/{TOTAL:,} ({100 * frac:.1f}%)  fps {fps:.0f}  ETA {eta_h:.1f} h\n')
        print('-' * 42)
        for k, v in rows:
            print(f'| {k:<24}| {fmt(v):<12}|')
        print('-' * 42)
        if os.path.exists(NPZ):
            d = np.load(NPZ)
            t, r, s = d['timesteps'], d['results'].mean(1), d['successes'].mean(1)
            print(f'\n{B}EvalCallback{R0} (10 eps, held-at-end success)   best {s.max():.1f} @ {t[s.argmax()]:,}'
                  f'   mean last 10 {s[-10:].mean():.2f}')
            for i in range(max(0, len(t) - 10), len(t)):
                c = G if s[i] >= 0.7 else Y if s[i] >= 0.5 else ''
                print(f'  {t[i]:>9,}  reward {r[i]:8.1f}   {c}success {s[i]:.1f}{R0}  {"▇" * int(s[i] * 10)}')
        if os.environ.get('ONCE'):
            break
        time.sleep(20)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
