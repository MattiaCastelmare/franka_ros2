"""Live dashboard for the sac_v10 runs: progress, ETA and the latest SB3 metrics.

    python3 franka_sim/runs/eval_all/train_dashboard.py [run ...]   (refresh 15 s, Ctrl-C to quit)
"""
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.normpath(os.path.join(HERE, '..'))
RUNS = sys.argv[1:] or ['sac_v10', 'sac_v10_obs51', 'sac_v10_g997']
TOTAL = 2_000_000
KEYS = ['total_timesteps', 'fps', 'episodes', 'ep_len_mean', 'ep_rew_mean',
        'success_rate', 'collision_rate', 'min_surface_dist', 'ent_coef', 'critic_loss']
GREEN, YELLOW, RED, BOLD, DIM, RESET = ('\033[32m', '\033[33m', '\033[31m', '\033[1m',
                                        '\033[2m', '\033[0m')


def last_table(path):
    """Metrics from the last complete SB3 table in the log (read from the tail)."""
    try:
        with open(path, 'rb') as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 60_000))
            text = f.read().decode(errors='ignore')
    except OSError:
        return {}, ''
    vals = {}
    for k in KEYS:
        m = re.findall(rf'\|\s+{k}\s+\|\s+([-\d.e+]+)', text)
        if m:
            vals[k] = float(m[-1])
    err = next((ln for ln in text.splitlines()[::-1]
                if 'Traceback' in ln or 'Error' in ln), '')
    return vals, err


def alive(run):
    out = subprocess.run(['pgrep', '-f', '--', f'--exp-name {run} '], capture_output=True, text=True)
    out2 = subprocess.run(['pgrep', '-f', '--', f'--exp-name {run}$'], capture_output=True, text=True)
    return bool(out.stdout.strip() or out2.stdout.strip())


def bar(frac, width=30):
    n = int(round(frac * width))
    return '█' * n + '░' * (width - n)


def fmt_eta(s):
    if s is None or s != s or s < 0:
        return '--'
    h, m = divmod(int(s) // 60, 60)
    return f'{h}h{m:02d}m'


def main():
    while True:
        lines = [f'{BOLD}sac_v10 training dashboard{RESET}   {time.strftime("%H:%M:%S")}'
                 f'   {DIM}(refresh 15 s, Ctrl-C to quit){RESET}', '']
        for run in RUNS:
            log = os.path.join(RUNS_DIR, f'{run}_train.log')
            v, err = last_table(log)
            ts = v.get('total_timesteps', 0)
            fps = v.get('fps', 0)
            frac = min(1.0, ts / TOTAL)
            running = alive(run)
            if frac >= 1.0 and not running:
                state = f'{GREEN}DONE{RESET}'
            elif running:
                state = f'{GREEN}running{RESET}'
            else:
                state = f'{RED}STOPPED{RESET}'
            eta = (TOTAL - ts) / fps if fps > 0 and running else None
            lines.append(f'{BOLD}{run:15s}{RESET} {state}')
            lines.append(f'  {bar(frac)} {100 * frac:5.1f}%  {ts / 1e6:5.3f}M / 2M'
                         f'   {fps:4.0f} fps   ETA {fmt_eta(eta)}')
            sr = v.get('success_rate')
            col = GREEN if (sr or 0) >= 0.5 else YELLOW if (sr or 0) >= 0.2 else RED
            lines.append(
                f'  held-success {col}{(sr if sr is not None else float("nan")):5.2f}{RESET}'
                f'   ep_len {v.get("ep_len_mean", float("nan")):5.0f}'
                f'   ep_rew {v.get("ep_rew_mean", float("nan")):7.0f}'
                f'   episodes {v.get("episodes", 0):6.0f}')
            lines.append(
                f'  collision/step {v.get("collision_rate", float("nan")):.4f}'
                f'   min_dist {v.get("min_surface_dist", float("nan")):+.3f} m'
                f'   ent_coef {v.get("ent_coef", float("nan")):.3g}'
                f'   critic_loss {v.get("critic_loss", float("nan")):.3g}')
            if err and not running:
                lines.append(f'  {RED}{err.strip()[:110]}{RESET}')
            lines.append('')
        lines.append(f'{DIM}held-success = target still within 5 cm at the END of the episode '
                     f'(terminate_on_success off){RESET}')
        sys.stdout.write('\033[H\033[2J' + '\n'.join(lines) + '\n')
        sys.stdout.flush()
        time.sleep(15)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
