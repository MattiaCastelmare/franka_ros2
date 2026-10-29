"""Plot a run log and print the contact metrics.

    ros2 run fr3_student_sim plot_run runs/my_controller_20261028_120000.npz
    ros2 run fr3_student_sim plot_run runs/*.npz --no-show      # PNGs only, many runs

For each log ``X.npz`` it writes ``X.png`` next to it and prints the ground-truth
facts of the first contact (time, entry speed, peak force and penetration, duration).
Everything else (energies, effective mass, work, detection delay ...) is computed
by you from the log. In Python::

    from fr3_student_sim.plot_run import load_run, metrics
    run = load_run('runs/....npz')     # dict of arrays: run['t'], run['q'], run['F_n'], ...
    run['meta']                        # parameters of the run (task, wall, realism ...)
    print(metrics(run))

Logged arrays (one row per 1 kHz plant sample):
    t, q, dq            time [s], joint positions/velocities
    tau_cmd, tau        your torque before / after clipping to TAU_MAX
    tau_wall            wall force mapped to the joints (J_c^T F)
    p, v                tool position / velocity
    F, F_n, delta, v_n  wall force vector, normal force, penetration, penetration rate
    contact             1 while in contact
    hand_depth          how deep the hand BODY is inside the wall (it should stay <= 0)
    ctrl_ms             time spent in Controller.update [ms]
    user_<key>          everything you put in self.log
"""
from __future__ import annotations

import argparse
import json

import numpy as np


def load_run(path: str) -> dict:
    with np.load(path, allow_pickle=False) as z:
        run = {k: z[k] for k in z.files if k != 'meta'}
        run['meta'] = json.loads(str(z['meta']))
    return run


def metrics(run: dict) -> dict:
    """Ground-truth facts of the run and of its first contact. Energies, effective
    mass, work, detection delay, tracking error ... are yours to compute from the log."""
    t = run['t']
    out = {'duration_s': float(t[-1]), 'stop_reason': run['meta'].get('stop_reason', ''),
           'samples': int(len(t)),
           'missed_samples_pct': 100.0 * (1.0 - len(t) / max(round((t[-1] - t[0]) / 1e-3) + 1, 1)),
           'tau_saturated_samples':
           int(np.sum(np.any(np.abs(run['tau_cmd'] - run['tau']) > 1e-9, axis=1))),
           'controller_ms_mean': float(np.mean(run['ctrl_ms'])),
           'controller_ms_max': float(np.max(run['ctrl_ms'])),
           'max_hand_depth_mm': 1e3 * float(np.max(run['hand_depth'])) if 'hand_depth' in run else 0.0}
    in_c = np.where(run['contact'] > 0)[0]
    if in_c.size == 0:
        out['contact'] = False
        return out
    i0 = int(in_c[0])
    after = np.where(run['contact'][i0:] == 0)[0]
    i1 = i0 + int(after[0]) if after.size else len(t)
    out.update({
        'contact': True,
        't_contact_s': float(t[i0]),
        'contact_duration_s': float(t[i1 - 1] - t[i0]),
        'entry_speed_mps': float(run['v_n'][i0]),
        'peak_force_N': float(np.max(run['F_n'][i0:i1])),
        'peak_penetration_mm': 1e3 * float(np.max(run['delta'][i0:i1])),
    })
    return out


def plot(run: dict, path_png: str, show: bool = True):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = run['t']
    user = [k for k in run if k.startswith('user_') and k not in ('user_p_ref',)]
    fig, ax = plt.subplots(4 + len(user), 1, figsize=(10, 2.2 * (4 + len(user))), sharex=True)
    ax[0].plot(t, run['p'])
    if 'user_p_ref' in run:
        ax[0].set_prop_cycle(None)
        ax[0].plot(t, run['user_p_ref'], '--')
    ax[0].set_ylabel('tool p [m]\n(-- ref)')
    ax[1].plot(t, run['tau'])
    ax[1].set_ylabel('tau [N m]')
    ax[2].plot(t, run['dq'])
    ax[2].set_ylabel('dq [rad/s]')
    ax[3].plot(t, run['F_n'], 'k')
    ax3 = ax[3].twinx()
    ax3.plot(t, 1e3 * np.maximum(run['delta'], 0), 'r', lw=0.8)
    ax3.set_ylabel('penetration [mm]', color='r')
    ax[3].set_ylabel('wall F_n [N]')
    for a, k in zip(ax[4:], user):
        a.plot(t, run[k])
        a.set_ylabel(k[5:])
    ax[-1].set_xlabel('t [s]')
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle(run['meta']['controller'].rsplit('/', 1)[-1])
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    fig.savefig(path_png, dpi=110)
    if show:
        plt.show()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--no-show', action='store_true', help='only write the PNGs')
    a = ap.parse_args()
    for path in a.logs:
        run = load_run(path)
        print(f'\n== {path}')
        for k, v in metrics(run).items():
            print(f'  {k:30s} {v:.4g}' if isinstance(v, float) else f'  {k:30s} {v}')
        plot(run, path[:-4] + '.png', show=not a.no_show)


if __name__ == '__main__':
    main()
