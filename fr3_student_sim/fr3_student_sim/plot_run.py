"""Plot a run log and print the contact metrics.

    ros2 run fr3_student_sim plot_run runs/my_controller_20261028_120000.npz
    ros2 run fr3_student_sim plot_run runs/*.npz --no-show      # PNGs only, many runs

For each log ``X.npz`` it writes ``X.png`` next to it and prints the ground-truth
facts of the first contact (time, entry speed, peak force and penetration, duration).
Logs with a momentum observer also produce ``X_observer.png`` with the seven
joint residuals, their detection thresholds, the residual norm and detection times.
Everything else (energies, effective mass, work, detection delay ...) is computed
by you from the log. In Python::

    from fr3_student_sim.plot_run import load_run, metrics
    run = load_run('runs/....npz')     # dict of arrays: run['t'], run['q'], run['F_n'], ...
    run['meta']                        # parameters of the run (task, wall, realism ...)
    print(metrics(run))

Logged arrays (one row per 1 kHz plant sample):
    t, q, dq            time [s], joint positions/velocities
    tau_cmd, tau        your torque before / after clipping to TAU_MAX
    tau_applied         delayed, clipped actuator torque over the sample interval;
                        gravity compensated, virtual wall excluded
    tau_wall            wall force mapped to the joints (J_c^T F)
    p, v                tool position / velocity
    F, F_n, delta, v_n  wall force vector, normal force, penetration, penetration rate
    contact             1 while in contact
    hand_depth          how deep the hand BODY is inside the wall (it should stay <= 0)
    ctrl_ms             time spent in Controller.update [ms]
    residual            momentum-observer residual on the seven joints [N m]
    residual_norm       Euclidean norm of the residual [N m]
    collision_detected  1 while any joint residual exceeds its threshold
    detection_event     1 at the start of each detected collision
    detection_times     times of the detection events [s] (one entry per event)
    residual_threshold  per-joint absolute residual thresholds [N m] (7 entries)
    user_<key>          everything you put in self.log
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

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


def detection_times(run: dict) -> np.ndarray:
    """Detection onsets from the log; older logs have no detection events."""
    if 'detection_times' in run:
        return np.asarray(run['detection_times'], dtype=float)
    if 'detection_event' in run:
        return run['t'][np.asarray(run['detection_event'], dtype=bool)]
    if 'collision_detected' in run:
        detected = np.asarray(run['collision_detected'], dtype=bool)
        return run['t'][detected & ~np.r_[False, detected[:-1]]]
    return np.empty(0)


def _mark_detections(axes, times):
    for axis in axes:
        for index, time in enumerate(times):
            axis.axvline(time, color='tab:red', alpha=0.65, lw=0.8,
                         label='detection onset' if index == 0 else None)


def _observer_figure(run: dict, plt):
    """Keep observer diagnostics separate from the tracking/contact figure."""
    t = run['t']
    residual = np.asarray(run['residual'])
    thresholds = run.get('residual_threshold')
    events = detection_times(run)
    fig, ax = plt.subplots(9, 1, figsize=(10, 18), sharex=True)
    for joint, axis in enumerate(ax[:7]):
        axis.plot(t, residual[:, joint], label='residual', lw=1.0)
        if thresholds is not None:
            threshold = np.asarray(thresholds)[joint]
            axis.axhline(threshold, color='tab:orange', ls='--', lw=0.8,
                         label='threshold')
            axis.axhline(-threshold, color='tab:orange', ls='--', lw=0.8)
        axis.set_ylabel(f'r{joint + 1} [N m]')
    ax[7].plot(t, run.get('residual_norm', np.linalg.norm(residual, axis=1)), 'k')
    ax[7].set_ylabel('||r|| [N m]')
    detected = np.asarray(run.get('collision_detected', np.zeros(len(t))), dtype=bool)
    ax[8].step(t, detected.astype(int), where='post', color='tab:red')
    ax[8].set_yticks([0, 1], labels=['no', 'yes'])
    ax[8].set_ylim(-0.1, 1.1)
    ax[8].set_ylabel('detected')
    ax[8].set_xlabel('t [s]')
    _mark_detections(ax, events)
    ax[0].legend(loc='upper right', fontsize=8)
    for axis in ax:
        axis.grid(alpha=0.3)
    fig.suptitle('Generalized momentum observer and collision detection')
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    return fig


def _plot_signal(axis, t, values, name, labels=None, **style):
    """Label Cartesian components, FR3 joints, and scalar/custom log signals."""
    lines = axis.plot(t, values, **style)
    if labels is None:
        if len(lines) == 3:
            labels = ('x', 'y', 'z')
        elif len(lines) == 7:
            labels = [f'joint {i + 1}' for i in range(7)]
        elif len(lines) == 1:
            labels = [name]
        else:
            labels = [f'{name}[{i}]' for i in range(len(lines))]
    for line, label in zip(lines, labels):
        line.set_label(label)


def _signal_legend(axis, secondary=None):
    """Place labels above the panel; include curves on its secondary axis."""
    handles, labels = axis.get_legend_handles_labels()
    if secondary is not None:
        extra_handles, extra_labels = secondary.get_legend_handles_labels()
        handles += extra_handles
        labels += extra_labels
    if handles:
        axis.legend(handles, labels, loc='lower left', bbox_to_anchor=(0, 1.02),
                    ncol=min(len(labels), 7), fontsize=8)


def plot(run: dict, path_png: str, show: bool = True):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = run['t']
    user = [k for k in run if k.startswith('user_') and k not in ('user_p_ref',)]
    fig, ax = plt.subplots(4 + len(user), 1, figsize=(10, 2.2 * (4 + len(user))), sharex=True)
    _plot_signal(ax[0], t, run['p'], 'position')
    if 'user_p_ref' in run:
        ax[0].set_prop_cycle(None)
        _plot_signal(ax[0], t, run['user_p_ref'], 'reference',
                     labels=('x ref', 'y ref', 'z ref'), linestyle='--')
    ax[0].set_ylabel('tool p [m]\n(-- ref)')
    _plot_signal(ax[1], t, run['tau'], 'torque')
    ax[1].set_ylabel('tau [N m]')
    _plot_signal(ax[2], t, run['dq'], 'joint velocity')
    ax[2].set_ylabel('dq [rad/s]')
    ax[3].plot(t, run['F_n'], 'k', label='normal force')
    ax3 = ax[3].twinx()
    ax3.plot(t, 1e3 * np.maximum(run['delta'], 0), 'r', lw=0.8, label='penetration')
    ax3.set_ylabel('penetration [mm]', color='r')
    ax[3].set_ylabel('wall F_n [N]')
    for a, k in zip(ax[4:], user):
        _plot_signal(a, t, run[k], k[5:])
        a.set_ylabel(k[5:])
    events = detection_times(run)
    _mark_detections((ax[0], ax[3]), events)
    ax[-1].set_xlabel('t [s]')
    for a in ax:
        a.grid(alpha=0.3)
        _signal_legend(a, secondary=ax3 if a is ax[3] else None)
    fig.suptitle(run['meta']['controller'].rsplit('/', 1)[-1])
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(path_png, dpi=110)
    print(f'  plot: {path_png}')
    observer_fig = None
    if 'residual' in run:
        observer_fig = _observer_figure(run, plt)
        path = Path(path_png)
        observer_path = path.with_name(path.stem + '_observer' + path.suffix)
        observer_fig.savefig(observer_path, dpi=110)
        print(f'  observer plot: {observer_path}')
    if show:
        plt.show()
    plt.close(fig)
    if observer_fig is not None:
        plt.close(observer_fig)


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
        if 'residual' in run:
            print(f'  detection_times_s: {detection_times(run).tolist()}')
        plot(run, path[:-4] + '.png', show=not a.no_show)


if __name__ == '__main__':
    main()
