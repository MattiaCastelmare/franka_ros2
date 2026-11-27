"""Plot a run log and print the contact metrics.

    ros2 run fr3_student_sim plot_run runs/circle_baseline_20261109_180000.npz
    ros2 run fr3_student_sim plot_run runs/*.npz --no-show      # write figures for many runs

For each log ``X.npz`` it writes ``X.png`` next to it and prints first-impact
metrics: peak force, normal impulse, maximum penetration, and the normal
kinetic energy, mass and speed from the sample immediately before contact.
Logs with a momentum observer also produce ``X_observer.png`` with the seven
joint residuals, their detection thresholds, the residual norm and detection times.
With the FR3 model and wall geometry available, it also compares the estimated
and simulated normal force. The estimate inverts the observer filter when its
gain was logged. For a complete first impact with the required observer data,
``X_force_estimation.png`` compares the simulated force with the two estimates
from the filtered residual and its filter inversion.
``X_collision.png`` contains six panels for the first compression-release cycle:
directional mass, normal velocity, contact force, penetration, normal impulse
and a pre-contact normal kinetic-energy summary. The interval ends at the first
detachment or the start of recompression, whichever occurs first.
Circular runs also produce ``X_all_collisions.png`` with the same quantities
over the full run, using one cycle per recorded contact. Optimized logs use
ten main panels. Baseline logs use the generic layout for their user signals.
Further analyses (detection delay, tracking error ...) can be computed from
the log. In Python::

    from fr3_sim.analysis.plot_run import load_run, metrics
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
    ctrl_ms             wall time spent in the last Controller.update [ms]
    controller_updated  true only at samples with a Controller.update call
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
import warnings
from pathlib import Path

import numpy as np

from fr3_sim.analysis.plot_collision import collision_figure
from fr3_sim.analysis.plot_all_collisions import all_collisions_figure, is_circular_run
from fr3_sim.analysis.impact_metrics import first_impact
from fr3_sim.analysis.timing import timing_metrics


def load_run(path: str) -> dict:
    with np.load(path, allow_pickle=False) as z:
        run = {k: z[k] for k in z.files if k != 'meta'}
        run['meta'] = json.loads(str(z['meta']))
    return run


def metrics(run: dict) -> dict:
    """Run diagnostics and the same first-impact metrics used by the figure."""
    t = run['t']
    out = {'duration_s': float(t[-1]), 'stop_reason': run['meta'].get('stop_reason', ''),
           'samples': int(len(t)),
           'tau_saturated_samples':
           int(np.sum(np.any(np.abs(run['tau_cmd'] - run['tau']) > 1e-9, axis=1))),
           'max_hand_depth_mm': 1e3 * float(np.max(run['hand_depth'])) if 'hand_depth' in run else 0.0}
    out.update(timing_metrics(run))
    out.update(first_impact(run)['metrics'])
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


def _invert_observer_filter(run: dict, residual: np.ndarray) -> np.ndarray | None:
    """Undo the observer's implicit Euler filter on the saved joint residuals."""
    observer = run['meta'].get('observer')
    if not isinstance(observer, dict):
        return None
    gain = np.asarray(observer.get('gain'), dtype=float)
    if gain.shape not in ((), (7,)) or not np.all(np.isfinite(gain) & (gain > 0)):
        return None

    dt = np.diff(run['t'])[:, None]
    torque = np.full_like(residual, np.nan, dtype=float)
    # The first residual comes from initialization and has no preceding sample.
    np.divide(np.diff(residual, axis=0), dt * gain, out=torque[1:],
              where=np.isfinite(dt) & (dt > 0))
    torque[1:] += residual[1:]
    return torque


def _estimate_normal_force(run: dict, joint_torque: np.ndarray) -> np.ndarray | None:
    """Fit joint_torque = (J_tip.T @ normal) * F_n for a fingertip contact."""
    wall = run['meta'].get('wall', {})
    if 'q' not in run or 'normal' not in wall or not wall.get('enabled', True):
        return None
    q = np.asarray(run['q'], dtype=float)
    normal = np.asarray(wall['normal'], dtype=float)
    if q.ndim != 2 or q.shape[1] != 7 or q.shape != joint_torque.shape or normal.shape != (3,):
        return None
    normal_length = np.linalg.norm(normal)
    if not np.isfinite(normal_length) or normal_length == 0:
        return None
    normal = normal / normal_length

    try:
        from fr3_sim.simulation.fr3_model import FR3Model
        robot = FR3Model()
    except (ImportError, LookupError, OSError) as exc:
        warnings.warn(f'Normal force estimate unavailable: {exc}', stacklevel=2)
        return None

    force = np.full(len(q), np.nan)
    valid = np.all(np.isfinite(q) & np.isfinite(joint_torque), axis=1)
    for i in np.flatnonzero(valid):
        torque_per_newton = robot.jacobian_pos(q[i]).T @ normal
        sensitivity_squared = torque_per_newton @ torque_per_newton
        if sensitivity_squared > 1e-12:
            force[i] = (torque_per_newton @ joint_torque[i]) / sensitivity_squared
    return force


def _observer_figure(run: dict, plt):
    """Keep the nine/ten observer panels readable in a two-column figure."""
    from matplotlib.ticker import MaxNLocator

    t = run['t']
    residual = np.asarray(run['residual'])
    thresholds = run.get('residual_threshold')
    events = detection_times(run)
    torque = _invert_observer_filter(run, residual)
    normal_force = _estimate_normal_force(run, residual if torque is None else torque)
    fig, grid = plt.subplots(5, 2, figsize=(6.27, 7.4), sharex=True)
    # Read down each column: r1-r5, then r6/r7/norm/force/detection.
    ax = grid.T.ravel()
    if normal_force is None:
        fig.delaxes(ax[8])
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
    if normal_force is not None:
        if 'F_n' in run:
            ax[8].plot(t, run['F_n'], label='simulated wall force', color='0.35', lw=1.0)
        label = ('estimate from filtered residual' if torque is None
                 else 'estimate with observer filter inversion')
        ax[8].plot(t, normal_force, label=label, color='tab:blue', ls='--', lw=1.2)
        ax[8].set_ylabel('Force [N]')
        handles, _ = ax[8].get_legend_handles_labels()
        labels = (['Simulated'] if 'F_n' in run else [])
        labels.append('Filtered residual' if torque is None else 'Filter inversion')
        ax[8].legend(handles, labels, loc='lower right', bbox_to_anchor=(1, 1.01),
                     ncol=2, fontsize=7.5, frameon=False, columnspacing=0.7,
                     handlelength=1.3, borderaxespad=0)

    detected = np.asarray(run.get('collision_detected', np.zeros(len(t))), dtype=bool)
    ax[-1].step(t, detected.astype(int), where='post', color='tab:red')
    ax[-1].set_yticks([0, 1], labels=['no', 'yes'])
    ax[-1].set_ylim(-0.1, 1.1)
    ax[-1].set_ylabel('detected')
    _mark_detections(fig.axes, events)
    handles, labels = ax[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.55, 0.968),
               ncol=3, fontsize=8.5, frameon=False, handlelength=1.5,
               columnspacing=1.2)
    for axis in fig.axes:
        bottom = axis in (ax[4], ax[-1])
        axis.yaxis.label.set_fontsize(10)
        axis.tick_params(axis='both', labelsize=9, labelbottom=bottom, pad=2)
        axis.xaxis.set_major_locator(MaxNLocator(nbins=3))
        if axis is not ax[-1]:
            axis.yaxis.set_major_locator(MaxNLocator(nbins=3, min_n_ticks=3))
        if bottom:
            axis.set_xlabel('t [s]', fontsize=10, labelpad=3)
        axis.grid(alpha=0.3)
    fig.suptitle('Generalized momentum observer and collision detection',
                 fontsize=11, y=0.995)
    fig.subplots_adjust(left=0.13, right=0.985, bottom=0.07, top=0.91,
                        wspace=0.60, hspace=0.48)
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


def _first_impact_force_figure(run: dict, plt):
    """Compare the two observer estimates with the logged force at first impact."""
    if 'residual' not in run or 'q' not in run:
        return None
    impact = first_impact(run)
    metrics = impact['metrics']
    if not metrics['first_impact_complete']:
        return None

    residual = np.asarray(run['residual'])
    corrected_torque = _invert_observer_filter(run, residual)
    if corrected_torque is None:
        return None

    t = impact['t']
    onset = metrics['t_contact_s']
    window = (t >= onset - 0.05) & (t <= metrics['first_impact_end_s'])
    # Invert before slicing so every displayed sample can use its predecessor.
    samples = {'meta': run['meta'], 'q': np.asarray(run['q'])[window]}
    filtered = _estimate_normal_force(samples, residual[window])
    corrected = _estimate_normal_force(samples, corrected_torque[window])
    during_impact = t[window] >= onset
    if any(force is None or not np.any(np.isfinite(force[during_impact]))
           for force in (filtered, corrected)):
        return None

    time_ms = 1000 * (t[window] - onset)
    title = 'First-impact force reconstruction'
    if is_circular_run(run):
        title = 'Circular: ' + title.lower()
    elif 'approach' in run['meta'].get('params', {}):
        title = 'Perpendicular: ' + title.lower()
    with plt.rc_context({'font.size': 11, 'axes.spines.top': False,
                         'axes.spines.right': False}):
        fig, axis = plt.subplots(figsize=(6.4, 3.05))
        axis.plot(time_ms, impact['normal_force_N'][window], color='0.2', lw=1.7,
                  label='Simulated force')
        axis.plot(time_ms, filtered, color='#D55E00', ls='--', lw=1.5,
                  label='Filtered residual')
        axis.plot(time_ms, corrected, color='#0072B2', lw=1.4, label='Filter inversion')
        axis.axvline(0, color='0.5', ls=':', lw=0.8)
        axis.set(xlabel=r'$t-t_c$ [ms]', ylabel='Normal force [N]', title=title)
        axis.grid(alpha=0.25)
        axis.legend(loc='upper left', fontsize=9, frameon=False)
        fig.tight_layout()
    return fig


def plot(run: dict, path_png: str, show: bool = True):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    # Removed controller names remain log-format identifiers, never imports.
    if Path(run['meta'].get('controller', '')).name in (
            'optimized_hqp.py', 'optimized_hqp_perp.py', 'optimized_3phases_perp.py',
            'optimized_hqp_full_online.py', 'optimized_hqp_full_online_perp.py',
            'optimized_full_online_circular.py',
            'optimized_full_online_qp_circular.py', 'optimized_full_online_qp_perp.py',
            'optimized_perp_full_online.py',
            'impedance_hqp.py', 'controller_impedance_hqp.py'):
        from fr3_sim.analysis.plot_impedance import impedance_figure
        fig = impedance_figure(run, plt)
        _save_figures(run, fig, path_png, show, plt)
        return
    t = run['t']
    user = [k for k in run if k.startswith('user_') and k not in ('user_p_ref',)]
    panels = 4 + len(user)
    fig, ax = plt.subplots(panels, 1, figsize=(10, 2.2 * panels), sharex=True)
    position_axis, torque_axis, velocity_axis, contact_axis = ax[:4]
    _plot_signal(position_axis, t, run['p'], 'position')
    if 'user_p_ref' in run:
        position_axis.set_prop_cycle(None)
        _plot_signal(position_axis, t, run['user_p_ref'], 'reference',
                     labels=('x ref', 'y ref', 'z ref'), linestyle='--')
    position_axis.set_ylabel('tool p [m]\n(-- ref)')
    _plot_signal(torque_axis, t, run['tau'], 'torque')
    torque_axis.set_ylabel('tau [N m]')
    _plot_signal(velocity_axis, t, run['dq'], 'joint velocity')
    velocity_axis.set_ylabel('dq [rad/s]')
    contact_axis.plot(t, run['F_n'], 'k', label='normal force')
    penetration_axis = contact_axis.twinx()
    penetration_axis.plot(t, 1e3 * np.maximum(run['delta'], 0), 'r', lw=0.8,
                          label='penetration')
    penetration_axis.set_ylabel('penetration [mm]', color='r')
    contact_axis.set_ylabel('wall F_n [N]')
    for a, k in zip(ax[4:], user):
        _plot_signal(a, t, run[k], k[5:])
        a.set_ylabel(k[5:])
    events = detection_times(run)
    _mark_detections((position_axis, contact_axis), events)
    ax[-1].set_xlabel('t [s]')
    for a in ax:
        a.grid(alpha=0.3)
        _signal_legend(a, secondary=penetration_axis if a is contact_axis else None)
    fig.suptitle(run['meta']['controller'].rsplit('/', 1)[-1])
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    _save_figures(run, fig, path_png, show, plt)


def _save_figures(run, fig, path_png, show, plt):
    """Save the tracking, contact and available observer figures for a run."""
    path = Path(path_png)
    fig.savefig(path_png, dpi=110)
    print(f'  plot: {path_png}')
    observer_fig = None
    force_fig = None
    if 'residual' in run:
        observer_fig = _observer_figure(run, plt)
        observer_path = path.with_name(path.stem + '_observer' + path.suffix)
        observer_fig.savefig(observer_path, dpi=200)
        print(f'  observer plot: {observer_path}')
        force_fig = _first_impact_force_figure(run, plt)
        if force_fig is not None:
            force_path = path.with_name(path.stem + '_force_estimation.png')
            force_fig.savefig(force_path, dpi=200)
            print(f'  force estimation plot: {force_path}')
    collision_fig = collision_figure(run, plt)
    collision_path = path.with_name(path.stem + '_collision' + path.suffix)
    collision_fig.savefig(collision_path, dpi=110)
    print(f'  collision plot: {collision_path}')
    all_collisions_fig = None
    if is_circular_run(run):
        all_collisions_fig = all_collisions_figure(run, plt)
        all_collisions_path = path.with_name(path.stem + '_all_collisions' + path.suffix)
        all_collisions_fig.savefig(all_collisions_path, dpi=110)
        print(f'  all collisions plot: {all_collisions_path}')
    if show:
        plt.show()
    for figure in (fig, observer_fig, force_fig, collision_fig, all_collisions_fig):
        if figure is not None:
            plt.close(figure)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--no-show', action='store_true', help='write figures without opening windows')
    a = ap.parse_args()
    for path in a.logs:
        if not path:
            ap.error('empty log path; check that the shell pattern matched an existing .npz file')
        if not Path(path).is_file():
            ap.error(f'log file not found: {path}')
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
