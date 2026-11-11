"""Compact diagnostics for the impedance/HQP experiment; other plots unchanged."""

import numpy as np

from fr3_sim.simulation.fr3_model import Q_MAX, Q_MIN


def impedance_figure(run, plt):
    t = run['t']
    fig, axes = plt.subplots(10, 1, figsize=(11, 24), sharex=True)
    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    for i, name in enumerate(('x', 'y', 'z')):
        color = colors[i]
        axes[0].plot(t, run['p'][:, i], color=color, label=name)
        axes[0].plot(t, run['user_p_ref'][:, i], '--', color=color, label=name + ' ref')
        axes[1].plot(t, 1000 * run['user_position_error'][:, i], label=name)
    for i in range(7):
        label = f'joint {i + 1}'
        axes[2].plot(t, run['tau'][:, i], label=label)
        axes[3].plot(t, run['dq'][:, i], label=label)
        margin = np.minimum(run['q'][:, i] - Q_MIN[i], Q_MAX[i] - run['q'][:, i])
        axes[7].plot(t, margin, label=label)
    axes[4].plot(t, run['F_n'], color='k', label='normal contact force')
    axes[5].plot(t, run['user_mass_kg'], label='directional mass')
    axes[6].plot(t, run['v_n'], label='actual normal velocity')
    axes[6].plot(t, run['user_normal_speed_ref'], '--', label='reference')
    configured_margin = run['meta']['params']['safety']['position_margin']
    axes[7].axhline(float(np.min(configured_margin)), color='k', ls='--', label='QP margin')
    axes[7].axhline(0.05, color='r', ls=':', label='runner stop')
    axes[8].plot(t, run['user_qp_primary_status'], label='primary status')
    axes[8].plot(t, run['user_qp_secondary_status'], '--', label='secondary status')
    axes[8].plot(t, run['user_observer_valid'], ':', label='observer valid')
    if 'user_startup_scale' in run:
        axes[8].plot(t, run['user_startup_scale'], color='tab:purple', label='startup scale')
    axes[9].plot(t, run['ctrl_ms'], label='Controller.update')
    period = 1000 / run['meta']['control_rate']
    axes[9].axhline(period, color='r', ls='--', label='period at RTF=1')
    labels = ('tool p [m]', 'position error [mm]', 'torque [N m]', 'dq [rad/s]',
              'contact force [N]', 'mass [kg]', 'normal speed [m/s]',
              'physical joint margin [rad]', 'status / valid', 'compute [ms]')
    for axis, label in zip(axes, labels):
        axis.set_ylabel(label)
        axis.grid(alpha=0.3)
        axis.legend(loc='upper right', fontsize=7, ncol=4)
    axes[-1].set_xlabel('simulation time [s]')
    fig.suptitle('Cartesian impedance and hierarchical QP')
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    return fig
