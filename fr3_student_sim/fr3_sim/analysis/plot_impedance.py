"""Compact HQP diagnostics, including historical Cartesian-impedance logs."""

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
        if 'user_contact_p_ref' in run:
            axes[0].plot(t, run['user_contact_p_ref'][:, i], ':', color=color,
                         label=name + ' contact equilibrium')
        if 'user_contact_position_error' in run:
            axes[1].plot(t, 1000 * run['user_contact_position_error'][:, i], ':',
                         color=color, label=name + ' contact error')
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
    safety = run['meta']['params']['safety']
    configured_margin = safety['position_margin']
    soft = safety.get('soft_constraints', {})
    elastic = bool(soft.get('enabled', False))
    axes[7].axhline(float(np.min(configured_margin)), color='k', ls='--',
                    label='preferred margin (soft)' if elastic else 'QP margin')
    if elastic and 'hard_position_margin' in soft:
        axes[7].axhline(float(np.min(soft['hard_position_margin'])), color='tab:orange',
                        ls='-.', label='hard QP margin')
    axes[7].axhline(0.05, color='r', ls=':', label='runner stop')
    axes[8].plot(t, run['user_qp_primary_status'], label='tracking status' if elastic else 'primary status')
    axes[8].plot(t, run['user_qp_secondary_status'], '--', label='redundancy status' if elastic else 'secondary status')
    axes[8].plot(t, run['user_observer_valid'], ':', label='observer valid')
    if 'user_qp_safety_status' in run:
        axes[8].plot(t, run['user_qp_safety_status'], '-.', label='safety status')
    if 'user_soft_slack_normalized_max' in run:
        axes[8].plot(t, run['user_soft_slack_normalized_max'], color='tab:orange',
                     label='max normalized slack')
    if 'user_startup_scale' in run:
        axes[8].plot(t, run['user_startup_scale'], color='tab:purple', label='startup scale')
    if 'user_phase' in run:
        axes[8].step(t, run['user_phase'], where='post', color='tab:brown',
                     label='phase (0 approach, 1 transition, 2 contact)')
    if 'user_contact_blend' in run:
        axes[8].plot(t, run['user_contact_blend'], color='tab:cyan', label='contact blend')
    if 'user_secondary_scale' in run:
        axes[8].plot(t, run['user_secondary_scale'], '--', color='tab:green',
                     label='mass / posture scale')
    if 'user_contact_latched' in run:
        axes[8].plot(t, run['user_contact_latched'], ':', color='tab:pink',
                     label='contact latched')
    axes[9].plot(t, run['ctrl_ms'], label='Controller.update')
    if 'user_qp_safety_ms' in run:
        axes[9].plot(t, run['user_qp_safety_ms'], lw=0.8, alpha=0.7, label='safety QP')
    period = 1000 / run['meta']['control_rate']
    axes[9].axhline(period, color='r', ls='--', label='period at RTF=1')
    labels = ('tool p [m]', 'position error [mm]', 'torque [N m]', 'dq [rad/s]',
              'contact force [N]', 'mass [kg]', 'normal speed [m/s]',
              'physical joint margin [rad]', 'status / valid / slack' if elastic else 'status / valid', 'compute [ms]')
    for axis, label in zip(axes, labels):
        axis.set_ylabel(label)
        axis.grid(alpha=0.3)
        axis.legend(loc='upper right', fontsize=7, ncol=4)
    axes[-1].set_xlabel('simulation time [s]')
    title = ('Three-phase approach / transition / contact with HQP' if 'user_phase' in run
             else 'Cartesian tracking and hierarchical QP' if 'user_tracking_acceleration' in run
             else 'Cartesian impedance and hierarchical QP')
    fig.suptitle(title)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    return fig
