"""Offline first-impact figure using only signals already stored in a log.

The selected interval contains the first compression and release, ending at
detachment or the start of recompression. Incoming normal kinetic energy is
measured before contact; cumulative positive contact work is not reported.
No dynamics are evaluated and no later impact contributes to the metrics.
"""
from __future__ import annotations

import numpy as np

from fr3_student_sim.impact_metrics import first_impact


def collision_signals(run: dict) -> dict:
    """Return first-impact signals and metrics, preserving optional NaNs.

    Position-error norm remains an optional diagnostic for callers, without
    being displayed or used in the first-impact metrics.
    """
    signals = first_impact(run)
    signals['contact_time_s'] = signals['metrics']['t_contact_s']
    samples = len(signals['t'])
    error = None
    if 'user_p_ref' in run and 'p' in run:
        reference = np.asarray(run['user_p_ref'], dtype=float)
        position = np.asarray(run['p'], dtype=float)
        if reference.shape != (samples, 3) or position.shape != (samples, 3):
            raise ValueError('Tool position and reference must contain one 3-vector per timestamp')
        error = reference - position
    elif 'user_position_error' in run:
        error = np.asarray(run['user_position_error'], dtype=float)
        if error.shape != (samples, 3):
            raise ValueError('Position error must contain one 3-vector per timestamp')
    signals['position_error_mm'] = None if error is None else 1e3 * np.linalg.norm(error, axis=1)
    return signals


def _value(value, unit, decimals=3):
    if value is None or not np.isfinite(value):
        return 'Not available'
    return f'{value:.{decimals}f} {unit}'


def collision_figure(run: dict, plt):
    """Show the first compression-release cycle and incoming energy.

    Five time panels include 50 ms before recorded contact and stop at the
    selected cycle end. The sixth panel reports the pre-contact energy and
    the mass and speed used to calculate it. Missing/incomplete data are
    labelled explicitly. The normal impulse starts at recorded contact.
    """
    signals = collision_signals(run)
    summary = signals['metrics']
    t = signals['t']
    contact_time = summary['t_contact_s']
    end_time = summary['first_impact_end_s']
    relative = contact_time is not None
    origin = contact_time if relative else 0.0
    if t.size:
        left = max(float(t[0]), contact_time - 0.05) if relative else float(t[0])
        right = end_time if end_time is not None else float(t[-1])
        visible = (t >= left) & (t <= right)
        selected_t = t[visible]
        # Include the interpolated zero-speed boundary before recompression.
        append_end = (selected_t.size and right > selected_t[-1]
                      and right <= t[-1])
        draw_t = np.r_[selected_t, right] if append_end else selected_t
    else:
        visible = np.zeros(t.shape, dtype=bool)
        draw_t = t
        append_end = False
        left = right = None

    fig, grid = plt.subplots(3, 2, figsize=(11, 10), facecolor='white')
    axes = list(grid.flat)
    time_axes = axes[:5]
    panels = (
        ('mass_kg', 'Directional mass', r'$m_n$ [kg]'),
        ('normal_speed_mps', 'Normal speed (+ into wall)', r'$v_n$ [m/s]'),
        ('normal_force_N', 'Normal contact force', r'$F_n$ [N]'),
        ('penetration_mm', 'Penetration', r'$\delta$ [mm]'),
    )
    for axis, (key, title, units) in zip(time_axes, panels):
        values = signals[key]
        if values is None:
            axis.text(0.5, 0.5, 'Not logged', transform=axis.transAxes, ha='center', va='center')
        else:
            draw_values = values[visible]
            if append_end:
                draw_values = np.r_[draw_values, np.interp(right, t, values)]
            axis.plot(draw_t - origin, draw_values, color='tab:blue', lw=1.5)
        axis.set_title(title, fontsize=11)
        axis.set_ylabel(units, fontsize=10)

    impulse_axis = axes[4]
    impact_t = signals['impact_t']
    if impact_t.size:
        before = draw_t[draw_t < contact_time]
        impulse_axis.plot(
            np.r_[before, impact_t] - origin,
            np.r_[np.zeros(len(before)), signals['impact_impulse_Ns']],
            color='tab:blue', lw=1.5)
    else:
        impulse_axis.text(0.5, 0.5, 'First cycle unavailable',
                          transform=impulse_axis.transAxes, ha='center', va='center')
    impulse_axis.set_title('Normal impulse — first cycle', fontsize=11)
    impulse_axis.set_ylabel(r'$I_n$ [N s]', fontsize=10)

    pre = signals['precontact_index']
    for axis, key, value in (
            (axes[0], 'mass_kg', summary['entry_mass_kg']),
            (axes[1], 'normal_speed_mps', summary['entry_speed_mps'])):
        if pre is not None and value is not None:
            axis.plot(t[pre] - origin, value, 'o', color='tab:orange', ms=4)
        unit, digits = ('kg', 3) if key == 'mass_kg' else ('m/s', 5)
        axis.text(0.98, 0.95, 'Before contact: ' + _value(value, unit, digits),
                  transform=axis.transAxes, ha='right', va='top', fontsize=9,
                  bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': 'none'})

    for axis, values, value, label, unit in (
            (axes[2], signals['impact_force_N'], summary['peak_force_N'], 'Peak', 'N'),
            (axes[3], signals['impact_penetration_mm'],
             summary['peak_penetration_mm'], 'Maximum', 'mm')):
        if impact_t.size and values is not None and value is not None:
            index = int(np.argmax(values))
            axis.plot(impact_t[index] - origin, values[index], 'o', color='tab:orange', ms=4)
        axis.text(0.98, 0.95, f'{label}: ' + _value(value, unit),
                  transform=axis.transAxes, ha='right', va='top', fontsize=9,
                  bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': 'none'})
    impulse_axis.text(0.98, 0.95, 'Impulse: ' + _value(summary['normal_impulse_Ns'], 'N s', 4),
                      transform=impulse_axis.transAxes, ha='right', va='top', fontsize=9,
                      bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': 'none'})

    for index, axis in enumerate(time_axes):
        if relative:
            axis.axvline(0.0, color='#c49a00', ls='--', lw=1.1)
            if end_time is not None:
                axis.axvline(end_time - origin, color='0.4', ls=':', lw=1.0)
        if left is not None and left < right:
            axis.set_xlim(left - origin, right - origin + 0.03 * (right - left))
        axis.tick_params(labelsize=9, labelbottom=index in (3, 4))
        if index in (3, 4):
            axis.set_xlabel(r'$t-t_c$ [s]' if relative else 't [s]')
        axis.grid(alpha=0.3)

    energy_axis = axes[5]
    energy_axis.axis('off')
    energy_axis.set_title('Normal kinetic energy before contact', fontsize=11)
    energy = summary['entry_normal_kinetic_energy_J']
    energy_axis.text(0.5, 0.84, r'$E_n^- = \frac{1}{2}m_n^- (v_n^-)^2$',
                     transform=energy_axis.transAxes, ha='center', va='center', fontsize=16)
    energy_axis.text(0.5, 0.63, _value(None if energy is None else 1e3 * energy, 'mJ'),
                     transform=energy_axis.transAxes, ha='center', va='center', fontsize=22)
    energy_axis.text(0.5, 0.44,
                     r'$m_n^-$ = ' + _value(summary['entry_mass_kg'], 'kg') + '\n'
                     + r'$v_n^-$ = ' + _value(summary['entry_speed_mps'], 'm/s', 5),
                     transform=energy_axis.transAxes, ha='center', va='top', fontsize=11, linespacing=1.7)
    stamp = summary['precontact_sample_s']
    note = (f'Last sample before contact: t = {stamp:.6f} s'
            if stamp is not None else 'No pre-contact sample available')
    energy_axis.text(0.5, 0.10, note, transform=energy_axis.transAxes,
                     ha='center', fontsize=9, color='0.4')

    fig.suptitle('First impact: compression and release', fontsize=14, y=0.99)
    if relative:
        reasons = {'detachment': 'detachment', 'recompression': 'start of recompression',
                   'end_of_log': 'incomplete: log ended',
                   'normal_speed_not_logged': 'cycle unavailable: normal speed not logged'}
        reason = reasons.get(summary['first_impact_end_reason'], 'cycle unavailable')
        subtitle = f'Contact at t = {contact_time:.3f} s; {reason}'
        if end_time is not None:
            subtitle += f' at t = {end_time:.6f} s'
        if signals['precontact_index'] is None:
            subtitle += '; incomplete: contact already present at log start'
        subtitle += '\nDashed gold: recorded contact onset; dotted grey: interval end'
    else:
        subtitle = 'No contact recorded' if signals['normal_force_N'] is not None else 'Contact force not logged'
    fig.text(0.5, 0.955, subtitle, ha='center', va='top', fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.90), h_pad=2.0)
    return fig
