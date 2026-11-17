"""Full-run circular contact figure, with one compression/release per entry."""
from __future__ import annotations

import numpy as np

from fr3_sim.analysis.impact_metrics import all_impacts, first_impact


def is_circular_run(run: dict) -> bool:
    """Restrict the extra figure to the three circular controller adapters."""
    name = str(run.get('meta', {}).get('controller', '')).replace('\\', '/').rsplit('/', 1)[-1]
    return name in ('baseline.py', 'optimized_hqp.py', 'optimized_full_online_circular.py')


def all_collisions_figure(run: dict, plt):
    """Show all recorded contacts on absolute simulation time.

    Continuous signals span the full log. Per-entry metrics use the same
    compression/release definition as the existing first-impact figure.
    Impulse starts again at zero for each entry; energy uses the preceding
    sample. Observer threshold events do not create contact episodes.
    """
    signals = first_impact(run)
    impacts = all_impacts(run)
    t = signals['t']
    fig, grid = plt.subplots(3, 2, figsize=(13, 10), sharex=True, facecolor='white')
    axes = list(grid.flat)
    panels = (
        ('mass_kg', 'Directional mass', r'$m_n$ [kg]'),
        ('normal_speed_mps', 'Normal speed (+ into wall)', r'$v_n$ [m/s]'),
        ('normal_force_N', 'Normal contact force', r'$F_n$ [N]'),
        ('penetration_mm', 'Penetration', r'$\delta$ [mm]'),
    )
    for axis, (key, title, unit) in zip(axes, panels):
        values = signals[key]
        if values is None:
            axis.text(0.5, 0.5, 'Not logged', transform=axis.transAxes,
                      ha='center', va='center')
        else:
            axis.plot(t, values, color='tab:blue', lw=1.2)
        axis.set_title(title, fontsize=11)
        axis.set_ylabel(unit)

    for axis, key in ((axes[0], 'entry_mass_kg'), (axes[1], 'entry_speed_mps')):
        available = [impact['metrics'] for impact in impacts
                     if impact['metrics'][key] is not None]
        if available:
            axis.scatter([item['precontact_sample_s'] for item in available],
                         [item[key] for item in available], s=22,
                         color='tab:orange', zorder=3, label='Before contact')

    impulse_axis, energy_axis = axes[4:]
    impulse_axis.set_title('Normal impulse per contact — first cycle', fontsize=11)
    impulse_axis.set_ylabel(r'$I_n$ [N s]')
    energy_axis.set_title('Normal kinetic energy before each contact', fontsize=11)
    energy_axis.set_ylabel(r'$E_n^-$ [mJ]')
    has_impulse = False
    energy_labels = set()
    for number, impact in enumerate(impacts, 1):
        summary = impact['metrics']
        onset = summary['t_contact_s']
        complete = bool(summary['first_impact_complete'])
        for axis in axes:
            axis.axvline(onset, color='#c49a00', ls='--', lw=0.7, alpha=0.5)
        # Stagger numbers so closely spaced rebounds remain distinguishable.
        axes[2].annotate(str(number), (onset, 0.98 if number % 2 else 0.89),
                         xycoords=axes[2].get_xaxis_transform(),
                         xytext=(3, -3), textcoords='offset points',
                         va='top', fontsize=8, color='0.35')

        impact_t = impact['impact_t']
        if impact_t.size:
            has_impulse = True
            impulse_axis.plot(impact_t, impact['impact_impulse_Ns'],
                              color='tab:blue', lw=1.3, ls='-' if complete else '--')
            impulse_axis.plot(impact_t[-1], impact['impact_impulse_Ns'][-1],
                              marker='o' if complete else 'x', ms=4,
                              color='tab:orange' if complete else 'tab:red')
            for axis, key in ((axes[2], 'impact_force_N'),
                              (axes[3], 'impact_penetration_mm')):
                values = impact[key]
                if values is not None:
                    peak = int(np.argmax(values))
                    axis.plot(impact_t[peak], values[peak], 'o',
                              color='tab:orange', ms=3, zorder=3)

        energy = summary['entry_normal_kinetic_energy_J']
        if energy is not None:
            label = 'Complete cycle' if complete else 'Incomplete cycle'
            energy_axis.scatter(summary['precontact_sample_s'], 1e3 * energy,
                                marker='o' if complete else 'x', s=30,
                                color='tab:blue' if complete else 'tab:red',
                                label=label if label not in energy_labels else None)
            energy_labels.add(label)
            energy_axis.annotate(str(number), (summary['precontact_sample_s'], 1e3 * energy),
                                 xytext=(4, 5), textcoords='offset points', fontsize=8)

    if not has_impulse:
        note = ('Normal speed not logged' if impacts and signals['normal_speed_mps'] is None
                else 'No contact cycle available')
        impulse_axis.text(0.5, 0.5, note, transform=impulse_axis.transAxes,
                          ha='center', va='center')
    if energy_labels:
        if 'Incomplete cycle' in energy_labels:
            energy_axis.legend(loc='upper left', fontsize=8)
        energy_axis.margins(y=0.2)
    else:
        energy_axis.text(0.5, 0.5, 'Pre-contact energy unavailable',
                         transform=energy_axis.transAxes, ha='center', va='center')

    # Match the first-impact labels, once per panel, across contact metrics.
    # In particular, mass/speed are pre-contact values, not full-run extrema.
    labels = (
        ('entry_mass_kg', 'Before contact', 'kg', 3, 1.0),
        ('entry_speed_mps', 'Before contact', 'm/s', 5, 1.0),
        ('peak_force_N', 'Peak', 'N', 3, 1.0),
        ('peak_penetration_mm', 'Maximum', 'mm', 3, 1.0),
        ('normal_impulse_Ns', 'Impulse', 'N s', 4, 1.0),
        ('entry_normal_kinetic_energy_J', 'Energy', 'mJ', 3, 1e3),
    )
    for axis, (key, label, unit, decimals, factor) in zip(axes, labels):
        available = (impact['metrics'] for impact in impacts
                     if impact['metrics'][key] is not None
                     and np.isfinite(impact['metrics'][key]))
        selected = max(available, key=lambda summary: abs(summary[key]), default=None)
        value = ('Not available' if selected is None else
                 f'{factor * selected[key]:.{decimals}f} {unit}')
        if selected is not None and not selected['first_impact_complete']:
            value += ' (partial)'
        axis.text(0.98, 0.95, f'{label}: {value}', transform=axis.transAxes,
                  ha='right', va='top', fontsize=9,
                  bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': 'none'})

    for axis in axes:
        if t.size > 1:
            axis.set_xlim(t[0], t[-1])
        axis.grid(alpha=0.3)
        axis.tick_params(labelsize=9)
    for axis in axes[4:]:
        axis.set_xlabel('Simulation time [s]')
    if impacts:
        complete_count = sum(bool(impact['metrics']['first_impact_complete']) for impact in impacts)
        subtitle = (f'{len(impacts)} recorded contacts; {complete_count} complete first cycles, '
                    f'{len(impacts) - complete_count} incomplete\n'
                    'Gold dashed: contact onset; orange: pre-contact samples / cycle peaks; '
                    'impulse resets at each contact\n'
                    'Numeric labels: largest magnitude across contact metrics')
    else:
        subtitle = ('No contact recorded' if signals['normal_force_N'] is not None
                    else 'Contact force not logged')
    fig.suptitle('Circular experiment: all wall contacts', fontsize=14, y=0.99)
    fig.text(0.5, 0.955, subtitle, ha='center', va='top', fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.90), h_pad=2.0)
    return fig
