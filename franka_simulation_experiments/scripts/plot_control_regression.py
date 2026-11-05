#!/usr/bin/env python3
"""Plot recorded paired SIM regressions; do not discard unsuccessful episodes."""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root')
    args = ap.parse_args()
    root = Path(args.root)
    results = json.loads((root / 'summary.json').read_text())['results']
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names = ('control_clear', 'control_static', 'control_approach', 'control_stick')
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for variant, offset, color in [('baseline', -.18, '#8a919a'), ('corrected', .18, '#197d9b')]:
        rows = [[r['closed_loop'] for r in results
                 if r['variant'] == variant and r['scenario'] == s and r['execution_complete']]
                for s in names]
        values = [
            [[r.get('pentagon', {}).get('p95_m', np.nan)*1000 for r in group] for group in rows],
            [[np.nan if r.get('min_gap') is None else r['min_gap']*1000 for r in group] for group in rows],
            [[sum(r.get('reflex_trips', 0) > 0 for r in group)] for group in rows],
            [[r.get('qp_rate_hz', np.nan) for r in group] for group in rows],
        ]
        for ax, metric in zip(axes.flat, values):
            y, low, high = [], [], []
            for group in metric:
                data = np.array(group, dtype=float)
                data = data[np.isfinite(data)]
                med = float(np.median(data)) if len(data) else np.nan
                y.append(med)
                low.append(med-float(data.min()) if len(data) else 0.)
                high.append(float(data.max())-med if len(data) else 0.)
            valid = np.isfinite(y)
            ax.bar((np.arange(4)+offset)[valid], np.asarray(y)[valid],
                   width=.34, color=color, label=variant,
                   yerr=np.array([low, high])[:, valid], capsize=3, alpha=.9)
    titles = ('TCP tracking error, 95th percentile [mm]',
              'Minimum true CP clearance [mm]',
              'Runs with reflex flags', 'Achieved QP rate [Hz of simulation time]')
    for ax, title in zip(axes.flat, titles):
        ax.set_title(title, loc='left', fontsize=11)
        ax.set_xticks(range(4), ['Clear', 'Static', 'Approach', 'Stick'])
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
        ax.spines[['top', 'right']].set_visible(False)
    axes[0, 1].axhline(150, color='#b53a34', ls='--', lw=1, label='d_safe = 150 mm')
    axes[0, 1].text(.98, 150, 'd_safe = 150 mm', ha='right', va='bottom',
                    transform=axes[0, 1].get_yaxis_transform(), color='#b53a34', fontsize=9)
    axes[0, 1].text(0, 10, 'no obstacle', ha='center', fontsize=9, color='#666666')
    axes[1, 0].set_yticks([0, 1, 2, 3]); axes[1, 0].set_ylim(0, 3.5)
    axes[1, 1].axhline(240, color='#b53a34', ls='--', lw=1, label='240 Hz evidence check')
    axes[1, 1].text(.98, 240, '240 Hz evidence check', ha='right', va='bottom',
                    transform=axes[1, 1].get_yaxis_transform(), color='#b53a34', fontsize=9)
    axes[0, 0].legend(frameon=False)
    fig.suptitle('IRIM2 control corrections — SIM, three seeds per scenario\n'
                 'Bars: median; whiskers: seed range. TCP errors in obstacle scenes include avoidance.', fontsize=13)
    fig.savefig(root / 'comparison.png', dpi=170)
    plt.close(fig)
    print(root / 'comparison.png')


if __name__ == '__main__':
    main()
