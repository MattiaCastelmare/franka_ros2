#!/usr/bin/env python3
"""Analyse and plot a Franka pentagon-commander CSV log.

Reads a CSV produced by ``pentagon_qddot_commander`` (see ``_init_logger`` in
that node) and generates, in a fresh ``plots_<timestamp>/`` folder:

  * end-effector position / velocity / acceleration tracking figures,
  * per-joint position / velocity / acceleration / torque tracking figures,
  * a tracking-statistics CSV (RMS / mean / max / std of every error signal),
  * a summary dashboard of the global controller signals,
  * a ``report.md`` with the headline numbers and links to every image.

For every quantity the layout is:

    ┌───────────────────────────┬─────────────────────┐
    │ Desired vs Actual         │ Error (RMS/max/mean) │
    └───────────────────────────┴─────────────────────┘

Velocities and accelerations that are not measured directly are reconstructed
by numerical differentiation (optionally Savitzky–Golay smoothed via SciPy).

Usage
-----
    python3 plot_franka_log.py pentagon_run.csv
    python3 plot_franka_log.py pentagon_run.csv --outdir my_plots --no-smooth
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use('Agg')  # headless: render straight to PNG, no display needed
import matplotlib.pyplot as plt  # noqa: E402

# SciPy is used only to smooth numerically-differentiated signals; optional.
try:
    from scipy.signal import savgol_filter
    _HAVE_SCIPY = True
except Exception:  # noqa: BLE001
    _HAVE_SCIPY = False


# ── Global style ───────────────────────────────────────────────────────────
NUM_JOINTS = 7
DPI = 300
WIDE = (16, 8)            # base "large figure" size
C_DES = '#1f77b4'        # desired  → blue   (consistent everywhere)
C_ACT = '#ff7f0e'        # actual   → orange
C_ERR = '#d62728'        # error    → red

plt.style.use('default')
plt.rcParams.update({
    'font.size':        12,
    'axes.titlesize':   13,
    'axes.labelsize':   12,
    'legend.fontsize':  11,
    'xtick.labelsize':  11,
    'ytick.labelsize':  11,
    'figure.titlesize': 17,
    'axes.grid':        True,
    'grid.alpha':       0.30,
    'lines.linewidth':  1.6,
    'figure.autolayout': False,   # we call tight_layout() explicitly
})


# ── Small numeric helpers ──────────────────────────────────────────────────
def error_stats(err: np.ndarray):
    """Return (RMS, mean-abs, max-abs, std) of an error array."""
    err = np.asarray(err, dtype=float)
    err = err[np.isfinite(err)]
    if err.size == 0:
        return 0.0, 0.0, 0.0, 0.0
    rms = float(np.sqrt(np.mean(err ** 2)))
    mae = float(np.mean(np.abs(err)))
    mxe = float(np.max(np.abs(err)))
    std = float(np.std(err))
    return rms, mae, mxe, std


def numerical_derivative(x, t, smooth=True, window=21, poly=3):
    """d/dt of *x* sampled at times *t* (handles non-uniform spacing).

    Optionally smoothed with a Savitzky–Golay filter to tame differentiation
    noise — useful when reconstructing velocity/acceleration from position.
    """
    x = np.asarray(x, dtype=float)
    t = np.asarray(t, dtype=float)
    d = np.gradient(x, t)
    if smooth and _HAVE_SCIPY and d.size > 7:
        w = min(window, d.size)
        if w % 2 == 0:               # savgol needs an odd window
            w -= 1
        if w >= poly + 2:
            d = savgol_filter(d, w, poly)
    return d


def col(df: pd.DataFrame, name: str, n: int) -> np.ndarray:
    """Column *name* as float array, or zeros if absent."""
    if name in df.columns:
        return df[name].to_numpy(dtype=float)
    return np.zeros(n)


def has_signal(df: pd.DataFrame, name: str) -> bool:
    """True if column exists and is not entirely zero/NaN."""
    if name not in df.columns:
        return False
    v = df[name].to_numpy(dtype=float)
    return bool(np.any(np.isfinite(v) & (v != 0.0)))


# ── Tracking-figure factory ────────────────────────────────────────────────
class Report:
    """Accumulates stat rows + image links while figures are produced."""

    def __init__(self, outdir: Path):
        self.outdir = outdir
        self.stats: list[dict] = []     # rows for tracking_statistics.csv
        self.images: list[str] = []     # generated PNG filenames (relative)
        self.err: dict[str, np.ndarray] = {}  # signal → error array (for report)

    def register(self, signal: str, err: np.ndarray):
        rms, mae, mxe, std = error_stats(err)
        self.stats.append({
            'Signal':         signal,
            'RMS':            rms,
            'Mean Abs Error': mae,
            'Max Abs Error':  mxe,
            'Std Error':      std,
        })
        self.err[signal] = np.asarray(err, dtype=float)
        return rms, mae, mxe, std

    def tracking_figure(self, t, rows, suptitle, fname, time_label='Time [s]'):
        """One figure, one row per quantity: left = des vs act, right = error.

        *rows* is a list of dicts with keys:
            signal  – name used in the statistics table
            label   – human-readable label for titles
            desired – array
            actual  – array
            unit    – y-axis unit string (optional)
        """
        n = len(rows)
        fig, axes = plt.subplots(
            n, 2, figsize=(WIDE[0], max(4.0 * n, 6.0)), squeeze=False)
        for r, row in enumerate(rows):
            des = np.asarray(row['desired'], dtype=float)
            act = np.asarray(row['actual'], dtype=float)
            err = des - act
            rms, mae, mxe, _ = self.register(row['signal'], err)
            unit = row.get('unit', '')

            axL, axR = axes[r][0], axes[r][1]
            axL.plot(t, des, color=C_DES, label='Desired')
            axL.plot(t, act, color=C_ACT, label='Actual', alpha=0.85)
            axL.set_title(f"{row['label']} — Desired vs Actual")
            axL.set_xlabel(time_label)
            axL.set_ylabel(unit)
            axL.legend(loc='best')

            axR.plot(t, err, color=C_ERR, label='Error')
            axR.axhline(0.0, color='k', lw=0.8, alpha=0.5)
            axR.set_title(
                f"{row['label']} Error   "
                f"RMS={rms:.4g}  max={mxe:.4g}  mean={mae:.4g}")
            axR.set_xlabel(time_label)
            axR.set_ylabel(f'Error {unit}'.strip())
            axR.legend(loc='best')

        fig.suptitle(suptitle, fontweight='bold')
        fig.tight_layout(rect=(0, 0, 1, 0.98))
        out = self.outdir / fname
        fig.savefig(out, dpi=DPI)
        plt.close(fig)
        self.images.append(fname)


# ── Main pipeline ──────────────────────────────────────────────────────────
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Plot a Franka pentagon CSV log.')
    ap.add_argument('csv', help='Path to the CSV log file.')
    ap.add_argument('--outdir', default=None,
                    help='Output folder (default: plots_<timestamp>/).')
    ap.add_argument('--no-smooth', action='store_true',
                    help='Disable Savitzky–Golay smoothing of derivatives.')
    args = ap.parse_args(argv)

    csv_path = Path(args.csv)
    if not csv_path.is_file():
        print(f'ERROR: file not found: {csv_path}', file=sys.stderr)
        return 1

    df = pd.read_csv(csv_path)
    if 'time' not in df.columns or len(df) < 3:
        print('ERROR: CSV needs a "time" column and at least 3 rows.',
              file=sys.stderr)
        return 1

    n = len(df)
    t = df['time'].to_numpy(dtype=float)
    t = t - t[0]                          # start the time axis at 0 s
    smooth = not args.no_smooth

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    outdir = Path(args.outdir) if args.outdir else \
        csv_path.resolve().parent / f'plots_{stamp}'
    outdir.mkdir(parents=True, exist_ok=True)
    print(f'Reading : {csv_path}  ({n} rows)')
    print(f'Output  : {outdir}')
    if not _HAVE_SCIPY:
        print('NOTE: SciPy not available — derivatives left unsmoothed.')

    rep = Report(outdir)

    def deriv(x):
        return numerical_derivative(x, t, smooth=smooth)

    # ── 1. End-effector position ───────────────────────────────────────────
    rep.tracking_figure(
        t,
        [
            {'signal': 'ee_x', 'label': 'EE X', 'unit': '[m]',
             'desired': col(df, 'ee_x_des', n), 'actual': col(df, 'ee_x', n)},
            {'signal': 'ee_y', 'label': 'EE Y', 'unit': '[m]',
             'desired': col(df, 'ee_y_des', n), 'actual': col(df, 'ee_y', n)},
            {'signal': 'ee_z', 'label': 'EE Z', 'unit': '[m]',
             'desired': col(df, 'ee_z_des', n), 'actual': col(df, 'ee_z', n)},
        ],
        'End-Effector Position Tracking', 'ee_position_tracking.png')

    # ── 2. End-effector velocity (actual = d/dt position) ──────────────────
    ee_vx = deriv(col(df, 'ee_x', n))
    ee_vy = deriv(col(df, 'ee_y', n))
    ee_vz = deriv(col(df, 'ee_z', n))
    rep.tracking_figure(
        t,
        [
            {'signal': 'ee_vx', 'label': 'EE Vx', 'unit': '[m/s]',
             'desired': col(df, 'vx_des', n), 'actual': ee_vx},
            {'signal': 'ee_vy', 'label': 'EE Vy', 'unit': '[m/s]',
             'desired': col(df, 'vy_des', n), 'actual': ee_vy},
            {'signal': 'ee_vz', 'label': 'EE Vz', 'unit': '[m/s]',
             'desired': col(df, 'vz_des', n), 'actual': ee_vz},
        ],
        'End-Effector Velocity Tracking  (actual = d/dt position)',
        'ee_velocity_tracking.png')

    # ── 3. End-effector acceleration (actual = d/dt velocity) ──────────────
    ee_ax = deriv(ee_vx)
    ee_ay = deriv(ee_vy)
    ee_az = deriv(ee_vz)
    rep.tracking_figure(
        t,
        [
            {'signal': 'ee_ax', 'label': 'EE Ax', 'unit': '[m/s²]',
             'desired': col(df, 'ax_des', n), 'actual': ee_ax},
            {'signal': 'ee_ay', 'label': 'EE Ay', 'unit': '[m/s²]',
             'desired': col(df, 'ay_des', n), 'actual': ee_ay},
            {'signal': 'ee_az', 'label': 'EE Az', 'unit': '[m/s²]',
             'desired': col(df, 'az_des', n), 'actual': ee_az},
        ],
        'End-Effector Acceleration Tracking  (actual = d²/dt² position)',
        'ee_acceleration_tracking.png')

    # ── 4–7. Per-joint figures ─────────────────────────────────────────────
    torque_present = any(
        has_signal(df, f'tau_des_{i}') or has_signal(df, f'tau_meas_{i}')
        for i in range(1, NUM_JOINTS + 1))

    for i in range(1, NUM_JOINTS + 1):
        # 4. position
        rep.tracking_figure(
            t,
            [{'signal': f'joint{i}_position', 'label': f'Joint {i} position',
              'unit': '[rad]',
              'desired': col(df, f'q_des_{i}', n), 'actual': col(df, f'q{i}', n)}],
            f'Joint {i} — Position Tracking', f'joint{i}_position.png')

        # 5. velocity
        rep.tracking_figure(
            t,
            [{'signal': f'joint{i}_velocity', 'label': f'Joint {i} velocity',
              'unit': '[rad/s]',
              'desired': col(df, f'dq_des_{i}', n), 'actual': col(df, f'dq{i}', n)}],
            f'Joint {i} — Velocity Tracking', f'joint{i}_velocity.png')

        # 6. acceleration (actual = d/dt measured joint velocity)
        qddot_act = deriv(col(df, f'dq{i}', n))
        rep.tracking_figure(
            t,
            [{'signal': f'joint{i}_acceleration', 'label': f'Joint {i} acceleration',
              'unit': '[rad/s²]',
              'desired': col(df, f'qddot_des_{i}', n), 'actual': qddot_act}],
            f'Joint {i} — Acceleration Tracking  (actual = d/dt measured velocity)',
            f'joint{i}_acceleration.png')

        # 7. torque (only if measured/desired torque columns carry data)
        if torque_present:
            rep.tracking_figure(
                t,
                [{'signal': f'joint{i}_torque', 'label': f'Joint {i} torque',
                  'unit': '[N·m]',
                  'desired': col(df, f'tau_des_{i}', n),
                  'actual':  col(df, f'tau_meas_{i}', n)}],
                f'Joint {i} — Torque Tracking', f'joint{i}_torque.png')

    # ── 8. Tracking statistics CSV ─────────────────────────────────────────
    stats_df = pd.DataFrame(
        rep.stats,
        columns=['Signal', 'RMS', 'Mean Abs Error', 'Max Abs Error', 'Std Error'])
    stats_path = outdir / 'tracking_statistics.csv'
    stats_df.to_csv(stats_path, index=False)
    print(f'Stats   : {stats_path.name}  ({len(stats_df)} signals)')

    # ── 9. Summary dashboard ───────────────────────────────────────────────
    make_summary_dashboard(df, t, n, outdir, rep)

    # ── 11. Markdown report ────────────────────────────────────────────────
    write_report(rep, stats_df, csv_path, outdir, n, t)

    print(f'Done. {len(rep.images)} figures + stats + report in {outdir}')
    return 0


def _norm_rows(*arrays) -> np.ndarray:
    """Per-sample Euclidean norm of several equal-length arrays."""
    stack = np.vstack([np.asarray(a, dtype=float) for a in arrays])
    return np.sqrt(np.sum(stack ** 2, axis=0))


def make_summary_dashboard(df, t, n, outdir, rep: Report):
    """Eight time-synchronised global controller signals on one figure."""
    qddot_norm = _norm_rows(*[col(df, f'qddot_des_{i}', n)
                              for i in range(1, NUM_JOINTS + 1)])
    # Prefer desired torque for the norm; fall back to measured.
    if any(has_signal(df, f'tau_des_{i}') for i in range(1, NUM_JOINTS + 1)):
        tau_norm = _norm_rows(*[col(df, f'tau_des_{i}', n)
                                for i in range(1, NUM_JOINTS + 1)])
        tau_lbl = '‖τ_des‖'
    else:
        tau_norm = _norm_rows(*[col(df, f'tau_meas_{i}', n)
                                for i in range(1, NUM_JOINTS + 1)])
        tau_lbl = '‖τ_meas‖'

    if has_signal(df, 'ee_error_norm'):
        ee_norm = col(df, 'ee_error_norm', n)
    else:  # reconstruct from component errors if the column is missing
        ee_norm = _norm_rows(col(df, 'ex', n), col(df, 'ey', n), col(df, 'ez', n))

    panels = [
        ('Cartesian error norm ‖e‖ [m]', ee_norm,                 'C3'),
        ('Manipulability w',             col(df, 'manipulability', n), 'C0'),
        ('Damping λ²',                   col(df, 'lambda_sq', n),  'C1'),
        ('Commanded ‖q̈_des‖ [rad/s²]',  qddot_norm,               'C2'),
        (f'Torque norm {tau_lbl} [N·m]', tau_norm,                 'C4'),
        ('Phase variable s',             col(df, 's', n),          'C5'),
        ('Phase rate s_dot',             col(df, 's_dot', n),      'C6'),
        ('Phase accel s_ddot',           col(df, 's_ddot', n),     'C8'),
    ]

    fig, axes = plt.subplots(4, 2, figsize=(16, 14), squeeze=False)
    flat = axes.flatten()
    for ax, (title, data, color) in zip(flat, panels):
        ax.plot(t, data, color=color)
        ax.set_title(title)
        ax.set_xlabel('Time [s]')
        # λ² spans orders of magnitude → log scale when strictly positive.
        if title.startswith('Damping') and np.all(data > 0):
            ax.set_yscale('log')
    fig.suptitle('Summary Dashboard — Global Controller Signals',
                 fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(outdir / 'summary_dashboard.png', dpi=DPI)
    plt.close(fig)
    rep.images.append('summary_dashboard.png')


def write_report(rep: Report, stats_df: pd.DataFrame, csv_path: Path,
                 outdir: Path, n: int, t: np.ndarray):
    """Emit report.md with headline numbers and links to every image."""

    def combined_rms(signals):
        """RMS of the Euclidean-norm error across a group of signals."""
        arrs = [rep.err[s] for s in signals if s in rep.err]
        if not arrs:
            return float('nan')
        norm = _norm_rows(*arrs)
        return float(np.sqrt(np.mean(norm ** 2)))

    ee_pos_rms = combined_rms(['ee_x', 'ee_y', 'ee_z'])
    ee_vel_rms = combined_rms(['ee_vx', 'ee_vy', 'ee_vz'])
    ee_acc_rms = combined_rms(['ee_ax', 'ee_ay', 'ee_az'])

    # Worst tracking signal = largest RMS error in the table.
    worst = stats_df.loc[stats_df['RMS'].idxmax()] if len(stats_df) else None

    lines: list[str] = []
    lines.append('# Franka Tracking Report')
    lines.append('')
    lines.append(f'- **Source CSV:** `{csv_path.name}`')
    lines.append(f'- **Samples:** {n}')
    lines.append(f'- **Duration:** {t[-1]:.2f} s '
                 f'(~{(n - 1) / max(t[-1], 1e-9):.1f} Hz)')
    lines.append(f'- **Generated:** {datetime.now().isoformat(timespec="seconds")}')
    lines.append('')

    lines.append('## Headline statistics')
    lines.append('')
    lines.append('| Quantity | RMS error |')
    lines.append('| --- | --- |')
    lines.append(f'| EE position ‖·‖ | {ee_pos_rms:.4g} m |')
    lines.append(f'| EE velocity ‖·‖ | {ee_vel_rms:.4g} m/s |')
    lines.append(f'| EE acceleration ‖·‖ | {ee_acc_rms:.4g} m/s² |')
    lines.append('')

    if worst is not None:
        lines.append('## Worst tracking error')
        lines.append('')
        lines.append(f'- **Signal:** `{worst["Signal"]}`')
        lines.append(f'- **RMS:** {worst["RMS"]:.4g}')
        lines.append(f'- **Max abs error:** {worst["Max Abs Error"]:.4g}')
        lines.append('')

    # Per-joint RMS tables ---------------------------------------------------
    def joint_table(title, suffix, unit):
        rows = []
        for i in range(1, NUM_JOINTS + 1):
            sig = f'joint{i}_{suffix}'
            row = stats_df.loc[stats_df['Signal'] == sig]
            if not row.empty:
                rows.append(f'| {i} | {row["RMS"].iloc[0]:.4g} {unit} |')
        if rows:
            lines.append(f'## {title}')
            lines.append('')
            lines.append('| Joint | RMS error |')
            lines.append('| --- | --- |')
            lines.extend(rows)
            lines.append('')

    joint_table('Per-joint position RMS', 'position', 'rad')
    joint_table('Per-joint velocity RMS', 'velocity', 'rad/s')
    joint_table('Per-joint acceleration RMS', 'acceleration', 'rad/s²')
    joint_table('Per-joint torque RMS', 'torque', 'N·m')

    # Image gallery ----------------------------------------------------------
    lines.append('## Figures')
    lines.append('')
    for img in rep.images:
        lines.append(f'### {img}')
        lines.append('')
        lines.append(f'![{img}]({img})')
        lines.append('')

    (outdir / 'report.md').write_text('\n'.join(lines), encoding='utf-8')


if __name__ == '__main__':
    sys.exit(main())
