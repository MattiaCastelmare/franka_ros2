"""The Phase 5 tables of franka_simulation/docs/gz_cbf_experiments_report.md, from a finished batch.

``write(batch_dir)`` rebuilds ONLY the text between ``<!-- AUTO:BEGIN -->``
and ``<!-- AUTO:END -->``; everything outside it (findings, proposals,
deviations) is hand-written and kept. Every table is SIM. The offline column
(cbf_scenarios.py, column 1) is computed here and cached in
``<batch>/offline.json``.

Aggregation over seeds: numbers are the MEDIAN with [min, max] where it
matters; booleans are counts "k/N".
"""
from __future__ import annotations

import csv
import json
import os
import time

import numpy as np

from . import scenario_catalog, sim_config

BEGIN, END = '<!-- AUTO:BEGIN -->', '<!-- AUTO:END -->'
CONDS = ('residual', 'kf', 'imm', 'imm+pred')


# ── reading ──────────────────────────────────────────────────────────────────

def _val(s):
    if s in ('', 'None', 'nan', None):
        return None
    if s in ('True', 'False'):
        return s == 'True'
    try:
        return float(s)
    except ValueError:
        return s


def load_rows(batch_dir: str) -> list[dict]:
    with open(os.path.join(batch_dir, 'cells.csv')) as f:
        rows = [{k: _val(v) for k, v in r.items()} for r in csv.DictReader(f)]
    for r in rows:                      # rows written before batch._flat knew this
        if r.get('condition') == 'residual':
            r['trk_rms'] = None
    return rows


def select(rows, **kw):
    return [r for r in rows if all(r.get(k) == v for k, v in kw.items())]


def _nums(rows, key):
    return np.array([r[key] for r in rows if isinstance(r.get(key), (int, float))
                     and not isinstance(r.get(key), bool) and np.isfinite(r[key])], float)


def med(rows, key, fmt='{:+.3f}', spread=False):
    x = _nums(rows, key)
    if not x.size:
        return '–'
    s = fmt.format(float(np.median(x)))
    if spread and x.size > 1 and x.max() > x.min():
        s += f' [{fmt.format(float(x.min()))}, {fmt.format(float(x.max()))}]'
    return s


def count(rows, key):
    v = [r.get(key) for r in rows if isinstance(r.get(key), bool)]
    return f'{sum(v)}/{len(v)}' if v else '–'


def total(rows, key):
    x = _nums(rows, key)
    return f'{int(x.sum())}' if x.size else '–'


def causes(rows):
    tot = {}
    for r in rows:
        for kv in (r.get('causes') or '').split(';'):
            if ':' in kv:
                k, v = kv.rsplit(':', 1)
                tot[k] = tot.get(k, 0) + int(float(v))
    if not tot:
        return '–'
    short = {'- ok': 'ok', '1 not-in-rows': 'no-rows', '2 untracked': 'untracked',
             '3 velocity-untrusted': 'untrusted', '4 too-late': 'late'}
    return ' '.join(f'{short.get(k, k)} {v}' for k, v in sorted(tot.items()))


def table(header, rows):
    out = ['| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
    out += ['| ' + ' | '.join(str(c) for c in r) + ' |' for r in rows]
    return '\n'.join(out)


# ── column 1 ─────────────────────────────────────────────────────────────────

#: cbf_scenarios keeps ONE frame in flight: a frame captured while the previous
#: one is undelivered replaces it and pushes the delivery time out by another
#: t_cam. With t_cam = 13 ms that never delivers anything above 1/0.013 = 77 fps
#: (at 90 fps last_msg stays None: no rows, no barrier; measured min h ≈ −0.25 m
#: in every scenario). 60 fps is the highest rate it admits, 30 is what it ships.
OFFLINE_FPS = (30.0, 60.0)


def offline_column(batch_dir: str, names, fps_list=OFFLINE_FPS) -> dict:
    path = os.path.join(batch_dir, 'offline.json')
    if os.path.isfile(path):
        with open(path) as f:
            return json.load(f)
    from . import offline
    out = {}
    for cond in ('residual', 'kf'):
        for fps in fps_list:
            res = offline.run(list(names), cond, fps=fps)
            for n, r in res.items():
                out[f'{n}|{cond}|{fps:g}'] = {k: r[k] for k in (
                    'min_h', 'neg_duration_s', 'contact', 'task_done', 'task_final_err',
                    'jerk_p99', 'slack_max', 'fallback_levels', 'livelock_escapes', 'flags')}
    with open(path, 'w') as f:
        json.dump(out, f, indent=1, default=float)
    return out


# ── the tables ───────────────────────────────────────────────────────────────

def t_quality(rows):
    body = []
    for perc, label in (('true', 'full chain'), ('emulated', 'emulated perception')):
        rr = select(rows, perception=perc)
        if not rr:
            continue
        body.append([label, len(rr), sum(1 for r in rr if r.get('ok')),
                     med(rr, 'rtf', '{:.2f}'), f"{np.percentile(_nums(rr, 'rtf'), 10):.2f}",
                     f"{_nums(rr, 'rtf').min():.2f}",
                     med(rr, 'qp_rate_hz', '{:.0f}'),
                     f"{np.percentile(_nums(rr, 'qp_rate_hz'), 10):.0f}",
                     sum(1 for r in rr if r.get('reflex') == 'PASS'),
                     total(rr, 'reflex_trips'), total(rr, 'rate_clips'),
                     sum(1 for r in rr if isinstance(r.get('rtf'), float) and r['rtf'] < 0.9),
                     med(rr, 'rows_hz', '{:.0f}'), med(rr, 'rows_lat_p50', '{:.0f}'),
                     med(rr, 'rows_lat_p95', '{:.0f}'), med(rr, 'capage_p50', '{:.0f}'),
                     med(rr, 'capage_p95', '{:.0f}')])
    return table(['cells', 'N', 'ok', 'RTF median', 'RTF p10', 'RTF min', 'QP Hz median',
                  'QP Hz p10', 'reflex PASS', 'reflex trips', 'τ-rate clips', 'RTF < 0.9',
                  'rows Hz', 'capture→rows p50 [ms]', 'p95', 'CBF sensing age p50 [ms]',
                  'p95'], body)


def t_estimator(rows, names):
    body = []
    for n in names:
        for c in CONDS:
            rr = select(rows, scenario=n, condition=c, perception='true')
            if not rr or not _nums(rr, 'est_pool_n').size:
                continue
            body.append([n, c, 'residual' if c == 'residual' else 'tracker',
                         med(rr, 'est_rms', '{:.3f}'), med(rr, 'est_bias', '{:+.3f}'),
                         med(rr, 'est_lag_ms', '{:+.0f}'), med(rr, 'est_noise', '{:.3f}'),
                         med(rr, 'est_pool_rms', '{:.3f}'), med(rr, 'est_pool_bias', '{:+.3f}'),
                         med(rr, 'est_pool_n', '{:.0f}'),
                         med(rr, 'res_rms', '{:.3f}'), med(rr, 'trk_rms', '{:.3f}')])
    return table(['scenario', 'condition', 'CBF uses', 'RMS [m/s]', 'bias [m/s]', 'lag [ms]',
                  'noise floor [m/s]', 'pooled RMS', 'pooled bias', 'pooled n',
                  'RMS residual', 'RMS tracker'], body)


def t_tracks(rows, names):
    body = []
    for n in names:
        for c in CONDS[1:]:
            rr = select(rows, scenario=n, condition=c, perception='true')
            if not rr or not _nums(rr, 'est_n').size:
                continue
            body.append([n, c, med(rr, 'confirm_ms', '{:.0f}'), med(rr, 'trust_ms', '{:.0f}'),
                         med(rr, 'n_ids', '{:.0f}'), total(rr, 'id_switches'),
                         med(rr, 'sig_k1', '{:.2f}'), med(rr, 'sig_k2', '{:.2f}'),
                         med(rr, 'sig_k3', '{:.2f}')])
    return table(['scenario', 'condition', 'first row → track [ms]', 'first row → trusted [ms]',
                  'track ids', 'id switches (Σ)', '≤1σ', '≤2σ', '≤3σ'], body)


def t_closed(rows, names, perception='true'):
    body = []
    for n in names:
        for c in CONDS:
            rr = select(rows, scenario=n, condition=c, perception=perception)
            if not rr:
                continue
            body.append([n, c, len(rr), med(rr, 'min_h', spread=True), med(rr, 'neg_s', '{:.2f}'),
                         count(rr, 'contact'), count(rr, 'task_done'),
                         med(rr, 'slack_obs', '{:.2f}'), med(rr, 'slack_cap', '{:.2f}'),
                         total(rr, 'fb_box') + '/' + total(rr, 'fb_brake'), total(rr, 'escapes'),
                         med(rr, 'jerk_p99', '{:.0f}'), total(rr, 'reflex_trips'),
                         med(rr, 'rtf', '{:.2f}'), med(rr, 'qp_rate_hz', '{:.0f}'), causes(rr)])
    return table(['scenario', 'condition', 'N', 'min h [m]', 'h<0 [s]', 'contact', 'task done',
                  'slack obs', 'slack cap', 'fallback box/brake', 'escapes', 'jerk p99',
                  'reflex trips', 'RTF', 'QP Hz', 'causes (passes)'], body)


def t_three(rows, off, names):
    body = []
    for n in names:
        for src, cond in (('residual', 'residual'), ('tracker', 'kf')):
            o30, o60 = off.get(f'{n}|{cond}|30'), off.get(f'{n}|{cond}|60')
            e30 = [r for r in select(rows, scenario=n, condition=cond, perception='emulated')
                   if r.get('emulated_fps') == 30]
            e90 = [r for r in select(rows, scenario=n, condition=cond, perception='emulated')
                   if r.get('emulated_fps') in (None, 90)]
            fc = select(rows, scenario=n, condition=cond, perception='true')

            def o(x, k, fmt='{:+.3f}'):
                if x is None:
                    return '–'
                v = x[k]
                return ('1/1' if v else '0/1') if isinstance(v, bool) else fmt.format(v)
            body.append([n, src,
                         o(o30, 'min_h'), med(e30, 'min_h'), o(o60, 'min_h'), med(e90, 'min_h'),
                         med(fc, 'min_h'),
                         o(o30, 'neg_duration_s', '{:.2f}'), med(e30, 'neg_s', '{:.2f}'),
                         med(e90, 'neg_s', '{:.2f}'), med(fc, 'neg_s', '{:.2f}'),
                         o(o30, 'contact'), count(e30, 'contact'), count(e90, 'contact'),
                         count(fc, 'contact'),
                         o(o30, 'task_done'), count(e30, 'task_done'), count(e90, 'task_done'),
                         count(fc, 'task_done'), causes(e90), causes(fc)])
    return table(['scenario', 'v_obs source', 'min h off@30', 'min h emul@30', 'min h off@60',
                  'min h emul@90', 'min h full', 'h<0 off@30', 'h<0 emul@30', 'h<0 emul@90',
                  'h<0 full', 'contact off@30', 'contact emul@30', 'contact emul@90',
                  'contact full', 'task off@30', 'task emul@30', 'task emul@90', 'task full',
                  'causes emul@90', 'causes full'], body)


def t_pentagon(rows, names):
    body = []
    for n in names:
        for c in CONDS:
            rr = select(rows, scenario=n, condition=c, perception='true')
            if rr:
                body.append([n, c, len(rr), med(rr, 'ee_p95', '{:.3f}'), med(rr, 'min_h', spread=True),
                             count(rr, 'contact'), med(rr, 'neg_s', '{:.2f}'),
                             med(rr, 'jerk_p99', '{:.0f}'), total(rr, 'escapes'),
                             total(rr, 'reflex_trips'), med(rr, 'rtf', '{:.2f}'), causes(rr)])
    return table(['scenario', 'condition', 'N', 'EE err p95 [m]', 'min h [m]', 'contact',
                  'h<0 [s]', 'jerk p99', 'escapes', 'reflex trips', 'RTF', 'causes'], body)


def t_throws(rows, names):
    body = []
    for n in names:
        for c in CONDS:
            rr = select(rows, scenario=n, condition=c, perception='true')
            if rr:
                body.append([n, c, len(rr), med(rr, 'min_h', spread=True), count(rr, 'contact'),
                             med(rr, 'confirm_ms', '{:.0f}'), med(rr, 'trust_ms', '{:.0f}'),
                             med(rr, 'est_pool_rms', '{:.2f}'), causes(rr)])
    return table(['scenario', 'condition', 'N', 'min h [m]', 'contact', 'row → track [ms]',
                  'row → trusted [ms]', 'v_obs RMS pooled [m/s]', 'causes (passes)'], body)


def t_flags(rows, flag_rows):
    body = []
    for r in sorted(flag_rows, key=lambda r: (r['sets'], r['scenario'])):
        base = [b for b in select(rows, scenario=r['scenario'], condition=r['condition'],
                                  perception='true') if b.get('seed') == r.get('seed')]
        b = base[0] if base else {}

        def f(x, k, fmt='{:+.3f}'):
            v = x.get(k)
            if isinstance(v, bool):
                return 'yes' if v else 'no'
            return fmt.format(v) if isinstance(v, (int, float)) else '–'
        body.append([r['sets'], r['scenario'], f(b, 'min_h'), f(r, 'min_h'), f(b, 'neg_s', '{:.2f}'),
                     f(r, 'neg_s', '{:.2f}'), f(b, 'contact'), f(r, 'contact'),
                     f(b, 'jerk_p99', '{:.0f}'), f(r, 'jerk_p99', '{:.0f}'),
                     causes([b]) if b else '–', causes([r])])
    return table(['flag', 'scenario', 'min h kf', 'min h flag', 'h<0 kf', 'h<0 flag',
                  'contact kf', 'contact flag', 'jerk p99 kf', 'jerk p99 flag', 'causes kf',
                  'causes flag'], body)


def build(batch_dir: str) -> str:
    allrows = load_rows(batch_dir)
    rows = [r for r in allrows if not r.get('sets')]
    flag_rows = [r for r in allrows if r.get('sets')]
    cfg = scenario_catalog.load()
    cat = list(cfg['catalog'])
    pent = list(cfg['pentagon']['scenarios'])
    throws = [n for n in scenario_catalog.throw_names(cfg)
              if any(r['scenario'] == n for r in rows)]
    off = offline_column(batch_dir, cat)
    man = {}
    p = os.path.join(batch_dir, 'batch_manifest.json')
    if os.path.isfile(p):
        with open(p) as f:
            man = json.load(f)
    git = man.get('git') or sim_config.git_provenance()
    parts = [
        BEGIN,
        f'_Generated by `run_gz_scenarios --report` on {time.strftime("%Y-%m-%d %H:%M")} '
        f'from `{os.path.relpath(batch_dir, sim_config.sim_runs_root())}` '
        f'({len(rows)} cells, git `{(git.get("sha") or "?")[:7]}`'
        f'{" dirty" if git.get("dirty") else ""}). **Every number below is SIM.**_',
        '', '### A. Run quality (SIM)', '', t_quality(rows), '',
        '### B. Estimator vs truth, full chain (SIM, Phase 5.1)', '',
        'Per scenario and condition, median over seeds. "CBF uses" is the estimator the '
        'condition feeds the barrier; RMS / bias / lag are of that one, against n̂ᵀv_true on the '
        'samples whose obstacle point lies on the sphere, per control point with >= 30 samples '
        '(compare_vobs); "pooled" = over the on-obstacle samples of every control point (a fast '
        'ball is in front of each one for a few frames only). Lag: NEGATIVE = the estimate trails. '
        'The last two columns score both estimators on the same run (the residual is recomputed '
        'from the run\'s own rows). Noise floor: compare_vobs static windows on a static obstacle.',
        '', t_estimator(rows, cat + pent + throws), '',
        '#### B.2 Tracks: confirmation, identity, σ calibration (SIM)', '',
        'σ = sqrt(n̂ᵀP_vv n̂) from the message; share of |v_trk − v_true| ≤ kσ on rows the CBF '
        'trusts (Gaussian 0.68 / 0.95 / 1.00).', '', t_tracks(rows, cat + pent + throws), '',
        '### C. Closed loop, full chain (SIM, Phase 5.2)', '',
        'min h from the TRUE gaps of the real control-point set (h = gap − d_safe, contact = gap '
        '< 0); slack per family from the CBFDIAG lines (sampled at diag_period_s); fallback = QP '
        'ticks at box-only / brake in the episode; causes = approach_probe.py causes over '
        'ball_throw_eval.py score passes of the moving obstacles (residual runs: rows and CBF lead '
        'only, approach_probe\'s track causes do not apply with tracking off).', '',
        t_closed(rows, cat), '',
        '### D. Three columns: offline / Gazebo + emulated perception / Gazebo full chain (SIM, '
        'Phase 5.3)', '',
        'off = cbf_scenarios.py (double integrator, emulated perception) at 30 fps as shipped '
        'and at 60 fps, the highest rate its one-frame-in-flight delivery admits with t_cam = '
        '13 ms (at 90 fps it delivers nothing). emul = the Gazebo plant and the whole stack with '
        'cbf_scenarios\' perception emulation (emulated_perception) at 30 fps (seed 0: same '
        'rate as off@30, so the gap is the plant) and at 90 fps (the simulated chain\'s rate, so '
        'emul@90 → full is the perception). full = simulated D455 → real_time_distance → '
        'tracker (kf for the tracker source). Counts are over seeds.', '', t_three(rows, off, cat), '',
        '### E. Pentagon task with obstacles (SIM)', '', t_pentagon(rows, pent), '',
        '### F. Throws (SIM)', '', t_throws(rows, throws), '',
        *(['### G. kf with one track-consuming CBF flag off (SIM, seed 0, optional cross)', '',
           t_flags(rows, flag_rows), ''] if flag_rows else []),
        END]
    return '\n'.join(parts)


def default_report_path() -> str:
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.join(here, '..', '..', '..', 'franka_simulation', 'docs'),
                 '/ros2_ws/src/franka_simulation/docs'):
        if os.path.isdir(cand):
            return os.path.normpath(os.path.join(cand, 'gz_cbf_experiments_report.md'))
    return os.path.join(os.getcwd(), 'gz_cbf_experiments_report.md')


def write(batch_dir: str, path: str | None = None) -> str:
    path = path or default_report_path()
    auto = build(batch_dir)
    if os.path.isfile(path):
        with open(path) as f:
            old = f.read()
        if BEGIN in old and END in old:
            new = old[:old.index(BEGIN)] + auto + old[old.index(END) + len(END):]
        else:
            new = old.rstrip() + '\n\n' + auto + '\n'
    else:
        new = '# Gazebo CBF experiments: report (SIM)\n\n' + auto + '\n'
    with open(path, 'w') as f:
        f.write(new)
    return path
