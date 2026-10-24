"""The Phase 5 grid: cells (scenario × condition × seed × perception), each a fresh Gazebo.

A cell is one ``gz_cbf_stack.launch.py`` run in ``<batch>/cells/<cell>/``,
scored right after it ends:

* ``closed_loop.summarize_run``   (always)
* ``estimator_metrics.score_run`` (obstacles and a perception that publishes rows)
* ``attribution.attribute_run``   (moving obstacles)

and written to ``<cell>/cell_result.json`` plus one flat row of
``<batch>/cells.csv``. A cell whose result file exists is skipped, so an
interrupted batch resumes where it stopped. Every row carries SIM, the RTF
and the achieved QP rate.
"""
from __future__ import annotations

import csv
import json
import os
import time
import traceback

from . import attribution, closed_loop, estimator_metrics, runner, scenario_catalog, sim_config

CONDITIONS = ('residual', 'kf', 'imm', 'imm+pred')


def cell_name(c: dict) -> str:
    p = {'true': 'gz', 'emulated': 'emul', 'false': 'noperc'}[c['perception']]
    if c['perception'] == 'emulated' and c.get('emulated_fps', 90) != 90:
        p += f'{c["emulated_fps"]:g}'
    cond = c['condition'].replace('+', 'P')
    for k, v in sorted((c.get('sets') or {}).items()):
        cond += f'-{k}={v}'
    return f'{c["scenario"]}__{cond}__{p}__s{c["seed"]}'


def emulated30_grid() -> list[dict]:
    """Column 2 at cbf_scenarios' own 30 fps: plant-only difference to column 1 @30."""
    cfg = scenario_catalog.load()
    return [dict(scenario=s, condition=c, seed=0, perception='emulated', emulated_fps=30)
            for s in cfg['catalog'] for c in ('residual', 'kf')]


def phase5_grid() -> list[dict]:
    """Seed 0 of everything first, so a partial batch is already a full table."""
    cfg = scenario_catalog.load()
    catalog = list(cfg['catalog'])
    pent = list(cfg['pentagon']['scenarios'])
    ball = [n for n in scenario_catalog.throw_names(cfg) if n.startswith('throw_ballistic_')]
    sweep_a = [n for n in scenario_catalog.throw_names(cfg) if n.startswith('throw_a_')]
    cells = []
    for seed in (0, 1, 2):
        for s in catalog:
            cells += [dict(scenario=s, condition=c, seed=seed, perception='true') for c in CONDITIONS]
            cells += [dict(scenario=s, condition=c, seed=seed, perception='emulated')
                      for c in ('residual', 'kf')]
        if seed < 2:
            for s in pent + ball:
                cells += [dict(scenario=s, condition=c, seed=seed, perception='true')
                          for c in CONDITIONS]
        if seed == 0:
            for s in sweep_a:
                cells += [dict(scenario=s, condition=c, seed=0, perception='true')
                          for c in ('residual', 'kf', 'imm+pred')]
    return cells


#: the CBF flags that consume the track (spec Phase 5, optional cross)
TRACK_CONSUMERS = ('vobs_in_hdot', 'uncertainty_margin', 'latency_compensation')


def flags_grid() -> list[dict]:
    """kf with each track-consuming flag OFF in turn, on the scenarios with moving obstacles."""
    scen = ('slow', 'slow_noisy', 'fast_seen_early', 'ball_slow', 'ball', 'throw_ballistic_v3',
            'pentagon_crossing')
    return [dict(scenario=s, condition='kf', seed=0, perception='true', sets={f: False})
            for f in TRACK_CONSUMERS for s in scen]


def _flat(c: dict, r: dict) -> dict:
    cl, est, att = r.get('closed_loop', {}), r.get('estimator', {}), r.get('attribution', {})
    used = est.get('used')
    e = (est.get('residual') if used == 'v_res' else est.get('tracker')) or {}
    tr = est.get('tracks', {})
    po = list((tr.get('per_obstacle') or {}).values())
    causes = {}
    for a in att.values():
        cnt = (attribution.residual_causes(a.get('passes', []), a.get('need_ms', 450.0))
               if c['condition'] == 'residual' else a.get('counts', {}))
        for k, v in cnt.items():
            causes[k] = causes.get(k, 0) + v
    fl = cl.get('fallback_levels') or {}
    sl = cl.get('slack_max') or {}
    return dict(
        SIM=True, cell=cell_name(c),
        **{'emulated_fps': None, **{k: v for k, v in c.items() if k != 'sets'}},
        sets=';'.join(f'{k}={v}' for k, v in sorted((c.get('sets') or {}).items())),
        ok=r.get('ok'), wall_s=r.get('wall_s'),
        rtf=cl.get('rtf'), qp_rate_hz=cl.get('qp_rate_hz'), reflex=cl.get('reflex_verdict'),
        reflex_trips=cl.get('reflex_trips'), rate_clips=cl.get('rate_clips'),
        min_h=cl.get('min_h'), neg_s=cl.get('neg_duration_s'), contact=cl.get('contact'),
        task_done=cl.get('task_done'), task_err=cl.get('task_final_err'),
        jerk_p99=cl.get('jerk_p99'), slack_any=cl.get('slack_max_any'),
        **{f'slack_{k}': sl.get(k) for k in closed_loop.FAMILIES},
        fb_box=fl.get('box-only'), fb_brake=fl.get('brake'), escapes=cl.get('livelock_escapes'),
        ee_p95=(cl.get('pentagon') or {}).get('p95_m'),
        rows_hz=cl.get('rows_rate_hz'), rows_lat_p50=cl.get('rows_latency_ms_p50'),
        rows_lat_p95=cl.get('rows_latency_ms_p95'), capage_p50=cl.get('capage_ms_p50'),
        capage_p95=cl.get('capage_ms_p95'),
        est=used, est_lag_ms=e.get('lag_ms'), est_bias=e.get('bias'), est_rms=e.get('rms'),
        est_pool_bias=e.get('pooled_bias'), est_pool_rms=e.get('pooled_rms'),
        est_pool_n=e.get('pooled_n'),
        est_noise=e.get('noise_floor'), est_n=e.get('n'),
        res_rms=(est.get('residual') or {}).get('rms'),
        # tracking is OFF in the residual condition: nothing to score
        trk_rms=None if c['condition'] == 'residual' else (est.get('tracker') or {}).get('rms'),
        confirm_ms=min((p['confirm_ms'] for p in po), default=None),
        trust_ms=min((p['trust_ms'] for p in po), default=None),
        id_switches=sum(p['id_switches'] for p in po) if po else None,
        n_ids=sum(p['n_ids'] for p in po) if po else None,
        sig_k1=(tr.get('sigma_calibration') or {}).get('k1'),
        sig_k2=(tr.get('sigma_calibration') or {}).get('k2'),
        sig_k3=(tr.get('sigma_calibration') or {}).get('k3'),
        passes=sum(a.get('n_parsed', 0) for a in att.values()) if att else None,
        causes=';'.join(f'{k}:{v}' for k, v in sorted(causes.items())))


def run_one(batch_dir: str, c: dict, *, timeout_s: float = 400.0) -> dict:
    rd = os.path.join(batch_dir, 'cells', cell_name(c))
    res_path = os.path.join(rd, 'cell_result.json')
    if os.path.isfile(res_path):
        with open(res_path) as f:
            return json.load(f)
    os.makedirs(rd, exist_ok=True)
    out = dict(cell=c, SIM=True)
    extra = ({'emulated_fps': f'{c["emulated_fps"]:g}'} if c.get('emulated_fps') else None)
    rr = runner.run_cell(rd, condition=c['condition'], scenario=c['scenario'], seed=c['seed'],
                         perception=c['perception'], timeout_s=timeout_s, extra_args=extra,
                         sets=c.get('sets'))
    out.update(wall_s=rr['wall_s'], timed_out=rr['timed_out'], leftovers=rr['leftovers'],
               returncode=rr['returncode'])
    try:
        out.update(score_cell(rd, c))
        out['ok'] = (not rr['timed_out'] and not rr['leftovers']
                     and out['closed_loop'].get('min_h') is not None)
    except Exception:                                          # noqa: BLE001
        out['ok'] = False
        out['error'] = traceback.format_exc()
    with open(res_path, 'w') as f:
        json.dump(out, f, indent=1, default=_json_default)
    return out


def sim_config_manifest(rd):
    from . import analysis as A
    return A.load_json(os.path.join(rd, 'run_manifest_sim.json')) or {}


def _json_default(o):
    try:
        import numpy as np
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
    except ImportError:
        pass
    return str(o)


def run_grid(batch_dir: str, cells: list[dict], *, log=print) -> str:
    """Run the cells not yet done; cells.csv is rebuilt from EVERY cell of the batch dir,
    so several grids (phase5, emulated30, flags) can share one batch."""
    import glob
    os.makedirs(batch_dir, exist_ok=True)
    mp = os.path.join(batch_dir, 'batch_manifest.json')
    man = {}
    if os.path.isfile(mp):
        with open(mp) as f:
            man = json.load(f)
    man.setdefault('SIM', True)
    man.setdefault('plant', 'gazebo')
    man.setdefault('runs', []).append({'git': sim_config.git_provenance(), 'n_cells': len(cells),
                                       'started': time.strftime('%Y-%m-%d %H:%M:%S'),
                                       'cells': cells})
    man['git'] = man['runs'][-1]['git']
    with open(mp, 'w') as f:
        json.dump(man, f, indent=1)
    csv_path = os.path.join(batch_dir, 'cells.csv')
    t0 = time.time()
    for i, c in enumerate(cells, 1):
        r = run_one(batch_dir, c)
        rows = []
        for p in sorted(glob.glob(os.path.join(batch_dir, 'cells', '*', 'cell_result.json'))):
            with open(p) as f:
                rr = json.load(f)
            rows.append(_flat(rr['cell'], rr))
        write_csv(csv_path, rows)
        cl = r.get('closed_loop', {})
        log(f'[{i}/{len(cells)} {time.time() - t0:7.0f}s] {cell_name(c):48s} ok={r.get("ok")} '
            f'min_h={cl.get("min_h", float("nan")):+.3f} rtf={cl.get("rtf") or float("nan"):.2f} '
            f'qp={cl.get("qp_rate_hz") or float("nan"):.0f}Hz wall={r.get("wall_s", 0):.0f}s')
    return csv_path


def score_cell(rd: str, c: dict) -> dict:
    """The three scorers on a finished run directory (no Gazebo)."""
    out = {'closed_loop': closed_loop.summarize_run(rd)}
    obstacles = (sim_config_manifest(rd).get('scenario_resolved') or {}).get('obstacles') or []
    if obstacles and c['perception'] != 'false':
        out['estimator'] = estimator_metrics.score_run(rd, condition=c['condition'],
                                                       sets=c.get('sets'))
    if any(o.get('law', 'static') != 'static' for o in obstacles):
        out['attribution'] = attribution.attribute_run(rd)
    return out


def _rescore_one(res_path: str) -> str:
    with open(res_path) as f:
        r = json.load(f)
    rd = os.path.dirname(res_path)
    for k in ('closed_loop', 'estimator', 'attribution', 'error'):
        r.pop(k, None)
    try:
        r.update(score_cell(rd, r['cell']))
        r['ok'] = (not r.get('timed_out') and not r.get('leftovers')
                   and r['closed_loop'].get('min_h') is not None)
    except Exception:                                          # noqa: BLE001
        r['ok'] = False
        r['error'] = traceback.format_exc()
    r['rescored'] = time.strftime('%Y-%m-%d %H:%M:%S')
    with open(res_path, 'w') as f:
        json.dump(r, f, indent=1, default=_json_default)
    return res_path


def rescore(batch_dir: str, *, workers: int = 4, log=print) -> str:
    """Re-run the scorers on every cell of a batch (after a scoring fix), rebuild cells.csv."""
    import glob
    from concurrent.futures import ProcessPoolExecutor
    paths = sorted(glob.glob(os.path.join(batch_dir, 'cells', '*', 'cell_result.json')))
    with ProcessPoolExecutor(max_workers=workers) as ex:
        for i, p in enumerate(ex.map(_rescore_one, paths), 1):
            if i % 25 == 0 or i == len(paths):
                log(f'rescored {i}/{len(paths)}')
    rows = []
    for p in paths:
        with open(p) as f:
            rr = json.load(f)
        rows.append(_flat(rr['cell'], rr))
    csv_path = os.path.join(batch_dir, 'cells.csv')
    write_csv(csv_path, rows)
    return csv_path


def write_csv(path: str, rows: list[dict]) -> None:
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    tmp = path + '.tmp'
    with open(tmp, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, path)
