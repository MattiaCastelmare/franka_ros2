"""Where the columns diverge: ball_throw_eval.py score + approach_probe.py causes, unchanged (Phase 5.3).

For every MOVING obstacle of a run, the truth is written in the format
``ball_throw_eval.py score --truth`` reads (``t``, ``ok``, ``p``; parked
samples are not ok) and the shipped scorer runs on the run's bag as a
subprocess. Its pass table goes to ``approach_probe.cmd_causes``, which gives
each pass the first of its four causes (not in rows / untracked / velocity
untrusted / too late) or ok.

One adaptation, classification-preserving: approach_probe's table regex reads a
lead as ``(\\w+)ms``, which does not match a NEGATIVE lead (the CBF first acted
AFTER the closest approach, inside ball_throw_eval's +0.1 s window). Such a pass
is silently dropped. Negative leads are rewritten to ``0ms`` before parsing: a
lead counts only through ``isfinite`` and ``< need_ms``, and 0 gives the same
answer as any negative number for both, so no pass changes cause.
"""
from __future__ import annotations

import contextlib
import io
import os
import re
import subprocess
import sys
import tempfile
import types

import numpy as np

from .estimator_metrics import fe_scripts

PARKED_Z = -10.0
_NEG_LEAD = re.compile(r'-\d+ms')


def truth_files(run_dir: str, out_dir: str | None = None, stride: int = 2) -> list[tuple]:
    """[(obstacle name, law, path)] in ball_throw_eval's truth format, moving obstacles only."""
    from . import analysis as A
    man = A.load_json(os.path.join(run_dir, 'run_manifest_sim.json')) or {}
    obs = (man.get('scenario_resolved') or {}).get('obstacles') or []
    T = A.load_truth(run_dir)
    if T is None or not obs:
        return []
    out_dir = out_dir or run_dir
    t = np.asarray(T['t'])[::stride]
    P = np.asarray(T['p'])[::stride]
    res = []
    for k, o in enumerate(obs):
        if o.get('law', 'static') == 'static':
            continue
        p = P[:, k, :]
        ok = np.isfinite(p).all(axis=1) & (p[:, 2] > PARKED_Z)
        path = os.path.join(out_dir, f'truth_bte_{o["name"]}.npz')
        np.savez(path, t=t, ok=ok, p=np.where(ok[:, None], p, 0.0))
        res.append((o['name'], o['law'], path))
    return res


def score(run_dir: str, truth_path: str, *, min_speed: float, rate_hz: float = 90.0,
          extra: tuple = ()) -> str:
    scripts = fe_scripts()
    cmd = [sys.executable, os.path.join(scripts, 'ball_throw_eval.py'), 'score',
           os.path.join(run_dir, 'bag'), '--truth', truth_path, '--rate-hz', str(rate_hz),
           '--min-speed', str(min_speed), *extra]
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=600, cwd=scripts,
                       env=dict(os.environ, OPENBLAS_NUM_THREADS='1'))
    return p.stdout + (('\n[stderr]\n' + p.stderr) if p.returncode else '')


def causes(score_text: str, need_ms: float = 450.0) -> dict:
    fe_scripts()
    import approach_probe
    fixed = _NEG_LEAD.sub('0ms', score_text)
    with tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False) as f:
        f.write(fixed)
        path = f.name
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            rows = approach_probe.cmd_causes(types.SimpleNamespace(file=path, need_ms=need_ms))
    finally:
        os.unlink(path)
    n_passes = 0
    m = re.search(r'(\d+) ball passes', score_text)
    if m:
        n_passes = int(m.group(1))
    counts = {}
    for r in rows:
        counts[r['cause']] = counts.get(r['cause'], 0) + 1
    return dict(need_ms=need_ms, n_passes_scored=n_passes, n_parsed=len(rows),
                n_negative_leads=len(_NEG_LEAD.findall(score_text)), counts=counts,
                passes=[{k: (None if isinstance(v, float) and not np.isfinite(v) else v)
                         for k, v in r.items()} for r in rows],
                summary=buf.getvalue().strip())


def residual_causes(passes: list[dict], need_ms: float = 450.0) -> dict:
    """approach_probe's order WITHOUT causes 2 and 3, for runs with tracking off.

    approach_probe assumes the tracker feeds the barrier: with tracking off every
    pass is '2 untracked' by construction, which says nothing about the residual
    estimator. Its other two tests need no track: the obstacle in the rows
    (row lead finite) and the CBF acting at least need_ms before the closest
    approach.
    """
    counts = {}
    for p in passes:
        row, cbf = p.get('row'), p.get('cbf')
        if row is None:
            c = '1 not-in-rows'
        elif cbf is None or cbf < need_ms:
            c = '4 too-late'
        else:
            c = '- ok'
        counts[c] = counts.get(c, 0) + 1
    return counts


def attribute_run(run_dir: str, *, need_ms: float = 450.0) -> dict:
    """Per moving obstacle: the scorer's table and the causes of its passes."""
    out = {}
    for name, law, path in truth_files(run_dir):
        from . import analysis as A
        man = A.load_json(os.path.join(run_dir, 'run_manifest_sim.json')) or {}
        o = next(x for x in man['scenario_resolved']['obstacles'] if x['name'] == name)
        speed = max(float(np.linalg.norm(o.get('v0', (0, 0, 0)))), float(o.get('speed', 0.0)))
        # the scorer ignores the obstacle below --min-speed: half its own speed
        txt = score(run_dir, path, min_speed=max(0.1, 0.5 * speed))
        with open(os.path.join(run_dir, f'ball_throw_eval_{name}.txt'), 'w') as f:
            f.write(txt)
        out[name] = dict(law=law, speed=speed, **causes(txt, need_ms))
    return out
