#!/usr/bin/env python3
"""Recorded simulation evidence: which code produced it, and how complete it is (chapter 12 §9). Host.

Reads (read-only) the untracked `franka_simulation_experiments/sim_runs/`: every
`run_manifest_sim.json`, every `batches/**/cell_result.json`, and the `truth.npz` of the 40 most
recent runs. Nothing is written. Without the directory every line becomes INFO and nothing fails.

  1. F-6-1: the git SHA of every run manifest: distinct SHAs, dirty share, none at the baseline,
     all ancestors of it                                                                    (Runtime (recorded))
  2. the batch cells: ok / timed out / QP-rate verdict UNKNOWN / ok with contact / ok with a
     reflex FAIL                                                                            (Runtime (recorded))
  3. F-6-5: the truth grid inside the episode window: rate and the largest hole; whether the holes
     fall at the 5 s save period (uniformly placed holes would give 0.20)                   (Runtime (recorded))
"""
import collections
import glob
import json
import os
import subprocess
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common6 as C6  # noqa: E402
import numpy as np  # noqa: E402

R = C6.C.Report('check_sim_runs (chapter 12 §9)')
root = C6.SIM_RUNS
if not os.path.isdir(root):
    R.info(f'no {root}: the Runtime (recorded) audit is skipped')
    sys.exit(R.finish())


def load(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


# 1
mans = [p for p in glob.glob(os.path.join(root, '**', 'run_manifest_sim.json'), recursive=True)]
shas, dirty, nosha = collections.Counter(), 0, 0
for p in mans:
    m = load(p) or {}
    g = m.get('git') or {}
    if not g.get('sha'):
        nosha += 1
        continue
    shas[g['sha'][:7]] += 1
    dirty += bool(g.get('dirty'))
base = subprocess.run(['git', '-C', C6.C.REPO, 'rev-parse', C6.BASELINE], capture_output=True,
                      text=True).stdout.strip()
anc = {s: subprocess.run(['git', '-C', C6.C.REPO, 'merge-base', '--is-ancestor', s, base]).returncode == 0
       for s in shas}
R.info(f'1. {len(mans)} run manifests; {nosha} without a SHA; {sum(shas.values())} with one, {dirty} of them '
       f'from a dirty tree; SHAs {dict(shas.most_common())}')
R.fact(f'1. F-6-1: no recorded run is at the baseline {C6.BASELINE}, and every SHA is an ancestor of it',
       C6.BASELINE not in shas and all(anc.values()), f'non-ancestors: {[s for s, a in anc.items() if not a]}')

# 2
cells = [load(p) for p in glob.glob(os.path.join(root, 'batches', '**', 'cell_result.json'), recursive=True)]
cells = [c for c in cells if c]
c = collections.Counter()
for r in cells:
    cl = r.get('closed_loop') or {}
    c['ok'] += bool(r.get('ok'))
    c['timed_out'] += bool(r.get('timed_out'))
    c['min_h None'] += cl.get('min_h') is None
    c['rate UNKNOWN'] += cl.get('qp_rate_ok') is None
    c['ok + contact'] += bool(r.get('ok') and cl.get('contact'))
    c['ok + reflex FAIL'] += bool(r.get('ok') and cl.get('reflex_verdict') == 'FAIL')
    c['ok + rate UNKNOWN'] += bool(r.get('ok') and cl.get('qp_rate_ok') is None)
R.info(f'2. {len(cells)} batch cells: {dict(c)}')

# 3
runs = []
for p in glob.glob(os.path.join(root, '**', 'truth.npz'), recursive=True):
    runs.append((os.path.getmtime(p), p))
hz, gap, at5 = [], [], []
for _, p in sorted(runs)[-40:]:
    m = load(os.path.join(os.path.dirname(p), 'run_manifest_sim.json')) or {}
    ep = m.get('episode') or {}
    if ep.get('t_start_sim') is None or ep.get('t_end_sim') is None:
        continue
    try:
        with np.load(p) as z:
            t = z['t']
    except Exception:   # noqa: BLE001  (a truncated file is itself a finding)
        continue
    w = t[(t >= ep['t_start_sim']) & (t <= ep['t_end_sim'])]
    if w.size < 3:
        continue
    d = np.diff(w)
    hz.append(w.size / (w[-1] - w[0]))
    gap.append(d.max())
    big = np.flatnonzero(np.diff(t) > 0.05)
    at5 += [float(((t[i] - t[0]) % 5.0)) for i in big]
if hz:
    hz, gap = np.array(hz), np.array(gap)
    R.info(f'3. {hz.size} recent runs: truth samples per sim second in the episode min/median/max '
           f'{hz.min():.0f}/{np.median(hz):.0f}/{hz.max():.0f}; largest hole per run median {1e3 * np.median(gap):.0f} ms, '
           f'max {1e3 * gap.max():.0f} ms')
    a = np.array(at5)
    near = float(np.mean(np.minimum(a, 5.0 - a) < 0.5)) if a.size else float('nan')
    R.info(f'3. {a.size} holes > 50 ms; share within 0.5 s of a multiple of 5 s after the first sample: {near:.2f}')
    R.info(f'3. runs with a hole > 50 ms inside the episode window: {int(np.sum(gap > 0.05))}/{gap.size}')
    R.fact(f'3. F-6-5: the holes fall at the truth node\'s 5 s save period ({near:.2f} of them within 0.5 s of a '
           'multiple of 5 s; 0.20 expected if they were placed at random)', a.size > 0 and near >= 0.8)
else:
    R.info('3. no truth.npz with an episode window found')
sys.exit(R.finish())
