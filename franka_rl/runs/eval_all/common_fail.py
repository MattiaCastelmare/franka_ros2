"""Failures shared by the ensembles on the 400 held-out seeds (3000:100 + 4000:300): are they infeasible tasks or a policy defect?
   python3 common_fail.py   (run from runs/eval_all)"""
import json, glob, re, os, numpy as np
from collections import defaultdict, Counter
def load(pat):
    E = {}
    for f in glob.glob(pat):
        for e in json.load(open(f))['eps']: E[e['seed']] = e
    return E
ENS = ['ens_soup9', 'ens_mix13', 'ens_mix6', 'ens_soup3', 'ens_avg3', 'ens_v12new3', 'ens_mix3', 'ens_prec3']
# every model ever run on these seeds = "oracle": a seed nobody holds is a candidate infeasible task
ALL = defaultdict(lambda: defaultdict(dict))
for d in ('avg', 'v12', 'v11conf'):
    for f in glob.glob(f'traces/{d}/*.json'):
        tag, sc = re.match(r'(.+)_(static|dynamic)_\d+\.json', os.path.basename(f)).groups()
        for e in json.load(open(f))['eps']: ALL[sc][f'{d}/{tag}'][e['seed']] = e
def cat(e):
    if e['coll']: return 'coll'
    if e['held']: return 'held'
    if e['reached']: return 'near' if e['err_f'] < 0.08 else 'lost'
    return 'barrier' if min(e['d'][-20:]) < 0.17 else 'never'
for sc in ('dynamic', 'static'):
    M = ALL[sc]; seeds = sorted(M['avg/ens_soup9'])
    nmod = len(M); held_by = {s: sum(m.get(s, {}).get('held', False) for m in M.values()) for s in seeds}
    ens_fail = {s: sum(not M[f'avg/{t}'][s]['held'] for t in ENS) for s in seeds}
    common = [s for s in seeds if ens_fail[s] == len(ENS)]
    some = [s for s in seeds if 0 < ens_fail[s] < len(ENS)]
    print(f'\n===== {sc}: {nmod} models; ens_soup9 fails {sum(not M["avg/ens_soup9"][s]["held"] for s in seeds)}; '
          f'failed by ALL {len(ENS)} ensembles {len(common)}, by some {len(some)}')
    print('   held by how many of the', nmod, 'models (common failures):', sorted(Counter(held_by[s] for s in common).items()))
    print('   never held by any model:', sum(held_by[s] == 0 for s in seeds))
    E9 = M['avg/ens_soup9']
    print('   ens_soup9 category of common failures:', Counter(cat(E9[s]) for s in common))
    rows = []
    for s in common:
        e = E9[s]; tg = np.array(e['target']); ob = np.array(e['ob']); ee = np.array(e['ee'])
        otd = np.linalg.norm(ob - tg, axis=1)
        rows.append((s, cat(e), held_by[s], e['err_f'], e['tot'], tg, otd.min(), otd[-20:].mean(), np.mean(e['d'][-20:]), np.mean(e['iv'][-20:]), e['dmin']))
    H = [E9[s] for s in seeds if E9[s]['held']]
    def stat(f, xs): return f'{np.median(xs):.3f} [{np.percentile(xs, 10):.3f}-{np.percentile(xs, 90):.3f}]'
    print('   median [p10-p90]            common-fail             held (ens_soup9)')
    print('   target z                  ', stat(0, [r[5][2] for r in rows]), '   ', stat(0, [e['target'][2] for e in H]))
    print('   final err (m)             ', stat(0, [r[3] for r in rows]), '   ', stat(0, [e['err_f'] for e in H]))
    print('   d_min last 1 s (m)        ', stat(0, [r[8] for r in rows]), '   ', stat(0, [np.mean(e['d'][-20:]) for e in H]))
    print('   CBF interv. last 1 s      ', stat(0, [r[9] for r in rows]), '   ', stat(0, [np.mean(e['iv'][-20:]) for e in H]))
    print('   obst-target dist, last 1s ', stat(0, [r[7] for r in rows]), '   ', stat(0, [np.linalg.norm(np.array(e['ob'][-20:]) - e['target'], axis=1).mean() for e in H]))
    print('   time on target            ', stat(0, [r[4] for r in rows]), '   ', stat(0, [e['tot'] for e in H]))
    print('   seed  cat     heldby err   tot   tgt xyz              otd_min otd_end d_end iv_end')
    for r in sorted(rows, key=lambda r: -r[2]):
        print(f'   {r[0]} {r[1]:7s} {r[2]:3d}   {r[3]:.3f} {r[4]:.2f}  {np.round(r[5], 2)!s:20s} {r[6]:.3f}   {r[7]:.3f}  {r[8]:.3f} {r[9]:.2f}')
