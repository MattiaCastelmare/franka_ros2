"""Summarise traces/stress: held / reached / min d_min / below-d_safe / collisions per condition, model, scenario."""
import json, glob, os, re
from math import comb
from collections import defaultdict
T = 'traces/stress'
def mcn(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)
G = defaultdict(dict)
for f in glob.glob(f'{T}/*.json'):
    cond, tag, sc, s0 = re.match(r'(.+)_(prec|v10c|zero|onnx)_(static|dynamic)_(\d+)\.json', os.path.basename(f)).groups()
    for e in json.load(open(f))['eps']: G[(cond, tag, sc)][e['seed']] = e
order = ['nominal', 'latency', 'obsnoise', 'jointnoise', 'dynamics', 'allrand', 'servo', 'fastobs', 'bigobs', 'qinit', 'long10s']
out = {}
print(f'{"cond":11s} {"sc":7s} | {"prec":>5s} {"v10c":>5s} {"+/-":>7s} {"p":>6s} | {"dmin p/v":>9s} {"<dsafe p/v":>11s} {"coll p/v":>8s} {"floor":>5s} | onnx zero')
for cond in order:
    for sc in ('dynamic', 'static'):
        P, V = G.get((cond, 'prec', sc)), G.get((cond, 'v10c', sc))
        if not P or not V: continue
        k = [s for s in P if s in V]
        hp = sum(P[s]['held'] for s in k); hv = sum(V[s]['held'] for s in k)
        pl = sum(P[s]['held'] and not V[s]['held'] for s in k); mi = sum(V[s]['held'] and not P[s]['held'] for s in k)
        dm = lambda E: min(E[s]['dmin'] for s in k)
        lt = lambda E: sum(E[s]['dmin'] < 0.15 for s in k)
        co = lambda E: sum(E[s]['coll'] for s in k)
        fl = lambda E: sum(E[s]['floor'] for s in k)
        extra = ''
        for t in ('onnx', 'zero'):
            X = G.get((cond, t, sc))
            if X: extra += f' {t} {sum(e["held"] for e in X.values())}/{len(X)}'
        print(f'{cond:11s} {sc:7s} | {hp:5d} {hv:5d} {f"+{pl}/-{mi}":>7s} {mcn(pl, mi):6.3f} | {1000*dm(P):4.0f}/{1000*dm(V):<4.0f} {lt(P):5d}/{lt(V):<5d} {co(P):3d}/{co(V):<3d} {fl(P)}/{fl(V)} |{extra}  (n={len(k)})')
        out[f'{cond}:{sc}'] = dict(n=len(k), prec=hp, v10c=hv, plus=pl, minus=mi, p=mcn(pl, mi), dmin_p=dm(P), dmin_v=dm(V),
                                   lt_p=lt(P), lt_v=lt(V), coll_p=co(P), coll_v=co(V), floor_p=fl(P), floor_v=fl(V),
                                   reach_p=sum(P[s]['reached'] for s in k), reach_v=sum(V[s]['reached'] for s in k))
json.dump(out, open('stress_summary.json', 'w'), indent=1)
