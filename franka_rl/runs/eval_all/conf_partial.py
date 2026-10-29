"""Partial comparison on traces/avg_conf against the prec3.75 tag (common seeds only)."""
import json, glob
from math import comb
def load(t, sc):
    E = {}
    for f in glob.glob(f'traces/avg_conf/{t}_{sc}_*.json'):
        for e in json.load(open(f))['eps']: E[e['seed']] = e
    return E
def mcn(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1, 2 * sum(comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)
H = lambda E, k: sum(E[s]['held'] for s in k)
for sc in ('dynamic', 'static'):
    R = load('prec3.75', sc); V = load('v10c', sc); print(sc, 'prec3.75 seeds so far', len(R), ' v10c', len(V))
    for t in ('avg_k4', 'ens_prec3', 'ens_soup3', 'ens_avg3', 'ens_soup9', 'v10c'):
        E = load(t, sc); k = [s for s in E if s in R]
        if not k: continue
        p = sum(E[s]['held'] and not R[s]['held'] for s in k); m = sum(R[s]['held'] and not E[s]['held'] for s in k)
        print(f'  {t:10s} {H(E, E)}/{len(E)} | on {len(k)} common: {H(E, k)} vs prec {H(R, k)}  +{p}/-{m} p={mcn(p, m):.4f}'
              f'  coll {sum(e["coll"] for e in E.values())} dmin {1000*min(e["dmin"] for e in E.values()):.0f}')
