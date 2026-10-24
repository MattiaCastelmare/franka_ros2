"""Summarise traces/v11 vs the v10c 2.5M baseline on the 140 paired seeds (held, reached, McNemar, d_min, floor)."""
import json, glob, os, re, sys
from collections import defaultdict
from math import comb
T = os.path.dirname(os.path.abspath(__file__)) + '/traces'
def load(files):
    eps = {}
    for f in files:
        for e in json.load(open(f))['eps']: eps[e['seed']] = e
    return eps
def base(sc):
    return load([f'{T}/v10c/2500000_{sc}.json'] + glob.glob(f'{T}/v10c/fresh_checkpoints/sac_2500000_steps_{sc}_*.json'))
def mcnemar(b, c):  # exact two-sided
    n = b + c
    if n == 0: return 1.0
    k = min(b, c); return min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n)
B = {sc: base(sc) for sc in ('static', 'dynamic')}
groups = defaultdict(list)
for f in glob.glob(f'{T}/v11/*.json'):
    m = re.match(r'(.+)_(\d+|final_model|best_model)_(static|dynamic)_(\d+)\.json', os.path.basename(f))
    groups[(m[1], m[2], m[3])].append(f)
rows = []
print(f'{"run":28s} {"step":>11s} {"sc":7s} {"n":>3s} {"held":>4s} {"base":>4s} {"+":>3s} {"-":>3s} {"p":>7s} {"reach":>5s} {"dmin":>5s} {"coll":>4s} {"floor":>5s}')
for sc in ('dynamic', 'static'):
    print(f'{"sac_v10c (baseline)":28s} {"2500000":>11s} {sc:7s} {len(B[sc]):3d} {sum(e["held"] for e in B[sc].values()):4d}')
for (r, s, sc), fs in sorted(groups.items(), key=lambda k: (k[0][0], k[0][2], k[0][1].zfill(12))):
    eps = load(fs); b = B[sc]; common = [k for k in eps if k in b]
    held = sum(eps[k]['held'] for k in common); bh = sum(b[k]['held'] for k in common)
    plus = sum(eps[k]['held'] and not b[k]['held'] for k in common); minus = sum(b[k]['held'] and not eps[k]['held'] for k in common)
    dmin = min(e['dmin'] for e in eps.values()); coll = sum(e['coll'] for e in eps.values()); fl = sum(e['floor'] for e in eps.values())
    print(f'{r:28s} {s:>11s} {sc:7s} {len(common):3d} {held:4d} {bh:4d} {plus:3d} {minus:3d} {mcnemar(plus, minus):7.4f} '
          f'{sum(e["reached"] for e in eps.values()):5d} {1000*dmin:5.0f} {coll:4d} {fl:5d}')
