"""Averaged/ensemble policies vs ft_prec 3.75M and v10c 2.5M on the same seeds.   avg_summary.py DIR [REF_DIR]"""
import json, glob, os, re, sys
from math import comb
from collections import defaultdict
D = sys.argv[1]; RD = sys.argv[2] if len(sys.argv) > 2 else 'traces/v11conf'
def mcn(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)
def load(pat):
    eps = {}
    for f in glob.glob(pat):
        for e in json.load(open(f))['eps']: eps[e['seed']] = e
    return eps
G = defaultdict(dict)
for f in glob.glob(f'{D}/*.json'):
    tag, sc = re.match(r'(.+)_(static|dynamic)_\d+\.json', os.path.basename(f)).groups()
    G[(tag, sc)].update(load(f))
REF = {('prec3.75', sc): load(f'{RD}/sac_v11_ft_prec_3750000_{sc}_*.json') for sc in ('static', 'dynamic')}
REF.update({('v10c', sc): load(f'{RD}/sac_v10c_2500000_{sc}_*.json') for sc in ('static', 'dynamic')})
print(f'{"model":12s} | {"dyn":>4s} {"vs prec":>9s} {"p":>6s} | {"sta":>4s} {"vs prec":>9s} {"p":>6s} | {"sum":>4s} | {"vs v10c sta p":>13s} | coll dmin <dsafe')
tags = sorted({t for t, _ in G}) + ['prec3.75', 'v10c']
for t in tags:
    row = []; tot = 0; ok = True
    for sc in ('dynamic', 'static'):
        E = G.get((t, sc)) or REF.get((t, sc)); R = REF[('prec3.75', sc)]
        if not E: ok = False; break
        k = [s for s in E if s in R]
        h = sum(E[s]['held'] for s in k); tot += h
        pl = sum(E[s]['held'] and not R[s]['held'] for s in k); mi = sum(R[s]['held'] and not E[s]['held'] for s in k)
        row.append((h, len(k), pl, mi, mcn(pl, mi), E, k))
    if not ok: continue
    V = REF[('v10c', 'static')]; E, k = row[1][5], row[1][6]
    pv = mcn(sum(E[s]['held'] and not V[s]['held'] for s in k), sum(V[s]['held'] and not E[s]['held'] for s in k))
    allE = [e for r in row for e in (r[5][s] for s in r[6])]
    print(f'{t:12s} | {row[0][0]:4d} {f"+{row[0][2]}/-{row[0][3]}":>9s} {row[0][4]:6.3f} | {row[1][0]:4d} {f"+{row[1][2]}/-{row[1][3]}":>9s} {row[1][4]:6.3f} | {tot:4d} | {pv:13.4f} |'
          f' {sum(e["coll"] for e in allE):3d} {1000*min(e["dmin"] for e in allE):4.0f} {sum(e["dmin"] < 0.15 for e in allE):4d}   (n={row[0][1]}/{row[1][1]})')
