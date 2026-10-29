"""v12 checkpoints on the 400 held-out seeds (3000:100 + 4000:300) vs v12_cont (same start) and ft_prec 3.75M -> v12_summary.txt."""
import json, glob, os, re
from math import comb
from collections import defaultdict
def mcn(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)
def load(pat):
    E = {}
    for f in glob.glob(pat):
        for e in json.load(open(f))['eps']: E[e['seed']] = e
    return E
G = defaultdict(dict)
for f in glob.glob('traces/v12/*.json'):
    tag, sc = re.match(r'(.+)_(static|dynamic)_\d+\.json', os.path.basename(f)).groups()
    G[(tag, sc)].update(load(f))
REF = {sc: load(f'traces/v11conf/sac_v11_ft_prec_3750000_{sc}_*.json') for sc in ('static', 'dynamic')}
lines = [f'{"run@step":32s} | {"dyn":>4s} {"vs prec3.75":>11s} {"p":>6s} | {"sta":>4s} {"vs prec3.75":>11s} {"p":>6s} | sum | coll floor dmin <dsafe']
lines.append(f'{"ft_prec 3.75M (reference)":32s} | {sum(e["held"] for e in REF["dynamic"].values()):4d} {"":>11s} {"":>6s} | {sum(e["held"] for e in REF["static"].values()):4d}')
for tag in sorted({t for t, _ in G}, key=lambda t: (t.rsplit('_', 1)[0], int(t.rsplit('_', 1)[1]))):
    cells = []; tot = 0; allE = []
    for sc in ('dynamic', 'static'):
        E = G.get((tag, sc), {}); R = REF[sc]; k = [s for s in E if s in R]
        if len(k) < 400: cells.append(f'{"(" + str(len(k)) + "/400)":>24s}'); continue
        h = sum(E[s]['held'] for s in k); tot += h; allE += [E[s] for s in k]
        p = sum(E[s]['held'] and not R[s]['held'] for s in k); m = sum(R[s]['held'] and not E[s]['held'] for s in k)
        cells.append(f'{h:4d} {f"+{p}/-{m}":>11s} {mcn(p, m):6.3f}')
    safety = (f'{sum(e["coll"] for e in allE):4d} {sum(e["floor"] for e in allE):5d} {1000*min(e["dmin"] for e in allE):4.0f} {sum(e["dmin"] < 0.15 for e in allE):6d}'
              if allE else '')
    lines.append(f'{tag:32s} | {cells[0]} | {cells[1]} | {tot:3d} | {safety}')
open('v12_summary.txt', 'w').write('\n'.join(lines) + '\n'); print('\n'.join(lines))
