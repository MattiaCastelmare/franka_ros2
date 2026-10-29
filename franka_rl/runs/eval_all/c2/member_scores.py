"""Per-checkpoint shield-OFF score of every 24-D shield-off member on the 400 held-out seeds (selection set).
score = OFF held (dyn+sta) − 3·OFF collisions; also ON held. Prints the best checkpoint per run.
    cd runs/eval_all && python3 c2/member_scores.py
"""
import json, glob, re, os
from collections import defaultdict
SRC = {'traces/c1_eval': None, 'traces/b3_final': None, 'traces/b4_eval': None}
EXCL = ('full', 'vobs')            # 27-D obs members can't share an ensemble with 24-D ones
R = defaultdict(lambda: defaultdict(dict))
for D in SRC:
    for f in glob.glob(f'{D}/*.json'):
        m = re.match(r'(.+)_(\d+)k_(off|on)_(static|dynamic)_\d+\.json', os.path.basename(f))
        if not m: continue
        run, k, cond, sc = m.groups()
        if any(x in run for x in EXCL) or not (run.startswith(('c2_off', 'c1_off', 'ftv10c', 'clear'))): continue
        for e in json.load(open(f))['eps']: R[(run, int(k))][(cond, sc)][e['seed']] = e
rows = []
for (run, k), C in R.items():
    if not all(len(C.get((c, s), {})) == 400 for c in ('off',) for s in ('static', 'dynamic')): continue
    h = sum(e['held'] for s in ('static', 'dynamic') for e in C[('off', s)].values())
    co = sum(e['coll'] for s in ('static', 'dynamic') for e in C[('off', s)].values())
    on = sum(e['held'] for s in ('static', 'dynamic') for e in C.get(('on', s), {}).values())
    non = sum(len(C.get(('on', s), {})) for s in ('static', 'dynamic'))
    rows.append((run, k, h, co, h - 3 * co, on, non))
best = {}
for r in sorted(rows, key=lambda r: (r[0], r[1])):
    print(f'{r[0]:14s} {r[1]:5d}k  OFF held {r[2]:3d} coll {r[3]:2d} score {r[4]:4d} | ON held {r[5]:3d}/{r[6]}')
    if r[0] not in best or r[4] > best[r[0]][4]: best[r[0]] = r
print('\nbest per run:'); [print(f'  {b[0]:14s} {b[1]}k score {b[4]}') for b in sorted(best.values(), key=lambda b: -b[4])]
