"""Clean-episode table over traces/d1 + traces/d2 (same seeds, same protocol).

    cd franka_rl/runs/eval_all && python3 d2/summary_d2.py [REGEX]      # REGEX filters tags
    FRESH=1 python3 d2/summary_d2.py ...                                 # traces/d2_fresh (seeds 5000:400)

clean = held at 5 s, no obstacle collision, no pos/vel/acc/self_coll/sing tick.
Per tag and condition: clean of 800 (dyn + static), failure split (not held / collision /
held but violated), and an exact McNemar test on clean vs big11clip on the same seeds.
"""
import glob
import json
import os
import re
import sys
from collections import defaultdict
from math import comb

HERE = os.path.dirname(os.path.abspath(__file__))
REQ = ('pos', 'vel', 'acc', 'self_coll', 'sing')
REF = os.environ.get('REF', 'big11clip')   # REF=big11clip_full for MODE=full tags


def clean(e):
    return e['held'] and not e['coll'] and not any(e['viol_steps'][k] for k in REQ)


def mcnemar(b, c):
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    return min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n)


G = defaultdict(dict)   # (tag, cond) -> {(sc, seed): episode}
DIRS = (tuple(os.environ['DIRS'].split(',')) if os.environ.get('DIRS')   # DIRS=e1_multi: other trace dirs
        else ('d2_fresh',) if os.environ.get('FRESH') else ('d1', 'd2'))
for f in [f for d in DIRS for f in glob.glob(os.path.join(HERE, '..', 'traces', d, '*.json'))]:
    m = re.match(r'(.+?)_(nom|rnd)_(static|dynamic)_\d+\.json', os.path.basename(f))
    tag, cond, sc = m.groups()
    for e in json.load(open(f))['eps']:
        G[(tag, cond)][(sc, e['seed'])] = e

pat = re.compile(sys.argv[1]) if len(sys.argv) > 1 else None
print(f'{"tag":22s} cond    n  dyn  sta  CLEAN | nh coll viol | nh_reached nh_err | vs {REF}: +/- p')
for (tag, cond) in sorted(G, key=lambda k: (k[1], -sum(map(clean, G[k].values())))):
    if pat and not pat.search(tag) and tag != REF:
        continue
    v = G[(tag, cond)]
    n = len(v)
    cl = {k: clean(e) for k, e in v.items()}
    dyn = sum(c for (sc, _), c in cl.items() if sc == 'dynamic')
    sta = sum(c for (sc, _), c in cl.items() if sc == 'static')
    nh = [e for e in v.values() if not e['held'] and not e['coll']]
    coll = sum(e['coll'] for e in v.values())
    viol = sum(e['held'] and not e['coll'] and not clean(e) for e in v.values())
    nh_err = sorted(e['err_f'] for e in nh)[len(nh) // 2] if nh else 0
    ref = G.get((REF, cond), {})
    common = [k for k in v if k in ref]
    b = sum(cl[k] and not clean(ref[k]) for k in common)
    c = sum(clean(ref[k]) and not cl[k] for k in common)
    flag = '' if n == 800 else '  (incomplete)'
    print(f'{tag:22s} {cond}  {n:4d} {dyn:4d} {sta:4d}  {dyn + sta:4d}  | {len(nh):3d} {coll:3d} {viol:3d} | '
          f'{sum(e["reached"] for e in nh):4d}  {nh_err:.3f} | +{b} -{c} p={mcnemar(b, c):.2g}{flag}')
