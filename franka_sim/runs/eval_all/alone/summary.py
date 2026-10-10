"""Summary of traces/alone (eval_alone.sh): task + constraint metrics per controller / mode / scenario.

    cd runs/eval_all && python3 alone/summary.py [--json out.json]

clean = held at 5 s, no obstacle collision, and none of pos / vel / acc /
self_coll / sing violated at any tick: the "policy alone" requirement.
Per family: episodes with at least one violating tick, and the worst excess.
"""
import glob
import json
import os
import re
import statistics as st
import sys
from collections import defaultdict

T = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'traces', 'alone')
REQ = ('pos', 'vel', 'acc', 'self_coll', 'sing')
EXTRA = ('slew', 'torque', 'floor', 'workspace')
G = defaultdict(dict)
for f in glob.glob(f'{T}/*.json'):
    tag, mode, sc = re.match(r'(.+?)_(alone|obstacle_off|full)_(static|dynamic)_\d+\.json', os.path.basename(f)).groups()
    for e in json.load(open(f))['eps']:
        G[(tag, mode, sc)][e['seed']] = e

ORDER = ['big11', 'soup9', 'prec', 'pd', 'zero']
rows = []
for key in sorted(G, key=lambda k: (ORDER.index(k[0]) if k[0] in ORDER else 99, k[1], k[2])):
    v = list(G[key].values()); n = len(v)
    fam = {k: sum(e['viol_steps'][k] > 0 for e in v) for k in REQ + EXTRA}
    worst = {k: max(e['viol_max'][k] for e in v) for k in REQ + EXTRA}
    clean = sum(e['held'] and not e['coll'] and not any(e['viol_steps'][k] for k in REQ) for e in v)
    rows.append(dict(controller=key[0], mode=key[1], scenario=key[2], episodes=n,
                     held=sum(e['held'] for e in v), coll=sum(e['coll'] for e in v), clean=clean,
                     **{f'ep_{k}': fam[k] for k in fam}, **{f'max_{k}': round(worst[k], 4) for k in worst},
                     sigma_min_p05=round(sorted(e['sigma_min'] for e in v)[n // 20], 4),
                     self_gap_min=round(min(e['self_gap'] for e in v), 4),
                     self_gap_p05=round(sorted(e['self_gap'] for e in v)[n // 20], 4)))

hdr = f'{"ctrl":6s} {"mode":12s} {"scen":7s} {"n":>4s} {"held":>5s} {"coll":>4s} {"CLEAN":>5s} | ' + \
      ' '.join(f'{k[:5]:>5s}' for k in REQ) + ' | ' + ' '.join(f'{k[:5]:>5s}' for k in EXTRA) + ' | sig5% gapmin'
print(hdr)
for r in rows:
    print(f'{r["controller"]:6s} {r["mode"]:12s} {r["scenario"]:7s} {r["episodes"]:4d} {r["held"]:5d} {r["coll"]:4d} {r["clean"]:5d} | '
          + ' '.join(f'{r["ep_" + k]:5d}' for k in REQ) + ' | ' + ' '.join(f'{r["ep_" + k]:5d}' for k in EXTRA)
          + f' | {r["sigma_min_p05"]:.3f} {r["self_gap_min"]:+.3f}')
print('\nworst excess (acc rad/s², vel rad/s, pos rad, sing = sigma_floor − σ, self_coll m):')
for r in rows:
    print(f'{r["controller"]:6s} {r["mode"]:12s} {r["scenario"]:7s} ' + ' '.join(f'{k}={r["max_" + k]}' for k in REQ + ('torque',)))
if '--json' in sys.argv:
    json.dump(rows, open(sys.argv[sys.argv.index('--json') + 1], 'w'), indent=1)
