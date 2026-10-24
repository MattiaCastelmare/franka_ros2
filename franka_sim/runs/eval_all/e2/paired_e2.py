"""e2 paired table: per d2 parent, clean/800 of the parent net, its G and S grafts, in the four protocols.

    cd franka_sim/runs/eval_all && python3 e2/paired_e2.py [1000k|2000k ...]

Columns: one/multi sphere × alone/full (traces/e2_one, traces/e2_multi, eval_e2.sh protocol). Rows: d2par_<p>
(24-D parent, same checkpoint as the e2 start), e2_g_<p>_<k>, e2_s_<p>_<k>; then the mean over parents and the
paired S − G difference with an exact McNemar test pooled over the parents present.
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
PARENTS = ['p_s5', 'p4_s1', 'p_p2', 'b_1', 'b_2', 'p4_p1']
COLS = [('one', ''), ('one', '_full'), ('multi', ''), ('multi', '_full')]


def clean(e):
    return e['held'] and not e['coll'] and not any(e['viol_steps'][k] for k in REQ)


def load(scene, tag):
    out = {}
    for f in glob.glob(os.path.join(HERE, '..', 'traces', f'e2_{scene}', f'{tag}_nom_*.json')):
        m = re.match(r'(.+?)_nom_(static|dynamic)_\d+\.json', os.path.basename(f))
        if m.group(1) != tag:
            continue
        for e in json.load(open(f))['eps']:
            out[(m.group(2), e['seed'])] = clean(e)
    return out


def mcnemar(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)


for k in (sys.argv[1:] or ['1000k', '2000k']):
    print(f'=== +{k}   clean/800   ' + '  '.join(f'{s}{x or "_alone"}'.rjust(12) for s, x in COLS))
    sums = defaultdict(list)
    pair = defaultdict(lambda: [0, 0])
    for p in PARENTS:
        rows = {'par': f'd2par_{p}', 'G': f'e2_g_{p}_{k}', 'S': f'e2_s_{p}_{k}'}
        data = {r: [load(s, t + x) for s, x in COLS] for r, t in rows.items()}
        for r, t in rows.items():
            cells = []
            for i, d in enumerate(data[r]):
                if len(d) == 800:
                    cells.append(f'{sum(d.values()):12d}')
                    sums[(r, i)].append(sum(d.values()))
                else:
                    cells.append(' ' * 12 if not d else f'{len(d):>9d}ep?')
            print(f'{p:6s} {r:3s} {t:22s}' + ''.join(cells))
        for i in range(len(COLS)):
            g, s = data['G'][i], data['S'][i]
            if len(g) == 800 and len(s) == 800:
                pair[i][0] += sum(s[x] and not g[x] for x in g)
                pair[i][1] += sum(g[x] and not s[x] for x in g)
    for r in ('par', 'G', 'S'):
        print(f'{"mean":6s} {r:3s} {"":22s}' + ''.join(
            f'{sum(sums[(r, i)]) / len(sums[(r, i)]):12.1f}' if sums[(r, i)] else ' ' * 12 for i in range(len(COLS))))
    print(f'{"S-G":6s} {"":3s} {"pooled +S/-S, McNemar":22s}' + ''.join(
        f'{f"+{pair[i][0]}/-{pair[i][1]} p{mcnemar(*pair[i]):.0e}":>12s}' if sum(pair[i]) else ' ' * 12
        for i in range(len(COLS))))
