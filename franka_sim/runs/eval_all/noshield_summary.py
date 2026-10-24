"""Test 1 (2026-10-05): obstacle CBF OFF at test time vs the same policy with the shield ON, paired per seed.
    cd runs/eval_all && python3 noshield_summary.py            (reads traces/noshield + the shield-on references)
held/coll/<dsafe are episode counts; 'iv' = mean shield intervention per recorded step (rad/s^2, every 5th step);
'shield busy' = fraction of recorded steps with intervention > 0.5 in the shield-ON run (how much the policy leaned on it).
"""
import json, glob, re, os
from math import comb
from collections import defaultdict

def mcn(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)

def load(pat):
    eps = {}
    for f in glob.glob(pat):
        for e in json.load(open(f))['eps']: eps[e['seed']] = e
    return eps

ON = {'ens_soup9': 'traces/avg/ens_soup9_{sc}_*.json',
      'prec3.75': 'traces/v11conf/sac_v11_ft_prec_3750000_{sc}_*.json',
      'v10c': 'traces/v11conf/sac_v10c_2500000_{sc}_*.json',
      'zero': 'traces/stress/nominal_zero_{sc}_*.json'}
OFF = defaultdict(dict)
for f in glob.glob('traces/noshield/*.json'):
    tag, sc = re.match(r'(.+)_(static|dynamic)_\d+\.json', os.path.basename(f)).groups()
    OFF[(tag, sc)].update(load(f))

mean = lambda xs: sum(xs) / max(len(xs), 1)
ZOFF = {sc: OFF.get(('zero', sc), {}) for sc in ('static', 'dynamic')}
print(f'{"model":10s} {"scen":7s} {"n":>4s} | {"held ON":>7s} {"OFF":>4s} {"+/-":>8s} {"p":>6s} | {"coll OFF":>8s} {"(zero coll too)":>15s} | '
      f'{"<dsafe ON":>9s} {"OFF":>4s} | {"dmin ON":>7s} {"OFF":>5s} mm | {"busy ON":>7s} | coll t (s) median')
for tag in ('zero', 'v10c', 'prec3.75', 'ens_soup9'):
    for sc in ('dynamic', 'static'):
        E = OFF.get((tag, sc), {}); R = load(ON[tag].format(sc=sc))
        k = sorted(s for s in E if s in R)
        if not E: continue
        if not k:   # no paired shield-on reference for these seeds
            k = sorted(E); R = None
        h_off = sum(E[s]['held'] for s in k); col = [s for s in k if E[s]['coll']]
        z = ZOFF[sc]; zc = sum(1 for s in col if s in z and z[s]['coll'])
        tcol = sorted(E[s]['t'][-1] for s in col)
        row = f'{tag:10s} {sc:7s} {len(k):4d} | '
        if R:
            h_on = sum(R[s]['held'] for s in k)
            pl = sum(E[s]['held'] and not R[s]['held'] for s in k); mi = sum(R[s]['held'] and not E[s]['held'] for s in k)
            busy = mean([mean([v > 0.5 for v in R[s]['iv']]) for s in k])
            row += f'{h_on:7d} {h_off:4d} {f"+{pl}/-{mi}":>8s} {mcn(pl, mi):6.3f} | '
        else:
            row += f'{"-":>7s} {h_off:4d} {"":>8s} {"":>6s} | '
        row += f'{len(col):8d} {f"{zc}/{len(col)}" if tag != "zero" else "":>15s} | '
        row += (f'{sum(R[s]["dmin"] < 0.15 for s in k):9d} ' if R else f'{"-":>9s} ') + f'{sum(E[s]["dmin"] < 0.15 for s in k):4d} | '
        row += (f'{1000 * min(R[s]["dmin"] for s in k):7.0f} ' if R else f'{"-":>7s} ') + f'{1000 * min(E[s]["dmin"] for s in k):5.0f}    | '
        row += (f'{busy:7.2f} | ' if R else f'{"-":>7s} | ') + (f'{tcol[len(tcol) // 2]:.2f}' if tcol else '-')
        print(row)
