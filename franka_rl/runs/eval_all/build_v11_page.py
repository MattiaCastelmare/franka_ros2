"""Collect everything the v11 results page shows into one JSON.   build_v11_page.py WIN_RUN:STEP OUT.json"""
import json, glob, os, re, sys
from math import comb
H = os.path.dirname(os.path.abspath(__file__)); T = H + '/traces'
win_r, win_s = sys.argv[1].split(':'); out = sys.argv[2]
RUNS = [('sac_v11_ft_lr1e4', 'lr 1e-4', 'riferimento dei fine-tune'),
        ('sac_v11_ft_lr1e4_s2', 'lr 1e-4, altro seed', 'misura il rumore tra training'),
        ('sac_v11_ft_prec', '+ bonus di precisione', 'w_prec 2 · (1 − tanh(d / 5 cm))'),
        ('sac_v11_ft_static', '+ 75 % ostacolo fermo', 'static_fraction 0.5 → 0.75'),
        ('sac_v11_ft_obspot', '+ shaping ostacolo', 'potenziale su d_min, w 2, 0.30 m'),
        ('sac_v11_ft_combo', '+ precisione + 75 % fermo', 'prec e static insieme'),
        ('sac_v11_ft_utd2', '+ 2 gradient step', 'gradient_steps 1 → 2'),
        ('sac_v11_scratch_prec', 'da zero + precisione', 'ricetta v10 + w_prec, 2M step')]
def load(files):
    eps = {}
    for f in files:
        for e in json.load(open(f))['eps']: eps[e['seed']] = e
    return eps
def mcn(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)
def agg(eps):
    v = list(eps.values())
    return dict(n=len(v), held=sum(e['held'] for e in v), reached=sum(e['reached'] for e in v),
                coll=sum(e['coll'] for e in v), floor=sum(e['floor'] for e in v),
                dmin=round(1000 * min(e['dmin'] for e in v), 1),
                dlt=sum(e['dmin'] < 0.15 for e in v))
def cat(e):
    if e['held']: return 'held'
    if e['reached']: return 'near' if e['err_f'] < 0.08 else 'lost'
    return 'barrier' if min(e['d'][-20:]) < 0.17 else 'never'
def pair(a, b):
    k = [s for s in a if s in b]
    p = sum(a[s]['held'] and not b[s]['held'] for s in k); m = sum(b[s]['held'] and not a[s]['held'] for s in k)
    return dict(plus=p, minus=m, p=mcn(p, m))
SC = ('dynamic', 'static')
base140 = {sc: load([f'{T}/v10c/2500000_{sc}.json'] + glob.glob(f'{T}/v10c/fresh_checkpoints/sac_2500000_steps_{sc}_*.json')) for sc in SC}
# checkpoint grid on the 140 seeds
grid = {}
for f in glob.glob(f'{T}/v11/*.json'):
    m = re.match(r'(.+)_(\d+|final_model|best_model)_(static|dynamic)_(\d+)\.json', os.path.basename(f))
    grid.setdefault(m[1], {}).setdefault(m[2], {}).setdefault(m[3], []).append(f)
G = {}
for r, steps in grid.items():
    for s, scs in steps.items():
        row = {}
        for sc, fs in scs.items():
            eps = load(fs)
            if len(eps) < 140: continue
            row[sc] = {**agg(eps), **pair(eps, base140[sc])}
        if len(row) == 2: G.setdefault(r, {})[s] = row
# held-out 100 seeds
conf = {}
cb = {sc: load(glob.glob(f'{T}/v11conf/sac_v10c_2500000_{sc}_*.json')) for sc in SC}
for f in glob.glob(f'{T}/v11conf/*.json'):
    m = re.match(r'(.+)_(\d+)_(static|dynamic)_(\d+)\.json', os.path.basename(f))
    conf.setdefault(f'{m[1]}:{m[2]}', set()).add(m[3])
C = {}
for k in conf:
    r, s = k.split(':'); row = {}
    for sc in SC:
        eps = load(glob.glob(f'{T}/v11conf/{r}_{s}_{sc}_*.json'))
        row[sc] = {**agg(eps), **(pair(eps, cb[sc]) if r != 'sac_v10c' else {})}
    C[k] = row
# winner vs baseline, per seed (140 + 100 held-out), with light traces for the explorer
def wload(r, s, sc):  # held-out seeds only: the winner was NOT selected on these
    return load(glob.glob(f'{T}/v11conf/{r}_{s}_{sc}_*.json'))
def slim(e):
    st = 2  # every 10th step (traces are already every 5th)
    return dict(held=e['held'], reached=e['reached'], cat=cat(e), ef=round(1000 * e['err_f'], 1), dm=round(1000 * e['dmin'], 1),
                tot=e['tot'], coll=e['coll'], floor=e['floor'],
                err=[round(1000 * x) for x in e['err'][::st]], d=[round(1000 * x) for x in e['d'][::st]],
                xy=[[round(p[0], 3), round(p[1], 3)] for p in e['ee'][::st]],
                ob=[[round(p[0], 3), round(p[1], 3)] for p in e['ob'][::st]], tg=e['target'][:2])
P = {}
for sc in SC:
    w = wload(win_r, win_s, sc); b = wload('sac_v10c', '2500000', sc)
    P[sc] = [dict(seed=s, w=slim(w[s]), b=slim(b[s])) for s in sorted(w) if s in b]
fails = {sc: {who: {} for who in ('w', 'b')} for sc in SC}
for sc in SC:
    for row in P[sc]:
        for who in ('w', 'b'):
            c = row[who]['cat']; fails[sc][who][c] = fails[sc][who].get(c, 0) + 1
curves = json.load(open(H + '/v11_curves.json'))
json.dump(dict(runs=RUNS, win=f'{win_r}:{win_s}', grid=G, conf=C, paired=P, fails=fails, curves=curves),
          open(out, 'w'), separators=(',', ':'))
print('wrote', out, os.path.getsize(out) // 1024, 'kB')
