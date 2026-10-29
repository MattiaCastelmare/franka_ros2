"""Data for the b3 page (2026-10-06) -> OUT/data.json.   python3 b3_page_data.py OUT BEST_STEP  (run from runs/eval_all)
OFF = obstacle CBF rows off at test time, ON = shield on.  Held-out seeds 3000:100 + 4000:300 (screen: 3000:100)."""
import json, glob, re, os, sys, numpy as np
from math import comb
OUT, BEST = sys.argv[1], sys.argv[2]
SC = ('dynamic', 'static')
MOD = '../../models'
def mcn(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)
def load(pat):
    E = {}
    for f in glob.glob(pat):
        for e in json.load(open(f))['eps']: E[e['seed']] = e
    return E
def tcoll(e):   # time of the collision [s] (last trace sample), None if no collision
    return e['t'][-1] if e['coll'] else None
def stats(E, seeds=None):
    v = [E[s] for s in (seeds if seeds is not None else E)]
    held = [e for e in v if e['held']]
    return dict(n=len(v), held=sum(e['held'] for e in v), reached=sum(e['reached'] for e in v), coll=sum(e['coll'] for e in v),
                coll1=sum(e['coll'] and tcoll(e) <= 1.0 for e in v), below=sum(e['dmin'] < 0.15 - 1e-3 for e in v),
                dmin=round(1000 * min(e['dmin'] for e in v)) if v else 0,
                err=round(1000 * float(np.median([e['err_f'] for e in held])), 1) if held else None,
                tot=round(float(np.median([e['tot'] for e in held])), 3) if held else None)

# ---- training curves (EvalCallback, deterministic; b3 runs evaluate with their own config = shield OFF)
RUNS = ['base', 'base_s2', 'base_s3', 'margin30', 'wobs5', 'coll200', 'prec', 'obs51', 'noterm', 'ft_v10c']
curves = {}
for r in ['b3_' + x for x in RUNS] + ['v10']:
    z = np.load(f'{MOD}/sac_{r}/evaluations.npz')
    curves[r] = dict(t=(z['timesteps'] / 1e6).round(3).tolist(), succ=z['successes'].mean(1).round(3).tolist(),
                     len=z['ep_lengths'].mean(1).round(0).astype(int).tolist())
# training-log safety metrics (rollouts, stochastic policy): collision_rate per step, success_rate
logs = {}
for r in ['b3_' + x for x in RUNS] + ['v10']:
    t, cr, sr, ms = [], [], [], []
    blk = {}
    for line in open(f'../sac_{r}_train.log', errors='ignore'):
        m = re.match(r'\|\s+(\w+)\s+\|\s+([-\d.e+]+)\s+\|', line)
        if m: blk[m.group(1)] = float(m.group(2))
        if line.startswith('---') and 'total_timesteps' in blk and 'collision_rate' in blk:
            t.append(blk['total_timesteps']); cr.append(blk['collision_rate']); sr.append(blk.get('success_rate', 0)); blk = {}
    t, cr, sr = map(np.asarray, (t, cr, sr))
    if len(t) == 0: continue
    edges = np.linspace(t.min(), t.max(), 61); idx = np.digitize(t, edges[1:-1])
    logs[r] = dict(t=[round(float(t[idx == i].mean()) / 1e6, 3) for i in range(60) if (idx == i).any()],
                   coll=[round(float(cr[idx == i].mean()), 4) for i in range(60) if (idx == i).any()],
                   succ=[round(float(sr[idx == i].mean()), 3) for i in range(60) if (idx == i).any()])

# ---- 100-seed screen: scratch runs at 1M and 2M, + references on the same seeds
screen = {}
for step, d in (('1M', 'b3_1000k'), ('2M', 'b3_2000k')):
    for f in glob.glob(f'traces/{d}/*.json'):
        tag, cond, sc = re.match(r'(.+)_(off|on)_(static|dynamic)_\d+\.json', os.path.basename(f)).groups()
        if tag == 'b3_ft_v10c': continue
        screen.setdefault(tag, {}).setdefault(step, {}).setdefault(cond, {}).setdefault(sc, {}).update(
            {e['seed']: e for e in json.load(open(f))['eps']})
screen = {t: {st: {c: {sc: stats(E) for sc, E in C.items()} for c, C in S.items()} for st, S in T.items()} for t, T in screen.items()}

# ---- 400-seed benchmark: ft_v10c checkpoints, start point v10c 2.5M, references
REF = {('v10c 2.5M', 'off'): 'traces/noshield/v10c_{sc}_*.json', ('v10c 2.5M', 'on'): 'traces/v11conf/sac_v10c_2500000_{sc}_*.json',
       ('ft_prec 3.75M', 'off'): 'traces/noshield/prec3.75_{sc}_*.json', ('ft_prec 3.75M', 'on'): 'traces/v11conf/sac_v11_ft_prec_3750000_{sc}_*.json',
       ('ens_soup9', 'off'): 'traces/noshield/ens_soup9_{sc}_*.json', ('ens_soup9', 'on'): 'traces/avg/ens_soup9_{sc}_*.json',
       ('zero', 'off'): 'traces/noshield/zero_{sc}_*.json'}
S = {}
for (lab, c), p in REF.items():
    for sc in SC: S[(lab, c, sc)] = load(p.format(sc=sc))
steps = sorted({int(re.match(r'ftv10c_(\d+)k_', os.path.basename(f)).group(1)) for f in glob.glob('traces/b3_final/*.json')})
for k in steps:
    for c in ('off', 'on'):
        for sc in SC: S[(f'ftv10c_{k}k', c, sc)] = load(f'traces/b3_final/ftv10c_{k}k_{c}_{sc}_*.json')
seeds = {sc: sorted(S[('v10c 2.5M', 'off', sc)]) for sc in SC}
for key, E in S.items():
    assert set(seeds[key[2]]) <= set(E), (key, len(E))
grid = {f'{k}': {c: {sc: stats(S[(f'ftv10c_{k}k', c, sc)], seeds[sc]) for sc in SC} for c in ('off', 'on')} for k in steps}
grid['2500'] = {c: {sc: stats(S[('v10c 2.5M', c, sc)], seeds[sc]) for sc in SC} for c in ('off', 'on')}
refs = {lab: {c: {sc: stats(S[(lab, c, sc)], seeds[sc]) for sc in SC} for c in ('off', 'on') if (lab, c, 'static') in S}
        for lab in ('v10c 2.5M', 'ft_prec 3.75M', 'ens_soup9', 'zero')}
screen100 = {lab: {c: {sc: stats(S[(lab, c, sc)], [x for x in seeds[sc] if x < 3100]) for sc in SC} for c in ('off', 'on') if (lab, c, 'static') in S}
             for lab in ('v10c 2.5M', 'ft_prec 3.75M', 'ens_soup9', 'zero')}
# paired comparison of the chosen ft_v10c checkpoint vs its start (v10c 2.5M) and vs ens_soup9
B = f'ftv10c_{BEST}k'
def paired(a, b, c, sc, field):
    A, Bb = S[(a, c, sc)], S[(b, c, sc)]
    pl = sum(A[s][field] and not Bb[s][field] for s in seeds[sc]); mi = sum(Bb[s][field] and not A[s][field] for s in seeds[sc])
    return dict(plus=pl, minus=mi, p=round(mcn(pl, mi), 4))
pairs = {f'{a}|{b}|{c}|{sc}|{fld}': paired(a, b, c, sc, fld)
         for a, b in ((B, 'v10c 2.5M'), (B, 'ens_soup9'), (B, 'ft_prec 3.75M')) for c in ('off', 'on') for sc in SC for fld in ('held', 'coll')}
# collision-time histogram (OFF): when do collisions happen
ctime = {lab: {sc: [tcoll(S[(lab, 'off', sc)][s]) for s in seeds[sc] if S[(lab, 'off', sc)][s]['coll']] for sc in SC}
         for lab in (B, 'v10c 2.5M', 'ens_soup9', 'zero')}

# ---- explorer: OFF traces of v10c 2.5M, ft_v10c BEST, ens_soup9, + ft_v10c BEST ON
EX = [(('v10c 2.5M', 'off'), 'v10c 2.5M · OFF'), ((B, 'off'), f'ft_v10c {int(BEST) / 1000:g}M · OFF'),
      (('ens_soup9', 'off'), 'ens_soup9 · OFF'), ((B, 'on'), f'ft_v10c {int(BEST) / 1000:g}M · ON')]
def ds(a, n=50):
    a = np.asarray(a); idx = np.linspace(0, len(a) - 1, min(n, len(a))).round().astype(int); return a[idx]
ex = dict(models=[l for _, l in EX], eps={})
for sc in SC:
    out = {}
    for s in seeds[sc]:
        e0 = S[(B, 'off', sc)][s]
        o = dict(tg=[round(v, 3) for v in e0['target']], ob=(ds(np.array(e0['ob'])[:, :2]) * 1000).round().astype(int).tolist(), m=[])
        for (lab, c), _ in EX:
            e = S[(lab, c, sc)][s]
            o['m'].append(dict(h=int(e['held']), r=int(e['reached']), c=int(e['coll']), tc=tcoll(e), ef=round(1000 * e['err_f']), dm=round(1000 * e['dmin']),
                               err=(ds(e['err']) * 1000).round().astype(int).tolist(), d=(ds(e['d']) * 1000).round().astype(int).tolist(),
                               xy=(ds(np.array(e['ee'])[:, :2]) * 1000).round().astype(int).tolist()))
        out[s] = o
    ex['eps'][sc] = out
json.dump(dict(curves=curves, logs=logs, screen=screen, screen100=screen100, grid=grid, refs=refs, pairs=pairs, ctime=ctime, ex=ex, best=BEST),
          open(os.path.join(OUT, 'data.json'), 'w'), separators=(',', ':'))
print('ok', os.path.getsize(os.path.join(OUT, 'data.json')))
