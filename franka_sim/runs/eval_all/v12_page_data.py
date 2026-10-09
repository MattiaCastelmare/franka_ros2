"""Data for the v12 comparison page -> OUT/data.json.   python3 v12_page_data.py OUT  (run from runs/eval_all)"""
import json, glob, re, os, sys, numpy as np
from math import comb
OUT = sys.argv[1]
SC = ('dynamic', 'static')
def mcn(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)
def load(pat):
    E = {}
    for f in glob.glob(pat):
        for e in json.load(open(f))['eps']: E[e['seed']] = e
    return E
def src(key):  # key -> {sc: eps}
    d, tag = key.split('/', 1)
    return {sc: load(f'traces/{d}/{tag}_{sc}_*.json') for sc in SC}
# (key, label, group, note)
MODELS = [
 ('v11conf/sac_v10c_2500000', 'v10c 2.5M', 'single', 'miglior modello prima di v11'),
 ('v11conf/sac_v11_ft_prec_3750000', 'ft_prec 3.75M', 'single', 'miglior singolo v11, riferimento'),
 ('v12/sac_v12_prec_s3_3750000', 'prec_s3 3.75M', 'v12', 'replica di ft_prec da v10c 2.5M + buffer'),
 ('v12/sac_v12_prec_s2_3500000', 'prec_s2 3.5M', 'v12', 'replica di ft_prec, altro seed'),
 ('v12/sac_v12_lr3e5_4750000', 'lr3e5 4.75M', 'v12', 'da ft_prec senza buffer, lr 3e-5'),
 ('v12/sac_v12_prec4_4750000', 'prec4 4.75M', 'v12', 'da ft_prec senza buffer, w_prec 4'),
 ('v12/sac_v12_cont_4750000', 'cont 4.75M', 'v12', 'da ft_prec senza buffer, nessun cambio'),
 ('v12/sac_v12_hard_4750000', 'hard 4.75M', 'v12', 'curriculum: 30% target bassi + bloccati'),
 ('v12/sac_v12_hard_slack_4750000', 'hard_slack 4.75M', 'v12', 'hard + w_slack 2'),
 ('v12/sac_v12_slack_4750000', 'slack 4.75M', 'v12', 'w_slack 0.5 → 2'),
 ('avg/avg_k4', 'avg_k4', 'wavg', 'media dei pesi di 4 checkpoint ft_prec 3.25–4.0M'),
 ('avg/soup9', 'soup9 (pesi)', 'wavg', 'media dei pesi di 9 checkpoint di 3 run diversi'),
 ('avg/ens_prec3', 'ens_prec3', 'ens', '3 checkpoint di ft_prec (3.5/3.75/4.0M)'),
 ('avg/ens_soup3', 'ens_soup3', 'ens', 'ft_prec + lr1e4 + lr1e4_s2 a 3.75M'),
 ('avg/ens_avg3', 'ens_avg3', 'ens', '3 medie dei pesi, una per run'),
 ('avg/ens_soup9', 'ens_soup9', 'ens', '3 run × 3 checkpoint; esportato in ONNX'),
 ('avg/ens_v12new3', 'ens_v12new3', 'ens', 'solo membri v12: prec_s3, lr3e5, prec_s2'),
 ('avg/ens_mix3', 'ens_mix3', 'ens', 'ft_prec + prec_s3 + lr3e5'),
 ('avg/ens_mix6', 'ens_mix6', 'ens', 'ens_soup3 + i 3 membri v12'),
 ('avg/ens_mix13', 'ens_mix13', 'ens', 'ens_soup9 + 4 membri v12'),
]
S = {k: src(k) for k, *_ in MODELS}
REF = S['v11conf/sac_v11_ft_prec_3750000']
def row(E):
    r = {}
    for sc in SC:
        e = E[sc]; R = REF[sc]; k = sorted(set(e) & set(R))
        pl = sum(e[s]['held'] and not R[s]['held'] for s in k); mi = sum(R[s]['held'] and not e[s]['held'] for s in k)
        held = [e[s] for s in k if e[s]['held']]
        r[sc] = dict(n=len(k), held=sum(e[s]['held'] for s in k), plus=pl, minus=mi, p=round(mcn(pl, mi), 4),
                     coll=sum(e[s]['coll'] for s in k), floor=sum(e[s]['floor'] for s in k),
                     dmin=round(1000 * min(e[s]['dmin'] for s in k)), below=sum(e[s]['dmin'] < 0.15 - 1e-3 for s in k),
                     tot=round(float(np.median([x['tot'] for x in held])), 3) if held else 0,
                     err=round(1000 * float(np.median([x['err_f'] for x in held])), 1) if held else 0)
    return r
models = [dict(key=k, label=l, group=g, note=n, **row(S[k])) for k, l, g, n in MODELS]
# feasible benchmark for ens_soup9
FE = {sc: dict(S['avg/ens_soup9'][sc]) for sc in SC}
for f in glob.glob('traces/feas_eval/ens_soup9_*.json'):
    e = json.load(open(f))['eps'][0]; FE['static' if '_static_' in f else 'dynamic'][e['seed']] = e
feas = {sc: dict(held=sum(e['held'] for e in FE[sc].values()), n=len(FE[sc]), coll=sum(e['coll'] for e in FE[sc].values())) for sc in SC}
# v12 checkpoint grid
grid = {}
for f in glob.glob('traces/v12/*.json'):
    tag, sc = re.match(r'(.+)_(static|dynamic)_\d+\.json', os.path.basename(f)).groups()
    run, step = tag.rsplit('_', 1)
    grid.setdefault(run.replace('sac_v12_', ''), {}).setdefault(step, {}).setdefault(sc, 0)
    grid[run.replace('sac_v12_', '')][step][sc] += sum(e['held'] for e in json.load(open(f))['eps'])
# failure classes for ens_soup9 (static from feasibility analysis; see project memory 2026-10-02)
F = {json.load(open(f))['seed']: json.load(open(f)) for f in glob.glob('traces/feas/common_*.json')}
E9 = S['avg/ens_soup9']
cls = {}
for s, e in E9['static'].items():
    if e['held']: continue
    if s in F: cls[s] = 'infeasible' if not F[s]['feasible'] else ('marginal' if F[s]['min_err'] > 0.02 or s == 3093 else 'nearbase')
    else: cls[s] = 'inconsistent'
pts = [dict(seed=s, x=e['target'][0], y=e['target'][1], z=e['target'][2], c=cls.get(s, 'held')) for s, e in sorted(E9['static'].items())]
ENS = ['avg/' + t for t in ('ens_soup9', 'ens_mix13', 'ens_mix6', 'ens_soup3', 'ens_avg3', 'ens_v12new3', 'ens_mix3', 'ens_prec3')]
dyn_common = [s for s, e in E9['dynamic'].items() if all(not S[k]['dynamic'][s]['held'] for k in ENS)]
fails = dict(static={c: sum(v == c for v in cls.values()) for c in ('infeasible', 'marginal', 'nearbase', 'inconsistent')},
             dynamic=dict(common=len(dyn_common), other=sum(not e['held'] for e in E9['dynamic'].values()) - len(dyn_common)),
             pts=pts)
# explorer traces: 6 models, 50 points per episode
EX = [('v11conf/sac_v11_ft_prec_3750000', 'ft_prec 3.75M'), ('avg/ens_soup9', 'ens_soup9'), ('v12/sac_v12_prec_s3_3750000', 'prec_s3 3.75M'),
      ('v12/sac_v12_lr3e5_4750000', 'lr3e5 4.75M'), ('v12/sac_v12_cont_4000000', 'cont 4.0M'), ('v11conf/sac_v10c_2500000', 'v10c 2.5M')]
for k, _ in EX:
    if k not in S: S[k] = src(k)
def ds(a, n=50):
    a = np.asarray(a); idx = np.linspace(0, len(a) - 1, min(n, len(a))).round().astype(int); return a[idx]
ex = dict(models=[l for _, l in EX], eps={})
for sc in SC:
    out = {}
    for s in sorted(E9[sc]):
        ob = ds(np.array(E9[sc][s]['ob'])[:, :2]); tg = E9[sc][s]['target']
        o = dict(tg=[round(v, 3) for v in tg], ob=(ob * 1000).round().astype(int).tolist(), m=[])
        for k, _ in EX:
            e = S[k][sc][s]
            o['m'].append(dict(h=int(e['held']), r=int(e['reached']), c=int(e['coll']), ef=round(1000 * e['err_f']), dm=round(1000 * e['dmin']),
                               err=(ds(e['err']) * 1000).round().astype(int).tolist(), d=(ds(e['d']) * 1000).round().astype(int).tolist(),
                               xy=(ds(np.array(e['ee'])[:, :2]) * 1000).round().astype(int).tolist()))
        out[s] = o
    ex['eps'][sc] = out
json.dump(dict(models=models, feas=feas, grid=grid, fails=fails, ex=ex, ref_label='ft_prec 3.75M'),
          open(os.path.join(OUT, 'data.json'), 'w'), separators=(',', ':'))
print('ok', os.path.getsize(os.path.join(OUT, 'data.json')))
