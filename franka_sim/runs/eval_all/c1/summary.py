"""c1 summary (2026-10-08): every tag in the given trace dirs vs references on the SAME seeds.
    cd runs/eval_all && python3 c1/summary.py traces/c1_eval [traces/c1_p0 ...]   [REFSEEDS=fresh]
held = target held at 5 s; rnh = reached but not held; nr = never reached; coll = collision; <ds = below d_safe.
McNemar (exact, two-sided) vs REF_ON = ft_prec 3.75M (ON) / REF_OFF = ftv10c 4.0M (OFF).
"""
import json, glob, re, os, sys
from math import comb
from collections import defaultdict
FRESH = os.environ.get('REFSEEDS') == 'fresh'

def load(pat):
    eps = {}
    for f in glob.glob(pat):
        for e in json.load(open(f))['eps']: eps[e['seed']] = e
    return eps

def mcnemar(b, c):
    n = b + c
    if n == 0: return 1.0
    k = min(b, c)
    return min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n)

G = defaultdict(dict)
for D in sys.argv[1:]:
    for f in glob.glob(f'{D}/*.json'):
        m = re.match(r'(.+)_(off|on)_(static|dynamic)_\d+\.json', os.path.basename(f))
        if m: G[m.groups()].update(load(f))
if FRESH:
    REF = {('REF ens_soup9', 'on'): 'traces/avg_conf/ens_soup9_{sc}_*.json', ('REF ft_prec', 'on'): 'traces/avg_conf/prec3.75_{sc}_*.json',
           ('REF avg_k4', 'on'): 'traces/avg_conf/avg_k4_{sc}_*.json', ('REF v10c', 'on'): 'traces/avg_conf/v10c_{sc}_*.json'}
else:
    REF = {
        ('REF ens_soup9', 'off'): 'traces/noshield/ens_soup9_{sc}_*.json', ('REF ens_soup9', 'on'): 'traces/avg/ens_soup9_{sc}_*.json',
        ('REF ft_prec', 'off'): 'traces/noshield/prec3.75_{sc}_*.json', ('REF ft_prec', 'on'): 'traces/v11conf/sac_v11_ft_prec_3750000_{sc}_*.json',
        ('REF prec_s3', 'on'): 'traces/v12/sac_v12_prec_s3_3750000_{sc}_*.json',
        ('REF prec_s2', 'on'): 'traces/v12/sac_v12_prec_s2_3500000_{sc}_*.json',
        ('REF ftv10c4M', 'off'): 'traces/b3_final/ftv10c_4000k_off_{sc}_*.json', ('REF ftv10c4M', 'on'): 'traces/b3_final/ftv10c_4000k_on_{sc}_*.json',
        ('REF zero', 'off'): 'traces/noshield/zero_{sc}_*.json',
    }
for (tag, cond), p in REF.items():
    for sc in ('static', 'dynamic'):
        E = load(p.format(sc=sc))
        if E: G[(tag, cond, sc)] = E
seeds = {sc: set().union(*[set(E) for (t, c, s), E in G.items() if s == sc and not t.startswith('REF')] or [set()]) for sc in ('static', 'dynamic')}
for k in list(G):
    if k[0].startswith('REF') and seeds[k[2]]:
        G[k] = {s: e for s, e in G[k].items() if s in seeds[k[2]]}
refname = {'on': 'REF ft_prec', 'off': 'REF ftv10c4M'}

def cell(t, c, sc):
    E = G.get((t, c, sc))
    if not E: return f'{"-":>34s}'
    v = list(E.values()); n = len(v)
    held = sum(e['held'] for e in v); coll = sum(e['coll'] for e in v)
    rnh = sum(e['reached'] and not e['held'] and not e['coll'] for e in v)
    ds = sum(e['dmin'] < 0.15 for e in v)
    R = G.get((refname[c], c, sc), {})
    common = [s for s in E if s in R]
    b = sum(E[s]['held'] and not R[s]['held'] for s in common); cc = sum(R[s]['held'] and not E[s]['held'] for s in common)
    p = mcnemar(b, cc) if common and not t == refname[c] else 1.0
    return f'{held:3d} {rnh:3d} {coll:3d} {ds:3d} {n:3d} +{b:<2d}-{cc:<3d}p{p:5.3f}'

hdr = 'held rnh coll <ds   n  vs-ref'
print(f'{"":20s}| {"OFF dynamic":^34s} | {"OFF static":^34s} | {"ON dynamic":^34s} | {"ON static":^34s}')
print(f'{"model":20s}| ' + ' | '.join(f'{hdr:>34s}' for _ in range(4)))
tags = sorted({t for t, _, _ in G if not t.startswith('REF')}) + sorted({t for t, _, _ in G if t.startswith('REF')})
for t in tags:
    print(f'{t:20s}| ' + ' | '.join(cell(t, c, sc) for c in ('off', 'on') for sc in ('dynamic', 'static')))
