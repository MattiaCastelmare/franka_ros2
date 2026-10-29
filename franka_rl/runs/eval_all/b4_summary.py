"""b4 summary (2026-10-07, from b3_summary.py + REF ft_v10c 4.0M): shield-off fine-tunes vs references on the SAME seeds.
    cd runs/eval_all && python3 b4_summary.py traces/b4_eval
OFF = obstacle CBF rows off at test time (what an end-to-end policy must survive), ON = shield on.
held = target held at 5 s; coll = episodes ending in a collision; <ds = episodes going below d_safe 0.15 m.
"""
import json, glob, re, os, sys
from collections import defaultdict
D = sys.argv[1]

def load(pat):
    eps = {}
    for f in glob.glob(pat):
        for e in json.load(open(f))['eps']: eps[e['seed']] = e
    return eps

G = defaultdict(dict)
for f in glob.glob(f'{D}/*.json'):
    tag, cond, sc = re.match(r'(.+)_(off|on)_(static|dynamic)_\d+\.json', os.path.basename(f)).groups()
    G[(tag, cond, sc)].update(load(f))
REF = {
    ('REF ens_soup9', 'off'): 'traces/noshield/ens_soup9_{sc}_*.json', ('REF ens_soup9', 'on'): 'traces/avg/ens_soup9_{sc}_*.json',
    ('REF ft_prec', 'off'): 'traces/noshield/prec3.75_{sc}_*.json', ('REF ft_prec', 'on'): 'traces/v11conf/sac_v11_ft_prec_3750000_{sc}_*.json',
    ('REF v10c', 'off'): 'traces/noshield/v10c_{sc}_*.json', ('REF v10c', 'on'): 'traces/v11conf/sac_v10c_2500000_{sc}_*.json',
    ('REF zero', 'off'): 'traces/noshield/zero_{sc}_*.json',
    ('REF ftv10c4M', 'off'): 'traces/b3_final/ftv10c_4000k_off_{sc}_*.json', ('REF ftv10c4M', 'on'): 'traces/b3_final/ftv10c_4000k_on_{sc}_*.json',
}
seeds = {sc: set().union(*[set(E) for (t, c, s), E in G.items() if s == sc]) for sc in ('static', 'dynamic')}
for (tag, cond), p in REF.items():
    for sc in ('static', 'dynamic'):
        E = load(p.format(sc=sc)); G[(tag, cond, sc)] = {s: E[s] for s in seeds[sc] if s in E}

def cell(E):
    if not E: return f'{"-":>21s}'
    n = len(E); v = list(E.values())
    return (f'{sum(e["held"] for e in v):3d} {sum(e["coll"] for e in v):3d} {sum(e["dmin"] < 0.15 for e in v):3d} '
            f'{1000 * min(e["dmin"] for e in v):5.0f} {n:4d}')

hdr = 'held coll <ds dminmm    n'
print(f'{"":14s}| {"OFF dynamic":^21s} | {"OFF static":^21s} | {"ON dynamic":^21s} | {"ON static":^21s}')
print(f'{"model":14s}| {hdr:>21s} | {hdr:>21s} | {hdr:>21s} | {hdr:>21s}')
tags = sorted({t for t, _, _ in G if not t.startswith('REF')}) + sorted({t for t, _, _ in G if t.startswith('REF')})
for t in tags:
    print(f'{t:14s}| ' + ' | '.join(cell(G.get((t, c, sc))) for c in ('off', 'on') for sc in ('dynamic', 'static')))
