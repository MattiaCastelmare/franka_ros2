"""Final metrics table (2026-10-09). Per model / seed set / shield condition / scenario:
held % [Wilson 95 % CI], std of held % over 25-seed chunks, collisions, episodes below d_safe, median min distance,
mean time on target, median final error (held eps), mean episode length (collision ends an episode early).
    cd runs/eval_all && python3 c2/final_table.py
"""
import json, glob, math, numpy as np
M = {  # name: {(set, cond): glob with {sc}}
 'ens_soup9 (prev deploy)': {('held-out', 'ON'): 'traces/avg/ens_soup9_{sc}_*.json', ('held-out', 'OFF'): 'traces/noshield/ens_soup9_{sc}_*.json',
                             ('fresh', 'ON'): 'traces/avg_conf/ens_soup9_{sc}_*.json', ('fresh', 'OFF'): 'traces/c1_conf/ens_soup9_off_{sc}_*.json'},
 'ens_off_big11 (new)':     {('held-out', 'ON'): 'traces/c2_ens/ens_big11_on_{sc}_*.json', ('held-out', 'OFF'): 'traces/c2_ens/ens_big11_off_{sc}_*.json',
                             ('fresh', 'ON'): 'traces/c1_conf/ens_big11_on_{sc}_*.json', ('fresh', 'OFF'): 'traces/c1_conf/ens_big11_off_{sc}_*.json'},
 'ens_off_mix6 (new, P0)':  {('held-out', 'ON'): 'traces/c1_p0/ens_off_mix6_on_{sc}_*.json', ('held-out', 'OFF'): 'traces/c1_p0/ens_off_mix6_off_{sc}_*.json',
                             ('fresh', 'ON'): 'traces/c1_conf/ens_off_mix6_on_{sc}_*.json', ('fresh', 'OFF'): 'traces/c1_conf/ens_off_mix6_off_{sc}_*.json'},
 'ft_prec 3.75M (single)':  {('held-out', 'ON'): 'traces/v11conf/sac_v11_ft_prec_3750000_{sc}_*.json', ('held-out', 'OFF'): 'traces/noshield/prec3.75_{sc}_*.json',
                             ('fresh', 'ON'): 'traces/avg_conf/prec3.75_{sc}_*.json'},
 'ft_v10c 4.0M (single)':   {('held-out', 'ON'): 'traces/b3_final/ftv10c_4000k_on_{sc}_*.json', ('held-out', 'OFF'): 'traces/b3_final/ftv10c_4000k_off_{sc}_*.json'},
}
def wilson(k, n, z=1.96):
    p = k / n; d = 1 + z * z / n; c = (p + z * z / (2 * n)) / d; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return 100 * (c - h), 100 * (c + h)
print(f'{"model":25s} {"set":8s} {"sh":3s} {"scen":7s} {"held % [95% CI]":>22s} {"chunk sd":>8s} {"coll":>4s} {"<ds":>4s} {"dmin mm":>7s} {"tot":>5s} {"err mm":>6s} {"len":>5s}')
for name, G in M.items():
    for (st, cond), pat in G.items():
        for sc in ('dynamic', 'static'):
            fs = sorted(glob.glob(pat.format(sc=sc)))
            E = {e['seed']: e for f in fs for e in json.load(open(f))['eps']}
            if len(E) != 400: print(name, st, cond, sc, 'n =', len(E)); continue
            v = [E[s] for s in sorted(E)]
            k = sum(e['held'] for e in v); lo, hi = wilson(k, 400)
            ch = [100 * np.mean([e['held'] for e in v[i:i + 25]]) for i in range(0, 400, 25)]
            L = np.mean([round(e['t'][-1] * 100) for e in v])
            print(f'{name:25s} {st:8s} {cond:3s} {sc:7s} {100 * k / 400:6.1f} [{lo:5.1f},{hi:5.1f}] ({k:3d}) {np.std(ch):8.1f} {sum(e["coll"] for e in v):4d} '
                  f'{sum(e["dmin"] < 0.15 for e in v):4d} {1000 * np.median([e["dmin"] for e in v]):7.0f} {np.mean([e["tot"] for e in v]):5.2f} '
                  f'{1000 * np.median([e["err_f"] for e in v if e["held"]]):6.1f} {L:5.0f}')
