"""Where does a model still fail? Bins failure rate by target/obstacle geometry on the held-out traces."""
import json, glob, sys, numpy as np
T = 'traces/v11conf'
def load(r, s, sc):
    eps = {}
    for f in glob.glob(f'{T}/{r}_{s}_{sc}_*.json'):
        for e in json.load(open(f))['eps']: eps[e['seed']] = e
    return eps
def cat(e):
    if e['held']: return 'held'
    if e['reached']: return 'near' if e['err_f'] < 0.08 else 'lost'
    return 'barrier' if min(e['d'][-20:]) < 0.17 else 'never'
def seg_dist(p, a, b):
    ab = b - a; t = np.clip(np.dot(p - a, ab) / max(1e-9, ab @ ab), 0, 1); return np.linalg.norm(p - (a + t * ab))
for sc in ('static', 'dynamic'):
    W = load('sac_v11_ft_prec', '3750000', sc); B = load('sac_v10c', '2500000', sc)
    rows = []
    for s, e in W.items():
        tg = np.array(e['target']); ee0 = np.array(e['ee'][0]); ob0 = np.array(e['ob'][0]); ob = np.array(e['ob'])
        obc = ob.mean(0)
        rows.append(dict(seed=s, c=cat(e), cb=cat(B[s]), tz=tg[2], tx=tg[0], ty=abs(tg[1]), dist0=np.linalg.norm(tg - ee0),
                         blk=seg_dist(obc, ee0, tg) < 0.23, otd=np.linalg.norm(obc - tg), oz=obc[2],
                         errv=np.array(e['ee'][-1]) - tg, vend=e['err'][-1], dmin=e['dmin'], tot=e['tot']))
    n = len(rows); print(f'\n===== {sc}  n={n}  held {sum(r["c"]=="held" for r in rows)}')
    def binrate(key, edges, label):
        out = []
        for lo, hi in zip(edges[:-1], edges[1:]):
            g = [r for r in rows if lo <= r[key] < hi]
            if g: out.append(f'[{lo:.2f},{hi:.2f}) n={len(g):3d} fail {100*sum(r["c"]!="held" for r in g)/len(g):4.0f}% (v10c {100*sum(r["cb"]!="held" for r in g)/len(g):3.0f}%)')
        print(label); [print('   ', o) for o in out]
    binrate('tz', [0.25, 0.35, 0.45, 0.55, 0.66], 'target z')
    binrate('tx', [0.3, 0.4, 0.5, 0.61], 'target x')
    binrate('ty', [0, 0.12, 0.24, 0.36], 'target |y|')
    binrate('otd', [0, 0.30, 0.40, 0.50, 2], 'obstacle-centre to target (mean over episode)')
    binrate('dist0', [0, 0.3, 0.45, 0.6, 2], 'start EE to target')
    for blk in (True, False):
        g = [r for r in rows if r['blk'] == blk]
        print(f'path blocked={blk}: n={len(g)} fail {100*sum(r["c"]!="held" for r in g)/len(g):.0f}% (v10c {100*sum(r["cb"]!="held" for r in g)/len(g):.0f}%)',
              {c: sum(r['c'] == c for r in g) for c in ('near', 'lost', 'barrier', 'never')})
    near = [r for r in rows if r['c'] == 'near']
    if near:
        ev = np.array([r['errv'] for r in near])
        print(f'near-miss final error vector mean (mm) x {1000*ev[:,0].mean():.0f} y {1000*ev[:,1].mean():.0f} z {1000*ev[:,2].mean():.0f};'
              f' |dominant axis| counts', np.bincount(np.abs(ev).argmax(1), minlength=3), ' time-on-target median', np.median([r['tot'] for r in near]))
        print('   z error: below target', int((ev[:, 2] < -0.02).sum()), 'above', int((ev[:, 2] > 0.02).sum()), ' obstacle-target dist median', np.median([r['otd'] for r in near]).round(2))
    bar = [r for r in rows if r['c'] in ('barrier', 'never')]
    if bar: print(f'never-reached n={len(bar)}: blocked {sum(r["blk"] for r in bar)}, obstacle-target median {np.median([r["otd"] for r in bar]):.2f}, '
                  f'final err median {np.median([r["vend"] for r in bar])*1:.0f} mm, v10c also failed {sum(r["cb"]!="held" for r in bar)}')
    lost = [r for r in rows if r['c'] == 'lost']
    if lost: print(f'lost after reach n={len(lost)}: final err median {np.median([r["vend"] for r in lost]):.0f} mm, tot median {np.median([r["tot"] for r in lost]):.2f}, otd median {np.median([r["otd"] for r in lost]):.2f}')
