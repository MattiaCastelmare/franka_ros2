#!/usr/bin/env python3
"""Approach-speed envelope (docs/approach_speed_envelope.md): hit/miss per speed x scenario, plus WHICH constraint binds.

Uses cbf_scenarios.run_scenario UNCHANGED (real ConstraintBuilder + OSQP, double-integrator arm, emulated
perception). Only the perception numbers are set from measurements (approach_probe.py rates / depth):
    fps 76        processed frames/s of the live node (bag ball_throws_3: 75.8 Hz)
    t_cam 0.048   capture stamp -> CBF receipt, median (same bag)
    t_sep 0.15    release -> ball is a cluster of its own (approach_probe.py depth, ~0.12-0.18 s median)
Constraint attribution = counterfactual: re-run the cell with ONE constraint relaxed; the relaxation that
flips a hit into a miss (else gives the largest clearance gain) is the binding one.

    python3 scripts/approach_envelope.py [--json out.json] [--speeds 0.5 1 ...]
"""
import argparse, json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cbf_scenarios as C

Y = np.array([0.0, 1.0, 0.0])


def _fifo_run_scenario():
    """cbf_scenarios.run_scenario with a delivery QUEUE instead of one in-flight slot.

    The shipped emulator keeps a single pending frame, so whenever capture->receipt latency exceeds the frame
    period (48 ms vs 13 ms live) every frame is overwritten before it arrives and the filter sees nothing
    (verified: ball 1.5 m/s, fps 76, t_cam .013 -> retreat 0). Patched at run time from the source, asserted
    exact, so cbf_scenarios.py itself is untouched."""
    import inspect
    src = inspect.getsource(C.run_scenario)
    a = ("            last_frame = (t_cap, snap, cps)\n            msg_t_recv = t_cap + t_cam\n"
         "            pending = last_frame\n"
         "        if last_msg is None or (msg_t_recv >= 0 and t >= msg_t_recv and last_msg[0] < pending[0]):\n"
         "            if msg_t_recv >= 0 and t >= msg_t_recv:\n"
         "                last_msg = pending\n")
    b = ("            fifo.append((t_cap + t_cam, (t_cap, snap, cps)))\n"
         "        while fifo and fifo[0][0] <= t:\n"
         "            last_msg = fifo.pop(0)[1]\n")
    assert a in src and "    last_msg = None " in src
    src = src.replace(a, b).replace("    last_msg = None ", "    fifo = []\n    last_msg = None ", 1)
    ns = dict(C.__dict__)
    exec(compile(src, 'run_scenario_fifo', 'exec'), ns)
    return ns['run_scenario']


run_scenario = _fifo_run_scenario()
R_BALL = 0.036
BASE = dict(fps=76.0, t_cam=0.048, t_sep=0.15, max_thresh=0.7, over={})
# one relaxation each; 'physical' is the rest: every one at once
RELAX = {
    'perception latency (t_cam->0.013)': dict(t_cam=0.013),
    'ball/thrower separation (t_sep->0)': dict(t_sep=0.0),
    'publish gate (0.7->1.5 m)': dict(max_thresh=1.5),
    'track trust (fast_min_frames 3->1)': dict(over=dict(obstacle_velocity_fast_min_frames=1)),
    'arm authority (box 10, margin .9, slew 5)': dict(over=dict(
        velocity_box_margin=0.9, max_qddot_delta=5.0)),
}


def cell(kind, v, L, cfg, seed=0):
    p = {**BASE, **cfg}
    over = dict(p['over'])
    if 'arm authority' in str(cfg.get('_tag', '')):
        pass
    P = C.shipped_params(**over)
    if cfg.get('_wide_box'):
        P.qddot_accel_limits = [10.0] * 7
    F0 = C.Filter(P)
    flange = F0.control_points(C.Q0, np.zeros(C.NV))[-1][1]
    if kind == 'a':   d0, tsep = 2.5, 0.0                       # tracked from far
    elif kind == 'b': d0, tsep = 0.7 + R_BALL + C.R_CAP, 0.0    # first seen AT the publish gate
    else:             d0, tsep = L, p['t_sep']                  # thrown from L, visible t_sep after release
    d0 = max(d0 - v * tsep, 0.3)
    T = min(d0 / v + 0.5, 6.0)
    r = run_scenario('cell', P, [C.Sphere(flange + d0 * Y, vel=-v * Y, r=R_BALL)], T=T, fps=p['fps'],
                       t_cam=p['t_cam'], max_thresh=p['max_thresh'], seed=seed)
    return r['min_h'] + P.d_safe, r['v_obs_max']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speeds', type=float, nargs='*', default=[0.5, 1, 1.5, 2, 3, 4, 5, 6])
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    cases = [('a', 0)] + [('b', 0)] + [('c', L) for L in (1.5, 2.0, 3.0, 4.0)]
    out = []
    print('gap = min surface gap [cm] (<0 = hit). binding = relaxation that flips/most improves.')
    for kind, L in cases:
        name = {'a': '(a) tracked from afar', 'b': '(b) first seen at 0.7 m gate'}.get(kind, f'(c) aimed throw, release at {L} m')
        print(f'\n{name}')
        print(' v[m/s]  gap[cm]  hit | binding constraint (gap if relaxed alone)')
        for v in a.speeds:
            g0, vo = cell(kind, v, L, {})
            rel = {}
            for k, cfg in RELAX.items():
                cfg = dict(cfg)
                if k.startswith('arm authority'):
                    cfg['_wide_box'] = True
                rel[k], _ = cell(kind, v, L, cfg)
            allr = {}
            for cfg in RELAX.values():
                for kk, vv in cfg.items():
                    allr[kk] = {**allr.get(kk, {}), **vv} if isinstance(vv, dict) else vv
            allr['_wide_box'] = True
            g_all, _ = cell(kind, v, L, allr)
            hit = g0 < 0
            if not hit:
                bind = '-'
            else:
                flips = [k for k, g in rel.items() if g >= 0]
                if flips:
                    bind = ' + '.join(flips) + ' (each alone flips)'
                else:
                    k = max(rel, key=rel.get)
                    bind = (f'none alone; best {k} ({rel[k]*100:+.0f} cm)' if rel[k] - g0 > 0.01 else 'none alone') + \
                           (f'; all together {g_all*100:+.0f} cm -> ' + ('flight time' if g_all < 0 else 'combination'))
            print(f' {v:5.1f}  {g0*100:7.1f}  {"HIT" if hit else " ok"} | {bind}', flush=True)
            out.append(dict(case=name, v=v, gap=g0, hit=hit, relaxed=rel, all=g_all, binding=bind))
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
