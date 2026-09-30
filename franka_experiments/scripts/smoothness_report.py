#!/usr/bin/env python3
"""Smoothness of the commanded / realised joint motion in a recorded bag.

Reads /NS_1/qddot_safe (CBF output), /NS_1/qddot_nom, /NS_1/franka/joint_states
(fast, joint order scrambled -> mapped by name) or /NS_1/joint_states, and reports
per-run: command jerk (d qddot_safe/dt) percentiles, |qddot_safe - qddot_nom| when the
filter acts, realised joint acceleration vs command (trk_err), and peak velocity.

    python3 scripts/smoothness_report.py rosbag/ball_throws_3 [rosbag/ball_throws_4 ...]
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ball_throw_eval import read_bag, QDD_SAFE, QDD_NOM

K = [f'fr3_joint{i}' for i in range(1, 8)]
FAST = '/NS_1/franka/joint_states'


def load(bag):
    safe, nom, js = [], [], []
    for tp, recv, m in read_bag(bag, {QDD_SAFE, QDD_NOM, FAST}):
        if tp == QDD_SAFE: safe.append((recv, np.array(m.data[:7])))
        elif tp == QDD_NOM: nom.append((recv, np.array(m.data[:7])))
        else:
            d = dict(zip(m.name, zip(m.position, m.velocity)))
            if all(k in d for k in K):
                js.append((recv, np.array([d[k][0] for k in K]), np.array([d[k][1] for k in K])))
    return safe, nom, js


def pct(x, p=(50, 90, 99, 100)):
    return ' '.join(f'p{q}={np.percentile(x, q):7.2f}' for q in p)


def report(bag):
    safe, nom, js = load(bag)
    ts = np.array([s[0] for s in safe]); Qs = np.array([s[1] for s in safe])
    tn = np.array([s[0] for s in nom]); Qn = np.array([s[1] for s in nom])
    print(f'== {bag}: {len(safe)} safe, {len(nom)} nom, {len(js)} joint_states')
    if len(ts) < 3: return
    dt = np.diff(ts); h = float(np.median(dt))
    # recv-time jitter (executor batching) would turn a 5 rad/s^2 tick step into 2700 rad/s^3:
    # use the nominal tick, the filter's own clock.
    jerk = np.diff(Qs, axis=0) / h
    print(f' safe rate {1/h:.0f} Hz')
    dev = np.abs(Qs - Qn[np.clip(np.searchsorted(tn, ts) - 1, 0, len(tn) - 1)])
    act = dev.max(axis=1) > 0.5
    print(f' filter active {act.mean()*100:.1f}% of ticks; |safe-nom| when active: {pct(dev.max(axis=1)[act]) if act.any() else "-"}')
    print(f' |command jerk| rad/s^3 all   : {pct(np.abs(jerk).max(axis=1))}')
    if act[1:].any():
        print(f' |command jerk| rad/s^3 active: {pct(np.abs(jerk).max(axis=1)[act[1:]])}')
    print(f' |qddot_safe| max over joints  : {pct(np.abs(Qs).max(axis=1))}')
    # chatter: the part of q̈_safe faster than ~5 Hz (minus a 5-tick moving average)
    k = 5; ker = np.ones(k) / k
    sm = np.stack([np.convolve(Qs[:, i], ker, mode='same') for i in range(7)], axis=1)
    hf = np.abs(Qs - sm).max(axis=1)[k:-k]
    flips = (np.diff(np.sign(Qs[:, :]), axis=0) != 0) & (np.abs(Qs[1:]) > 0.5) & (np.abs(Qs[:-1]) > 0.5)
    print(f' HF chatter |q̈ - MA5| rad/s^2 : {pct(hf)}   big sign flips/s (|q̈|>.5 both sides): {flips.sum() / (ts[-1]-ts[0]):.2f}')
    if js:
        tj = np.array([j[0] for j in js]); V = np.array([j[2] for j in js])
        # realised accel from velocity on a 30 ms grid to reject 1 kHz quantisation noise
        g = np.arange(tj[0], tj[-1], 0.01); Vg = np.array([np.interp(g, tj, V[:, i]) for i in range(7)]).T
        A = (Vg[6:] - Vg[:-6]) / 0.06; J = (A[6:] - A[:-6]) / 0.06
        print(f' realised |accel| rad/s^2      : {pct(np.abs(A).max(axis=1))}')
        print(f' realised |jerk| rad/s^3       : {pct(np.abs(J).max(axis=1))}')
        print(f' peak |qdot| per joint         : {np.round(np.abs(V).max(axis=0), 2)}')
        ia = np.clip(np.searchsorted(ts, g[6:-6] + 0.03) - 1, 0, len(ts) - 1)
        e = np.abs(A[3:-3] - Qs[ia][3:len(A) - 3 + 0 if False else None][:len(A[3:-3])]) if False else None
        m = min(len(A), len(ia)); err = np.abs(A[:m] - Qs[ia[:m]]).max(axis=1)
        print(f' trk_err |a_real - safe| max_j : {pct(err)}')


if __name__ == '__main__':
    for b in sys.argv[1:]:
        report(b)
