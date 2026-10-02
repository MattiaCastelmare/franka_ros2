#!/usr/bin/env python3
"""One-page verdict on a recorded run: did the arm move, and was the motion smooth and inside its limits?

Reads /NS_1/qddot_safe (CBF output), /NS_1/qddot_nom, /NS_1/franka/joint_states (~1 kHz, joint order
scrambled -> mapped by name) or /NS_1/joint_states (30 Hz), /NS_1/ee_actual + /NS_1/ee_desired, and prints

  * whether the arm MOVED AT ALL (ball_throws_4 sat still for 135 s under a 6-7 rad/s² command, and its
    "tracking error" was command minus zero — every number below is meaningless then, so it says so first);
  * command: jerk, peak, chatter (the part faster than ~5 Hz), sign flips, how often the filter overrides the nominal;
  * realised: acceleration, jerk, tracking error |a_real - q̈_safe| (when the arm moves);
  * velocity against the FIRMWARE envelope (libfranka's position-based curve, what joint_velocity_violation trips on);
  * end-effector tracking error against the commanded path;
  * a PASS / WATCH / FAIL line per criterion, thresholds from docs/ball_throw_closed_loop.md.

    python3 scripts/smoothness_report.py rosbag/<run> [rosbag/<run2> ...]

Inside the ROS container, sourced, with the package source on PYTHONPATH (append: PYTHONPATH=$PWD:$PYTHONPATH).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ball_throw_eval import read_bag, QDD_SAFE, QDD_NOM  # noqa: E402

K = [f'fr3_joint{i}' for i in range(1, 8)]
FAST = '/NS_1/franka/joint_states'
SLOW = '/NS_1/joint_states'
EE_ACT, EE_DES = '/NS_1/ee_actual', '/NS_1/ee_desired'


def load(bag):
    safe, nom, js, slow, ea, ed = [], [], [], [], [], []
    for tp, recv, m in read_bag(bag, {QDD_SAFE, QDD_NOM, FAST, SLOW, EE_ACT, EE_DES}):
        if tp == QDD_SAFE:
            safe.append((recv, np.array(m.data[:7])))
        elif tp == QDD_NOM:
            nom.append((recv, np.array(m.data[:7])))
        elif tp in (FAST, SLOW):
            d = dict(zip(m.name, zip(m.position, m.velocity)))
            if all(k in d for k in K):
                (js if tp == FAST else slow).append(
                    (recv, np.array([d[k][0] for k in K]), np.array([d[k][1] for k in K])))
        elif tp == EE_ACT:
            ea.append((recv, np.array([m.point.x, m.point.y, m.point.z])))
        else:
            ed.append((recv, np.array([m.point.x, m.point.y, m.point.z])))
    return safe, nom, (js or slow), ea, ed


def pct(x, p=(50, 90, 99, 100)):
    return ' '.join(f'p{q}={np.percentile(x, q):7.2f}' for q in p)


def verdict(name, value, watch, fail, unit='', fmt='.2f', lower_is_better=True):
    bad = (lambda v, t: v > t) if lower_is_better else (lambda v, t: v < t)
    tag = 'FAIL ' if bad(value, fail) else ('WATCH' if bad(value, watch) else 'PASS ')
    print(f'   [{tag}] {name}: {value:{fmt}}{unit}   (watch > {watch}, fail > {fail})' if lower_is_better
          else f'   [{tag}] {name}: {value:{fmt}}{unit}   (watch < {watch}, fail < {fail})')
    return tag


def report(bag):
    safe, nom, js, ea, ed = load(bag)
    print(f'== {bag}: {len(safe)} q̈_safe, {len(nom)} q̈_nom, {len(js)} joint_states, {len(ea)} ee_actual')
    if len(safe) < 10:
        print('   no /NS_1/qddot_safe in this bag')
        return
    ts = np.array([s[0] for s in safe]); Qs = np.array([s[1] for s in safe])
    tn = np.array([s[0] for s in nom]); Qn = np.array([s[1] for s in nom])
    h = float(np.median(np.diff(ts)))
    dur = ts[-1] - ts[0]
    print(f'   duration {dur:.0f} s, q̈_safe at {1 / h:.0f} Hz')

    # ── did the arm move? ────────────────────────────────────────────────────
    moved = True
    if js:
        tj = np.array([j[0] for j in js]); Q = np.array([j[1] for j in js]); V = np.array([j[2] for j in js])
        sp = np.abs(V).max(axis=1)
        frac_moving = float(np.mean(sp > 0.05))
        print(f'   arm moving (max|q̇| > 0.05 rad/s) {100 * frac_moving:.0f} % of the time, peak |q̇| per joint {np.round(np.abs(V).max(axis=0), 2)}')
        if ea:
            E = np.array([e[1] for e in ea])
            print(f'   ee_actual range [m] {np.round(np.ptp(E, axis=0), 3)}')
        if frac_moving < 0.05 or float(np.abs(V).max()) < 0.1:
            moved = False
            print('   *** THE ARM DID NOT MOVE. Command-vs-realised numbers are command minus zero; do not read them. ***')
            print('       (robot stopped / Desk user-stop / brakes / reflex?  the torque command was published regardless)')
    else:
        print('   no joint states in this bag: realised motion cannot be judged')

    # ── command ──────────────────────────────────────────────────────────────
    nom_at = Qn[np.clip(np.searchsorted(tn, ts) - 1, 0, len(tn) - 1)] if len(nom) else np.zeros_like(Qs)
    dev = np.abs(Qs - nom_at).max(axis=1)
    act = dev > 0.5
    jerk = np.abs(np.diff(Qs, axis=0)) / h              # the filter's own tick, not the receive jitter
    jmax = jerk.max(axis=1)
    k = 5
    ker = np.ones(k) / k
    sm = np.stack([np.convolve(Qs[:, i], ker, mode='same') for i in range(7)], axis=1)
    hf = np.abs(Qs - sm).max(axis=1)[k:-k]
    # reversals of the FILTER's contribution (q̈_safe - q̈_nom), not of q̈ itself: the circle path alone swings
    # the joint accelerations through +-1.5 every 6 s cycle, which says nothing about avoidance
    contrib = Qs - nom_at
    state = np.zeros(7)
    rev = 0
    for row in contrib:
        for j in range(7):
            if abs(row[j]) > 1.0 and np.sign(row[j]) != state[j]:
                rev += 1 if state[j] != 0 else 0
                state[j] = np.sign(row[j])
    print(f'   command: filter overrides the nominal on {100 * act.mean():.1f} % of ticks; when it does |q̈_safe - q̈_nom| {pct(dev[act]) if act.any() else "-"}')
    print(f'   command jerk [rad/s³]   all   : {pct(jmax)}')
    if act[1:].any():
        print(f'   command jerk [rad/s³]   active: {pct(jmax[act[1:]])}')
    print(f'   |q̈_safe| max over joints       : {pct(np.abs(Qs).max(axis=1))}')
    print(f'   chatter |q̈ - MA5| [rad/s²]     : {pct(hf)};  sign reversals of the filter contribution (>1): {rev / dur * 60:.1f} / min')

    # ── realised ─────────────────────────────────────────────────────────────
    worst = []
    if js and moved:
        g = np.arange(tj[0], tj[-1], 0.01)
        Vg = np.array([np.interp(g, tj, V[:, i]) for i in range(7)]).T
        A = (Vg[6:] - Vg[:-6]) / 0.06
        J = (A[6:] - A[:-6]) / 0.06
        print(f'   realised |accel| [rad/s²]     : {pct(np.abs(A).max(axis=1))}')
        print(f'   realised |jerk|  [rad/s³]     : {pct(np.abs(J).max(axis=1))}')
        ia = np.clip(np.searchsorted(ts, g[6:-6] + 0.03) - 1, 0, len(ts) - 1)
        m = min(len(A), len(ia))
        err = np.abs(A[:m] - Qs[ia[:m]]).max(axis=1)
        print(f'   trk_err |a_real - q̈_safe|     : {pct(err)}')
        # velocity against the firmware envelope
        from franka_experiments.utils.cbf_hard_limits import fr3_velocity_envelope
        ratio = np.empty(len(tj))
        who = np.empty(len(tj), int)
        for i in range(len(tj)):
            up, lo = fr3_velocity_envelope(Q[i])
            r = np.where(V[i] >= 0, V[i] / np.maximum(up, 1e-3), V[i] / np.minimum(lo, -1e-3))
            ratio[i], who[i] = float(np.max(r)), int(np.argmax(r))
        print(f'   |q̇| / firmware envelope       : p99 {np.percentile(ratio, 99):.2f}  max {ratio.max():.2f} (joint {who[int(np.argmax(ratio))] + 1})')
        worst.append(verdict('peak |q̇| / firmware envelope', float(ratio.max()), 0.8, 0.95))
        worst.append(verdict('realised jerk p99 [rad/s³]', float(np.percentile(np.abs(J).max(axis=1), 99)), 250, 500, fmt='.0f'))
        worst.append(verdict('trk_err p99 [rad/s²]', float(np.percentile(err, 99)), 6, 10))
    worst.append(verdict('command jerk p99 [rad/s³]', float(np.percentile(jmax, 99)), 200, 500, fmt='.0f'))
    worst.append(verdict('peak |q̈_safe| [rad/s²]', float(np.abs(Qs).max()), 6.1, 10.1))

    # ── end-effector tracking ────────────────────────────────────────────────
    if ea and ed and moved:
        te = np.array([e[0] for e in ea]); E = np.array([e[1] for e in ea])
        tdd = np.array([e[0] for e in ed]); D = np.array([e[1] for e in ed])
        i = np.clip(np.searchsorted(tdd, te), 0, len(tdd) - 1)
        e_ee = np.linalg.norm(E - D[i], axis=1) * 100
        print(f'   EE tracking error [cm]        : {pct(e_ee)}')
    if not moved:
        worst.append('FAIL ')
    print('   overall:', 'FAIL' if any(w.strip() == 'FAIL' for w in worst) else
          ('WATCH' if any(w.strip() == 'WATCH' for w in worst) else 'PASS'))


if __name__ == '__main__':
    for b in sys.argv[1:]:
        report(b)
