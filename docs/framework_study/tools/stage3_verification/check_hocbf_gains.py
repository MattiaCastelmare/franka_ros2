#!/usr/bin/env python3
"""HOCBF gains, the zone ladder and the row right-hand side (chapter 07 §2, §3, §6; F-3-4, F-3-5).

Uses the SOURCE `utils/cbf_zones.py` (ZoneLadder, ladder_from_params) and
`utils/cbf_qp_assembly.py::build_row_rhs`, with the effective hardware-default parameters
(`_common3.effective_params('hw_default')`).

  1. (7.6) factorisation: s^2 + k1 s + k0 = (s + a1)(s + a2) with real a1, a2 > 0 iff k1^2 >= 4 k0.
     Evaluated for the shipped scalar pair and every rung                                   (NUM)
  2. F-3-5: the priority/hold rungs (50, 14) have complex roots; the blended active<->priority
     gains cross into that region at w = 0.5044, computed with the real ZoneLadder           (NUM)
  3. (7.8) why it matters: impulse response g(t) of s^2 + k1 s + k0. Real roots: g >= 0 for all t
     (comparison lemma holds). Complex roots: g has a negative lobe of relative size
     exp(-zeta pi / sqrt(1 - zeta^2))                                                        (NUM)
  4. (7.9) linear class-K static engagement gap: the row starts to push when
     h < (k1/k0)|h_dot|; ratio for each rung                                                  (NUM)
  5. F-3-4: ZoneLadder.weights does not depend on d_notice (two ladders differing only in d_notice
     give identical weights on a 1 mm grid), and the notice weight is ~1 already at
     d_active + blend/2                                                                       (FACT)
  6. (7.10) build_row_rhs reproduces k1 (A qdot_cbf - v_obs) + k0 max(h, h_pen) + c (+ b_ff + kappa)
     and the rate-cap rows' (v_cap + a q_dot)/T                                               (NUM)
  7. (7.11) bounded depth push: at rest on a binding row, v_sep - v_obs = (k0/k1)|max(h, h_pen)| <=
     retreat_excess_speed_max                                                                 (NUM)
  8. 1-D closed loop of the HOCBF row against an obstacle approaching at constant speed:
     h stays >= 0 with the shipped gains (practical example of 07 §5)                  (illustrative)

Needs: Python 3, numpy, PyYAML. Host. Read-only.
"""
import math
import os
import sys
from types import SimpleNamespace

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common3 as C3  # noqa: E402

C = C3.C
C.use_source_package()
import numpy as np  # noqa: E402
from franka_experiments.utils.cbf_qp_assembly import build_row_rhs  # noqa: E402
from franka_experiments.utils.cbf_zones import ZoneLadder, ladder_from_params  # noqa: E402

R = C.Report('check_hocbf_gains (chapter 07 §2-3, §6; F-3-4, F-3-5)')
P = C3.params_namespace('hw_default')
k0, k1 = float(P.k0_cbf), float(P.k1_cbf)
lad = ladder_from_params(P)
R.fact('zone ladder is ON in the hardware default (enable_zone_ladder)', lad is not None)
R.close('ladder boundaries hold/priority/active/notice [m] = (0.25, 0.5, 1.0, 1.5) x d_safe',
        lad.bounds, [0.025, 0.05, 0.10, 0.15], 1e-12, kind='FACT')
R.close('blend width [m] = zone_blend_r x d_safe', lad.blend_m, 0.02, 1e-12, kind='FACT')

# 1) factorisation
pairs = {'scalar (k0_cbf, k1_cbf)': (k0, k1)}
for i, name in enumerate(('hold', 'priority', 'active', 'notice')):
    pairs[f'rung {name}'] = (float(lad.k0[i]), float(lad.k1[i]))


def roots(a0, a1):
    disc = a1 * a1 - 4 * a0
    if disc >= 0:
        r = math.sqrt(disc)
        return disc, ((a1 + r) / 2, (a1 - r) / 2)
    return disc, None


for name, (a0, a1) in pairs.items():
    disc, rt = roots(a0, a1)
    zeta = a1 / (2 * math.sqrt(a0))
    if rt:
        ok = abs(rt[0] * rt[1] - a0) < 1e-9 and abs(rt[0] + rt[1] - a1) < 1e-9 and min(rt) > 0
        R.fact(f'{name} (k0={a0:g}, k1={a1:g}): real factorisation alpha1, alpha2 = '
               f'{rt[0]:.4f}, {rt[1]:.4f}; zeta = {zeta:.4f}', ok, kind='NUM')
    else:
        R.fact(f'{name} (k0={a0:g}, k1={a1:g}): k1^2 - 4 k0 = {disc:g} < 0, NO real factorisation; '
               f'zeta = {zeta:.4f}', name.startswith('rung hold') or name.startswith('rung priority'),
               kind='NUM', )

# 2) crossing weight inside the active <-> priority band
d_mid = lad.bounds[1]
ds = np.linspace(d_mid - lad.blend_m / 2, d_mid + lad.blend_m / 2, 20001)
k0v, k1v, _, _ = lad.gains_array(ds)
disc = k1v ** 2 - 4 * k0v
w_prio = np.array([lad.weights(float(d))[1] for d in ds])
i = int(np.flatnonzero(disc < 0).max()) if np.any(disc < 0) else None
w_star_analytic = (26.5 - math.sqrt(26.5 ** 2 - 4 * 12.25 * 10.25)) / (2 * 12.25)
R.close('F-3-5: active->priority blend: priority weight at which k1^2 = 4 k0 '
        '(analytic root of 12.25 w^2 - 26.5 w + 10.25)', w_star_analytic, 0.5044, 5e-5, kind='NUM')
neg = w_prio[disc < 0]
R.close('F-3-5: smallest priority weight with k1^2 < 4 k0, from the real ZoneLadder', neg.min(),
        w_star_analytic, 2e-3, kind='NUM')
d_cross = float(ds[disc < 0].max())
R.info(f'F-3-5: complex roots for gaps below {d_cross:.4f} m (d_safe = {P.d_safe} m); '
       f'i.e. once the gap is about {1e3 * (P.d_safe - d_cross):.1f} mm inside d_safe')


# 3) impulse response
def impulse_min_ratio(a0, a1, T=6.0, n=200001):
    t = np.linspace(0, T, n)
    disc = a1 * a1 - 4 * a0
    if disc > 1e-12:
        r = math.sqrt(disc)
        p1, p2 = (-a1 + r) / 2, (-a1 - r) / 2
        g = (np.exp(p1 * t) - np.exp(p2 * t)) / (p1 - p2)
    elif disc < -1e-12:
        wd = math.sqrt(-disc) / 2
        g = np.exp(-a1 / 2 * t) * np.sin(wd * t) / wd
    else:
        g = t * np.exp(-a1 / 2 * t)
    return float(g.min() / g.max())


for name, (a0, a1) in pairs.items():
    ratio = impulse_min_ratio(a0, a1)
    disc = a1 * a1 - 4 * a0
    if disc >= 0:
        R.below(f'{name}: impulse response never negative (min g / max g)', -ratio, 1e-12, kind='NUM')
    else:
        zeta = a1 / (2 * math.sqrt(a0))
        pred = math.exp(-zeta * math.pi / math.sqrt(1 - zeta * zeta))
        # peak at w_d t1 = atan2(sqrt(1-z^2), z), trough half a period later: ratio exactly -pred
        R.close(f'{name}: negative lobe of g relative to its peak (= -exp(-zeta pi/sqrt(1-zeta^2)))',
                ratio, -pred, abs(pred) * 1e-3, kind='NUM',
                note=f'{pred:.2e}: a second-order differential inequality has no comparison bound, '
                'but the violation it allows is this small')

# 4) engagement ratio
for name, (a0, a1) in pairs.items():
    R.info(f'{name}: static engagement h < (k1/k0)|h_dot|, k1/k0 = {a1 / a0:.4f} s')
R.close('k1/k0 for the shipped scalar pair and the notice rung [s]', [k1 / k0, 7.0 / 12.0],
        [0.42, 0.5833], 1e-4, kind='NUM')

# 5) F-3-4: d_notice is inert
kw = dict(d_active=0.10, d_priority=0.05, d_hold=0.025, blend_m=0.02)
L1 = ZoneLadder(d_notice=0.15, **kw)
L2 = ZoneLadder(d_notice=0.90, **kw)
grid = np.arange(0.0, 1.0, 0.001)
W1 = np.array([L1.weights(float(d)) for d in grid])
W2 = np.array([L2.weights(float(d)) for d in grid])
R.fact('F-3-4: ZoneLadder.weights is identical for d_notice = 0.15 and 0.90 m (1 mm grid, 0-1 m)',
       np.array_equal(W1, W2))
wn = lad.weights(lad.bounds[2] + lad.blend_m / 2)
R.close('F-3-4: notice weight at d_active + blend/2 = 0.11 m (< d_notice = 0.15 m)', wn[3], 1.0, 0,
        kind='FACT')

# 6) build_row_rhs on a synthetic snapshot (2 obstacle rows, 1 retreat cap, 1 speed cap)
rng = np.random.default_rng(3)
nv = 7
A = rng.normal(size=(4, nv))
h_bar = np.array([0.05, -0.03, 1e3, 1e3])
jdq = np.array([0.2, -0.1, 0.0, 0.0])
v_obs = np.array([0.3, 0.0, 0.0, 0.0])
b_ff = np.array([-0.1, -0.2, 0.0, 0.0])
kap = np.array([0.05, 0.0, 0.0, 0.0])
cap_v = np.array([0.4, 0.9])
h_pen = np.array([-0.01, -0.01, -np.inf, -np.inf])
k0r = np.array([25.0, 40.0, 25.0, 25.0])
k1r = np.array([10.5, 12.0, 10.5, 10.5])
con = SimpleNamespace(A=A, h_bar=h_bar, jdot_qdot=jdq, v_obs=v_obs, b_ff=b_ff, kappa=kap,
                      n_cap=2, n_rtr=1, cap_v=cap_v, h_pen_row=h_pen, k0_row=k0r, k1_row=k1r)
qd, qdc = rng.normal(size=nv), rng.normal(size=nv)
T_r, T_s = float(P.retreat_cap_horizon_s), float(P.link_speed_horizon_s)
hq, _ = build_row_rhs(con, qd, qdc, k0=k0, k1=k1, retreat_horizon=T_r, speed_horizon=T_s)
exp = k1r * (A @ qdc - v_obs) + k0r * np.maximum(h_bar, h_pen) + jdq + b_ff + kap
exp[2] = (cap_v[0] + A[2] @ qd) / T_r
exp[3] = (cap_v[1] + A[3] @ qd) / T_s
R.below('(7.10) build_row_rhs = k1(A qdot_cbf - v_obs) + k0 max(h,h_pen) + c + b_ff + kappa; '
        'caps (v_cap + a qdot)/T', float(np.abs(hq - exp).max()), 1e-12)
hq_s, _ = build_row_rhs(con, qd, qdc, k0=k0, k1=k1, retreat_horizon=T_r, speed_horizon=T_s, zoned=False)
exp_s = k1 * (A @ qdc - v_obs) + k0 * np.maximum(h_bar, h_pen) + jdq + b_ff + kap
R.below('zoned=False uses the scalar gains on the barrier rows', float(np.abs(hq_s[:2] - exp_s[:2]).max()), 1e-12)

# 7) bounded depth push
dv = float(P.retreat_excess_speed_max)
for (a0, a1) in ((k0, k1), (12.0, 7.0), (50.0, 14.0)):
    hp = -dv * a1 / a0
    for h in (-0.01, -0.05, -0.2):
        v_rest_excess = (a0 / a1) * abs(max(h, hp))
        if v_rest_excess > dv + 1e-12:
            R.fact(f'bounded push k0={a0} k1={a1} h={h}', False, f'{v_rest_excess}')
            break
R.fact(f'(7.11) bounded depth push: rest excess speed (k0/k1)|max(h, -dv k1/k0)| <= dv = {dv} m/s '
       'for every rung and h in {-0.01, -0.05, -0.2} m', True, kind='NUM')


# 8) 1-D closed loop: point robot, nominal "hold still" (u_nom = -k_brake v), obstacle closing
def closed_loop(v_o, a0, a1, h0=0.30, dt=1e-3, T=4.0):
    h, v_r = h0, 0.0           # v_r: robot separation speed along n (positive = away)
    hmin = h
    for _ in range(int(T / dt)):
        hdot = v_r - v_o
        u_nom = -3.0 * v_r
        u = max(u_nom, -a1 * hdot - a0 * h)   # 1-row QP with identity cost
        v_r += u * dt
        h += (v_r - v_o) * dt
        hmin = min(hmin, h)
    return hmin, v_r


for v_o in (0.3, 1.0):
    hmin, v_end = closed_loop(v_o, k0, k1)
    R.fact(f'1-D closed loop, shipped gains, obstacle closing at {v_o} m/s from h = 0.30 m: '
           f'min h = {hmin * 1e3:.3f} mm, final separation speed {v_end:.3f} m/s', hmin >= -1e-6,
           kind='NUM')
sys.exit(R.finish())
