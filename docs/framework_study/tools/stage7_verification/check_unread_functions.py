#!/usr/bin/env python3
"""Production functions first reviewed in Stage 7 (chapter 16: F-7-1, F-7-3; chapter 08 (8.10)). Host.

Imports the baseline source only (`franka_experiments` from the repository; the guard refuses an
installed copy, F-2-2) and calls pure-numpy production functions on constructed inputs. Nothing
starts ROS.

1. F-7-1 (REPRO): `singularity_recovery.quintic_coeffs` meets q(0)=q₀, q̇(0)=v₀, q̈(0)=0,
   q(T)=q₁, q̈(T)=0, but q̇(T) = v₀ (the docstring promises 0). Written to PASS while the defect
   is present.
2. F-7-1 (REPRO): `plan_duration` with |v₀| ≤ vmax returns a T that meets the bounds;
   with |v₀,₂| = 0.6 > vmax = 0.5 it returns T₀·1.1⁶⁰ (no T can pass, the τ=0 sample has q̇ = v₀).
3. F-7-1 (NUM): the reference of case 2 leaves the FR3 joint-2 range (±1.7628 rad) by tens of rad.
4. F-7-1 (REPRO): the `SingularityRecovery` state machine, driven with the node's defaults, hands
   the arm a SETTLING reference whose velocity steps from v₀ to 0.
5. F-7-3 (NUM): `select_gamma(d − d_safe = 0.1, conf = 1)` = 2 + 10·e^{−1.8} = 3.653, not 6.0.
6. (NUM) `risk_slack_weight` ∈ [1, w_max]; w(ρ/2) = 1 + (w_max − 1)/2 (the knee).
7. (NUM) `joint_limit_risk_margin`: closing → h − q̇²/(2a); receding → h.
8. (NUM) `self_collision.segment_segment_closest` vs a dense brute-force minimum (random pairs,
   incl. parallel and degenerate segments), the closest-point ingredient of (8.10).
9. (NUM) `retreat_displacement_reachable` = ∫₀ᵗ `retreat_speed_reachable` (trapezoid), several t.
"""
import math
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'stage2_verification'))
import _common as C  # noqa: E402

FE = os.path.join(C.REPO, 'franka_experiments')
sys.path.insert(0, FE)


def main():
    R = C.Report('Stage 7 — functions first reviewed in Stage 7 (F-7-1, F-7-3, (8.10))')
    import franka_experiments.utils.singularity_recovery as SR
    import franka_experiments.utils.self_collision as SC
    import franka_experiments.utils.cbf_state_rows as CSR
    for m in (SR, SC, CSR):
        assert os.path.realpath(m.__file__).startswith(os.path.realpath(FE)), m.__file__
    R.info(f'imported from source: {os.path.relpath(SR.__file__, C.REPO)}')

    # 1 — boundary conditions
    q0, v0, q1, T = np.zeros(3), np.array([1.0, -0.5, 0.2]), np.array([0.5, 0.3, -0.1]), 2.0
    c = SR.quintic_coeffs(q0, v0, q1, T)
    qa, dqa, ddqa = SR.quintic_eval(c, 0.0)
    qb, dqb, ddqb = SR.quintic_eval(c, T)
    ok = (np.allclose(qa, q0) and np.allclose(dqa, v0) and np.allclose(ddqa, 0) and np.allclose(qb, q1)
          and np.allclose(ddqb, 0, atol=1e-12) and np.allclose(dqb, v0))
    R.fact('F-7-1 quintic ends at q̇(T) = v₀ (docstring says 0)', ok,
           f'q̇(T) = {np.round(dqb, 6).tolist()} = v₀ = {v0.tolist()}; q(T), q̈(T) as specified', kind='REPRO')

    # 2 — plan_duration convergence
    q0, qh = np.zeros(7), np.full(7, 0.4)
    v_ok = np.zeros(7); v_ok[1] = 0.3
    T_ok = SR.plan_duration(q0, v_ok, qh, 0.5, 2.0)
    cc = SR.quintic_coeffs(q0, v_ok, qh, T_ok)
    taus = np.linspace(0, T_ok, 400)
    peak_v = max(np.abs(SR.quintic_eval(cc, t)[1]).max() for t in taus)
    v_bad = np.zeros(7); v_bad[1] = 0.6
    T_bad = SR.plan_duration(q0, v_bad, qh, 0.5, 2.0)
    T0 = max(1.0, 1.875 * 0.4 / 0.5, math.sqrt(5.78 * 0.4 / 2.0))
    R.fact('F-7-1 plan_duration: converges for |v₀| ≤ vmax, returns T₀·1.1⁶⁰ for |v₀| > vmax',
           peak_v <= 0.5 + 1e-9 and abs(T_bad - T0 * 1.1 ** 60) < 1e-6 * T_bad,
           f'|v₀|=0.3: T={T_ok:.2f} s, peak {peak_v:.3f}; |v₀|=0.6: T={T_bad:.1f} s = T₀·1.1⁶⁰ (T₀={T0:.3f})',
           kind='REPRO')

    # 3 — excursion
    cb = SR.quintic_coeffs(q0, v_bad, qh, T_bad)
    qs = np.array([SR.quintic_eval(cb, t)[0][1] for t in np.linspace(0, T_bad, 4001)])
    R.below('F-7-1 joint-2 reference LEAVES the ±1.7628 rad range (checked as −max|q₂| ≤ −1.7628)',
            -float(np.abs(qs).max()), -1.7628, kind='NUM',
            note=f'max |q₂ ref| = {np.abs(qs).max():.1f} rad: the reference leaves the joint range')

    # 4 — state machine with node defaults
    rec = SR.SingularityRecovery(sigma_thr=0.07, progress_eps=0.05, err_thr=0.02, stall_s=3.0,
                                 vmax=0.5, amax=2.0, t_min=1.0, settle_s=0.5, cooldown_s=5.0)
    qd0 = np.zeros(7); qd0[3] = 0.3
    dt, t, refs = 0.01, 0.0, []
    while t < 30.0:
        r = rec.update(t, q0, qd0 if rec.state == rec.IDLE else np.zeros(7), qh, 0.03, 0.5)
        refs.append((rec.state, r))
        t += dt
    ret = [r for s, r in refs if s == rec.RETURNING and r is not None]
    sett = [r for s, r in refs if s == rec.SETTLING and r is not None]
    jump = float(np.abs(sett[0][1] - ret[-1][1]).max()) if ret and sett else float('nan')
    R.fact('F-7-1 state machine: velocity reference steps by ≈ v₀ at RETURNING → SETTLING',
           bool(ret and sett) and abs(jump - 0.3) < 0.02,
           f'{len(ret)} RETURNING ticks; step in q̇_ref at SETTLING = {jump:.3f} rad/s (v₀ = 0.3)', kind='REPRO')

    # 5 — select_gamma (needs ament_index at import: read the formula by calling it if importable)
    try:
        import franka_experiments.utils.cbf_utils as CU
        g = CU.select_gamma('safe', 1.0, d=0.30, d_safe=0.20)
        src = 'production select_gamma'
    except ImportError as exc:
        g = 2.0 + 10.0 * math.exp(-18.0 * 0.1)
        src = f'formula transcribed (import failed on host: {exc.name})'
        txt = open(os.path.join(FE, 'franka_experiments', 'utils', 'cbf_utils.py')).read()
        assert 'GAMMA_MIN = 2.0' in txt and 'GAMMA_MAX = 12.0' in txt and 'K         = 18.0' in txt
    R.close('F-7-3 select_gamma at δ = 0.1 m, conf 1 (docstring says ≈ 6.0)', g, 2 + 10 * math.exp(-1.8),
            1e-9, kind='NUM', note=src)

    # 6 — risk_slack_weight
    hs = np.linspace(-1.0, 2.0, 301)
    ws = np.array([CSR.risk_slack_weight(h, w_max=5.0, rho=0.30, alpha=6.0) for h in hs])
    knee = CSR.risk_slack_weight(0.15, w_max=5.0, rho=0.30, alpha=6.0)
    R.fact('risk_slack_weight ∈ [1, w_max], monotone non-increasing, knee at ρ/2',
           ws.min() >= 1.0 and ws.max() <= 5.0 and np.all(np.diff(ws) <= 1e-12) and abs(knee - 3.0) < 1e-12,
           f'range [{ws.min():.4f}, {ws.max():.4f}], w(ρ/2) = {knee:.6f}', kind='NUM')

    # 7 — joint_limit_risk_margin
    a = np.zeros(7); a[1] = -1.0                       # upper-limit row on joint 2
    qd = np.zeros(7); qd[1] = 1.02
    auth = np.full(7, 1.553)
    m_close = CSR.joint_limit_risk_margin(a, 0.40, qd, auth)
    m_away = CSR.joint_limit_risk_margin(a, 0.40, -qd, auth)
    R.fact('joint_limit_risk_margin: closing h − q̇²/2a, receding h',
           abs(m_close - (0.40 - 1.02 ** 2 / (2 * 1.553))) < 1e-12 and m_away == 0.40,
           f'closing {m_close:.4f}, receding {m_away:.4f}', kind='NUM')

    # 8 — segment/segment closest points
    rng = np.random.default_rng(7)
    worst = 0.0
    cases = [rng.normal(size=(4, 3)) for _ in range(200)]
    p = rng.normal(size=3); d = rng.normal(size=3)
    cases.append(np.array([p, p + d, p + [0.1, 0, 0], p + [0.1, 0, 0] + 2 * d]))   # parallel
    cases.append(np.array([p, p, p + d, p + 2 * d]))                                # first degenerate
    s = np.linspace(0, 1, 401)
    for P1, Q1, P2, Q2 in cases:
        _, _, dist = SC.segment_segment_closest(P1, Q1, P2, Q2)
        A = P1 + s[:, None] * (Q1 - P1)
        B = P2 + s[:, None] * (Q2 - P2)
        brute = np.min(np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2))
        worst = max(worst, dist - brute)
    R.below('segment_segment_closest ≤ brute-force grid minimum (202 pairs)', worst, 1e-12, kind='NUM',
            note='the grid can only over-estimate the true minimum')

    # 9 — displacement = integral of speed
    a = rng.normal(size=7) * 0.3
    qdot = rng.normal(size=7) * 0.2
    box = dict(qdot_max=np.full(7, 1.5), acc_lb=-np.full(7, 3.0), acc_ub=np.full(7, 3.0))
    err = 0.0
    for tt in (0.05, 0.2, 0.6, 2.0):
        ts = np.linspace(0, tt, 4001)
        v = np.array([CSR.retreat_speed_reachable(a, qdot, x, **box) for x in ts])
        integ = float(np.sum(0.5 * (v[1:] + v[:-1]) * np.diff(ts)))
        err = max(err, abs(CSR.retreat_displacement_reachable(a, qdot, tt, **box) - integ))
    R.below('retreat_displacement_reachable = ∫ retreat_speed_reachable', err, 1e-6, kind='NUM')
    return R.finish()


if __name__ == '__main__':
    sys.exit(main())
