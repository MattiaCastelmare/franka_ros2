"""Reset-time check: can the arm hold the target at all, with every hard CBF row satisfied?

``task.target_clearance`` only bounds the distance from the obstacle centre to
the target POINT. The barrier, however, acts on the control-point spheres: the
hand sphere (r 0.13) sits 0.10 m behind the TCP and the elbow (link4, r 0.09)
can be forced next to the obstacle by a far, lateral target. Measured on the
400 held-out seeds (runs/eval_all/feasibility.py, 2026-10-02): 7 of the 22
static episodes that EVERY ensemble failed had no joint configuration with the
TCP within target_tol of the target and all spheres at d >= d_safe (best 58-81 mm),
all with the obstacle 0.26-0.29 m from the target. No policy can hold those;
they cap the success rate and feed pure noise into the reward.

This solves the constrained IK

    min_q ||p_ee(q) - target||^2
    s.t.  ||p_i(q) - o|| - r_obs - r_i >= d_safe     (every control point)
          z_i(q) - r_i - floor_z - floor_margin >= 0 (every control point, if floor_enable)
          q_min + m <= q <= q_max - m

with SLSQP and analytic Jacobians on a PRIVATE MjData, so the episode state and
the env RNG are untouched. The base keep-out and the EE workspace box are left
out: targets live inside the ws box, and the keep-out only binds near the base,
far from the target region; leaving rows out can only call an infeasible target
feasible, never the reverse.

It is a NECESSARY condition: a feasible goal pose says nothing about whether
the CBF lets the arm get there. And SLSQP is local: a target is declared
infeasible only after every start failed, so a miss means "no start found one".
"""
from __future__ import annotations

import mujoco
import numpy as np
from scipy.optimize import minimize


class TargetIK:
    def __init__(self, model, qpos0, qadr, dadr, ee_site, cp_body, cp_radius, r_obs, d_safe,
                 q_min, q_max, q_margin=0.05, floor=None, n_starts=8, seed=0):
        """floor: None, or (floor_z, floor_margin). qpos0: full qpos used for the non-arm dofs."""
        self.m = model
        self.d = mujoco.MjData(model)
        self.d.qpos[:] = qpos0
        self.qadr, self.dadr = np.asarray(qadr), np.asarray(dadr)
        self.ee_site = ee_site
        self.cp_body = list(cp_body)
        self.cp_r = np.asarray(cp_radius, float)
        self.r_obs, self.d_safe = float(r_obs), float(d_safe)
        self.lo = np.asarray(q_min, float) + q_margin
        self.hi = np.asarray(q_max, float) - q_margin
        self.floor = floor
        # Fixed extra starts from a PRIVATE generator: the check is a pure
        # function of (target, obstacle, start pose) and never consumes the env RNG.
        rng = np.random.default_rng(seed)
        self.extra_starts = [rng.uniform(self.lo, self.hi) for _ in range(max(0, n_starts - 1))]
        self._jac = np.zeros((3, model.nv))
        self.evals = 0

    def _fk(self, q):
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)          # mj_jac needs cdof/subtree_com

    def _ee(self):
        mujoco.mj_jacSite(self.m, self.d, self._jac, None, self.ee_site)
        return self.d.site_xpos[self.ee_site].copy(), self._jac[:, self.dadr].copy()

    def _cps(self):
        P, J = [], []
        for b in self.cp_body:
            p = self.d.xpos[b].copy()
            mujoco.mj_jac(self.m, self.d, self._jac, None, p, b)
            P.append(p); J.append(self._jac[:, self.dadr].copy())
        return np.array(P), np.array(J)

    def min_error(self, target, obstacles, q_start, tol):
        """Smallest TCP-target distance over feasible poses, for EVERY obstacle centre in
        ``obstacles`` taken separately (the arm may re-configure as the obstacle moves).
        Returns the worst (largest) of those minima; stops early once one centre exceeds tol."""
        target = np.asarray(target, float)
        worst = 0.0
        for o in np.atleast_2d(obstacles):
            worst = max(worst, self._min_error_one(target, np.asarray(o, float), q_start, tol))
            if worst >= tol:
                break
        return worst

    def _min_error_one(self, target, o, q_start, tol):
        cache = {}

        def at(q):
            k = q.tobytes()
            if k not in cache:
                self._fk(q); self.evals += 1
                e, Je = self._ee(); P, J = self._cps()
                cache.clear(); cache[k] = (e, Je, P, J)
            return cache[k]

        def f(q):
            e, Je, _, _ = at(q); r = e - target
            return float(r @ r), 2.0 * Je.T @ r

        def g(q):
            _, _, P, _ = at(q)
            diff = P - o; dist = np.linalg.norm(diff, axis=1)
            rows = [dist - self.r_obs - self.cp_r - self.d_safe]
            if self.floor is not None:
                rows.append(P[:, 2] - self.cp_r - self.floor[0] - self.floor[1])
            return np.concatenate(rows)

        def gj(q):
            _, _, P, J = at(q)
            diff = P - o; dist = np.maximum(np.linalg.norm(diff, axis=1), 1e-9)
            rows = [np.einsum('ik,ikj->ij', diff / dist[:, None], J)]
            if self.floor is not None:
                rows.append(J[:, 2, :])
            return np.vstack(rows)

        best = np.inf
        for q0 in [np.clip(q_start, self.lo, self.hi)] + self.extra_starts:
            r = minimize(f, q0, jac=True, method='SLSQP', bounds=list(zip(self.lo, self.hi)),
                         constraints=[{'type': 'ineq', 'fun': g, 'jac': gj}],
                         options=dict(maxiter=200, ftol=1e-10))
            q = np.clip(r.x, self.lo, self.hi)
            if g(q).min() > -1e-4:                 # only constraint-satisfying poses count
                best = min(best, float(np.sqrt(f(q)[0])))
            if best < tol:
                break
        return best
