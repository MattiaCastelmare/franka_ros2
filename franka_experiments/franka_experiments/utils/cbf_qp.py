
# TODO[LEGACY]: no importer; the CBF filter builds and solves its OSQP problem inline | confidence: high | superseded-by: cbf_safety_filter inline OSQP + utils/cbf_qp_assembly.py | flagged: 2026-09-01
import numpy as np
import qpsolvers


class CBFQP:
    def __init__(self, nv: int, rho_slack: float, solver: str = "osqp"):
        self.nv = nv
        self.n_dec = nv + 1   # qddot (nv) + slack s (1)
        self.rho = float(rho_slack)
        self.solver = solver

        # Cost: (1/2) ||qddot - qddot_nom||^2 + (1/2) rho s^2
        self.P = np.eye(self.n_dec, dtype=np.float64)
        self.P[-1, -1] = self.rho

        self._warm = None
        self._prev_n_rows = -1   # detect problem-structure changes → reset warm start

    def solve(
        self,
        qddot_nom:  np.ndarray,
        qdot:       np.ndarray,
        q:          np.ndarray,
        dt:         float,
        A_cbf:      np.ndarray,   # (n_c, nv)  — constraint: A qddot + s >= b
        b_cbf:      np.ndarray,   # (n_c,)
        qddot_min:  np.ndarray,
        qddot_max:  np.ndarray,
        qdot_min:   np.ndarray,
        qdot_max:   np.ndarray,
        q_min:      np.ndarray,
        q_max:      np.ndarray,
        G_extra:    np.ndarray | None = None,  # (n_e, n_dec) — hard G x <= h (no slack)
        h_extra:    np.ndarray | None = None,  # (n_e,)
    ) -> tuple[np.ndarray | None, float | None]:
        """Return (qddot_safe, slack) or (None, None) if QP fails."""
        nv = self.nv

        # Linear cost: -qddot_nom^T qddot  (slack coeff = 0)
        q_vec = np.zeros(self.n_dec)
        q_vec[:nv] = -np.asarray(qddot_nom, dtype=np.float64)

        # Box on decision variable
        lb = np.concatenate([qddot_min, [0.0]])
        ub = np.concatenate([qddot_max, [1e6]])

        G_rows, h_rows = [], []

        # 1) CBF: A qddot + s >= b  →  -A qddot - s <= -b
        n_c = A_cbf.shape[0] if A_cbf.ndim == 2 else 0
        if n_c > 0:
            G_cbf = np.zeros((n_c, self.n_dec))
            G_cbf[:, :nv] = -A_cbf
            G_cbf[:, -1]  = -1.0
            G_rows.append(G_cbf)
            h_rows.append(-b_cbf)

        # 2) Velocity: qdot + qddot*dt in [qdot_min, qdot_max]
        Iv = dt * np.eye(nv)
        G_v = np.zeros((2 * nv, self.n_dec))
        G_v[:nv,  :nv] =  Iv
        G_v[nv:,  :nv] = -Iv
        h_v = np.concatenate([qdot_max - qdot, qdot - qdot_min])
        G_rows.append(G_v); h_rows.append(h_v)

        # 3) Position: q + qdot*dt + 0.5*qddot*dt^2 in [q_min, q_max]
        # h_p clipped to ≥ 0: if q is already outside the YAML limits (physically
        # valid but tighter than hardware), a negative h_p forces qddot >> ub, making
        # the QP infeasible and permanently locking the robot. Clipping to 0 relaxes
        # the constraint to "don't worsen the violation" — recovery happens naturally.
        Ip = 0.5 * dt**2 * np.eye(nv)
        G_p = np.zeros((2 * nv, self.n_dec))
        G_p[:nv,  :nv] =  Ip
        G_p[nv:,  :nv] = -Ip
        h_p = np.maximum(
            np.concatenate([q_max - q - qdot * dt,
                            q + qdot * dt - q_min]),
            0.0,
        )
        G_rows.append(G_p); h_rows.append(h_p)

        # 4) Extra hard constraints (e.g. torque-rate): G_extra already (n_e, n_dec)
        if G_extra is not None and G_extra.shape[0] > 0:
            G_rows.append(G_extra)
            h_rows.append(h_extra)

        G     = np.vstack(G_rows)
        h_vec = np.concatenate(h_rows)

        # Reset warm start when problem structure changes (different G row count).
        # Passing a stale warm start to OSQP with a different constraint count
        # causes a shape mismatch that silently fails or returns None.
        n_rows = G.shape[0]
        if n_rows != self._prev_n_rows:
            self._warm = None
            self._prev_n_rows = n_rows

        x = qpsolvers.solve_qp(
            P=self.P, q=q_vec,
            G=G, h=h_vec,
            lb=lb, ub=ub,
            solver=self.solver,
            initvals=self._warm,
            verbose=False,
        )

        if x is None or not np.all(np.isfinite(x)):
            self._warm = None
            return None, None

        self._warm = x.copy()
        return x[:nv].copy(), float(x[-1])
