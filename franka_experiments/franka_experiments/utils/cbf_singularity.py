"""HOCBF row keeping the arm away from kinematic singularities.

WHY THIS EXISTS
---------------
The obstacle rows constrain a *Cartesian* quantity, ``aᵀq̈`` with ``a = n̂ᵀJ``.
Near a singularity ``‖a‖`` collapses in some directions, so the QP has to ask
for a huge ``q̈`` to move the control point at all — the joint velocities blow
up while the Cartesian retreat stays modest. On hardware that is the "fast
obstacle → avoidance sends the arm through a singularity → the joints saturate
their velocity limit → ``joint_velocity_violation``" chain. The velocity box
underneath can only truncate the command once it is already asking for too
much; it cannot steer the pose away from the ill-conditioned region.

This module supplies the missing barrier, in exactly the row format the rest of
the filter already uses.

THE BARRIER
-----------
``h = σ_min(J̃) − σ_floor``, with ``σ_min`` the SMALLEST SINGULAR VALUE of the
frame Jacobian. Chosen over Yoshikawa's ``w = √det(J Jᵀ)`` deliberately:

* ``w`` is the PRODUCT of the singular values, so it stays healthy while one
  direction is already degenerate — precisely the case that hurts here;
* ``σ_min`` is the amplification bound the failure is made of: for any task
  velocity ``ẋ``, ``‖q̇‖ ≥ ‖ẋ‖ / σ_max`` and, for the worst direction,
  ``‖q̇‖ = ‖ẋ‖ / σ_min``. A floor on ``σ_min`` is *literally* a ceiling on how
  much the avoidance can amplify joint speed.

``J̃`` is the 6×7 Jacobian with its rotational rows scaled by ``rot_scale``
[m/rad], so that all six rows carry the same units and the SVD is not comparing
metres with radians. ``rot_scale`` is a characteristic arm length: with the
default 0.3 m a 1 rad/s spin counts like a 0.3 m/s translation.

RELATIVE DEGREE 2 → SAME HOCBF CONVENTION AS EVERY OTHER ROW
------------------------------------------------------------
    ḣ = ∇σᵀ q̇                    ⇒  a = ∇σ
    ḧ = ∇σᵀ q̈ + (d∇σ/dt)ᵀ q̇     ⇒  ċ = (d∇σ/dt)ᵀ q̇
    aᵀq̈ + s ≥ −k1·(aᵀq̇) − k0·h − ċ

``∇σ`` has no closed form here (it would need the derivative of an SVD through
Pinocchio), so it is a FORWARD FINITE DIFFERENCE over the 7 joints: 8 Jacobian
evaluations per rebuild, ~0.2 ms at the 50 Hz constraint rate. ``ċ`` differences
``∇σ`` between two rebuilds.

That drift estimate is the noisiest term in the row, and its sign matters: ``ċ``
enters ``h_qp`` additively, so a positive ``ċ`` RELAXES the constraint. It is
therefore clamped to ``≤ 0`` by default (``drift_relaxes=False``) — the estimate
can then only ever tighten the row, never open it on the strength of a numerical
artefact. Same conservative asymmetry the obstacle rows use for ``v_obs``.

No ROS here; pure numpy + Pinocchio, so it is unit-testable headless.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np
import pinocchio as pin

Row = Tuple[np.ndarray, float, float, str]


class SingularityRowBuilder:
    """Builds the (at most one) singularity HOCBF row for a frame.

    Owns a PRIVATE ``pin.Data``: the finite differences evaluate the Jacobian at
    perturbed configurations, and doing that on the node's shared ``CBFKinematics``
    data would clobber the snapshot the obstacle and self-collision rows are
    built from in the same pass.

    Args:
        model: Pinocchio model whose ``nv`` is the arm's (the hand-less FR3
            model the filter already carries — ``nv == 7``).
        frame_id: Frame whose Jacobian defines the barrier (the EE/flange).
        sigma_floor: [m/rad] barrier zero. ``h = σ_min − sigma_floor``.
        horizon: [m/rad] emit a row only while ``h < horizon``.
        eps: [rad] forward-difference step for ``∇σ``.
        rot_scale: [m/rad] weight applied to the three rotational Jacobian rows
            before the SVD, so the singular values are all in metres.
        min_leverage: drop the row when ``‖∇σ‖`` is below this — no leverage on
            q̈ means the row can only inject numerical noise into the QP.
        drift_relaxes: allow the finite-difference ``ċ`` to be positive (i.e. to
            LOOSEN the row). Default False — see the module docstring.
        label: diagnostic row label.
    """

    def __init__(
        self,
        model,
        frame_id: int,
        *,
        sigma_floor: float = 0.05,
        horizon: float = 0.08,
        eps: float = 1e-4,
        rot_scale: float = 0.3,
        min_leverage: float = 1e-3,
        drift_relaxes: bool = False,
        label: str = 'sing:sigma_min',
    ) -> None:
        self.model = model
        self.data = model.createData()
        self.frame_id = int(frame_id)
        self.sigma_floor = float(sigma_floor)
        self.horizon = float(horizon)
        self.eps = float(eps)
        self.rot_scale = float(rot_scale)
        self.min_leverage = float(min_leverage)
        self.drift_relaxes = bool(drift_relaxes)
        self.label = label

        self.nv = int(model.nv)
        # Row weighting applied before the SVD: translation rows as-is,
        # rotation rows scaled to metres. Precomputed once.
        self._w = np.ones(6)
        self._w[3:] = self.rot_scale

        # Last computed values — DIAGNOSTIC + the drift estimate's memory.
        self.last_sigma: float = float('nan')
        self._grad_prev: Optional[np.ndarray] = None
        self._t_prev: Optional[float] = None

    # ── Barrier value ───────────────────────────────────────────────────────

    def sigma(self, q: np.ndarray) -> float:
        """Smallest singular value of the (unit-weighted) frame Jacobian at q.

        Recomputes FK + Jacobians on this object's own ``pin.Data``, so it is
        safe to call at perturbed configurations while the caller's kinematics
        snapshot stays intact.
        """
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.getFrameJacobian(
            self.model, self.data, self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        return float(np.linalg.svd(self._w[:, None] * J, compute_uv=False)[-1])

    # ── Gradient ────────────────────────────────────────────────────────────

    def grad_sigma(self, q: np.ndarray, sigma0: Optional[float] = None) -> np.ndarray:
        """``∇σ`` by forward finite differences (nv extra Jacobian evaluations).

        Forward rather than central: half the cost, and the truncation error is
        O(eps) on a quantity the barrier already treats conservatively (the
        horizon and the ``sigma_floor`` margin both dwarf it at eps = 1e-4).
        """
        s0 = self.sigma(q) if sigma0 is None else float(sigma0)
        g = np.empty(self.nv)
        qp = np.array(q, dtype=np.float64, copy=True)
        for j in range(self.nv):
            qj = qp[j]
            qp[j] = qj + self.eps
            g[j] = (self.sigma(qp) - s0) / self.eps
            qp[j] = qj
        return g

    # ── Row ─────────────────────────────────────────────────────────────────

    def build(self, q: np.ndarray, qdot: np.ndarray, t: float) -> List[Row]:
        """Return ``[(a, h, jdq, label)]`` or ``[]`` when the row is not needed.

        Args:
            q: (nv,) joint configuration of the snapshot.
            qdot: (nv,) joint velocity of the snapshot.
            t: [s] timestamp of this rebuild, used only to difference ``∇σ``
                for the drift term. Must be monotonic.

        Returns:
            A list with at most one row, so the caller can splice it into its
            row lists with the same ``for ... in`` shape used by the other
            builders.
        """
        sigma = self.sigma(q)
        self.last_sigma = sigma
        h = sigma - self.sigma_floor
        if not np.isfinite(h) or h >= self.horizon:
            # Far from the ill-conditioned region: the linear HOCBF would be
            # non-binding anyway and the row would only inflate n_c (and with it
            # OSQP setup() churn). Reset the drift memory so the first row after
            # a long absence does not difference against a stale gradient.
            self._grad_prev = None
            self._t_prev = None
            return []

        a = self.grad_sigma(q, sigma0=sigma)
        if not np.all(np.isfinite(a)) or float(np.linalg.norm(a)) < self.min_leverage:
            # σ_min is non-smooth where the two smallest singular values cross;
            # there the finite difference is meaningless. Dropping the row is
            # the honest response — the velocity/position box underneath is
            # unaffected.
            self._grad_prev = None
            self._t_prev = None
            return []

        # ċ = (d∇σ/dt)ᵀ q̇, differenced between rebuilds. Zero on the first row
        # of an episode (no previous gradient) — the k1 term still anticipates.
        jdq = 0.0
        if self._grad_prev is not None and self._t_prev is not None:
            dt = t - self._t_prev
            if 1e-4 < dt < 0.5:
                jdq = float(((a - self._grad_prev) / dt) @ qdot)
        if not self.drift_relaxes:
            jdq = min(jdq, 0.0)     # may only ever tighten the row
        self._grad_prev = a
        self._t_prev = float(t)

        return [(a, float(h), float(jdq), self.label)]

    def describe(self) -> str:
        return (f'{self.label}: floor={self.sigma_floor:.3f} '
                f'horizon={self.horizon:.3f} rot_scale={self.rot_scale:.2f} '
                f'eps={self.eps:.1e}')
