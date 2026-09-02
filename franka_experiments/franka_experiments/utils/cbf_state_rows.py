"""Extra HOCBF rows for the QP: self-collision and joint position limits.

Both were previously handled — badly — outside the QP:

* self-collision was a post-solve VETO that replaced the command wholesale, so
  the arm stopped dead instead of steering around the problem;
* joint position limits were a per-joint BOX clip, a hard wall that saturates
  one joint at the last moment. Measured on hardware: joint4 sitting exactly on
  its braking curve had its box collapse to ``lb = 0``, the commander kept
  asking for -4 rad/s², and the 4 rad/s² clip on that single joint produced
  ``dnorm=10.989`` with ``dq_ort=10.941`` — the whole deviation dumped
  orthogonally to anything the CBF was doing, a command the arm could not track
  (``trk_err=8.428``), ending in a ``joint_velocity_violation`` reflex.

As HOCBF rows both engage GRADUALLY and the QP spreads the correction across
joints in the least-squares sense, exactly as it already does for obstacles.
The velocity/position box stays underneath as the hard floor — a row can be
relaxed by the slack, a box cannot.

Row convention (identical to the obstacle rows in cbf_safety_filter):

    aᵀ q̈ + s  ≥  −k1·(aᵀ q̇) − k0·h − ċ

so a builder returns ``(a, h, jdq, label)`` and the node appends it to the same
``rows_a / rows_h / rows_jdq / rows_link`` lists.

Everything here is pure numpy plus the caller's kinematics object; no ROS.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

from franka_experiments.utils.self_collision import (
    Capsule,
    segment_segment_closest,
)

Row = Tuple[np.ndarray, float, float, str]


# ── Joint position limits ────────────────────────────────────────────────────

def joint_limit_rows(
    q: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    margin: float,
    horizon: float,
    nv: int,
) -> List[Row]:
    """HOCBF rows keeping each joint inside ``[q_min+margin, q_max-margin]``.

    Two barriers per joint:

        upper:  h = q_max − margin − q ,  ḣ = −q̇  ⇒  a = −e_j
        lower:  h = q − margin − q_min ,  ḣ = +q̇  ⇒  a = +e_j

    ``ċ = 0`` exactly: ḧ = ∓q̈ has no drift term, unlike the Cartesian rows
    where J̇q̇ appears.

    A row is emitted only while ``h < horizon``; beyond that the linear HOCBF is
    non-binding anyway and the row would only inflate ``n_c`` (and with it OSQP
    ``setup()`` churn). Rows come out in a deterministic order — joint index,
    then upper before lower — so the QP's sparsity pattern is stable frame to
    frame and the warm start stays valid.
    """
    rows: List[Row] = []
    for j in range(nv):
        h_up = float(q_max[j] - margin - q[j])
        if h_up < horizon:
            a = np.zeros(nv)
            a[j] = -1.0
            rows.append((a, h_up, 0.0, f'q{j + 1}+'))
        h_lo = float(q[j] - margin - q_min[j])
        if h_lo < horizon:
            a = np.zeros(nv)
            a[j] = 1.0
            rows.append((a, h_lo, 0.0, f'q{j + 1}-'))
    return rows


# ── Self-collision ───────────────────────────────────────────────────────────

class SelfCollisionRowBuilder:
    """HOCBF rows for capsule pairs that are closing on each other.

    The barrier is the SURFACE gap between two capsules,

        h = ‖p_a − p_b‖ − r_a − r_b − margin

    with ``p_a``/``p_b`` the closest points of the two segments. Both points are
    attached to moving links, so the relative Jacobian is what matters:

        ḣ = n̂ᵀ(ṗ_a − ṗ_b) = n̂ᵀ(J_a − J_b) q̇   ⇒   a = n̂ᵀ(J_a − J_b)
        ċ = n̂ᵀ(J̇_a − J̇_b) q̇

    This is the part the old post-solve veto could not do: it only knew the gap,
    not which joint motion changes it, so it had no way to ask the QP for a
    *different* command — only to refuse the one it had.

    Pair selection is the caller's (see ``self_collision.build_capsule_pairs``
    with the exclusion list from Franka's own SRDF). Only pairs whose gap is
    below ``horizon`` produce a row, so a normal pose contributes nothing.
    """

    def __init__(
        self,
        capsules: Sequence[Capsule],
        pairs: Sequence[Tuple[int, int]],
        frame_ids: Sequence[int],
        arm_v_ids: Sequence[int],
        margin: float = 0.0,
        horizon: float = 0.15,
        max_rows: int = 4,
    ):
        if not capsules:
            raise ValueError(
                'no capsules — the URDF was built without with_sc:=true')
        self._caps = list(capsules)
        self._pairs = [(int(i), int(j)) for i, j in pairs]
        self._fids = [int(f) for f in frame_ids]
        # The *_sc model carries the hand and its finger joints, so Pinocchio's
        # nv exceeds the 7 the QP controls. Jacobian rows must be projected onto
        # the arm columns. Sound because the finger joints are held at zero
        # velocity, so they contribute nothing to ḣ — but the SLICE has to
        # happen after the full-width product, never before.
        self._arm_v = np.asarray(arm_v_ids, dtype=np.intp)
        self._margin = float(margin)
        self._horizon = float(horizon)
        self._max_rows = int(max_rows)

        n_cap = len(self._caps)
        self._loc_p1 = np.array([c.p1 for c in self._caps], dtype=np.float64)
        self._loc_p2 = np.array([c.p2 for c in self._caps], dtype=np.float64)
        self._radius = np.array([c.radius for c in self._caps])
        # Pre-allocated world endpoints, refilled each call.
        self._w1 = np.zeros((n_cap, 3))
        self._w2 = np.zeros((n_cap, 3))
        self.last_min_gap = float('inf')
        self.last_pair = ''

    @property
    def n_pairs(self) -> int:
        return len(self._pairs)

    @property
    def n_capsules(self) -> int:
        return len(self._caps)

    def describe(self) -> str:
        return (f'{len(self._caps)} capsules, {len(self._pairs)} pairs, '
                f'margin={self._margin:.3f} m, horizon={self._horizon:.3f} m, '
                f'max_rows={self._max_rows}')

    def _world_endpoints(self, kin) -> None:
        """Place every capsule using the *_sc frame placements from *kin*."""
        for c, fid in enumerate(self._fids):
            oMf = kin.data.oMf[fid]
            R, t = oMf.rotation, oMf.translation
            self._w1[c] = R @ self._loc_p1[c] + t
            self._w2[c] = R @ self._loc_p2[c] + t

    def build(self, kin, v_full: np.ndarray) -> List[Row]:
        """Rows for the pairs currently inside the horizon.

        *kin* must be a CBFKinematics already updated at the current q, q̇ with
        ``with_jdot=True`` — the caller owns that call so FK is paid once per
        tick no matter how many consumers need it.

        *v_full* is the FULL-width velocity of that model (nv, including any
        finger joints), not the 7-vector: the J̇ product needs every column.
        Returned rows are 7-wide, projected onto ``arm_v_ids``.
        """
        self._world_endpoints(kin)
        self.last_min_gap = float('inf')
        self.last_pair = ''

        # Cheap scan first: distance only, no Jacobians. Jacobians cost a
        # Pinocchio call each and are paid solely for the pairs that make a row.
        near: List[Tuple[float, int, np.ndarray, np.ndarray]] = []
        for k, (i, j) in enumerate(self._pairs):
            pa, pb, d = segment_segment_closest(
                self._w1[i], self._w2[i], self._w1[j], self._w2[j])
            gap = d - self._radius[i] - self._radius[j]
            if gap < self.last_min_gap:
                self.last_min_gap = gap
                self.last_pair = f'{self._caps[i].body[-6:]}/{self._caps[j].body[-6:]}'
            if gap - self._margin < self._horizon:
                near.append((gap, k, pa, pb))

        if not near:
            return []
        # Cap the row count: the closest pairs are the ones that matter, and an
        # unbounded n_c would churn OSQP's factorization.
        near.sort(key=lambda t: t[0])
        rows: List[Row] = []
        for gap, k, pa, pb in near[: self._max_rows]:
            i, j = self._pairs[k]
            delta = pa - pb
            norm = float(np.linalg.norm(delta))
            if norm < 1e-9:
                continue                     # exactly coincident: no normal
            n_w = delta / norm

            Ja, Jad = kin.point_jacobian(self._fids[i], pa)
            Jb, Jbd = kin.point_jacobian(self._fids[j], pb)
            # Full-width first, then project: slicing the Jacobian before the
            # product would silently drop the coupling through the wrist.
            a_full = n_w @ (Ja - Jb)
            a = np.ascontiguousarray(a_full[self._arm_v], dtype=np.float64)
            jdq = float(n_w @ ((Jad - Jbd) @ v_full))
            h = float(gap - self._margin)
            if not (np.all(np.isfinite(a)) and np.isfinite(h) and np.isfinite(jdq)):
                continue
            rows.append((a, h, jdq,
                         f'sc:{self._caps[i].body[-6:]}/{self._caps[j].body[-6:]}'))
        return rows


# ── Construction helper ──────────────────────────────────────────────────────

def build_self_collision_builder(
    urdf_path: str,
    resolve_frame_id,
    arm_v_ids: Sequence[int],
    exclude_pairs: Optional[Sequence[str]] = None,
    margin: float = 0.0,
    horizon: float = 0.15,
    max_rows: int = 4,
) -> SelfCollisionRowBuilder:
    """Parse the ``*_sc`` capsules from *urdf_path* and wire up a builder.

    ``exclude_pairs`` should carry Franka's own SRDF ``reason="Never"`` list —
    without it the builder checks 47 pairs, 23 of which the manufacturer states
    cannot collide at any configuration, and several of those (link1/link3 above
    all) interpenetrate by centimetres at the home pose purely because every
    capsule radius already includes ``safety_distance``.
    """
    from franka_experiments.utils.self_collision import (
        build_capsule_pairs, parse_sc_capsules)

    with open(urdf_path) as fh:
        capsules = parse_sc_capsules(fh.read())
    if not capsules:
        raise RuntimeError(
            f'{urdf_path} has no *_sc links — generated without with_sc:=true?')
    pairs = build_capsule_pairs(capsules, list(exclude_pairs or []))
    if not pairs:
        raise RuntimeError('every capsule pair was excluded')

    fids = []
    for c in capsules:
        fid = resolve_frame_id(c.frame)
        if fid is None:
            raise RuntimeError(f'frame "{c.frame}" not in the Pinocchio model')
        fids.append(int(fid))

    return SelfCollisionRowBuilder(capsules, pairs, fids, arm_v_ids,
                                   margin=margin, horizon=horizon,
                                   max_rows=max_rows)
