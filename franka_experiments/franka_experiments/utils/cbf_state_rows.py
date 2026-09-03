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


# ── Retreat cap ──────────────────────────────────────────────────────────────
#
# Every other row here bounds a barrier's rate from BELOW. These two helpers
# build the missing upper bound: how fast the filter is allowed to run away.
#
# The HOCBF row demands  n̂ᵀJ q̈ ≥ −k1·ḣ − k0·h̄ − ċ  and says nothing about the
# other side, so once h̄ < 0 the demanded floor grows with k0·|h̄| and the QP
# keeps accelerating the arm away for as long as the violation lasts. Fast
# retreat next to a person is its own hazard — and the arm is left with a large
# velocity and a large tracking error once the obstacle is gone.
#
# A cap row is the SAME direction n̂ᵀJ bounded from above, stored with a = −n̂ᵀJ
# so the node's generic ``G = [−A | −e_group]`` assembly turns it into an upper
# bound with no special case anywhere in the QP path.


def retreat_cap_speed(
    v_obs: float,
    h_bar: float,
    *,
    base: float,
    obs_gain: float,
    depth_gain: float,
    depth_speed_ref: float,
    engage_gap: float,
    max_speed: float,
) -> float:
    """[m/s] fastest separation rate a control point may be given.

        ramp   = clip(max(v_obs,0) / depth_speed_ref, 0, 1)
        relief = max_speed · clip(h̄ / engage_gap, 0, 1)
        v_cap  = min( max_speed,
                      base
                      + obs_gain·max(v_obs,0)
                      + depth_gain·max(−h̄,0)·ramp
                      + relief )

    The governing idea is PROPORTIONALITY: how fast the robot may back away is
    set by how fast the obstacle is coming in, not by how alarming the geometry
    looks. A person stepping slowly toward the arm should be answered by the arm
    stepping slowly away; anything else is a hazard of its own.

    Read as a function of ``h̄`` the cap is V-shaped, with its minimum exactly at
    the barrier:

    * ``h̄ ≥ engage_gap`` — ``relief`` saturates the cap at ``max_speed``. The
      arm is doing its task, not avoiding, and a cap here would throttle
      ordinary motion along an arbitrary obstacle normal. That was measured:
      ``retreat=+0.161/0.150`` with the nearest obstacle 0.389 m away and no
      avoidance happening at all. ``relief`` is a RAMP rather than an on/off
      gate on purpose — a gate would have to sit either above the distance
      where a fast approach first engages the barrier (useless) or below it
      (a step in the cap at the worst moment). ``engage_gap ≤ 0`` disables the
      relief entirely, so the cap applies at full strength everywhere — the
      pre-ramp behaviour, kept reachable from configuration.
    * ``h̄ = 0`` — ``relief`` is zero and the cap is purely proportional:
      ``base + obs_gain·v_obs``. This is where avoidance happens and where the
      shaping is meant to be tightest.
    * ``h̄ < 0`` — the depth term escalates, gated by ``ramp``.

    The terms:

    * ``base`` — the creep, not a retreat speed. It is what remains when the
      obstacle is stationary, and its only job is to let the arm walk out of a
      violation that nothing is making worse. Keep it small: it is the one term
      that is NOT proportional to anything.
    * ``obs_gain·v_obs`` — the proportional term, and the one that should
      dominate. At ``obs_gain = 1`` the arm may separate exactly as fast as the
      obstacle closes, which holds the gap without outrunning anything: at the
      barrier the constraint is then simply ``ḣ = n̂ᵀJq̇ − v_obs ≤ base``.
      ``v_obs`` is clamped to the approaching half upstream, so a receding
      obstacle contributes nothing.
    * ``depth_gain·max(−h̄,0)·ramp`` — escalation for a violated barrier, scaled
      by ``ramp``. Without ``ramp`` this is the hole in the proportionality: a
      slow obstacle that ends up deep inside ``d_safe`` authorises a fast
      retreat purely because the geometry is bad — exactly the "l'ostacolo si
      muove lentamente ma il robot si allontana troppo velocemente" case. With
      it, depth can only amplify an approach that is actually happening.
      ``depth_speed_ref`` is the closing speed at which the escalation reaches
      full strength; ``≤ 0`` disables the ramp and restores the older,
      speed-independent behaviour.
    * ``max_speed`` — absolute ceiling regardless of the above.

    The clamps are what make the escalation temporary: the instant the obstacle
    leaves ``d_safe`` and stops closing, both extra terms vanish and the cap is
    ``base`` plus whatever ``relief`` the distance now buys.

    This bound is SOFT — it has its own slack, priced well below the barrier's,
    so a genuine emergency still overrules it. Being proportional is a shaping
    policy, not a safety guarantee; the barrier remains the guarantee.
    """
    v = max(float(v_obs), 0.0)
    h = float(h_bar)
    ref = float(depth_speed_ref)
    gap = float(engage_gap)
    ramp = 1.0 if ref <= 0.0 else min(v / ref, 1.0)
    # gap <= 0 means NO relief, i.e. the cap applies at full strength at every
    # distance — the pre-ramp behaviour. Same convention as depth_speed_ref
    # above: a non-positive value disables the refinement rather than
    # reinterpreting it.
    relief = 0.0 if gap <= 0.0 else float(max_speed) * min(max(h / gap, 0.0), 1.0)
    return min(float(max_speed),
               float(base)
               + float(obs_gain) * v
               + float(depth_gain) * max(-h, 0.0) * ramp
               + relief)


def retreat_cap_rhs(
    A_cap: np.ndarray,      # (n, nv) the STORED rows, i.e. −n̂ᵀJ
    qdot: np.ndarray,       # (nv,) measured joint velocity
    cap_v: np.ndarray,      # (n,) per-row cap from retreat_cap_speed
    horizon: float,         # [s] enforcement horizon T
) -> np.ndarray:
    """RHS of the cap rows for ``G x ≤ h_qp``.

    With the row stored as ``a = −n̂ᵀJ``, the node's ``G[:, :nv] = −A`` gives
    ``+n̂ᵀJ q̈ − s ≤ h_qp``, and the one-step bound

        n̂ᵀJ (q̇ + q̈·T) ≤ v_cap

    is exactly ``h_qp = (v_cap − n̂ᵀJ q̇) / T``, with ``n̂ᵀJ q̇ = −(A_cap q̇)``.

    ``horizon`` is symmetric on purpose — see the parameter docs. Uses the RAW
    ``q̇``: this is a measured velocity, not the noisy derivative the barrier's
    ``k1`` term filters.
    """
    return (cap_v + A_cap @ qdot) / float(horizon)


def link_speed_cap(clearance: float, *, v_max: float, reaction_s: float) -> float:
    """[m/s] fastest a control point may travel, given the room it has left.

        v_allow = min( v_max, clearance / reaction_s )

    Two different bounds, and the tighter wins:

    * ``v_max`` is the flat ceiling — no part of the robot moves faster than
      this, whatever the configuration. The joint velocity box cannot express
      it: a folded or near-singular arm decouples joint speed from task speed
      in both directions, so modest ``q̇`` can whip a link and vice versa.
    * ``clearance / reaction_s`` is the GEOMETRIC bound, and the reason this
      exists at all. The barrier is rebuilt at 50 Hz from ~30 Hz perception,
      so between two evaluations a point travels ``v·Δt`` blind. Let it move
      faster than its own remaining gap divided by that blind time and it can
      cross the barrier BETWEEN samples — the constraint was never violated at
      any instant the QP looked at, and the links interpenetrate anyway. This
      is plain discretisation tunnelling, and no amount of k0/k1 tuning fixes
      it: the fix is to forbid the speed that makes the gap jumpable.

    ``reaction_s`` is that worst-case blind time: perception period + QP period
    + actuation lag, not the QP period alone.
    """
    return min(float(v_max), max(float(clearance), 0.0) / float(reaction_s))


def link_speed_row(
    Jp: np.ndarray,         # (3, nv) point Jacobian
    qdot: np.ndarray,       # (nv,)
    v_allow: float,         # [m/s] from link_speed_cap
    activate_frac: float,   # emit only above this fraction of v_allow
    label: str,
) -> Optional[Tuple[np.ndarray, float, str]]:
    """One task-space speed row for a control point, or ``None``.

    ``‖ṗ‖ ≤ v_allow`` is a norm bound, not linear. Linearised along the
    direction the point is ACTUALLY travelling,

        v̂ᵀJp q̇ ≤ v_allow ,      v̂ = Jp q̇ / ‖Jp q̇‖

    which is exact for the current motion (``v̂ᵀJp q̇ = ‖ṗ‖``) and tightens the
    one direction that matters. Returned as ``a = −v̂ᵀJp`` so the node's generic
    ``G = [−A | −e]`` turns it into an UPPER bound, exactly like a retreat-cap
    row — and it shares that row's RHS builder, :func:`retreat_cap_rhs`.

    ``None`` when the point is slower than ``activate_frac·v_allow``: below that
    the row is non-binding and would only inflate ``n_c`` (and OSQP ``setup()``
    churn) on every control point at once. Same horizon-gate tradeoff every
    other row family here makes.

    Returns:
        ``(a, v_allow, label)`` or ``None``.
    """
    v = Jp @ qdot
    speed = float(np.linalg.norm(v))
    if speed < activate_frac * v_allow or speed < 1e-9:
        return None
    v_hat = v / speed
    return (-(v_hat @ Jp), float(v_allow), label)


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
