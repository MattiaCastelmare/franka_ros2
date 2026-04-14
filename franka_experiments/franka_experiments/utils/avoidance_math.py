
"""Utilities shared by avoidance-related scripts.

Design goals
------------
- Keep the *import-time* dependencies minimal (numpy is always OK).
- Optional heavy dependencies (Pinocchio, OSQP, ROS message types) are imported
    lazily inside the few helpers that need them.

This lets controller scripts stay readable without forcing every runtime
environment to have all optional dependencies installed just to import this
module.
"""

from __future__ import annotations

import os
import math
import tempfile
import zlib
from typing import Any, Optional

import numpy as np


def clip_aabb(p: np.ndarray, half: np.ndarray) -> np.ndarray:
    """Clip a 3D point to an axis-aligned box [-half, +half] in the same frame."""
    p = np.array(p, dtype=float).reshape(3)
    half = np.array(half, dtype=float).reshape(3)
    return np.clip(p, -half, +half)


def outward_normal_aabb(p_inside: np.ndarray, half: np.ndarray) -> np.ndarray:
    """Best-effort outward normal when p is (numerically) on/inside an AABB.

    This is used when the closest point computation returns a point with near-zero
    separation from the box (contact / penetration), so we still need a stable
    direction to push outward.
    """
    p = np.array(p_inside, dtype=float).reshape(3)
    h = np.array(half, dtype=float).reshape(3)
    # distance to each face
    d = h - np.abs(p)
    axis = int(np.argmin(d))
    n = np.zeros(3, dtype=float)
    n[axis] = 1.0 if p[axis] >= 0.0 else -1.0
    return n


def closest_points_segment_aabb(
    a: np.ndarray,
    b: np.ndarray,
    half: np.ndarray,
    iters: int = 8,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Closest points between segment a-b and axis-aligned box [-half,+half].

    Returns (p_seg, p_box, t) where:
      p_seg = a + t*(b-a), t in [0,1]
      p_box = clip_aabb(p_seg)

    Implementation: alternating projections between convex sets (segment and AABB).
    For this small problem size it is fast and stable.
    """
    a = np.array(a, dtype=float).reshape(3)
    b = np.array(b, dtype=float).reshape(3)
    half = np.array(half, dtype=float).reshape(3)

    d = b - a
    dd = float(d @ d)
    if dd < 1e-12:
        p = a.copy()
        q = clip_aabb(p, half)
        return p, q, 0.0

    t = 0.5
    it = max(1, int(iters))
    for _ in range(it):
        p = a + t * d
        q = clip_aabb(p, half)
        t = float((q - a) @ d) / dd
        t = float(np.clip(t, 0.0, 1.0))

    p = a + t * d
    q = clip_aabb(p, half)
    return p, q, float(t)


def closest_points_on_segments(
    p0: np.ndarray,
    p1: np.ndarray,
    q0: np.ndarray,
    q1: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return closest points between segments p0-p1 and q0-q1."""
    p0 = np.array(p0, dtype=float).reshape(3)
    p1 = np.array(p1, dtype=float).reshape(3)
    q0 = np.array(q0, dtype=float).reshape(3)
    q1 = np.array(q1, dtype=float).reshape(3)

    u = p1 - p0
    v = q1 - q0
    w0 = p0 - q0

    a = float(u @ u)
    b = float(u @ v)
    c = float(v @ v)
    d = float(u @ w0)
    e = float(v @ w0)

    denom = a * c - b * b
    s = 0.0
    t = 0.0
    if denom > 1e-12:
        s = (b * e - c * d) / denom
        t = (a * e - b * d) / denom

    s = float(np.clip(s, 0.0, 1.0))
    t = float(np.clip(t, 0.0, 1.0))
    cp_p = p0 + s * u
    cp_q = q0 + t * v
    return cp_p, cp_q


def tangential_dir(dir_vec: np.ndarray) -> np.ndarray:
    """Return a unit tangential direction orthogonal to dir_vec (prefer world-up swirl)."""
    d = np.array(dir_vec, dtype=float).reshape(3)
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        return np.zeros(3)
    d = d / n
    up = np.array([0.0, 0.0, 1.0], dtype=float)
    t = np.cross(up, d)
    tn = float(np.linalg.norm(t))
    if tn < 1e-6:
        ax = np.array([1.0, 0.0, 0.0], dtype=float)
        t = np.cross(ax, d)
        tn = float(np.linalg.norm(t))
    if tn < 1e-9:
        return np.zeros(3)
    return t / tn


def capsule_segment_distance_to_obb(
    p0_world: np.ndarray,
    p1_world: np.ndarray,
    radius: float,
    center_world: np.ndarray,
    half_sizes: np.ndarray,
    R_world_from_box: np.ndarray,
    *,
    projection_iters: int = 8,
    repulsion_spread_enable: bool = False,
    repulsion_spread_samples: int = 5,
    repulsion_spread_half_length: float = 0.10,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray, list[dict]]:
    """Distance between a capsule segment (p0-p1, radius) and a single oriented box.

    This is a pure helper extracted from the controller to make the node easier to read.

    Args:
      p0_world, p1_world: capsule segment endpoints in world frame (3,)
      radius: capsule radius [m]
      center_world: box center in world frame (3,)
      half_sizes: box half-dimensions in box frame (3,)
      R_world_from_box: rotation matrix mapping box frame -> world frame (3x3)
      projection_iters: iterations for closest_points_segment_aabb in box frame
      repulsion_spread_enable/samples/half_length: optional region sampling along segment

    Returns:
      (dist, dir_world, p_seg_world, p_box_world, samples)

    Where:
      - dist = ||p_seg - p_box|| - radius
      - dir_world is a unit vector pointing from box to capsule
      - samples is a list of dicts with keys: p_seg, p_box, dir, distance, weight
    """
    p0 = np.array(p0_world, dtype=float).reshape(3)
    p1 = np.array(p1_world, dtype=float).reshape(3)
    center = np.array(center_world, dtype=float).reshape(3)
    half = np.array(half_sizes, dtype=float).reshape(3)
    R = np.array(R_world_from_box, dtype=float).reshape(3, 3)
    r = float(radius)

    # Transform segment into box-local frame (OBB -> AABB)
    a = R.T @ (p0 - center)
    b = R.T @ (p1 - center)

    p_seg_l, p_box_l, t_star = closest_points_segment_aabb(a, b, half, iters=int(projection_iters))

    diff_l = p_seg_l - p_box_l
    diff_n = float(np.linalg.norm(diff_l))
    if diff_n < 1e-9:
        dir_l = outward_normal_aabb(p_seg_l, half)
    else:
        dir_l = diff_l / diff_n

    p_seg_w = center + R @ p_seg_l
    p_box_w = center + R @ p_box_l
    dir_w = R @ dir_l
    dir_w = dir_w / (float(np.linalg.norm(dir_w)) + 1e-9)

    dist = float(np.linalg.norm(p_seg_w - p_box_w) - r)

    # Optional repulsion samples around the closest point for a smoother "region" effect.
    samples: list[dict] = []
    if repulsion_spread_enable and int(repulsion_spread_samples) >= 2:
        d_ab = b - a
        L = float(np.linalg.norm(d_ab))
        if L > 1e-9:
            half_len = max(0.0, float(repulsion_spread_half_length))
            dt = float(np.clip(half_len / L, 0.0, 0.5))
            n = int(repulsion_spread_samples)
            if (n % 2) == 0:
                n += 1
            offsets = np.linspace(-dt, +dt, n)
            sigma = max(1e-9, 0.5 * dt) if dt > 1e-9 else 1e-9

            for off in offsets:
                ti = float(np.clip(float(t_star) + float(off), 0.0, 1.0))
                pi_l = a + ti * d_ab
                qi_l = clip_aabb(pi_l, half)
                di_l = pi_l - qi_l
                di_n = float(np.linalg.norm(di_l))
                if di_n < 1e-9:
                    ni_l = outward_normal_aabb(pi_l, half)
                else:
                    ni_l = di_l / di_n

                pi_w = center + R @ pi_l
                qi_w = center + R @ qi_l
                ni_w = R @ ni_l
                ni_w = ni_w / (float(np.linalg.norm(ni_w)) + 1e-9)

                di = float(np.linalg.norm(pi_w - qi_w) - r)
                w = float(math.exp(-0.5 * float(off * off) / float(sigma * sigma)))
                samples.append(
                    {
                        "p_seg": pi_w,
                        "p_box": qi_w,
                        "dir": ni_w,
                        "distance": di,
                        "weight": float(w),
                    }
                )

    return dist, dir_w, p_seg_w, p_box_w, samples


def quat_from_z_axis_to_direction(direction: np.ndarray) -> np.ndarray:
    """Quaternion rotating +Z to the given direction.

    Returns quaternion as (x, y, z, w). This matches the marker code previously
    implemented in online_avoidance_controller.py.
    """
    d = np.array(direction, dtype=float).reshape(3)
    norm_dir = float(np.linalg.norm(d))
    if norm_dir < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    d = d / norm_dir
    v = np.cross(z_axis, d)
    c = float(np.dot(z_axis, d))

    # 180-degree rotation: z -> -z, pick X axis (same as previous implementation)
    if c <= -1.0 + 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    s = float(np.sqrt((1.0 + c) * 2.0))
    return np.array([v[0] / s, v[1] / s, v[2] / s, s / 2.0], dtype=float)


def enforce_halfspace_with_box(
    qdot_des: np.ndarray,
    j_row: np.ndarray,
    d_dot_min: float,
    max_abs_vel: float,
    iters: int = 4,
    eps: float = 1e-4,
    correction_l2_max: Optional[float] = None,
) -> tuple[np.ndarray, bool]:
    """Project a desired joint velocity into:  j_row @ qdot >= d_dot_min  and  |qdot_i| <= max_abs_vel.

    This is a small alternating-projection loop (fast, dependency-free) that approximates:

      min ||qdot - qdot_des||^2
      s.t. j_row @ qdot >= d_dot_min
           |qdot_i| <= max_abs_vel

    Args:
      qdot_des: desired joint velocity (N,)
      j_row: distance Jacobian row (N,), where d_dot ≈ j_row @ qdot
      d_dot_min: lower bound for distance rate
      max_abs_vel: symmetric per-joint box bound
      iters: number of alternating projections
      eps: tolerance on the half-space satisfaction
      correction_l2_max: optional L2 cap on the correction step (helps smoothness)

    Returns:
      (qdot_proj, ok)
    """
    qdot = np.array(qdot_des, dtype=float).reshape(-1)
    j = np.array(j_row, dtype=float).reshape(-1)
    n = int(qdot.shape[0])
    if n == 0:
        return qdot, True
    if j.shape[0] != n:
        # Best-effort: mismatch means we cannot enforce reliably.
        return np.clip(qdot, -float(max_abs_vel), +float(max_abs_vel)), False

    maxv = float(max_abs_vel)
    if maxv <= 0.0:
        return np.zeros_like(qdot), False

    jn2 = float(j @ j) + 1e-8
    qdot = np.clip(qdot, -maxv, +maxv)

    ok = False
    for _ in range(max(1, int(iters))):
        d_dot = float(j @ qdot)
        if d_dot >= float(d_dot_min) - float(eps):
            ok = True
            break

        lam = (float(d_dot_min) - d_dot) / jn2
        corr = lam * j

        if correction_l2_max is not None:
            try:
                cmax = float(correction_l2_max)
                if cmax > 0.0:
                    cn = float(np.linalg.norm(corr))
                    if cn > cmax:
                        corr *= cmax / (cn + 1e-9)
            except Exception:
                pass

        qdot = np.clip(qdot + corr, -maxv, +maxv)

    # Final check (even if we ran out of iterations)
    try:
        ok = ok or (float(j @ qdot) >= float(d_dot_min) - float(eps))
    except Exception:
        ok = False

    return qdot, ok


def check_halfspace_feasible(
    qdot: np.ndarray,
    a_row: np.ndarray,
    b_scalar: float,
    tol: float = 1e-6,
) -> bool:
    """Verify that a joint velocity satisfies the CBF half-space constraint.

    Checks:  a_row @ qdot >= b_scalar - tol

    This is the post-solver verification for the CBF safety filter.
    When this returns True the barrier constraint  J_d · q̇ ≥ −α(h)  holds
    within numerical tolerance.
    """
    q = np.array(qdot, dtype=float).reshape(-1)
    a = np.array(a_row, dtype=float).reshape(-1)
    if q.shape[0] != a.shape[0]:
        return False
    return float(a @ q) >= float(b_scalar) - float(tol)


def check_all_halfspaces_feasible(
    qdot: np.ndarray,
    G: np.ndarray,
    b: np.ndarray,
    tol: float = 1e-6,
) -> bool:
    """Verify that *qdot* satisfies **all** CBF half-space constraints.

    Checks:  G[i] @ qdot >= b[i] - tol   for every row i.

    Returns True only when every single constraint is met within tolerance.
    If G is empty (no constraints) the result is trivially True.
    """
    q = np.array(qdot, dtype=float).reshape(-1)
    G = np.array(G, dtype=float)
    b = np.array(b, dtype=float).reshape(-1)
    if G.size == 0 or G.ndim != 2:
        return True
    M = int(G.shape[0])
    for i in range(M):
        if float(G[i] @ q) < float(b[i]) - float(tol):
            return False
    return True


def capsule_risk_weight(
    cap_idx: int,
    last_idx: int,
    *,
    weight_last2: float = 2.0,
    weight_last3: float = 1.5,
    weight_default: float = 1.0,
) -> float:
    """Risk weight for a capsule based on proximity to end-effector.

    End-effector capsules are more "dangerous" and should maintain a larger
    safety distance.  This function maps a capsule index to a scalar weight
    that amplifies ``d_safe`` and (optionally) ``influence_distance``.

    Weight tiers (counted from the end)::

        last 2  (cap_idx >= last_idx - 1) → weight_last2   (default 2.0)
        last 3  (cap_idx == last_idx - 2) → weight_last3   (default 1.5)
        others                            → weight_default  (default 1.0)

    Args:
      cap_idx:  capsule index (0-based, e.g. 0..7 for FR3).
      last_idx: maximum capsule index among active capsules.
      weight_last2/last3/default: per-tier multipliers.

    Returns:
      Scalar weight  w ≥ weight_default.
    """
    if last_idx < 0 or cap_idx < 0:
        return float(weight_default)
    if cap_idx >= last_idx - 1:      # last 2 capsules
        return float(weight_last2)
    if cap_idx == last_idx - 2:      # 3rd from end
        return float(weight_last3)
    return float(weight_default)


def solve_cbf_qp_1constraint_box(
    qdot_des: np.ndarray,
    a_row: np.ndarray,
    b_scalar: float,
    max_vel: float,
    *,
    iters: int = 6,
    eps: float = 1e-4,
) -> tuple[np.ndarray, bool]:
    """CBF-QP safety filter: 1 linear constraint + symmetric box bounds.

    Solves:

      min  ‖q̇ − q̇_des‖²
      s.t. a_row · q̇ ≥ b_scalar          (CBF constraint)
           |q̇_i| ≤ max_vel   ∀ i         (joint velocity box)

    Three-stage algorithm
    ---------------------
    1. **Alternating projection** (via :func:`enforce_halfspace_with_box`):
       iterates half-space projection → box clip.  This finds a good
       approximate solution near *q̇_des* but may fail when the box clip
       after the last half-space projection re-introduces a small violation.

    2. **Post-verification** (via :func:`check_halfspace_feasible`):
       if the constraint is satisfied within tolerance, return immediately.

    3. **Best-effort fallback** — when alternating projection could not
       satisfy the constraint (box bounds conflict with the half-space),
       compute the *maximum feasible growth rate* inside the box:

           q̇_best_i = max_vel · sign(a_row_i)

       This is the closed-form solution of:

           argmax_{|q_i| ≤ v_max}  a_row · q

       (each component independently maximises its contribution to d_dot).

       • If a_row · q̇_best ≥ b_scalar → the problem *is* feasible and
         q̇_best satisfies the CBF; return (q̇_best, True).
       • Otherwise the constraint is **geometrically infeasible** inside the
         box (the robot's velocity limits physically cannot produce a large
         enough d_dot).  Return (q̇_best, False) — this is the safest
         possible action: it maximises the distance growth rate even though
         the hard CBF bound cannot be met.

    Degenerate cases
    ~~~~~~~~~~~~~~~~
    • a_row ≈ 0 (‖a_row‖ < 1e-12):  the constraint is vacuous (or
      numerically meaningless).  Return clip(q̇_des) as feasible.
    • max_vel ≤ 0:  only q̇ = 0 is admissible.  Return zeros.

    Args:
      qdot_des: nominal (desired) joint velocity (N,).
      a_row: constraint gradient row (N,), i.e. J_d.
      b_scalar: constraint lower bound, i.e. −k_α · h.
      max_vel: symmetric per-joint velocity bound [rad/s].
      iters: alternating projection iterations (default 6).
      eps: feasibility tolerance.

    Returns:
      (qdot_safe, feasible) — projected velocity and feasibility flag.
      ``feasible`` is True **iff** a_row · qdot_safe ≥ b_scalar − eps.
    """
    q_des = np.array(qdot_des, dtype=float).reshape(-1)
    a = np.array(a_row, dtype=float).reshape(-1)
    b = float(b_scalar)
    maxv = float(max_vel)
    n = q_des.shape[0]

    # --- Edge case: empty vector ---
    if n == 0:
        return q_des, True

    # --- Edge case: zero velocity bound ---
    if maxv <= 0.0:
        return np.zeros(n, dtype=float), (b <= float(eps))

    # --- Edge case: a_row numerically zero ---
    #     The constraint  0 · q̇ ≥ b  is satisfied iff b ≤ 0 (vacuous).
    #     No direction can influence d, so just clip the desired velocity.
    a_norm2 = float(a @ a)
    if a_norm2 < 1e-12:
        return np.clip(q_des, -maxv, +maxv), (b <= float(eps))

    # --- Stage 1: alternating projection (closest to qdot_des) ---
    qdot_proj, ok_ap = enforce_halfspace_with_box(
        qdot_des=q_des,
        j_row=a,
        d_dot_min=b,
        max_abs_vel=maxv,
        iters=int(iters),
        eps=float(eps),
    )

    # --- Stage 2: rigorous post-verification ---
    if check_halfspace_feasible(qdot_proj, a, b, tol=float(eps)):
        return qdot_proj, True

    # --- Stage 3: best-effort fallback ---
    #
    #   The alternating projection failed to reconcile the half-space with
    #   the box.  This happens when the box clip after the last half-space
    #   projection pushes the solution back outside the half-space.
    #
    #   Compute the point inside the box that maximises  a · q̇ :
    #       q̇_best_i = max_vel · sign(a_i)
    #   This is the LP vertex that gives the highest possible d_dot.
    qdot_best = maxv * np.sign(a)

    if check_halfspace_feasible(qdot_best, a, b, tol=float(eps)):
        # The problem IS feasible — alternating projection just failed
        # to find the solution close to qdot_des.  Return the LP vertex.
        return qdot_best, True

    # Geometrically infeasible: even the maximum-effort velocity cannot
    # satisfy the CBF bound.  Return it anyway — it maximises the distance
    # growth rate, which is the safest achievable action.
    return qdot_best, False


def pocs_project_halfspaces_with_box(
    qdot_nom: np.ndarray,
    G: np.ndarray,
    b: np.ndarray,
    max_abs_vel: float,
    iters: int = 3,
    eps: float = 1e-9,
    g_norm2_eps: float = 1e-12,
) -> np.ndarray:
    """Deterministic fallback projection (POCS) onto multiple half-spaces plus a box.

    Approximates:
      min ||q - qdot_nom||^2
      s.t.  G_i q >= b_i   for i=0..M-1
            |q_j| <= max_abs_vel

    This is the same algorithm previously implemented as `fallback_projection` in
    online_avoidance_controller.py.
    """
    q = np.array(qdot_nom, dtype=float).reshape(-1)
    G = np.array(G, dtype=float)
    b = np.array(b, dtype=float).reshape(-1)

    n = int(q.shape[0])
    if n == 0:
        return q

    maxv = float(max_abs_vel)
    if maxv <= 0.0:
        return np.zeros_like(q)

    q = np.clip(q, -maxv, +maxv)

    if G.size == 0:
        return q
    if G.ndim != 2:
        return q

    M = int(G.shape[0])
    if M <= 0:
        return q

    # Ensure b length
    if b.shape[0] < M:
        # best effort: pad with very negative values (inactive)
        b_pad = np.full((M,), -1e9, dtype=float)
        b_pad[: b.shape[0]] = b
        b = b_pad
    else:
        b = b[:M]

    for _ in range(max(1, int(iters))):
        for i in range(M):
            g = G[i, :].reshape(-1)
            if g.shape[0] != n:
                continue
            bi = float(b[i])
            g_norm2 = float(g @ g)
            if g_norm2 < float(g_norm2_eps):
                continue
            val = float(g @ q)
            if val + 1e-12 < bi:
                q = q + ((bi - val) / (g_norm2 + float(eps))) * g
                q = np.clip(q, -maxv, +maxv)

    return q


def skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix such that skew(v) @ w == v x w."""
    v = np.array(v, dtype=float).reshape(3)
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ],
        dtype=float,
    )


def point_jacobian_world(
    model: Any,
    data: Any,
    q: np.ndarray,
    fid: int,
    p_world: np.ndarray,
) -> np.ndarray:
    """3xN Jacobian of a point rigidly attached to frame `fid`, expressed in WORLD.

    This is Pinocchio-specific, but we keep it here to remove boilerplate from the
    controller node. The Pinocchio import is *lazy* so importing this module in
    contexts that only use the numpy helpers remains possible.
    """
    import pinocchio as pin  # type: ignore

    p_world = np.array(p_world, dtype=float).reshape(3)
    J6 = pin.computeFrameJacobian(model, data, q, int(fid), pin.ReferenceFrame.WORLD)

    Jv = np.array(J6[:3, :], dtype=float)
    Jw = np.array(J6[3:, :], dtype=float)

    oMf = data.oMf[int(fid)]
    r = (p_world - np.array(oMf.translation, dtype=float).reshape(3)).reshape(3)
    # v_point = v_origin + w x r = v_origin - skew(r) @ w
    return Jv - (skew(r) @ Jw)


def compute_closest_constraint_jacobian(
    closest_candidate: Optional[dict],
    model: Any,
    data: Any,
    q: np.ndarray,
    n_dof: int = 7,
) -> np.ndarray:
    """Compute the distance Jacobian row for the closest constraint.
    
    Args:
        closest_candidate: Candidate dict with 'kind', 'fid', 'p', 'n' (external/ground)
                          or 'fid_i', 'fid_j', 'p_i', 'p_j', 'n' (self-collision).
        model: Pinocchio model.
        data: Pinocchio data (must have computed FK).
        q: Joint configuration.
        n_dof: Number of degrees of freedom (default 7).
    
    Returns:
        Jacobian row (n_dof,) where d_dot ≈ j_row @ qdot.
        Returns zeros if computation fails.
    """
    j_row = np.zeros(n_dof, dtype=float)
    
    if closest_candidate is None:
        return j_row
    
    kind = str(closest_candidate.get("kind", "")).strip()
    
    try:
        if kind in ("external", "ground"):
            fid = int(closest_candidate.get("fid", -1))
            p = np.array(closest_candidate.get("p", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
            n = np.array(closest_candidate.get("n", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
            
            if fid >= 0:
                n_norm = float(np.linalg.norm(n))
                if n_norm > 1e-9:
                    n_unit = n / n_norm
                    Jp = point_jacobian_world(model, data, q, fid, p)
                    j_vec = (n_unit.reshape(1, 3) @ Jp).reshape(-1)
                    if j_vec.shape[0] == n_dof and np.all(np.isfinite(j_vec)):
                        j_row = j_vec
        
        elif kind == "self":
            fid_i = int(closest_candidate.get("fid_i", -1))
            fid_j = int(closest_candidate.get("fid_j", -1))
            p_i = np.array(closest_candidate.get("p_i", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
            p_j = np.array(closest_candidate.get("p_j", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
            n = np.array(closest_candidate.get("n", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
            
            n_norm = float(np.linalg.norm(n))
            if fid_i >= 0 and fid_j >= 0 and n_norm > 1e-9:
                n_unit = n / n_norm
                J_i = point_jacobian_world(model, data, q, fid_i, p_i)
                J_j = point_jacobian_world(model, data, q, fid_j, p_j)
                j_vec = (n_unit.reshape(1, 3) @ (J_i - J_j)).reshape(-1)
                if j_vec.shape[0] == n_dof and np.all(np.isfinite(j_vec)):
                    j_row = j_vec
    
    except Exception:
        pass
    
    return j_row


def build_reduced_pinocchio_model_from_urdf(
    urdf_xml: str,
    *,
    lock_keyword: str = "finger",
) -> tuple[Any, Any]:
    """Build a reduced Pinocchio model+data from an URDF XML string.

    This is extracted from the controller's `_init_pinocchio_and_capsules`.

    Notes:
      - Pinocchio's `buildModelFromUrdf` takes a file path, so we write a temp file.
      - We lock joints whose name contains `lock_keyword` (default: "finger").

    Returns:
      (model, data)
    """
    import pinocchio as pin  # type: ignore

    urdf_xml = str(urdf_xml)
    lock_keyword = str(lock_keyword)

    urdf_path = None
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as f:
            f.write(urdf_xml.encode())
            urdf_path = f.name

        model_full = pin.buildModelFromUrdf(urdf_path)
        lock = [model_full.getJointId(n) for n in model_full.names if lock_keyword in str(n)]
        model = pin.buildReducedModel(model_full, lock, pin.neutral(model_full))
        data = model.createData()
        return model, data
    finally:
        if urdf_path is not None:
            try:
                os.unlink(urdf_path)
            except Exception:
                pass


def build_capsules_for_link_pairs(
    *,
    model: Any,
    data: Any,
    link_pairs: list[tuple[str, str]],
    capsule_fractions: Any,
    capsule_radii: Any,
) -> tuple[dict[str, int], dict[str, list[dict[str, Any]]]]:
    """Build per-link capsules (3 per link) and the corresponding frame ids.

    Extracted from the controller's `_init_pinocchio_and_capsules`.

    Returns:
      (frame_ids, capsules)
    """
    import pinocchio as pin  # type: ignore

    frame_ids: dict[str, int] = {}
    capsules: dict[str, list[dict[str, Any]]] = {}

    for parent, child in link_pairs:
        for link in (parent, child):
            if link not in frame_ids:
                frame_ids[link] = int(model.getFrameId(link))

    q0 = pin.neutral(model)
    pin.forwardKinematics(model, data, q0)
    pin.updateFramePlacements(model, data)

    fr = list(capsule_fractions) if isinstance(capsule_fractions, list) else []
    if len(fr) != 6:
        fr = [0.00, 0.35, 0.25, 0.75, 0.60, 0.95]
    r = list(capsule_radii) if isinstance(capsule_radii, list) else []
    if len(r) != 3:
        r = [0.15, 0.12, 0.13]
    radius_single = float(max(r))

    for parent, child in link_pairs:
        fid_p = int(frame_ids[parent])
        fid_c = int(frame_ids[child])

        oMp = data.oMf[fid_p]
        oMc = data.oMf[fid_c]
        p_child_local = oMp.rotation.T @ (oMc.translation - oMp.translation)

        capsules[parent] = [
            {
                "p0": np.zeros_like(p_child_local),
                "p1": np.array(p_child_local, dtype=float),
                "radius": float(radius_single),
            }
        ]

    return frame_ids, capsules


def build_joint_to_joint_capsules(
    *,
    model: Any,
    capsule_radii: Any,
) -> dict[str, list[dict[str, Any]]]:
    """Build joint-to-joint capsules for FR3 robot.
    
    Creates exactly 8 capsules:
    - Capsule 0: fr3_link0  → fr3_joint1
    - Capsule 1: fr3_joint1 → fr3_joint2
    - Capsule 2: fr3_joint2 → fr3_joint3 (SHORTENED to link2 length)
    - Capsule 3: fr3_joint3 → fr3_joint4
    - Capsule 4: fr3_joint4 → fr3_joint5 (SHORTENED to link4 length)
    - Capsule 5: fr3_joint5 → fr3_joint6 (START from cap4 endpoint)
    - Capsule 6: fr3_joint6 → fr3_joint7
    - Capsule 7: fr3_joint7 → fr3_link8 (end-effector)
    
    This replaces the old link_pairs + capsule_fractions approach.
    
    Capsules 2, 4, 5 are modified based on exact link lengths from FR3 kinematics.yaml:
    - Link2 length: 0.316 m (from joint2 origin to joint3 origin along -y)
    - Link4 length: ~0.3927 m (from joint4 to joint5, accounting for offsets)
    
    Returns:
      capsules: Dict[capsule_name, List[capsule_info_dict]]
                Each capsule_info_dict contains:
                  - "type": "joint_joint", "frame_joint", or "joint_frame"
                  - "start_id": joint_id or frame_id
                  - "end_id": joint_id or frame_id
                  - "start_type": "joint" or "frame"
                  - "end_type": "joint" or "frame"
                  - "radius": float
                  - "link_idx": int (for exclusion logic)
                  - "target_length": float (optional, for shortening)
                  - "use_prev_capsule_end": bool (optional, for cap5)
    """
    import pinocchio as pin  # type: ignore
    
    # Link lengths from FR3 kinematics.yaml (explicit values from DH parameters)
    # Link2: joint2→joint3 has xyz=[0, -0.316, 0] => length = 0.316 m
    # Link4: joint4→joint5 has xyz=[-0.0825, 0.384, 0] => length = sqrt(0.0825² + 0.384²)
    LINK2_LENGTH = 0.15  # m
    LINK4_LENGTH = 0.15  # m (sqrt(0.0825**2 + 0.384**2))
    
    # Prepare radii (if not enough values, repeat the last one)
    radii = list(capsule_radii) if isinstance(capsule_radii, list) else []
    if not radii:
        radii = [0.12]
    while len(radii) < 8:
        radii.append(radii[-1])
    
    # Get frame ID for link0
    try:
        link0_frame_id = int(model.getFrameId("fr3_link0"))
    except Exception:
        # Fallback to universe frame if link0 doesn't exist
        link0_frame_id = 0
    
    # Get joint IDs for joints 1-7
    joint_ids = []
    for i in range(1, 8):
        joint_id = int(model.getJointId(f"fr3_joint{i}"))
        joint_ids.append(joint_id)
    
    # Get frame ID for end-effector link8
    ee_frame_id = int(model.getFrameId("fr3_link8"))
    
    capsules: dict[str, list[dict[str, Any]]] = {}
    
    # Capsule 0: link0 → joint1
    capsules["fr3_cap_0"] = [{
        "type": "frame_joint",
        "start_id": link0_frame_id,
        "end_id": joint_ids[0],
        "start_type": "frame",
        "end_type": "joint",
        "radius": float(radii[0]),
        "link_idx": 0,
        "name": "cap_link0_joint1",
    }]
    
    # Capsules 1-6: joint_i → joint_i+1
    # Special handling for capsules 2, 4, 5 (modified geometry)
    for i in range(6):
        cap_dict = {
            "type": "joint_joint",
            "start_id": joint_ids[i],
            "end_id": joint_ids[i + 1],
            "start_type": "joint",
            "end_type": "joint",
            "radius": float(radii[i + 1]),
            "link_idx": i + 1,
            "name": f"cap_joint{i+1}_joint{i+2}",
        }
        
        # Cap2 (i=1): Shorten to link2 length
        if i == 1:
            cap_dict["target_length"] = LINK2_LENGTH
        
        # Cap4 (i=3): Shorten to link4 length
        elif i == 3:
            cap_dict["target_length"] = LINK4_LENGTH
        
        # Cap5 (i=4): Start from cap4 endpoint instead of joint5
        elif i == 4:
            cap_dict["use_prev_capsule_end"] = True
        
        capsules[f"fr3_cap_{i+1}"] = [cap_dict]
    
    # Capsule 7: joint7 → link8
    capsules["fr3_cap_7"] = [{
        "type": "joint_frame",
        "start_id": joint_ids[6],
        "end_id": ee_frame_id,
        "start_type": "joint",
        "end_type": "frame",
        "radius": float(radii[7]),
        "link_idx": 7,
        "name": "cap_joint7_link8",
    }]
    
    return capsules


def ordered_joint_positions_from_joint_state(msg: Any, joint_names: list[str]) -> Optional[np.ndarray]:
    """Extract joint positions in a specified order from a JointState-like message.

    Returns None if some joint is missing.
    """
    try:
        name_to_idx = {str(n): int(i) for i, n in enumerate(getattr(msg, "name"))}
        pos = getattr(msg, "position")
        q = [pos[name_to_idx[str(n)]] for n in joint_names]
        return np.array(q, dtype=float)
    except Exception:
        return None


def filtered_collision_objects_from_planning_scene(msg: Any, excluded_substrings: list[str]) -> list[Any]:
    """Return PlanningScene world collision_objects excluding ids containing any substring.

    This is extracted from the controller's `_obstacle_cb`.
    """
    out: list[Any] = []

    try:
        objs = list(msg.world.collision_objects)
    except Exception:
        return []

    excl = [str(s).lower() for s in (excluded_substrings or [])]
    for o in objs:
        try:
            oid = str(getattr(o, "id", "")).lower()
        except Exception:
            oid = ""
        if any(ex in oid for ex in excl):
            continue
        out.append(o)
    return out


def quat_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion (x,y,z,w) to 3x3 rotation matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3, dtype=float)

    x /= n
    y /= n
    z /= n
    w /= n

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def stable_sign_from_id(text: str) -> float:
    """Deterministic +/-1 sign from a string id (no dependence on PYTHONHASHSEED)."""
    try:
        v = zlib.crc32(text.encode("utf-8"))
    except Exception:
        v = 0
    return 1.0 if (v % 2) == 0 else -1.0


def smooth_alpha(d: float, influence_distance: float, safety_margin: float) -> float:
    """
    Smooth activation in [0,1] with zero slope at the boundaries.
    This avoids velocity snaps when entering/exiting the influence zone.
    """
    d = float(d)
    influence_distance = float(influence_distance)
    safety_margin = float(safety_margin)

    if influence_distance <= safety_margin + 1e-12:
        return 1.0 if d <= safety_margin else 0.0

    # Normalize: t=0 at influence_distance, t=1 at safety_margin
    t = (influence_distance - d) / (influence_distance - safety_margin)
    t = max(0.0, min(1.0, float(t)))

    # Quintic smoothstep (C2 continuous, zero derivative at ends)
    return float(t ** 3 * (10.0 - 15.0 * t + 6.0 * t * t))


def smoothstep01(x: float) -> float:
    """C1-continuous smoothstep on [0,1]."""
    x = float(np.clip(float(x), 0.0, 1.0))
    return float(3.0 * x * x - 2.0 * x * x * x)


def ramp_smooth(d: float, d_hi: float, d_lo: float) -> float:
    """Smooth ramp 0..1 when d goes from d_hi to d_lo (d decreases).

    - returns 0 for d >= d_hi
    - returns 1 for d <= d_lo
    - smoothstep in-between
    """
    d = float(d)
    d_hi = float(d_hi)
    d_lo = float(d_lo)
    if d_hi <= d_lo + 1e-12:
        return 0.0
    if d >= d_hi:
        return 0.0
    if d <= d_lo:
        return 1.0
    x = (d_hi - d) / (d_hi - d_lo)
    return smoothstep01(x)


# Legacy velocity avoidance functions removed (staged_risk_weight, alpha_from_distance,
# qp_gamma_from_distance, beta_lpf_from_distance, posture_reference).
# Distance-only controller does not compute velocities.


def distance_capsule_to_collision_object_boxes(
    p0_world: np.ndarray,
    p1_world: np.ndarray,
    radius: float,
    obs: Any,
    *,
    box_projection_iters: int,
    repulsion_spread_enable: bool,
    repulsion_spread_samples: int,
    repulsion_spread_half_length: float,
) -> tuple[float, Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], list[dict]]:
    """Distance between a capsule segment and a MoveIt CollisionObject containing BOX primitives.

    This is the controller's former `_distance_capsule_to_box` but extracted.
    We keep it ROS-type-agnostic by accepting `obs` as Any and accessing the
    expected fields (`primitives`, `primitive_poses`).
    """
    best_d = 1e6
    best_dir: Optional[np.ndarray] = None
    best_p_seg: Optional[np.ndarray] = None
    best_p_box: Optional[np.ndarray] = None
    best_samples: list[dict] = []

    try:
        primitives = list(getattr(obs, "primitives"))
        poses = list(getattr(obs, "primitive_poses"))
    except Exception:
        return best_d, None, None, None, []

    for i, prim in enumerate(primitives):
        try:
            if int(getattr(prim, "type")) != int(getattr(prim, "BOX")):
                continue
        except Exception:
            continue

        if i >= len(poses):
            continue
        pose = poses[i]

        try:
            center = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            half = np.array(list(getattr(prim, "dimensions")), dtype=float) / 2.0
            q = pose.orientation
            R = quat_to_rot_matrix(float(q.x), float(q.y), float(q.z), float(q.w))
        except Exception:
            continue

        dist, dir_w, p_seg_w, p_box_w, samples = capsule_segment_distance_to_obb(
            p0_world=p0_world,
            p1_world=p1_world,
            radius=float(radius),
            center_world=center,
            half_sizes=half,
            R_world_from_box=R,
            projection_iters=int(box_projection_iters),
            repulsion_spread_enable=bool(repulsion_spread_enable),
            repulsion_spread_samples=int(repulsion_spread_samples),
            repulsion_spread_half_length=float(repulsion_spread_half_length),
        )

        # Enforce dir = normalize(p_seg - p_box) (box -> capsule).
        try:
            v = np.array(p_seg_w, dtype=float).reshape(3) - np.array(p_box_w, dtype=float).reshape(3)
            vn = float(np.linalg.norm(v))
            if vn > 1e-9:
                dir_w = v / vn
        except Exception:
            pass

        # Ensure sample directions follow the same convention.
        try:
            if isinstance(samples, list) and len(samples) > 0:
                fixed_samples = []
                for s in samples:
                    try:
                        ps = np.array(s.get("p_seg", p_seg_w), dtype=float).reshape(3)
                        pb = np.array(s.get("p_box", p_box_w), dtype=float).reshape(3)
                        vsp = ps - pb
                        vn = float(np.linalg.norm(vsp))
                        s2 = dict(s)
                        if vn > 1e-9:
                            s2["dir"] = vsp / vn
                        fixed_samples.append(s2)
                    except Exception:
                        fixed_samples.append(s)
                samples = fixed_samples
        except Exception:
            pass

        if float(dist) < float(best_d):
            best_d = float(dist)
            best_dir = np.array(dir_w, dtype=float).reshape(3)
            best_p_seg = np.array(p_seg_w, dtype=float).reshape(3)
            best_p_box = np.array(p_box_w, dtype=float).reshape(3)
            best_samples = list(samples)

    return float(best_d), best_dir, best_p_seg, best_p_box, best_samples


# Legacy CBF constraint builder and OSQP QP solver removed.
# Distance-only controller does not compute velocities or apply CBF safety filters.


def make_capsule_markers(
    p0: np.ndarray,
    p1: np.ndarray,
    radius: float,
    marker_id: int,
    *,
    stamp_msg: Any,
    frame_id: str = "world",
    ns: str = "capsules",
    rgba: tuple[float, float, float, float] = (0.9, 0.1, 0.1, 0.5),
) -> list[Any]:
    """Create RViz markers for a capsule (cylinder + 2 spheres).

    ROS message types are imported lazily.
    """
    from visualization_msgs.msg import Marker  # type: ignore
    from std_msgs.msg import ColorRGBA  # type: ignore

    p0 = np.array(p0, dtype=float).reshape(3)
    p1 = np.array(p1, dtype=float).reshape(3)
    radius = float(radius)

    markers: list[Any] = []

    cyl = Marker()
    cyl.header.frame_id = str(frame_id)
    cyl.header.stamp = stamp_msg
    cyl.ns = str(ns)
    cyl.id = int(marker_id)
    cyl.type = Marker.CYLINDER
    cyl.action = Marker.ADD

    center = (p0 + p1) / 2.0
    height = float(np.linalg.norm(p1 - p0))
    cyl.pose.position.x = float(center[0])
    cyl.pose.position.y = float(center[1])
    cyl.pose.position.z = float(center[2])

    direction = p1 - p0
    q = quat_from_z_axis_to_direction(direction)
    cyl.pose.orientation.x = float(q[0])
    cyl.pose.orientation.y = float(q[1])
    cyl.pose.orientation.z = float(q[2])
    cyl.pose.orientation.w = float(q[3])

    cyl.scale.x = 2.0 * radius
    cyl.scale.y = 2.0 * radius
    cyl.scale.z = float(height)
    cyl.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
    markers.append(cyl)

    for idx, pos in enumerate([p0, p1], start=1):
        sph = Marker()
        sph.header.frame_id = str(frame_id)
        sph.header.stamp = stamp_msg
        sph.ns = str(ns)
        sph.id = int(marker_id) + int(idx)
        sph.type = Marker.SPHERE
        sph.action = Marker.ADD
        sph.pose.position.x = float(pos[0])
        sph.pose.position.y = float(pos[1])
        sph.pose.position.z = float(pos[2])
        sph.pose.orientation.w = 1.0
        sph.scale.x = sph.scale.y = sph.scale.z = 2.0 * radius
        sph.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
        markers.append(sph)

    return markers


def make_distance_markers(
    p_cap: np.ndarray,
    p_obs: np.ndarray,
    d: float,
    marker_id: int,
    *,
    stamp_msg: Any,
    activation_distance: float,
    rgba: Optional[tuple[float, float, float, float]] = None,
    frame_id: str = "world",
    ns_line: str = "distances",
    ns_text: str = "distances_text",
    line_width: float = 0.005,
    text_size: float = 0.02,
) -> list[Any]:
    """Create RViz markers for a distance segment + label.

    Matches the controller's previous inline marker creation.
    """
    from visualization_msgs.msg import Marker  # type: ignore
    from geometry_msgs.msg import Point  # type: ignore
    from std_msgs.msg import ColorRGBA  # type: ignore

    p_cap = np.array(p_cap, dtype=float).reshape(3)
    p_obs = np.array(p_obs, dtype=float).reshape(3)
    d = float(d)

    if rgba is not None:
        
        try:
            r, g, b, a = rgba
            color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
        except Exception:
            color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)
    elif d <= float(activation_distance):
        color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
       
    else:
        color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)

    line_marker = Marker()
    line_marker.header.frame_id = str(frame_id)
    line_marker.header.stamp = stamp_msg
    line_marker.ns = str(ns_line)
    line_marker.id = int(marker_id)
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.scale.x = float(line_width)
    line_marker.color = color

    p1_point = Point()
    p1_point.x, p1_point.y, p1_point.z = float(p_cap[0]), float(p_cap[1]), float(p_cap[2])
    p2_point = Point()
    p2_point.x, p2_point.y, p2_point.z = float(p_obs[0]), float(p_obs[1]), float(p_obs[2])
    line_marker.points = [p1_point, p2_point]

    text_marker = Marker()
    text_marker.header.frame_id = str(frame_id)
    text_marker.header.stamp = stamp_msg
    text_marker.ns = str(ns_text)
    text_marker.id = int(marker_id) + 1
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.scale.z = float(text_size)
    text_marker.color = color

    mid_point = (p_cap + p_obs) / 2.0
    text_marker.pose.position.x = float(mid_point[0])
    text_marker.pose.position.y = float(mid_point[1])
    text_marker.pose.position.z = float(mid_point[2])
    text_marker.pose.orientation.w = 1.0
    text_marker.text = f"{d:.3f}m"

    return [line_marker, text_marker]