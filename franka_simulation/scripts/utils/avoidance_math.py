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

    for parent, child in link_pairs:
        fid_p = int(frame_ids[parent])
        fid_c = int(frame_ids[child])

        oMp = data.oMf[fid_p]
        oMc = data.oMf[fid_c]
        p_child_local = oMp.rotation.T @ (oMc.translation - oMp.translation)

        capsules[parent] = [
            {
                "p0": float(fr[0]) * p_child_local,
                "p1": float(fr[1]) * p_child_local,
                "radius": float(r[0]),
            },
            {
                "p0": float(fr[2]) * p_child_local,
                "p1": float(fr[3]) * p_child_local,
                "radius": float(r[1]),
            },
            {
                "p0": float(fr[4]) * p_child_local,
                "p1": float(fr[5]) * p_child_local,
                "radius": float(r[2]),
            },
        ]

    return frame_ids, capsules


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


def smooth_alpha(d: float, d_infl: float, d_safe: float) -> float:
    """
    Smooth activation in [0,1] with zero slope at the boundaries.
    This avoids velocity snaps when entering/exiting the influence zone.
    """
    d = float(d)
    d_infl = float(d_infl)
    d_safe = float(d_safe)

    if d_infl <= d_safe + 1e-12:
        return 1.0 if d <= d_safe else 0.0

    # Normalize: t=0 at d_infl, t=1 at d_safe
    t = (d_infl - d) / (d_infl - d_safe)
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


def staged_risk_weight(
    d: float,
    d_far: float = 0.30,
    d_mid: float = 0.20,
    d_near: float = 0.10,
    d_stop: float = 0.05,
) -> float:
    """Distance-based risk weight w(d) in [0,1] with staged ramps.

    Mapping:
      d >= d_far            -> w = 0
      d_far..d_mid          -> w ramps 0 .. 0.25
      d_mid..d_near         -> w ramps 0.25 .. 0.75
      d_near..d_stop        -> w ramps 0.75 .. 1.0
      d <= d_stop           -> w = 1
    """
    d = float(d)
    d_far = float(d_far)
    d_mid = float(d_mid)
    d_near = float(d_near)
    d_stop = float(d_stop)

    # enforce monotonic thresholds (best-effort)
    d_far = max(d_far, d_mid + 1e-9)
    d_mid = max(d_mid, d_near + 1e-9)
    d_near = max(d_near, d_stop + 1e-9)
    d_stop = max(0.0, d_stop)

    if d >= d_far:
        return 0.0
    if d <= d_stop:
        return 1.0

    if d > d_mid:
        x = ramp_smooth(d, d_far, d_mid)
        return 0.25 * x
    if d > d_near:
        x = ramp_smooth(d, d_mid, d_near)
        return 0.25 + 0.50 * x
    x = ramp_smooth(d, d_near, d_stop)
    return 0.75 + 0.25 * x


def alpha_from_distance(
    d: float,
    *,
    alpha_min: float,
    alpha_max: float,
    d_far: float,
    d_mid: float,
    d_near: float,
    d_stop: float,
) -> float:
    """Risk-scaled CBF alpha(d) = lerp(alpha_min, alpha_max, w(d))."""
    w = staged_risk_weight(float(d), d_far=float(d_far), d_mid=float(d_mid), d_near=float(d_near), d_stop=float(d_stop))
    a0 = float(alpha_min)
    a1 = float(alpha_max)
    return float(a0 + w * (a1 - a0))


def qp_gamma_from_distance(
    d: float,
    *,
    gamma_min: float,
    gamma_max: float,
    d_far: float,
    d_mid: float,
    d_near: float,
    d_stop: float,
) -> float:
    """Risk-scaled QP damping gamma(d) = lerp(gamma_min, gamma_max, w(d))."""
    w = staged_risk_weight(float(d), d_far=float(d_far), d_mid=float(d_mid), d_near=float(d_near), d_stop=float(d_stop))
    g0 = float(gamma_min)
    g1 = float(gamma_max)
    return float(g0 + w * (g1 - g0))


def beta_lpf_from_distance(
    d: float,
    *,
    beta_far: float,
    beta_near: float,
    d_far: float,
    d_mid: float,
    d_near: float,
    d_stop: float,
) -> float:
    """Risk-scaled output LPF coefficient beta in [0,1]."""
    w = staged_risk_weight(float(d), d_far=float(d_far), d_mid=float(d_mid), d_near=float(d_near), d_stop=float(d_stop))
    b = float(beta_far) + float(w) * (float(beta_near) - float(beta_far))
    return float(np.clip(b, 0.0, 1.0))


def posture_reference(
    posture_reference_param: Any,
    *,
    model: Any,
) -> Optional[np.ndarray]:
    """Return a 7D posture reference (radians) if available.

    Behaviour matches the controller's previous `_posture_reference`:
    - if `posture_reference_param` is a list of 7 values, use it
    - else try Pinocchio neutral(model) and take first 7 values
    - else return None
    """
    try:
        if isinstance(posture_reference_param, list) and len(posture_reference_param) == 7:
            return np.array([float(x) for x in posture_reference_param], dtype=float).reshape(7)
    except Exception:
        pass

    try:
        import pinocchio as pin  # type: ignore

        q0 = pin.neutral(model)
        q0 = np.array(q0, dtype=float).reshape(-1)
        if q0.shape[0] >= 7:
            return q0[:7].copy()
    except Exception:
        pass

    return None


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

        if float(dist) < float(best_d):
            best_d = float(dist)
            best_dir = np.array(dir_w, dtype=float).reshape(3)
            best_p_seg = np.array(p_seg_w, dtype=float).reshape(3)
            best_p_box = np.array(p_box_w, dtype=float).reshape(3)
            best_samples = list(samples)

    return float(best_d), best_dir, best_p_seg, best_p_box, best_samples


def build_cbf_constraints(
    candidates: list[dict],
    active_threshold: float,
    *,
    K: int,
    cbf_eps: float,
    cbf_d_safe: float,
    approach_speed_limit: float,
    # risk-scaled alpha
    alpha_min: float,
    alpha_max: float,
    risk_d_far: float,
    risk_d_mid: float,
    risk_d_near: float,
    stop_distance: float,
    # pinocchio
    model: Any,
    data: Any,
    q: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, int, Optional[dict]]:
    """Build top-K CBF constraints from hazard candidates.

    This matches the controller's former `compute_constraints` method.
    """
    K = max(0, int(K))
    G = np.zeros((K, 7), dtype=float)
    b = np.full((K,), -1e9, dtype=float)  # inactive constraints
    if K == 0:
        return G, b, 0, None

    act = [c for c in candidates if float(c.get("d", 1e9)) <= float(active_threshold)]
    act.sort(key=lambda x: float(x.get("d", 1e9)))

    m_active = min(K, len(act))
    active_best = act[0] if m_active > 0 else None

    eps = float(cbf_eps)
    for i in range(m_active):
        c = act[i]
        d = float(c["d"])
        kind = str(c.get("kind", ""))
        v_obs_proj = 0.0

        if kind in ("external", "ground"):
            fid = int(c["fid"])
            p = np.array(c["p"], dtype=float).reshape(3)
            n = np.array(c["n"], dtype=float).reshape(3)
            n = n / (float(np.linalg.norm(n)) + eps)
            Jp = point_jacobian_world(model, data, q, fid, p)
            g = (n.reshape(1, 3) @ Jp).reshape(-1)
        elif kind == "self":
            fid_i = int(c["fid_i"])
            fid_j = int(c["fid_j"])
            p_i = np.array(c["p_i"], dtype=float).reshape(3)
            p_j = np.array(c["p_j"], dtype=float).reshape(3)
            n = np.array(c["n"], dtype=float).reshape(3)
            n = n / (float(np.linalg.norm(n)) + eps)
            J_i = point_jacobian_world(model, data, q, fid_i, p_i)
            J_j = point_jacobian_world(model, data, q, fid_j, p_j)
            g = (n.reshape(1, 3) @ (J_i - J_j)).reshape(-1)
        else:
            continue

        alpha_i = alpha_from_distance(
            d,
            alpha_min=float(alpha_min),
            alpha_max=float(alpha_max),
            d_far=float(risk_d_far),
            d_mid=float(risk_d_mid),
            d_near=float(risk_d_near),
            d_stop=float(stop_distance),
        )

        bi = float(v_obs_proj - float(alpha_i) * (d - float(cbf_d_safe)))
        v_lim = float(approach_speed_limit)
        if v_lim > 0.0:
            bi = max(bi, -v_lim)

        G[i, :] = g
        b[i] = bi

    return G, b, m_active, active_best


class OsqpCbfQpSolver:
    """OSQP-based QP solver for the CBF safety filter.

    This wraps the controller's former `_init_osqp_solver` + `solve_qp_safety_filter`.
    All heavy deps are imported lazily.
    """

    def __init__(
        self,
        *,
        K: int,
        W_diag: np.ndarray,
        lambda_reg: float,
        rho_slack: float,
        max_abs_vel: float,
        max_iter: int = 100,
    ):
        self.K = int(K)
        self.W_diag = np.array(W_diag, dtype=float).reshape(-1)
        self.lambda_reg = float(lambda_reg)
        self.rho_slack = float(rho_slack)
        self.max_abs_vel = float(max_abs_vel)
        self.max_iter = int(max_iter)

        self.available = False
        self.init_status = "disabled"

        self._sp = None
        self._osqp_mod = None
        self._solver = None

        self._A_data_template = None
        self._A_data_work = None
        self._A_g_slices = None
        self._qp_q_work = None
        self._qp_l_work = None
        self._P_data_template = None
        self._P_data_work = None

        if self.K <= 0:
            self.available = False
            self.init_status = "disabled"
            return

        try:
            import osqp  # type: ignore
            import scipy.sparse as sp  # type: ignore

            self._sp = sp
            self._osqp_mod = osqp
            self._init_solver()
            self.available = True
            self.init_status = "ready"
        except Exception as e:
            self.available = False
            self._solver = None
            self.init_status = f"no_osqp:{e.__class__.__name__}"

    def _init_solver(self) -> None:
        K = int(self.K)
        if K <= 0:
            return

        sp = self._sp
        osqp = self._osqp_mod
        if sp is None or osqp is None:
            raise RuntimeError("OSQP/Scipy not available")

        n = 7 + K
        m = (2 * K) + 7

        P_diag = np.zeros(n, dtype=float)
        P_diag[:7] = 2.0 * (np.maximum(self.W_diag, 1e-9) + float(self.lambda_reg))
        P_diag[7:] = 2.0 * float(self.rho_slack)
        P = sp.diags(P_diag, offsets=0, format="csc")

        indptr = [0]
        indices: list[int] = []
        data: list[float] = []
        self._A_g_slices = []

        for j in range(7):
            col_rows = list(range(0, K)) + [2 * K + j]
            col_data = [0.0] * K + [1.0]
            start = len(data)
            indices.extend(col_rows)
            data.extend(col_data)
            end = start + K
            self._A_g_slices.append((start, end))
            indptr.append(len(data))

        for k in range(K):
            col_rows = [k, K + k]
            col_data = [1.0, 1.0]
            indices.extend(col_rows)
            data.extend(col_data)
            indptr.append(len(data))

        A = sp.csc_matrix(
            (
                np.array(data, dtype=float),
                np.array(indices, dtype=int),
                np.array(indptr, dtype=int),
            ),
            shape=(m, n),
        )

        l = np.full(m, -np.inf, dtype=float)
        u = np.full(m, +np.inf, dtype=float)
        l[:K] = -1e9
        l[K : 2 * K] = 0.0

        qmin = -float(self.max_abs_vel)
        qmax = +float(self.max_abs_vel)
        l[2 * K : 2 * K + 7] = qmin
        u[2 * K : 2 * K + 7] = qmax

        q = np.zeros(n, dtype=float)
        solver = osqp.OSQP()
        solver.setup(
            P=P,
            q=q,
            A=A,
            l=l,
            u=u,
            warm_start=True,
            verbose=False,
            polish=False,
            max_iter=int(self.max_iter),
        )

        self._solver = solver
        self._A_data_template = A.data.copy()
        self._A_data_work = A.data.copy()
        self._P_data_template = P.data.copy()
        self._P_data_work = P.data.copy()
        self._qp_q_work = q.copy()
        self._qp_l_work = l.copy()

    def solve(
        self,
        qdot_nom: np.ndarray,
        G: np.ndarray,
        b: np.ndarray,
        *,
        gamma: float,
        qdot_prev: np.ndarray,
    ) -> Optional[tuple[np.ndarray, float, str]]:
        if (not self.available) or (self._solver is None):
            return None

        K = int(self.K)
        if K <= 0:
            qn = np.array(qdot_nom, dtype=float).reshape(7)
            return qn, 0.0, "no_constraints"

        qdot_nom = np.array(qdot_nom, dtype=float).reshape(7)
        qdot_prev = np.array(qdot_prev, dtype=float).reshape(7)
        G = np.array(G, dtype=float)
        b = np.array(b, dtype=float).reshape(-1)

        gamma = float(max(0.0, gamma))

        self._qp_q_work[:] = 0.0
        self._qp_q_work[:7] = -2.0 * (np.maximum(self.W_diag, 1e-9) * qdot_nom + gamma * qdot_prev)

        self._qp_l_work[:K] = b.reshape(-1)[:K]

        self._A_data_work[:] = self._A_data_template
        for j in range(7):
            sl = self._A_g_slices[j]
            self._A_data_work[sl[0] : sl[1]] = G[:, j]

        if (self._P_data_template is not None) and (self._P_data_work is not None):
            self._P_data_work[:] = self._P_data_template
            self._P_data_work[:7] = self._P_data_template[:7] + (2.0 * gamma)

        try:
            if self._P_data_work is not None:
                self._solver.update(
                    Px=self._P_data_work,
                    q=self._qp_q_work,
                    l=self._qp_l_work,
                    Ax=self._A_data_work,
                )
            else:
                self._solver.update(q=self._qp_q_work, l=self._qp_l_work, Ax=self._A_data_work)
            res = self._solver.solve()
        except Exception:
            return None

        status = str(getattr(res.info, "status", ""))
        status_ok = status.lower().startswith("solved")
        if (not status_ok) or (res.x is None):
            return None

        x = np.array(res.x, dtype=float).reshape(-1)
        qdot = x[:7]
        slack = x[7 : 7 + K] if x.shape[0] >= (7 + K) else np.zeros(K)
        slack_max = float(np.max(slack)) if slack.size > 0 else 0.0
        return qdot, slack_max, status


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
    d_infl: float,
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

    if d < float(d_infl):
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
