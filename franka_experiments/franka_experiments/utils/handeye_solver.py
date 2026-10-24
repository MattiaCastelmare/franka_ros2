"""Eye-in-hand hand-eye calibration: the maths, with no ROS in it.

Setup: the camera is rigidly mounted on the flange, the AprilTag lies still on
the desk. For every sample *i* the chain ``base → tag`` closes through two
paths::

    T_BE(i) · X · T_CT(i) = Y

* ``T_BE(i)`` — robot forward kinematics, base → flange (``fr3_link8``);
* ``T_CT(i)`` — AprilTag detection, camera optical frame → tag;
* ``X = T_EC`` — flange → camera optical frame, the result we want;
* ``Y = T_BT`` — base → tag, a nuisance unknown solved alongside X.

This is the eye-in-hand dual of what ``handeye_calibration_node`` solves
(``X · T_CT = T_BE · Y`` with the camera fixed and the tag on the flange).

Pipeline (``solve``): closed-form initial guess (Kabsch on the rotation axes of
relative motions, then linear least squares on the translations) → robust
12-DOF refinement → MAD outlier rejection → refit on a train split → held-out
residuals. The pose generators (``bootstrap_ee_poses``,
``orbit_camera_poses``) build the motion that keeps the tag in view.

Pure numpy/scipy, so it is testable without a robot (test_handeye_solver.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


# ── SE(3) helpers ────────────────────────────────────────────────────────────

def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rotvec(R: np.ndarray) -> np.ndarray:
    """SO(3) log: rotation matrix (or stack of them) → axis·angle [rad]."""
    return Rotation.from_matrix(R).as_rotvec()


def from_rotvec(w) -> np.ndarray:
    return Rotation.from_rotvec(np.asarray(w, dtype=float)).as_matrix()


def angle_deg(R: np.ndarray) -> float:
    return math.degrees(float(np.linalg.norm(rotvec(R))))


def rotation_distance_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    return angle_deg(R1.T @ R2)


def _stack(Ts: Sequence[np.ndarray]) -> np.ndarray:
    return np.asarray(Ts, dtype=float).reshape(-1, 4, 4)


# ── Residuals ────────────────────────────────────────────────────────────────

def loop_closure_errors(
    X: np.ndarray, Y: np.ndarray,
    T_BE: Sequence[np.ndarray], T_CT: Sequence[np.ndarray],
) -> Tuple[np.ndarray, np.ndarray]:
    """Per-sample error of ``E_i = Y⁻¹ · T_BE(i) · X · T_CT(i)`` (≈ I).

    Returns ``(translation [m], rotation [deg])`` arrays. The translation is
    where sample *i* puts the tag relative to where Y says it is, so it already
    includes the camera's rotation error times the viewing distance.
    """
    A, B = _stack(T_BE), _stack(T_CT)
    E = inv_T(Y)[None] @ A @ X[None] @ B
    t = np.linalg.norm(E[:, :3, 3], axis=1)
    r = np.degrees(np.linalg.norm(rotvec(E[:, :3, :3]), axis=1))
    return t, r


# ── Closed-form initial guess ────────────────────────────────────────────────

# Pairs rotating less than this carry no usable axis; pairs close to π have an
# axis whose sign flips between the two sides of AX = XB.
_MIN_PAIR_ANGLE_DEG = 2.0
_MAX_PAIR_ANGLE_DEG = 170.0


def _relative_motions(A: np.ndarray, B: np.ndarray):
    """All pairs (i, j) as ``A_ij · X = X · B_ij`` with enough rotation.

    From ``A_i X B_i = A_j X B_j``:  ``A_j⁻¹ A_i · X = X · B_j B_i⁻¹``.
    """
    out = []
    n = len(A)
    for i in range(n):
        for j in range(i + 1, n):
            Aij = inv_T(A[j]) @ A[i]
            Bij = B[j] @ inv_T(B[i])
            ang = angle_deg(Aij[:3, :3])
            if _MIN_PAIR_ANGLE_DEG <= ang <= _MAX_PAIR_ANGLE_DEG:
                out.append((Aij, Bij))
    return out


def initial_guess(
    T_BE: Sequence[np.ndarray], T_CT: Sequence[np.ndarray],
) -> Tuple[np.ndarray, np.ndarray]:
    """Closed-form ``(X, Y)`` from relative motions.

    Rotation: ``R_A · R_X = R_X · R_B`` means the rotation axes satisfy
    ``α = R_X · β``, a Kabsch fit over all pairs. Translation:
    ``(R_A − I) · t_X = R_X · t_B − t_A``, stacked linear least squares.
    Both need rotations about at least two non-parallel axes — raises
    ``ValueError`` otherwise, rather than returning a confident wrong answer.
    """
    A, B = _stack(T_BE), _stack(T_CT)
    if len(A) < 3:
        raise ValueError(f'need at least 3 samples, have {len(A)}')
    pairs = _relative_motions(A, B)
    if len(pairs) < 2:
        raise ValueError('not enough rotation between samples '
                         f'(< {_MIN_PAIR_ANGLE_DEG}° between every pair)')

    alphas = np.array([rotvec(a[:3, :3]) for a, _ in pairs])
    betas = np.array([rotvec(b[:3, :3]) for _, b in pairs])
    H = betas.T @ alphas
    U, S, Vt = np.linalg.svd(H)
    if S[1] < 1e-6 * max(S[0], 1e-12):
        raise ValueError('all relative rotations share one axis: X is not '
                         'observable (rotate the flange about another axis)')
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R_X = Vt.T @ D @ U.T

    C = np.vstack([a[:3, :3] - np.eye(3) for a, _ in pairs])
    d = np.concatenate([R_X @ b[:3, 3] - a[:3, 3] for a, b in pairs])
    t_X, *_ = np.linalg.lstsq(C, d, rcond=None)
    X = make_T(R_X, t_X)

    Ys = A @ X[None] @ B
    R_Y = Rotation.from_matrix(Ys[:, :3, :3]).mean().as_matrix()
    t_Y = np.median(Ys[:, :3, 3], axis=0)
    return X, make_T(R_Y, t_Y)


# ── Nonlinear refinement ─────────────────────────────────────────────────────

def refine(
    X0: np.ndarray, Y0: np.ndarray,
    T_BE: Sequence[np.ndarray], T_CT: Sequence[np.ndarray],
    *,
    rot_weight: float = 1.0,
    trans_weight: float = 1.0,
    huber_scale: float = 0.005,
) -> Tuple[np.ndarray, np.ndarray]:
    """Jointly refine X and Y (12 DOF) minimising the SE(3) loop error.

    Parameterised as local perturbations ``X = X0 · exp(δX)`` so no rotation
    (a camera mounted upside down included) sits on a parameter singularity.
    Huber loss with ``huber_scale`` [m / rad]: a few-millimetre detection
    wobble is fitted as L2, a bad detection is down-weighted.
    """
    A, B = _stack(T_BE), _stack(T_CT)

    def unpack(x):
        X = X0 @ make_T(from_rotvec(x[0:3]), x[3:6])
        Y = Y0 @ make_T(from_rotvec(x[6:9]), x[9:12])
        return X, Y

    def residuals(x):
        X, Y = unpack(x)
        E = inv_T(Y)[None] @ A @ X[None] @ B
        return np.hstack([rot_weight * rotvec(E[:, :3, :3]),
                          trans_weight * E[:, :3, 3]]).ravel()

    sol = least_squares(residuals, np.zeros(12), method='trf', loss='huber',
                        f_scale=huber_scale, max_nfev=2000)
    return unpack(sol.x)


# ── Outliers and observability ───────────────────────────────────────────────

def mad_inlier_mask(
    t_err: np.ndarray, r_err: np.ndarray, *,
    k: float = 3.0, t_floor_m: float = 0.005, r_floor_deg: float = 1.0,
) -> np.ndarray:
    """Keep samples within ``median + k·σ_MAD`` on BOTH error channels.

    The floors stop a very clean dataset (MAD ≈ 0) from rejecting samples
    that are off by a fraction of a millimetre.
    """
    def keep(e, floor):
        med = float(np.median(e))
        mad = 1.4826 * float(np.median(np.abs(e - med)))
        return e <= max(med + k * mad, floor)
    return keep(np.asarray(t_err), t_floor_m) & keep(np.asarray(r_err), r_floor_deg)


def rotation_axis_sigmas(T_BE: Sequence[np.ndarray]) -> np.ndarray:
    """Singular values (normalised to [0, 1]) of the relative rotation axes.

    Three comparable values = rotations about every axis; a smallest value
    near zero = all motion about (nearly) one plane of axes, a poorly
    observable X.
    """
    A = _stack(T_BE)
    axes = []
    for i in range(len(A)):
        for j in range(i + 1, len(A)):
            w = rotvec((inv_T(A[j]) @ A[i])[:3, :3])
            if math.degrees(np.linalg.norm(w)) >= _MIN_PAIR_ANGLE_DEG:
                axes.append(w / np.linalg.norm(w))
    if len(axes) < 3:
        return np.zeros(3)
    return np.linalg.svd(np.array(axes), compute_uv=False) / math.sqrt(len(axes))


# ── Full solve ───────────────────────────────────────────────────────────────

@dataclass
class HandEyeResult:
    X: np.ndarray                  # flange → camera optical frame
    Y: np.ndarray                  # base → tag
    inliers: np.ndarray            # bool mask over the input samples
    train: np.ndarray              # indices of the samples X was fitted on
    test: np.ndarray               # held-out indices (may be empty)
    train_t_mm: float
    train_r_deg: float
    test_t_mm: Optional[float]
    test_r_deg: Optional[float]
    axis_sigmas: np.ndarray

    def verdict_errors(self) -> Tuple[float, float, str]:
        """Errors the verdict is judged on: held-out if any, else train."""
        if self.test_t_mm is not None:
            return self.test_t_mm, self.test_r_deg, 'test'
        return self.train_t_mm, self.train_r_deg, 'train'


def solve(
    T_BE: Sequence[np.ndarray], T_CT: Sequence[np.ndarray], *,
    validation_ratio: float = 0.2,
    seed: int = 123,
    outlier_k: float = 3.0,
    rot_weight: float = 1.0,
    trans_weight: float = 1.0,
) -> HandEyeResult:
    """Initial guess → refine → reject outliers → refit on train → validate."""
    A, B = _stack(T_BE), _stack(T_CT)
    n = len(A)
    X0, Y0 = initial_guess(A, B)
    X0, Y0 = refine(X0, Y0, A, B, rot_weight=rot_weight, trans_weight=trans_weight)

    t, r = loop_closure_errors(X0, Y0, A, B)
    mask = (mad_inlier_mask(t, r, k=outlier_k) if n >= 6
            else np.ones(n, dtype=bool))
    idx = np.flatnonzero(mask)
    n_test = (int(round(len(idx) * validation_ratio))
              if validation_ratio > 0.0 and len(idx) >= 8 else 0)
    perm = np.random.default_rng(seed).permutation(idx)
    test, train = np.sort(perm[:n_test]), np.sort(perm[n_test:])

    # Warm start from the all-sample fit; the refit converges to the train
    # optimum, so the held-out samples do not leak into X.
    X, Y = refine(X0, Y0, A[train], B[train],
                  rot_weight=rot_weight, trans_weight=trans_weight)

    tt, rt = loop_closure_errors(X, Y, A[train], B[train])
    if n_test:
        te, re = loop_closure_errors(X, Y, A[test], B[test])
        test_t_mm, test_r_deg = 1000.0 * float(te.mean()), float(re.mean())
    else:
        test_t_mm = test_r_deg = None
    return HandEyeResult(
        X=X, Y=Y, inliers=mask, train=train, test=test,
        train_t_mm=1000.0 * float(tt.mean()), train_r_deg=float(rt.mean()),
        test_t_mm=test_t_mm, test_r_deg=test_r_deg,
        axis_sigmas=rotation_axis_sigmas(A[mask]))


# ── Pose generation ──────────────────────────────────────────────────────────

def look_at(cam_pos, target, x_hint) -> np.ndarray:
    """Camera OPTICAL-frame rotation (z forward, x right, y down) in base.

    z points from ``cam_pos`` at ``target``; x is ``x_hint`` projected
    orthogonal to z, so consecutive poses keep the image "up" consistent and
    the wrist does not wind up.
    """
    z = np.asarray(target, float) - np.asarray(cam_pos, float)
    z /= np.linalg.norm(z)
    x = np.asarray(x_hint, float) - float(np.dot(x_hint, z)) * z
    if np.linalg.norm(x) < 1e-6:                  # hint parallel to z
        x = np.cross(z, [0.0, 0.0, 1.0] if abs(z[2]) < 0.9 else [1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def bootstrap_ee_poses(
    T_BE0: np.ndarray, *, rot_deg: float = 10.0, trans_m: float = 0.03,
) -> List[np.ndarray]:
    """Small moves around the start flange pose, needing no prior on X.

    ±rot_deg about each flange axis (three rotation axes make X observable)
    and ±trans_m along base x/y. At a typical 0.3 m viewing distance a 10°
    flange rotation shifts the tag by ~5 cm in the scene, well inside the
    D405 field of view.
    """
    poses = []
    for axis in range(3):
        for sign in (1.0, -1.0):
            w = np.zeros(3)
            w[axis] = sign * math.radians(rot_deg)
            poses.append(T_BE0 @ make_T(from_rotvec(w), np.zeros(3)))
    for d in ((trans_m, 0, 0), (-trans_m, 0, 0), (0, trans_m, 0), (0, -trans_m, 0)):
        T = T_BE0.copy()
        T[:3, 3] += d
        poses.append(T)
    return poses


def _pose_distance(T1: np.ndarray, T2: np.ndarray) -> float:
    """Blend [m + 0.2·rad]: 0.2 m of travel weighs like one radian of turn."""
    return (float(np.linalg.norm(T1[:3, 3] - T2[:3, 3]))
            + 0.2 * math.radians(rotation_distance_deg(T1[:3, :3], T2[:3, :3])))


def orbit_camera_poses(
    p_tag, T_BC0: np.ndarray, n: int, *,
    radius_range: Tuple[float, float] = (0.20, 0.40),
    max_tilt_deg: float = 35.0,
    azimuth_half_range_deg: float = 60.0,
    max_roll_deg: float = 30.0,
    accept: Optional[Callable[[np.ndarray], bool]] = None,
    rng: Optional[np.random.Generator] = None,
    n_candidates: Optional[int] = None,
) -> List[np.ndarray]:
    """Camera poses on a spherical cap above the tag, all looking at it.

    Camera position: distance in ``radius_range``, tilt from the base vertical
    up to ``max_tilt_deg`` (the tag lies on the desk, so the camera stays above
    it), azimuth within ±``azimuth_half_range_deg`` of the start view. Roll
    about the optical axis up to ±``max_roll_deg`` around the start image
    orientation. ``accept(T_BC)`` filters out poses the arm cannot or must not
    reach. From the candidates, ``n`` are kept by farthest-point sampling
    (diversity) and returned in greedy nearest-neighbour order from the start
    pose (short moves).
    """
    candidates = [T for T in sample_orbit_candidates(
        p_tag, T_BC0, n_candidates or max(30 * n, 600), radius_range=radius_range,
        max_tilt_deg=max_tilt_deg, azimuth_half_range_deg=azimuth_half_range_deg,
        max_roll_deg=max_roll_deg, rng=rng) if accept is None or accept(T)]
    return select_diverse_poses(candidates, T_BC0, n)


def sample_orbit_candidates(
    p_tag, T_BC0: np.ndarray, n_candidates: int, *,
    radius_range: Tuple[float, float] = (0.20, 0.40),
    max_tilt_deg: float = 35.0,
    azimuth_half_range_deg: float = 60.0,
    max_roll_deg: float = 30.0,
    rng: Optional[np.random.Generator] = None,
) -> List[np.ndarray]:
    """Unfiltered random look-at camera poses on the cap (see orbit_camera_poses).

    Split out so a caller with an expensive filter (IK) can spread the checks
    over many control ticks and then call ``select_diverse_poses``.
    """
    rng = rng if rng is not None else np.random.default_rng(0)
    p_tag = np.asarray(p_tag, dtype=float)
    u0 = T_BC0[:3, 3] - p_tag
    az0 = math.atan2(u0[1], u0[0]) if np.hypot(u0[0], u0[1]) > 1e-3 else 0.0
    x_hint = T_BC0[:3, 0]
    tilt_max = math.radians(max_tilt_deg)

    candidates: List[np.ndarray] = []
    for _ in range(n_candidates):
        r = rng.uniform(*radius_range)
        tilt = math.acos(1.0 - rng.uniform() * (1.0 - math.cos(tilt_max)))  # uniform on the cap
        az = az0 + math.radians(rng.uniform(-azimuth_half_range_deg, azimuth_half_range_deg))
        p = p_tag + r * np.array([math.sin(tilt) * math.cos(az),
                                  math.sin(tilt) * math.sin(az),
                                  math.cos(tilt)])
        roll = math.radians(rng.uniform(-max_roll_deg, max_roll_deg))
        candidates.append(make_T(look_at(p, p_tag, x_hint) @ from_rotvec([0.0, 0.0, roll]), p))
    return candidates


def select_diverse_poses(
    candidates: Sequence[np.ndarray], T_start: np.ndarray, n: int,
) -> List[np.ndarray]:
    """Farthest-point subset of ``n`` poses, ordered as a greedy nearest-neighbour
    tour from ``T_start`` (diverse samples, short moves)."""
    candidates = list(candidates)
    if not candidates:
        return []
    chosen = [int(np.argmin([_pose_distance(T, T_start) for T in candidates]))]
    d_min = np.array([_pose_distance(T, candidates[chosen[0]]) for T in candidates])
    while len(chosen) < min(n, len(candidates)):
        k = int(np.argmax(d_min))
        chosen.append(k)
        d_min = np.minimum(d_min, [_pose_distance(T, candidates[k]) for T in candidates])

    ordered, current, left = [], T_start, [candidates[k] for k in chosen]
    while left:
        k = int(np.argmin([_pose_distance(current, T) for T in left]))
        current = left.pop(k)
        ordered.append(current)
    return ordered
