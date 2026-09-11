"""Sim-to-real contract for the ONNX Safe-RL policy (``rl_policy_commander``).

Everything here is plain numpy / YAML so it can be unit-tested without a ROS
environment, and — more importantly — so the **observation contract** between
training and deployment lives in ONE place.

``franka_sim/envs/franka_cbf_env.py`` builds, every 100 Hz control tick::

    obs(24) = [ q(7), q̇(7), ee_pos(3), target(3), obstacle(3), d_min(1) ]
    action(7) ∈ [−1, 1]   →   q̈_nom = action · q̈_max   →   CBF filter

:func:`build_observation` rebuilds exactly that vector from robot topics and
:func:`action_to_qddot` applies exactly that scaling, so a policy trained in
``franka_sim`` sees the same numbers on hardware.

Obstacle mapping (sim ↔ real)
-----------------------------
In simulation the obstacle is a sphere of radius ``r_obs`` and the observation
carries its CENTRE; the barrier uses the SURFACE distance
``d = ‖p_cp − p_obs‖ − r_obs − r_cp``.

On the robot, ``MultiLinkDistance`` carries, per CONTROL POINT (several share
one ``robot_link_name``), the surface distance
``d = ‖p_cp − p_human‖ − r_cp`` (``distance_engine`` already subtracts the
capsule radius), the closest human point ``p_human`` and the unit normal
``n̂`` pointing obstacle → robot.  Both quantities therefore already mean
"distance between the robot capsule surface and the obstacle surface" and map
1:1.  The observation's obstacle SLOT is reconstructed as the centre of the
fictitious sim sphere tangent to the point cloud at ``p_human``::

    p_obs = p_human − n̂ · r_obs

which restores ``‖p_cp − p_obs‖ − r_obs − r_cp = d`` exactly.

One asymmetry is unavoidable and deliberately left in place: the real engine
clamps ``d`` at 0 (``np.maximum(..., 0)``) while the sim reports negative
penetration.  Real ``d_min`` therefore saturates at 0 instead of going
negative — the conservative direction (the policy never sees a "less bad than
reported" state), and the CBF filter downstream is the actual guarantee.
"""

from __future__ import annotations

import os
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from .constants import NUM_JOINTS

OBS_DIM: int = 2 * NUM_JOINTS + 10
"""Observation width: q(7) + q̇(7) + ee(3) + target(3) + obstacle(3) + d_min(1)."""

ACT_DIM: int = NUM_JOINTS
"""Action width: one normalised nominal acceleration per arm joint."""

_JOINT_KEYS: List[str] = [f'joint{i}' for i in range(1, NUM_JOINTS + 1)]


# ── Observation / action contract ────────────────────────────────────────────

def build_observation(
    q: np.ndarray,
    qdot: np.ndarray,
    ee_pos: np.ndarray,
    target: np.ndarray,
    obstacle: np.ndarray,
    d_min: float,
    out: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Assemble the 24-dim observation exactly as ``FrankaCBFEnv._get_obs``.

    *out* — when given a preallocated ``(OBS_DIM,)`` or ``(1, OBS_DIM)``
    float32 buffer — is filled in place and returned, so the control loop
    allocates nothing per tick.  Non-finite entries are zeroed (a NaN reaching
    the network would poison the whole action vector).
    """
    if out is None:
        out = np.zeros(OBS_DIM, dtype=np.float32)
    flat = out.reshape(-1)
    if flat.size != OBS_DIM:
        raise ValueError(f'out must hold {OBS_DIM} values, got {flat.size}')

    n = NUM_JOINTS
    flat[0:n]         = q
    flat[n:2 * n]     = qdot
    flat[2 * n:2 * n + 3] = ee_pos
    flat[2 * n + 3:2 * n + 6] = target
    flat[2 * n + 6:2 * n + 9] = obstacle
    flat[2 * n + 9]   = d_min
    np.nan_to_num(flat, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
    return out


def action_to_qddot(
    action: np.ndarray,
    qddot_max: np.ndarray,
    scale: float = 1.0,
    out: Optional[np.ndarray] = None,
) -> np.ndarray:
    """``a ∈ [−1,1]⁷ → q̈_nom = clip(a) · q̈_max · scale`` (sim step 1).

    *scale* is a deployment-only derate (``≤ 1``) for cautious first runs on
    hardware; it never widens the sim envelope.  Non-finite actions collapse to
    zero — an unusable policy output must not become an unbounded command.
    """
    a = np.clip(np.nan_to_num(np.asarray(action, dtype=np.float64).reshape(-1),
                              nan=0.0, posinf=0.0, neginf=0.0), -1.0, 1.0)
    if out is None:
        out = np.zeros(ACT_DIM, dtype=np.float64)
    np.multiply(a, qddot_max, out=out)
    out *= float(scale)
    return out


# ── Obstacle slot reconstruction ─────────────────────────────────────────────

def obstacle_centre(p_human: np.ndarray, n_hat: np.ndarray,
                    obstacle_radius: float) -> np.ndarray:
    """Centre of the sim-equivalent obstacle sphere tangent at *p_human*."""
    return np.asarray(p_human, dtype=np.float64) - \
        float(obstacle_radius) * np.asarray(n_hat, dtype=np.float64)


def nearest_obstacle(
    entries: Iterable[Tuple[str, float, np.ndarray, np.ndarray]],
    obstacle_radius: float,
    links: Optional[Sequence[str]] = None,
) -> Optional[Tuple[np.ndarray, float]]:
    """Closest link entry → ``(obstacle_centre, d_min)``, or ``None`` if empty.

    *entries* are ``(link_name, d, n̂, p_human)`` tuples as decoded from
    ``MultiLinkDistance`` (already validity-filtered by the caller).  *links*,
    when non-empty, restricts the search to those link names — use it to mirror
    the control-point subset the policy was trained against.
    """
    best = None
    allowed = set(links) if links else None
    for name, d, n_hat, p_human in entries:
        if allowed is not None and name not in allowed:
            continue
        if not np.isfinite(d):
            continue
        if best is None or d < best[1]:
            best = (name, float(d), n_hat, p_human)
    if best is None:
        return None
    return obstacle_centre(best[3], best[2], obstacle_radius), best[1]


def synthetic_obstacle(ee_pos: np.ndarray, centre: np.ndarray,
                       obstacle_radius: float) -> Tuple[np.ndarray, float]:
    """"No obstacle in sight" fallback slot: a sphere parked at *centre*.

    Used only when the perception pipeline was never started (distance topic
    never seen).  Keeping the slot geometrically self-consistent — ``d`` really
    is the EE-to-sphere surface distance — avoids feeding the network a
    contradictory (position, distance) pair.  A *stale* perception pipeline is
    a fault, not a fallback: the caller must stop commanding instead.
    """
    c = np.asarray(centre, dtype=np.float64)
    d = float(np.linalg.norm(np.asarray(ee_pos, dtype=np.float64) - c)
              - float(obstacle_radius))
    return c, d


# ── Config / model discovery ─────────────────────────────────────────────────

def qddot_max_from_limits(limits: dict) -> np.ndarray:
    """``joint_limits`` block → ``(7,)`` q̈_max, in ``joint1..joint7`` order.

    Both ``franka_sim/config.yaml`` and ``config/fr3_control.yaml`` use the same
    ``[q_min, q_max, q̇_max, q̈_max, τ_max]`` row layout.
    """
    return np.array([float(limits[k][3]) for k in _JOINT_KEYS], dtype=np.float64)


def joint_limits_mismatch(sim_limits: dict, robot_limits: dict,
                          tol: float = 1e-9) -> List[str]:
    """Report every ``joint_limits`` entry that differs between sim and robot.

    The two YAMLs are documented as mirrors; a silent divergence would rescale
    the policy's actions on hardware (the policy outputs a FRACTION of q̈_max),
    which is exactly the class of sim-to-real bug that is invisible until the
    robot moves.  Returned strings are meant to be logged verbatim.
    """
    out: List[str] = []
    for k in _JOINT_KEYS:
        s = sim_limits.get(k)
        r = robot_limits.get(k)
        if s is None or r is None:
            out.append(f'{k}: missing in {"sim" if s is None else "robot"} config')
            continue
        for i, name in enumerate(('q_min', 'q_max', 'qdot_max', 'qddot_max',
                                  'tau_max')):
            if i >= len(s) or i >= len(r):
                continue
            if abs(float(s[i]) - float(r[i])) > tol:
                out.append(f'{k}.{name}: sim={s[i]} robot={r[i]}')
    return out


def load_yaml(path: str) -> dict:
    with open(path, 'r') as fh:
        return yaml.safe_load(fh) or {}


def find_sim_root(start: str) -> str:
    """Locate the standalone ``franka_sim/`` module from *start*.

    ``franka_sim`` is deliberately NOT a ROS package (it must stay importable
    without ROS for training), so ``get_package_share_directory`` cannot find
    it.  Walk up from *start* — ``realpath`` first, so that under
    ``colcon build --symlink-install`` the installed node file resolves back
    into the source checkout — and return the first ``franka_sim`` directory
    that carries a ``config.yaml``.  Returns ``''`` when not found; callers
    then require an explicit path parameter.
    """
    d = os.path.dirname(os.path.realpath(start))
    while True:
        cand = os.path.join(d, 'franka_sim')
        if os.path.isfile(os.path.join(cand, 'config.yaml')):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            return ''
        d = parent


def resolve_model_path(model: str, sim_root: str = '') -> str:
    """Resolve the ``.onnx`` policy path (absolute, ``sim_root``-relative, cwd).

    Raises ``FileNotFoundError`` listing what was tried — a deployment node
    silently falling back to "no policy" would be worse than not starting.
    """
    tried: List[str] = []
    for cand in (model,
                 os.path.join(sim_root, model) if sim_root else None,
                 os.path.abspath(model)):
        if not cand:
            continue
        tried.append(cand)
        if os.path.isfile(cand):
            return os.path.abspath(cand)
    raise FileNotFoundError(
        'ONNX policy not found. Tried: ' + ', '.join(tried))


def resolve_sim_config_path(explicit: str, model_path: str,
                            sim_root: str = '') -> str:
    """Pick the config that describes how the policy was TRAINED.

    Preference order: explicit parameter → the ``config.yaml`` frozen next to
    the model by ``train.py`` (authoritative: it is the config the run actually
    used) → ``franka_sim/config.yaml``.  Returns ``''`` if nothing is found.
    """
    if explicit:
        return explicit
    frozen = os.path.join(os.path.dirname(os.path.abspath(model_path)),
                          'config.yaml')
    if os.path.isfile(frozen):
        return frozen
    if sim_root:
        default = os.path.join(sim_root, 'config.yaml')
        if os.path.isfile(default):
            return default
    return ''
