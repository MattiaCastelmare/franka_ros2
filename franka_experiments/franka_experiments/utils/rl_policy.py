"""Sim-to-real contract for the ONNX Safe-RL policy (``rl_policy_commander``).

Everything here is plain numpy / YAML so it can be unit-tested without a ROS
environment, and — more importantly — so the **observation contract** between
training and deployment lives in ONE place.

``franka_sim/envs/franka_cbf_env.py`` builds, every 100 Hz control tick::

    obs(24) = [ q(7), q̇(7), ee_pos(3), target(3), obstacle(3), d_min(1) ]
    + v_obs(3)                        if `obs.obstacle_velocity`
    + [dᵢ, n̂ᵢ] × n_cp                 if `obs.control_point_geometry`
    action(7) ∈ [−1, 1]   →   q̈_nom = action · q̈_max   →   CBF filter

:func:`build_observation` rebuilds exactly that vector from robot topics and
:func:`action_to_qddot` applies exactly that scaling, so a policy trained in
``franka_sim`` sees the same numbers on hardware.

The optional blocks are PREFIX EXTENSIONS: slots 0..23 keep their meaning and
their offsets, so a model frozen before they existed deploys unchanged.  Which
blocks a given policy wants is a property of the POLICY, not of this file:
``train.py`` freezes ``config.yaml`` next to the model and
:func:`resolve_sim_config_path` reads that copy back, so
:class:`ObsSpec` travels with the artifact.

:class:`ObsSpec` mirrors ``franka_sim/envs/obs_layout.py``.  ``franka_sim`` is
deliberately not a ROS package (it must stay importable without ROS, for
training), so the node cannot import it at runtime and the layout is written
out twice on purpose — exactly as the ``cbf:`` / ``joint_limits:`` config
blocks are.  ``test_observation_layout_mirrors_franka_sim`` loads the sim
module straight off disk and fails the suite on any drift.

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

import glob
import os
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from .constants import NUM_JOINTS

OBS_DIM: int = 2 * NUM_JOINTS + 10
"""Legacy observation width: q(7)+q̇(7)+ee(3)+target(3)+obstacle(3)+d_min(1).

Kept as a module constant because it is the width of every model up to
``sac_v4`` and the base of every longer layout.  For anything that depends on
what a SPECIFIC policy was trained with, use ``ObsSpec.dim``.
"""

CP_WIDTH: int = 4
"""Width of one control-point geometry block: dᵢ + n̂ᵢ."""

#: Fallback control points — mirror of
#: ``franka_sim.envs.obs_layout.DEFAULT_CONTROL_POINTS``.  Only reached by a
#: config that asks for the geometry block without listing `cbf.control_points`;
#: both sides must then invent the SAME list or the widths disagree.
DEFAULT_CONTROL_POINTS: Tuple[dict, ...] = (
    {'body': 'fr3_link3', 'radius': 0.09, 'robot_link': 'fr3_link3'},
    {'body': 'fr3_link4', 'radius': 0.09, 'robot_link': 'fr3_link4'},
    {'body': 'fr3_link5', 'radius': 0.09, 'robot_link': 'fr3_link5'},
    {'body': 'fr3_link6', 'radius': 0.08, 'robot_link': 'fr3_link6'},
    {'body': 'fr3_link7', 'radius': 0.07, 'robot_link': 'fr3_link7'},
    {'body': 'fr3_hand',  'radius': 0.13, 'robot_link': 'fr3_link8'},
)


@dataclass(frozen=True)
class ObsSpec:
    """Which optional observation blocks a policy carries — see module docstring.

    Mirror of ``franka_sim.envs.obs_layout.ObsSpec``.
    """

    n_joints: int = NUM_JOINTS
    obstacle_velocity: bool = False
    #: Robot link names, in `cbf.control_points` order.  Empty = block absent.
    control_points: Tuple[str, ...] = ()
    clip_distance: float = 1.2

    @property
    def n_cp(self) -> int:
        return len(self.control_points)

    @property
    def dim(self) -> int:
        return (2 * self.n_joints + 10
                + (3 if self.obstacle_velocity else 0)
                + CP_WIDTH * self.n_cp)

    @property
    def slots(self) -> List[Tuple[str, int, int]]:
        """``[(name, start, width), …]`` — the layout, in order."""
        n = self.n_joints
        out: List[Tuple[str, int, int]] = [
            ('q', 0, n), ('qdot', n, n), ('ee_pos', 2 * n, 3),
            ('target', 2 * n + 3, 3), ('obstacle', 2 * n + 6, 3),
            ('d_min', 2 * n + 9, 1),
        ]
        i = 2 * n + 10
        if self.obstacle_velocity:
            out.append(('v_obs', i, 3))
            i += 3
        for name in self.control_points:
            out.append((f'cp:{name}', i, CP_WIDTH))
            i += CP_WIDTH
        return out

    def describe(self) -> str:
        """One-line layout summary — log it wherever a policy is loaded."""
        return (f'obs({self.dim}) = ' +
                ' + '.join(f'{k}({w})' for k, _, w in self.slots))


#: The 24-dim layout every model up to ``sac_v4`` was trained on, and what a
#: config carrying no ``obs:`` block resolves to.
LEGACY_OBS_SPEC = ObsSpec()


def obs_spec_from_config(cfg: Optional[dict]) -> ObsSpec:
    """Training ``config.yaml`` → :class:`ObsSpec` (mirror of the sim helper).

    Pass the config frozen NEXT TO THE MODEL, not the repository default: the
    layout is a property of the artifact about to drive the arm.
    """
    cfg = cfg or {}
    o = cfg.get('obs') or {}
    cbf = cfg.get('cbf') or {}
    cps: Tuple[str, ...] = ()
    if bool(o.get('control_point_geometry', False)):
        entries = cbf.get('control_points') or DEFAULT_CONTROL_POINTS
        cps = tuple(str(c.get('robot_link', c['body'])) for c in entries)
    clip = o.get('clip_distance')
    clip = (float(clip) if clip is not None
            else float(cbf.get('cbf_obstacle_horizon', 1.2)))
    return ObsSpec(obstacle_velocity=bool(o.get('obstacle_velocity', False)),
                   control_points=cps, clip_distance=clip)

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
    v_obs: Optional[np.ndarray] = None,
    cp_geometry: Optional[np.ndarray] = None,
    spec: ObsSpec = LEGACY_OBS_SPEC,
    out: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Assemble the observation exactly as ``FrankaCBFEnv._get_obs``.

    *spec* defaults to the legacy 24-dim layout, so existing callers and every
    model up to ``sac_v4`` are unaffected.

    *cp_geometry* is ``(n_cp, 4)`` — ``[dᵢ, n̂ᵢ]`` per row, in
    ``spec.control_points`` order, as :func:`control_point_geometry` builds it.

    *out* — when given a preallocated ``(spec.dim,)`` or ``(1, spec.dim)``
    float32 buffer — is filled in place and returned, so the control loop
    allocates nothing per tick.  Non-finite entries are zeroed (a NaN reaching
    the network would poison the whole action vector).
    """
    if out is None:
        out = np.zeros(spec.dim, dtype=np.float32)
    flat = out.reshape(-1)
    if flat.size != spec.dim:
        raise ValueError(f'out must hold {spec.dim} values, got {flat.size}')

    n = spec.n_joints
    flat[0:n]         = q
    flat[n:2 * n]     = qdot
    flat[2 * n:2 * n + 3] = ee_pos
    flat[2 * n + 3:2 * n + 6] = target
    flat[2 * n + 6:2 * n + 9] = obstacle
    flat[2 * n + 9]   = min(float(d_min), spec.clip_distance)
    i = 2 * n + 10

    if spec.obstacle_velocity:
        if v_obs is None:
            raise ValueError('spec requires obstacle_velocity but v_obs is None')
        flat[i:i + 3] = v_obs
        i += 3

    if spec.n_cp:
        if cp_geometry is None:
            raise ValueError('spec requires control-point geometry but '
                             'cp_geometry is None')
        g = np.asarray(cp_geometry, dtype=np.float64).reshape(-1, CP_WIDTH)
        if g.shape[0] != spec.n_cp:
            raise ValueError(f'cp_geometry must have {spec.n_cp} rows, '
                             f'got {g.shape[0]}')
        g = g.copy()
        g[:, 0] = np.minimum(g[:, 0], spec.clip_distance)
        flat[i:i + CP_WIDTH * spec.n_cp] = g.reshape(-1)

    np.nan_to_num(flat, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
    return out


def control_point_geometry(
    entries: Iterable[Tuple[str, float, np.ndarray, np.ndarray]],
    spec: ObsSpec,
    out: Optional[np.ndarray] = None,
) -> np.ndarray:
    """``MultiLinkDistance`` entries → ``(n_cp, 4)`` ``[dᵢ, n̂ᵢ]`` block.

    *entries* are ``(link_name, d, n̂, p_human)`` as decoded by
    ``rl_policy_commander._obs_cb``.  The robot reports SEVERAL control points
    per link (``fr3_complete.yaml`` samples 11 points along the segment axes)
    while the sim carries one sphere at the body origin, so the NEAREST entry
    per link is the one that maps onto the sim's row — and it is the
    conservative pick either way.

    A link with nothing reported near it gets ``(clip_distance, 0, 0, 0)``: a
    zero normal is a distinguishable "no direction known" token, never a
    direction.  That is also what the sim emits for a degenerate row, so the
    two agree on the absence of information as well as on its presence.
    """
    if out is None:
        out = np.zeros((spec.n_cp, CP_WIDTH), dtype=np.float64)
    if out.shape != (spec.n_cp, CP_WIDTH):
        raise ValueError(f'out must be {(spec.n_cp, CP_WIDTH)}, got {out.shape}')
    out[:, 0] = spec.clip_distance
    out[:, 1:] = 0.0
    if not spec.n_cp:
        return out

    index = {name: i for i, name in enumerate(spec.control_points)}
    best: List[Optional[float]] = [None] * spec.n_cp
    for name, d, n_hat, _p_human in entries:
        i = index.get(name)
        if i is None or not np.isfinite(d):
            continue
        if best[i] is None or d < best[i]:
            best[i] = float(d)
            out[i, 0] = float(d)
            out[i, 1:] = n_hat
    return out


def obstacle_velocity(centre: np.ndarray, prev_centre: Optional[np.ndarray],
                      dt: float) -> np.ndarray:
    """Finite-difference obstacle velocity, mirroring ``_get_obs``.

    Deliberately a plain difference of the ESTIMATED centres with no extra
    smoothing: the sim differentiates its post-randomizer position and nothing
    else, so any filtering added here would be a block the policy never trained
    against.  The engine's own LPF is already upstream of both.

    Returns zeros on the first tick and on a non-positive *dt* — an unusable
    timestamp must not become an unbounded velocity.
    """
    if prev_centre is None or not np.isfinite(dt) or dt <= 0.0:
        return np.zeros(3)
    v = (np.asarray(centre, dtype=np.float64)
         - np.asarray(prev_centre, dtype=np.float64)) / float(dt)
    return np.nan_to_num(v, nan=0.0, posinf=0.0, neginf=0.0)


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


def find_latest_model(sim_root: str) -> str:
    """Newest exported ``.onnx`` under ``<sim_root>/models``, or ``''``.

    Backs the documented ``rl_onnx_model:=""`` default ("empty = newest model
    under franka_sim/models", ``config/launch_defaults.yaml``).  Ordered by
    MTIME, not by filename, for the same reason
    :func:`~franka_sim.scripts.evaluate_policy.latest_checkpoint` is:
    ``best_model.onnx`` is rewritten whenever eval improves, so "the newest
    file" and "the highest episode number" are different questions, and the
    newest file is the one that was just trained.

    Only ``.onnx`` is considered.  A ``.zip`` is a stable-baselines3 training
    artifact that ``rl_policy_commander`` cannot load at all (the robot carries
    onnxruntime, not torch), so silently selecting one would trade a clear
    "nothing to run" error for an obscure load failure.

    The CALLER must log the path it gets back.  Choosing an artifact implicitly
    is a convenience for tests and demos; on hardware the operator has to be
    able to read back which policy is about to move the arm.
    """
    if not sim_root:
        return ''
    models = os.path.join(sim_root, 'models')
    if not os.path.isdir(models):
        return ''
    hits = glob.glob(os.path.join(models, '**', '*.onnx'), recursive=True)
    if not hits:
        return ''
    return os.path.abspath(max(hits, key=os.path.getmtime))


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
