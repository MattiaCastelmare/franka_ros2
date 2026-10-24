"""Canonical layout of the policy observation vector.

ONE definition of what the network sees, so the training env and the
deployment node cannot drift apart silently.  Pure numpy: this module must
stay importable WITHOUT mujoco, gymnasium or ROS, because
``franka_experiments/test/test_rl_policy.py`` loads it straight off disk to
check the robot-side mirror against it.

Layout
------
The vector is a strict PREFIX EXTENSION of the 24-dim layout every model up to
``sac_v4`` was trained on — the first 24 entries keep their meaning and their
offsets forever, so an old frozen config keeps producing the old vector::

    [ 0: 7]  q            joint positions
    [ 7:14]  q̇            joint velocities
    [14:17]  ee_pos       end-effector position (world)
    [17:20]  target       target position (world)
    [20:23]  obstacle     obstacle sphere CENTRE (world)
    [23:24]  d_min        smallest control-point surface distance
    ── optional, `obs.obstacle_velocity` ────────────────────────────────────
    [24:27]  v_obs        obstacle centre velocity (world), finite difference
    ── optional, `obs.control_point_geometry` ───────────────────────────────
    [27:31]  d₀, n̂₀       per control point, in `cbf.control_points` ORDER
    [31:35]  d₁, n̂₁
      ...                 4 values × len(control_points)

Why the two optional blocks exist
---------------------------------
**Velocity.** The obstacle moves (sinusoidally in sim, freely in the world) and
the base layout carries only its POSITION.  From a single frame "approaching"
and "receding" are the same observation with opposite optimal actions, so the
task was a POMDP wearing an MDP's clothes.  No RL algorithm recovers from that;
it is a state defect, not a learning one.

**Control-point geometry.** ``FrankaCBFEnv._build_obstacles`` already computes,
per control point, the surface distance dᵢ and the obstacle→robot unit normal
n̂ᵢ — that is exactly what the CBF rows are built from.  The base layout threw
all of it away and kept ``min(dᵢ)``.  The policy was therefore strictly LESS
informed than the filter whose intervention it is supposed to pre-empt: it
could not know which link is threatened, nor which way it is about to be
pushed, which is precisely the information "go around on this side" requires.

It also changes what the observation can DESCRIBE.  ``obstacle(3)`` presumes a
single sphere of a radius fixed at training time; ``(dᵢ, n̂ᵢ)`` per link is
"how far is matter from this link, and in which direction", which is agnostic
to obstacle count and shape and is what the robot's point-cloud pipeline
natively produces.

Distances are CLIPPED at ``clip_distance`` here — in one place, so sim and
robot cannot disagree about what "far away" looks like.  Without it a link with
nothing near it contributes an unbounded number next to features of order
0.2 m, which is the sort of thing that quietly dominates a network's input
scaling.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

NUM_JOINTS: int = 7

#: Fallback control points — ``config.yaml`` `cbf.control_points` is the source
#: of truth.  ``robot_link`` names the link the robot's ``MultiLinkDistance``
#: reports for this body: the MuJoCo body ``fr3_hand`` and the URDF link
#: ``fr3_link8`` are the same frame (both link7 + 0.107 z; the hand is only
#: rotated −45° about z), and the deployment node has to look it up by the name
#: the message carries.
DEFAULT_CONTROL_POINTS: Tuple[dict, ...] = (
    {'body': 'fr3_link3', 'radius': 0.09, 'robot_link': 'fr3_link3'},
    {'body': 'fr3_link4', 'radius': 0.09, 'robot_link': 'fr3_link4'},
    {'body': 'fr3_link5', 'radius': 0.09, 'robot_link': 'fr3_link5'},
    {'body': 'fr3_link6', 'radius': 0.08, 'robot_link': 'fr3_link6'},
    {'body': 'fr3_link7', 'radius': 0.07, 'robot_link': 'fr3_link7'},
    {'body': 'fr3_hand',  'radius': 0.13, 'robot_link': 'fr3_link8'},
)

#: Width of one control-point geometry block: dᵢ + n̂ᵢ.
CP_WIDTH: int = 4


@dataclass(frozen=True)
class ObsSpec:
    """Which optional blocks the observation carries, and how wide it is.

    Frozen on purpose: ``train.py`` writes the config next to the model and
    ``rl_policy_commander`` reads THAT file back, so the spec is a property of
    the trained artifact, never of whatever ``config.yaml`` happens to say
    today.  A policy and its spec travel together or the slots shift under it.
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


def spec_from_config(cfg: Optional[dict]) -> ObsSpec:
    """Build the spec from a loaded ``config.yaml``.

    A config with no ``obs:`` block yields the legacy 24-dim spec, so every
    model frozen before this existed keeps deploying unchanged.
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


def assemble(
    spec: ObsSpec,
    q, qdot, ee_pos, target, obstacle, d_min,
    v_obs=None,
    cp_geometry=None,
    out: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Pack the observation, clipping distances and sanitising non-finites.

    *cp_geometry* is ``(n_cp, 4)`` — ``[dᵢ, n̂ᵢx, n̂ᵢy, n̂ᵢz]`` per row, in
    ``spec.control_points`` order.  A link with nothing reported near it is
    passed as ``(clip_distance, 0, 0, 0)``: a zero normal is a distinguishable
    "no direction known" token, not a direction.

    *out* — a preallocated ``(dim,)`` or ``(1, dim)`` float32 buffer — is
    filled in place, so the 100 Hz control path allocates nothing.
    """
    if out is None:
        out = np.zeros(spec.dim, dtype=np.float32)
    flat = out.reshape(-1)
    if flat.size != spec.dim:
        raise ValueError(f'out must hold {spec.dim} values, got {flat.size}')

    n = spec.n_joints
    flat[0:n] = q
    flat[n:2 * n] = qdot
    flat[2 * n:2 * n + 3] = ee_pos
    flat[2 * n + 3:2 * n + 6] = target
    flat[2 * n + 6:2 * n + 9] = obstacle
    flat[2 * n + 9] = min(float(d_min), spec.clip_distance)
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

    # A NaN anywhere poisons the whole action vector, so it never leaves here.
    np.nan_to_num(flat, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
    return out


def bounds(spec: ObsSpec, q_min, q_max, qdot_max,
           pos_limit: float = 2.0, vel_limit: float = 5.0):
    """``(low, high)`` float32 arrays for a ``gymnasium.spaces.Box``."""
    lo = [np.asarray(q_min, float), -np.asarray(qdot_max, float),
          np.full(3, -pos_limit), np.full(3, -pos_limit),
          np.full(3, -pos_limit), np.array([-1.0])]
    hi = [np.asarray(q_max, float), np.asarray(qdot_max, float),
          np.full(3, pos_limit), np.full(3, pos_limit),
          np.full(3, pos_limit), np.array([spec.clip_distance])]
    if spec.obstacle_velocity:
        lo.append(np.full(3, -vel_limit))
        hi.append(np.full(3, vel_limit))
    if spec.n_cp:
        # Per control point: d ∈ [−1, clip], n̂ ∈ [−1, 1]³.
        lo.append(np.tile([-1.0, -1.0, -1.0, -1.0], spec.n_cp))
        hi.append(np.tile([spec.clip_distance, 1.0, 1.0, 1.0], spec.n_cp))
    return (np.concatenate(lo).astype(np.float32),
            np.concatenate(hi).astype(np.float32))
