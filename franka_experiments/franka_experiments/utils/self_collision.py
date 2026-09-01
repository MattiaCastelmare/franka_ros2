"""Self-collision geometry from the OFFICIAL franka_description capsules.

franka_description models the robot's *internal* self-collision geometry as
capsules (cylinder + 2 sphere caps) attached to dedicated massless links named
``<link>_sc`` (see ``robots/common/utils.xacro``, macros ``link_with_sc`` /
``collision_capsule``).  They are emitted only when the xacro arg
``with_sc:=true`` is passed, and every radius already includes the xacro
``safety_distance`` margin (default 0.03 m).

This module parses those capsules straight out of the generated URDF — no
hand-tuned geometry — and provides the segment/segment distance math used by
the CBF self-collision rows.  Everything here is pure numpy + stdlib (no
Pinocchio, no ROS) so it is unit-testable without a robot environment.

Conventions
-----------
* A capsule is stored as a *segment* (two endpoints, in the local frame of its
  ``*_sc`` URDF link) plus a radius.  The ``*_sc`` links survive the Pinocchio
  URDF import as fixed frames, so world-frame endpoints are obtained with the
  frame placement ``oMf`` of that frame.
* Surface distance between two capsules = segment/segment distance − r_A − r_B.
* ``body`` is the physical link the capsule belongs to (``fr3_link5`` for the
  ``fr3_link5_sc`` frame); it drives the adjacency logic for pair selection.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import NamedTuple

import numpy as np


class Capsule(NamedTuple):
    frame:  str          # URDF *_sc link name (== Pinocchio frame name)
    body:   str          # physical link name (frame minus the trailing '_sc')
    p1:     np.ndarray   # (3,) segment endpoint, LOCAL *_sc frame
    p2:     np.ndarray   # (3,) segment endpoint, LOCAL *_sc frame
    radius: float        # capsule radius [m] (includes xacro safety_distance)


# ── URDF parsing ─────────────────────────────────────────────────────────────

def rpy_to_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF fixed-axis XYZ convention: R = Rz(y) @ Ry(p) @ Rx(r)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _origin_of(elem) -> tuple[np.ndarray, np.ndarray]:
    """Return (xyz, rpy) of an element's <origin> child (defaults to zeros)."""
    o = elem.find('origin')
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if o is not None:
        if o.get('xyz'):
            xyz = np.array([float(v) for v in o.get('xyz').split()])
        if o.get('rpy'):
            rpy = np.array([float(v) for v in o.get('rpy').split()])
    return xyz, rpy


def parse_sc_capsules(urdf_xml: str) -> list[Capsule]:
    """Extract the official self-collision capsules from a URDF string.

    Scans every link whose name ends in ``_sc`` and turns each
    ``<collision><geometry><cylinder>`` into a :class:`Capsule` (the two
    companion ``<sphere>`` collisions emitted by the ``collision_capsule``
    macro are the caps of the same capsule — the cylinder alone carries all
    the information, so spheres are deliberately ignored).

    The cylinder axis is its local z axis rotated by the collision origin rpy;
    endpoints = origin ± (length/2)·axis, expressed in the *_sc link frame.
    """
    root = ET.fromstring(urdf_xml)
    caps: list[Capsule] = []
    for link in root.iter('link'):
        name = link.get('name', '')
        if not name.endswith('_sc'):
            continue
        body = name[: -len('_sc')]
        for coll in link.findall('collision'):
            cyl = coll.find('geometry/cylinder')
            if cyl is None:
                continue
            radius = float(cyl.get('radius'))
            length = float(cyl.get('length'))
            xyz, rpy = _origin_of(coll)
            axis = rpy_to_matrix(*rpy) @ np.array([0.0, 0.0, 1.0])
            half = 0.5 * length * axis
            caps.append(Capsule(frame=name, body=body,
                                p1=xyz - half, p2=xyz + half, radius=radius))
    return caps


# ── Pair selection ───────────────────────────────────────────────────────────

def body_chain_index(body: str) -> int | None:
    """Map a physical link name to its position along the kinematic chain.

    ``*link0..7`` → 0..7; the hand (``*hand``) → 8 (bolted to link7).
    Returns None for unknown bodies (their capsules are skipped in pairing).
    """
    if body.endswith('hand'):
        return 8
    for i in range(8):
        if body.endswith(f'link{i}'):
            return i
    return None


def _adjacent(ia: int, ib: int) -> bool:
    """Chain-adjacent bodies cannot collide (single joint between them).

    The hand (8) is rigidly bolted to link7 and separated from link6 only by
    the joint7 z-rotation — both pairs are geometrically collision-free, so
    the hand is treated as adjacent to BOTH link7 and link6.
    """
    lo, hi = min(ia, ib), max(ia, ib)
    if hi - lo <= 1:
        return True
    if hi == 8 and lo >= 6:          # hand vs link6 / link7
        return True
    return False


def build_capsule_pairs(
    capsules: list[Capsule],
    exclude_bodies: list[str] | None = None,
) -> list[tuple[int, int]]:
    """All capsule index pairs whose bodies are non-adjacent on the chain.

    ``exclude_bodies`` is a list of ``"bodyA-bodyB"`` strings (matched on the
    trailing part of the body names, order-insensitive) for pairs the user
    wants dropped — e.g. pairs that sit permanently close in every legal
    configuration and would only add noise.
    """
    excl: set[frozenset] = set()
    for s in exclude_bodies or []:
        parts = s.replace(' ', '').split('-')
        if len(parts) == 2:
            excl.add(frozenset(parts))

    def _excluded(ba: str, bb: str) -> bool:
        for pair in excl:
            pa, pb = tuple(pair) if len(pair) == 2 else (next(iter(pair)),) * 2
            if (ba.endswith(pa) and bb.endswith(pb)) or \
               (ba.endswith(pb) and bb.endswith(pa)):
                return True
        return False

    pairs: list[tuple[int, int]] = []
    for i in range(len(capsules)):
        ci = body_chain_index(capsules[i].body)
        if ci is None:
            continue
        for j in range(i + 1, len(capsules)):
            cj = body_chain_index(capsules[j].body)
            if cj is None or capsules[i].body == capsules[j].body:
                continue
            if _adjacent(ci, cj):
                continue
            if _excluded(capsules[i].body, capsules[j].body):
                continue
            pairs.append((i, j))
    return pairs


# ── Segment/segment distance ─────────────────────────────────────────────────

def segment_segment_closest(
    p1: np.ndarray, q1: np.ndarray,
    p2: np.ndarray, q2: np.ndarray,
    eps: float = 1e-12,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Closest points between segments [p1,q1] and [p2,q2] (world frame).

    Returns ``(pa, pb, dist)`` with ``pa`` on segment 1, ``pb`` on segment 2.
    Robust to degenerate (point-like) and parallel segments — the standard
    clamped closed-form solution (Ericson, *Real-Time Collision Detection*,
    §5.1.9).
    """
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(d1 @ d1)
    e = float(d2 @ d2)
    f = float(d2 @ r)

    if a <= eps and e <= eps:              # both segments are points
        s = t = 0.0
    elif a <= eps:                          # first segment is a point
        s = 0.0
        t = min(1.0, max(0.0, f / e))
    else:
        c = float(d1 @ r)
        if e <= eps:                        # second segment is a point
            t = 0.0
            s = min(1.0, max(0.0, -c / a))
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            # den == 0 → parallel: pick s=0, then clamp t (any s works)
            s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = min(1.0, max(0.0, (b - c) / a))

    pa = p1 + s * d1
    pb = p2 + t * d2
    return pa, pb, float(np.linalg.norm(pa - pb))
