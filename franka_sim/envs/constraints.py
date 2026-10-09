"""Constraint monitor: what the robot would refuse, measured instead of prevented.

With ``env.shield: false`` nothing stands between the policy and the torque
chain, so the limits the shield used to enforce become things to MEASURE. This
module reads the state after every control tick and reports, per family:

  pos        q outside the EFFECTIVE position limits (config ∩ libfranka q_ref,
             the same limits cbf_filter anchors its box to)
  vel        q̇ outside the firmware velocity envelope at margin 1.0 — the exact
             condition of the joint_velocity_violation reflex
  acc        commanded |q̈| above the acceleration box the robot's QP clips to
             (joint_limits ∩ cbf.qddot_max_abs)
  slew       |q̈ − q̈_prev| above cbf.max_qddot_delta per tick (the robot QP's
             continuity bound, i.e. a jerk limit)
  torque     inverse-dynamics torque that had to be clipped to the joint limit
  self_coll  two self-collision capsules closer than
             constraints.self_collision_margin, or two robot meshes in contact
  sing       σ_min of the 6×7 EE Jacobian (rotation rows × rot_scale, as
             cbf_safety_filter on the robot) below constraints.sigma_floor
  floor      any robot geom touching the floor
  workspace  EE outside the cbf.ws_min/ws_max box

Self-collision uses the OFFICIAL franka_description capsules, the geometry the
robot's own self-collision rows use (assets/franka_fr3/
self_collision_capsules.yaml, written by scripts/extract_sc_capsules.py), with
the robot's pair rules: chain-adjacent links and the hand vs link6/link7 are
never checked, nor the pairs franka's SRDF marks "Never"
(fr3_control.yaml: self_collision_exclude_pairs). MuJoCo's mesh distance
(mj_geomDistance) was tried first and rejected: on these meshes it returned a
false 0 about 70 times per episode, with closest points several cm apart.

It only READS MuJoCo data, so a rollout is bit-identical with it on or off;
``test_policy_alone`` checks that.
"""
from __future__ import annotations

import os

import mujoco
import numpy as np
import yaml

from .cbf_filter import fr3_velocity_envelope

FAMILIES = ('pos', 'vel', 'acc', 'slew', 'torque', 'self_coll', 'sing', 'floor',
            'workspace')

_CAPSULES = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'assets',
                         'franka_fr3', 'self_collision_capsules.yaml')

# Copied from fr3_control.yaml (self_collision_exclude_pairs, "SRDF: Never").
ROBOT_EXCLUDE_PAIRS = (
    'link0-link2', 'link0-link3', 'link0-link4', 'link1-link3', 'link1-link4',
    'link2-link4', 'link2-link6', 'link3-link5', 'link3-link6', 'link3-link7',
    'link4-link6', 'link4-link7', 'link5-link7', 'hand-link3', 'hand-link4',
    'hand-link6')


def _chain_index(body):
    if body.endswith('hand'):
        return 8
    for i in range(8):
        if body.endswith(f'link{i}'):
            return i
    return None


def _adjacent(ia, ib):
    """Mirror of self_collision._adjacent: one joint apart, or hand vs link6/7."""
    lo, hi = min(ia, ib), max(ia, ib)
    return hi - lo <= 1 or (hi == 8 and lo >= 6)


def capsule_pairs(bodies, exclude=ROBOT_EXCLUDE_PAIRS):
    """Index pairs to check, by the robot's rules (self_collision.build_capsule_pairs)."""
    excl = [tuple(s.split('-')) for s in exclude]
    out = []
    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            a, b = bodies[i], bodies[j]
            ia, ib = _chain_index(a), _chain_index(b)
            if ia is None or ib is None or a == b or _adjacent(ia, ib):
                continue
            if any((a.endswith(x) and b.endswith(y)) or (a.endswith(y) and b.endswith(x))
                   for x, y in excl):
                continue
            out.append((i, j))
    return out


def segment_distance(p1, q1, p2, q2, eps=1e-12):
    """Closest distance between segments [p1,q1] and [p2,q2] (Ericson §5.1.9),
    the same closed form as franka_experiments' segment_segment_closest."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if a <= eps and e <= eps:
        s = t = 0.0
    elif a <= eps:
        s, t = 0.0, min(1.0, max(0.0, f / e))
    else:
        c = float(d1 @ r)
        if e <= eps:
            t, s = 0.0, min(1.0, max(0.0, -c / a))
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, min(1.0, max(0.0, (b - c) / a))
    return float(np.linalg.norm((p1 + s * d1) - (p2 + t * d2)))


class ConstraintMonitor:
    def __init__(self, env, cfg: dict):
        cfg = cfg or {}
        m = env.model
        self.env = env
        self.margin = float(cfg.get('self_collision_margin', 0.0))
        self.sigma_floor = float(cfg.get('sigma_floor', 0.05))
        self.rot_scale = float(cfg.get('sigma_rot_scale', 0.3))
        self.terminate = bool(cfg.get('terminate', False))
        # Which families end the episode when terminate is on (default: all).
        self.terminate_on = tuple(cfg.get('terminate_on', FAMILIES))
        cbf = env.cbf
        self.q_lo, self.q_hi = cbf.q_min.copy(), cbf.q_max.copy()
        self.acc_box = cbf.qddot_box.copy()
        self.slew = float(cbf.slew_delta)
        self.ws_min, self.ws_max = cbf.ws_min.copy(), cbf.ws_max.copy()

        caps = yaml.safe_load(open(cfg.get('capsules_file') or _CAPSULES))['capsules']
        self.cap_body = [c['body'] for c in caps]
        self.cap_bid = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
                                 for b in self.cap_body])
        self.cap_p1 = np.array([c['p1'] for c in caps], float)
        self.cap_p2 = np.array([c['p2'] for c in caps], float)
        self.cap_r = np.array([c['radius'] for c in caps], float)
        self.pairs = capsule_pairs(self.cap_body, cfg.get('exclude_pairs', ROBOT_EXCLUDE_PAIRS))

        # Physical contact between robot meshes (MuJoCo's own contact list),
        # for the same body pairs.
        names = set()
        for i, j in self.pairs:
            names.add(frozenset((self.cap_body[i], self.cap_body[j])))
        self._contact_pairs = names
        self._geom_body = {g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g]))
                           for g in range(m.ngeom)}
        self._jp = np.zeros((3, m.nv))
        self._jr = np.zeros((3, m.nv))

    def sigma_min(self):
        e = self.env
        mujoco.mj_jacSite(e.model, e.data, self._jp, self._jr, e._ee_site)
        J = np.vstack([self._jp[:, e._dadr], self.rot_scale * self._jr[:, e._dadr]])
        return float(np.linalg.svd(J, compute_uv=False)[-1])

    def self_gap(self):
        """(smallest capsule surface gap [m], 'bodyA|bodyB')."""
        d = self.env.data
        R = d.xmat[self.cap_bid].reshape(-1, 3, 3)
        x = d.xpos[self.cap_bid]
        p1 = x + np.einsum('kij,kj->ki', R, self.cap_p1)
        p2 = x + np.einsum('kij,kj->ki', R, self.cap_p2)
        best, label = 1.0, ''
        for i, j in self.pairs:
            g = segment_distance(p1[i], p2[i], p1[j], p2[j]) - self.cap_r[i] - self.cap_r[j]
            if g < best:
                best, label = g, f'{self.cap_body[i]}|{self.cap_body[j]}'
        return float(best), label

    def robot_contacts(self):
        """'bodyA|bodyB' for checked body pairs whose meshes MuJoCo has in contact."""
        d = self.env.data
        out = []
        for k in range(d.ncon):
            c = d.contact[k]
            a, b = self._geom_body[c.geom1], self._geom_body[c.geom2]
            if frozenset((a, b)) in self._contact_pairs:
                out.append(f'{a}|{b}')
        return out

    def measure(self, q, qdot, qddot_cmd, qddot_prev, tau_excess, ee):
        """One tick → ({family: excess ≥ 0}, diagnostics). Excess 0 = satisfied."""
        up, lo = fr3_velocity_envelope(q, margin=1.0)
        gap, pair = self.self_gap()
        touching = self.robot_contacts()
        sig = self.sigma_min()
        ex = {
            'pos': float(max(0.0, np.max(self.q_lo - q), np.max(q - self.q_hi))),
            'vel': float(max(0.0, np.max(qdot - up), np.max(lo - qdot))),
            'acc': float(max(0.0, np.max(np.abs(qddot_cmd) - self.acc_box))),
            'slew': float(max(0.0, np.max(np.abs(qddot_cmd - qddot_prev)) - self.slew)),
            'torque': float(tau_excess),
            'self_coll': float(max(0.0, self.margin - gap, 1e-6 if touching else 0.0)),
            'sing': float(max(0.0, self.sigma_floor - sig)),
            'floor': float(self.env._floor_contact()),
            'workspace': float(max(0.0, np.max(self.ws_min - ee), np.max(ee - self.ws_max))),
        }
        return ex, {'sigma_min': sig, 'self_gap': gap, 'self_pair': pair,
                    'self_contact': touching[0] if touching else ''}

    def violated(self, ex):
        return [k for k in FAMILIES if ex[k] > 0.0]

    def should_terminate(self, ex):
        return self.terminate and any(ex[k] > 0.0 for k in self.terminate_on)
