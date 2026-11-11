#!/usr/bin/env python3
"""FR3 geometry, duplicate control points, so3_log and the URDF hand (preserved from the Stage 2
`check2.py` and `home.py`, plus one new quantification marked NEW).

Verifies:
  - the home-pose frame origins table of 04 §3.1 and the segment lengths, constant over 200 random
    q (seed 1), including the zero-length segments 1 and 5                         (04 §3.1)
  - fr3_link6#0 == fr3_link6#1 for every q (F-2-1)                                 (04 §4.2)
  - utils.kinematics.so3_log against pin.log3 (4.11)                                 (04 §4.6)
  - in the with-hand URDF, hand.stl is the collision mesh of link fr3_hand, not fr3_link8
    (basis of F-2-9)                                                                  (05 §4.2)
  - the forearm sphere gap of F-2-4 (Derived formula, evaluated with the measured segment length)
  - NEW (not in the Stage 2 run): at the home pose, how far the correctly placed hand collision
    mesh reaches beyond the 11-sphere model (F-2-4, previously an Inferred magnitude) (05 §3.2)

Needs: the ROS 2 environment (xacro, franka_description), Pinocchio, numpy, PyYAML. Run inside the
`franka_ros2` container (README.md).
"""
import os
import re
import sys

sys.dont_write_bytecode = True   # read-only: never write __pycache__ into the repository
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common as C  # noqa: E402

C.use_source_package()
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
from franka_experiments.utils.kinematics import (  # noqa: E402
    CBFKinematics, build_urdf_no_hand, generate_urdf_from_xacro, so3_log)

R = C.Report('check_geometry_and_so3 (chapters 04 and 05)')
cfg = C.load_yaml(os.path.join(C.CONFIG, 'fr3_complete.yaml'))
p = build_urdf_no_hand()
try:
    m = pin.buildModelFromUrdf(p)
finally:
    os.unlink(p)
kin = CBFKinematics(m)
kin.configure_control_points(cfg['robot'], cfg['distance'])

# ── home-pose frame origins (04 §3.1 table) ────────────────────────────────────────────────
kin.update(C.Q_HOME, np.zeros(7))
home_table = {
    'fr3_link0': (0, 0, 0), 'fr3_link1': (0, 0, 0.333), 'fr3_link2': (0, 0, 0.333),
    'fr3_link3': (-0.2234, 0, 0.5564), 'fr3_link4': (-0.1651, 0, 0.6148),
    'fr3_link5': (0.2189, 0, 0.6973), 'fr3_link6': (0.2189, 0, 0.6973),
    'fr3_link7': (0.3069, 0, 0.6973), 'fr3_link8': (0.3069, 0, 0.5903)}
for name, ref in home_table.items():
    R.close(f'home origin {name} [m]', kin.data.oMf[m.getFrameId(name)].translation, ref, 6e-5)

# ── segment lengths over 200 random q and the duplicate control points (seed 1, as originally) ─
rng = np.random.default_rng(1)
lo, hi = m.lowerPositionLimit, m.upperPositionLimit
chain = ['fr3_link%d' % i for i in range(9)]
lens = {i: [] for i in range(8)}
seg_lens = {s['seg_idx']: [] for s in cfg['robot']['segments']}
dup = 0.0
for _ in range(200):
    q = lo + (hi - lo) * rng.random(7)
    kin.update(q, np.zeros(7), with_jdot=False)
    o = [kin.data.oMf[m.getFrameId(n)].translation for n in chain]
    for i in range(8):
        lens[i].append(np.linalg.norm(o[i + 1] - o[i]))
    for s in cfg['robot']['segments']:
        a = kin.data.oMf[m.getFrameId(s['start_link'])].translation
        b = kin.data.oMf[m.getFrameId(s['end_link'])].translation
        seg_lens[s['seg_idx']].append(np.linalg.norm(b - a))
    dup = max(dup, np.linalg.norm(kin.control_point_position('fr3_link6#0')
                                  - kin.control_point_position('fr3_link6#1')))
reported_next = [0.333, 0.0, 0.316, 0.0825, 0.39276, 0.0, 0.088, 0.107]
for i in range(8):
    span = max(lens[i]) - min(lens[i])
    R.close(f'distance {chain[i]} -> {chain[i + 1]} [m] (constant over q: spread {span:.1e})',
            np.mean(lens[i]), reported_next[i], 6e-5)
    R.below(f'  spread of that distance over 200 q [m]', span, 1e-9, kind='FACT')
for k, v in seg_lens.items():
    R.info(f'configured segment {k}: length min {min(v):.5f} max {max(v):.5f} m')
R.below('F-2-1: |fr3_link6#0 - fr3_link6#1| over 200 q [m]', dup, 1e-12, kind='FACT',
        note='reported at most 2e-16')

# ── so3_log vs pin.log3 (4.11) ─────────────────────────────────────────────────────────────
err = 0.0
for _ in range(2000):
    w = rng.normal(size=3)
    w = w / np.linalg.norm(w) * rng.uniform(0, np.pi)
    Rm = pin.exp3(w)
    err = max(err, np.abs(so3_log(Rm) - pin.log3(Rm)).max())
errpi = 0.0
for ang in [np.pi - 1e-7, np.pi - 1e-5, np.pi - 1e-3, 1e-7, 1e-5]:
    w = rng.normal(size=3)
    w = w / np.linalg.norm(w) * ang
    Rm = pin.exp3(w)
    e_ = np.abs(so3_log(Rm) - w).max()
    if ang > 3:   # at pi the sign of the axis is ambiguous; accept either
        e_ = min(e_, np.abs(so3_log(Rm) + w).max())
    errpi = max(errpi, e_)
R.below('so3_log vs pin.log3, 2000 random rotations', err, 1e-8, note='reported 1.4e-10')
R.below('so3_log near 0 and pi (sign at pi allowed)', errpi, 1e-5, note='reported <= 2.6e-6')

# ── hand collision mesh owner in the with-hand URDF ────────────────────────────────────────
xml = generate_urdf_from_xacro()


def collisions(link):
    mm = re.search(r'<link name="%s">(.*?)</link>' % link, xml, re.S)
    return re.findall(r'<collision.*?</collision>', mm.group(1), re.S) if mm else []


R.fact('URDF: hand.stl is a collision mesh of fr3_hand', any('hand.stl' in c for c in collisions('fr3_hand')))
R.fact('URDF: fr3_link8 has no hand.stl collision', not any('hand.stl' in c for c in collisions('fr3_link8')),
       f'{len(collisions("fr3_link8"))} collision element(s) on fr3_link8')
mj = re.search(r'<joint name="fr3_hand_joint".*?</joint>', xml, re.S)
rpy = re.search(r'rpy="([^"]+)"', mj.group(0)).group(1) if mj else ''
R.close('URDF: fr3_hand_joint rpy [rad]', [float(x) for x in rpy.split()] if rpy else [np.nan] * 3,
        [0, 0, -np.pi / 4], 1e-6, kind='FACT')

# ── F-2-4 (a): forearm gap between spheres, Derived formula ────────────────────────────────
L = float(np.mean(seg_lens[[s['seg_idx'] for s in cfg['robot']['segments']
                            if s['start_link'] == 'fr3_link4'][0]]))
r = 0.05
half = L / 3 / 2
gap = np.hypot(half, r) - r
R.close('F-2-4 forearm: point on the 5 cm tube midway between sphere centres, gap to the nearest '
        'sphere [m]', gap, 0.032, 6e-4, note=f'L = {L:.5f} m, centres at 1/3 and 2/3')

# ── F-2-4 (b) NEW: correctly placed hand mesh vs the 11 spheres at home ────────────────────
kin.update(C.Q_HOME, np.zeros(7))
cps = np.array([kin.control_point_position(lb) for lb in kin._control_points])
radii = 0.05
o8 = kin.data.oMf[m.getFrameId('fr3_link8')]
c, s = np.cos(-np.pi / 4), np.sin(-np.pi / 4)
Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
verts = C.read_stl_triangles(C.HAND_STL).reshape(-1, 3)
world = (o8.rotation @ (Rz @ verts.T)).T + o8.translation       # URDF placement (fr3_hand)
excess = np.min(np.linalg.norm(world[:, None, :] - cps[None, :, :], axis=2), axis=1) - radii
R.info(f'NEW F-2-4: hand vertices outside every control-point sphere: {np.mean(excess > 0) * 100:.0f} %; '
       f'largest distance of a hand vertex from the sphere model {excess.max():.4f} m '
       f'(home pose, URDF-correct hand placement)')
R.fact('NEW F-2-4: some hand surface lies outside the sphere model', excess.max() > 0,
       f'max excess {excess.max():.4f} m')
sys.exit(R.finish())
