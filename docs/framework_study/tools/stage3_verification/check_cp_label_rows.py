#!/usr/bin/env python3
"""F-2-10 downstream (F-3-12): duplicate wrist control points and label flips in the REAL row builder.

Same pipeline as check_held_row.py (define_control_points at q_home -> build_cp_messages ->
labelled_links -> _on_distances mapping -> ConstraintBuilder.build, hardware-default parameters).
Every control point sees a primary obstacle point H (cluster 0). In the middle third of the run a
second cluster H1 appears (multi_obstacle_k = 2 publishes one extra rank per control point).
H recedes at 0.3 m/s along the line from the wrist, so h_raw rises every frame and the asymmetric
recovery EMA (cbf_h_recovery_alpha) is what sets h.

Checks:
  1. with one cluster the wrist labels are fr3_link6#0, fr3_link6#1; with two they are
     #0, #0.1, #0.2, #0.3 (F-2-10)                                                          (FACT)
  2. the rows of fr3_link6#0 and its twin (#1 or #0.2) have identical a and c (duplicate rows) (NUM)
  3. at a flip to a NEW key (#1 -> #0.2) the twin's _h_smooth is empty, so h_raw is accepted at
     once (a loosening while the obstacle recedes); at the flip BACK the stale #1 state from
     before is resumed. #0, with a stable label, keeps smoothing                               (FACT)
  4. the twin's per-label velocity state (_obs_vel, _obs_vmed) is re-created at each flip     (FACT)

Needs: the ROS 2 container. Read-only.
"""
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _builder3 as B  # noqa: E402

C3, C = B.C3, B.C
import numpy as np  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from franka_experiments.utils.distance_engine import ControlPointResult, ObstacleHit  # noqa: E402
from franka_experiments.utils.distance_utils import define_control_points  # noqa: E402
from franka_experiments.utils.perception_msgs import build_cp_messages  # noqa: E402

R = C.Report('check_cp_label_rows (chapter 07 §8.2; F-2-10 -> F-3-12)')
P = C3.params_namespace('hw_default')
R.fact(f'multi_obstacle_k is a perception setting; this script publishes 1 extra rank in frames 10-19 '
       f'(as k = 2 does with a second cluster); cbf_h_recovery_alpha = {P.cbf_h_recovery_alpha}', True)
cfg = B.robot_cfg()
builder, kin, log = B.make_builder(P)
q = C.Q_HOME.copy()
kin.update(q, np.zeros(7), with_jdot=False)
m, d = kin.model, kin.data
tf = {n: (d.oMf[m.getFrameId(n)].rotation.copy(), d.oMf[m.getFrameId(n)].translation.copy())
      for n in cfg['robot']['segment_links']}
cps = define_control_points(tf, cfg['robot'], cfg['distance'])
wrist = cps[4]['point']
u = np.array([0.0, 0.6, 0.8])                 # direction from the wrist to the primary obstacle
H1 = np.array([0.4, -0.4, 0.9])
HZ = 90.0


def message(Hp, two, t):
    res = []
    for cp in cps:
        dv = cp['point'] - Hp
        extras = []
        if two:
            e1 = cp['point'] - H1
            extras = [ObstacleHit(float(np.linalg.norm(e1)) - 0.05, H1, e1 / np.linalg.norm(e1), 1, 1.5)]
        res.append(ControlPointResult(
            point=cp['point'], seg_idx=cp['seg_idx'], cp_idx=cp['cp_idx'], radius=0.05,
            start_link=cp['start_link'], end_link=cp['end_link'],
            distance=float(np.linalg.norm(dv)) - 0.05, direction=dv / np.linalg.norm(dv),
            closest_obstacle_point=Hp, closest_pixel=(1, 1), cluster_id=0, extras=extras, range_m=1.5))
    stamp = Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
    _, mld = build_cp_messages(res, 400, stamp, 'fr3_link0', cfg['robot']['segment_links'],
                               {'min_thresh': 0.08, 'max_thresh': 0.7}, 2.0, {})
    return mld


rows_log = []
t0 = 100.0
for k in range(30):
    t = t0 + k / HZ
    two = 10 <= k < 20
    Hp = wrist + (0.20 + 0.3 * k / HZ) * u          # receding: h_raw rises every frame
    items = B.obstacles_from_msg(message(Hp, two, t))
    con = builder.build(B.JointSnap(q, np.zeros(7), t), B.snap(items, t), t)
    rows = {lbl: (a, h, c) for (lbl, a, h, _, c) in B.obstacle_rows(con) if lbl.startswith('fr3_link6')}
    rows_log.append((k, two, rows, set(builder._obs_vel)))

lab1 = sorted(rows_log[5][2])
lab2 = sorted(rows_log[15][2])
R.fact('1. one cluster: wrist labels', lab1 == ['fr3_link6#0', 'fr3_link6#1'], str(lab1))
R.fact('1. two clusters: wrist labels (F-2-10)', lab2 == ['fr3_link6#0', 'fr3_link6#0.1', 'fr3_link6#0.2',
                                                          'fr3_link6#0.3'], str(lab2))
dmax = 0.0
for k, two, rows, _ in rows_log:
    twin = 'fr3_link6#0.2' if two else 'fr3_link6#1'
    a0, _, c0 = rows['fr3_link6#0']
    a1, _, c1 = rows[twin]
    dmax = max(dmax, float(np.abs(a0 - a1).max()), abs(c0 - c1))
R.below('2. rows of fr3_link6#0 and its twin: max |a0 - a1|, |c0 - c1| over all frames', dmax, 1e-12, kind='NUM',
        note='duplicates to rounding: the QP carries the same constraint twice')
R.info('frame  clusters  h(#0)     h(twin)   twin label')
flips = []
for k, two, rows, _ in rows_log:
    twin = 'fr3_link6#0.2' if two else 'fr3_link6#1'
    h0, h1 = rows['fr3_link6#0'][1], rows[twin][1]
    R.info(f'{k:4d}  {2 if two else 1:5d}   {h0:8.5f}  {h1:8.5f}  {twin}')
    if k in (10, 20):
        flips.append((k, h0, h1))
k, h0, h1 = flips[0]
R.fact(f'3. frame {k} (flip #1 -> #0.2, a NEW key): twin h {h1:.5f} > #0 h {h0:.5f}: no _h_smooth entry, so '
       f'h_raw is taken at once (loosening {1e3 * (h1 - h0):.2f} mm in one frame)', h1 > h0 + 1e-6)
k, h0, h1 = flips[1]
h9 = [r for r in rows_log if r[0] == 9][0][2]['fr3_link6#1'][1]
R.fact(f'3. frame {k} (flip back to #1, a STALE key): twin h {h1:.5f} != #0 h {h0:.5f}; the EMA resumes '
       f'from the value left at frame 9 ({h9:.5f}), i.e. 0.8 x {h9:.5f} + 0.2 x h_raw', abs(h1 - h0) > 1e-3
       and h1 > h9, 'the direction depends on the scene; here (receding) it is a tightening')
same = all(abs(rows['fr3_link6#0'][1] - rows['fr3_link6#1' if not two else 'fr3_link6#0.2'][1]) < 1e-12
           for k, two, rows, _ in rows_log if k in range(1, 10))
R.fact('3. control: with a stable label (frames 1-9) the twin h equals #0 h exactly', same)
keys = rows_log[15][3]
R.fact('4. per-label velocity state exists under BOTH twin keys after the flip (#1 and #0.2)',
       {'fr3_link6#1', 'fr3_link6#0.2'} <= keys, str(sorted(x for x in keys if x.startswith('fr3_link6'))))
sys.exit(R.finish())
