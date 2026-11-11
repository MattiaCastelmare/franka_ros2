#!/usr/bin/env python3
"""F-2-10: duplicate control points break the multi-obstacle rank labels (preserved `labels.py`).

Builds the 11 home-pose control points with the baseline `distance_utils.define_control_points`
from Pinocchio forward kinematics, gives every control point one extra obstacle rank (as
`multi_obstacle_k >= 2` does when a second cluster is in range), publishes them with the baseline
`perception_msgs.build_cp_messages`, and reads the labels back with `labelled_links`.
Repeats with fr3_link6's translation composed from fr3_link5 with a zero offset, which is what a
TF lookup of a zero-length fixed offset produces.

Expected (bug present at the baseline): the labels contain 'fr3_link6#0.2' and 'fr3_link6#0.3' and
no 'fr3_link6#1'. A control run without the duplicate (fr3_link6#1 shifted by 1 mm) shows the
labelling itself is otherwise correct.

Needs: the ROS 2 environment (franka_msgs, builtin_interfaces, xacro, franka_description),
Pinocchio, numpy, PyYAML. Run inside the `franka_ros2` container (README.md).
"""
import os
import sys

sys.dont_write_bytecode = True   # read-only: never write __pycache__ into the repository
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common as C  # noqa: E402

C.use_source_package()
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from franka_experiments.utils.distance_engine import ControlPointResult, ObstacleHit  # noqa: E402
from franka_experiments.utils.distance_utils import define_control_points  # noqa: E402
from franka_experiments.utils.kinematics import build_urdf_no_hand  # noqa: E402
from franka_experiments.utils.perception_msgs import build_cp_messages, labelled_links  # noqa: E402

R = C.Report('check_cp_labels (chapter 05 §6.2; F-2-10)')
cfg = C.load_yaml(os.path.join(C.CONFIG, 'fr3_complete.yaml'))
p = build_urdf_no_hand()
try:
    m = pin.buildModelFromUrdf(p)
finally:
    os.unlink(p)
d = m.createData()
pin.forwardKinematics(m, d, C.Q_HOME)
pin.updateFramePlacements(m, d)
tf = {n: (d.oMf[m.getFrameId(n)].rotation.copy(), d.oMf[m.getFrameId(n)].translation.copy())
      for n in cfg['robot']['segment_links']}
R.fact('fr3_link5 and fr3_link6 translations are exactly equal (Pinocchio FK)',
       np.array_equal(tf['fr3_link5'][1], tf['fr3_link6'][1]))
H0 = np.array([0.5, 0.3, 0.7])       # primary obstacle point
H1 = np.array([0.4, -0.4, 0.9])      # a second cluster: rank 1 for every control point


def labels_for(cps):
    res = []
    for cp in cps:
        dv = cp['point'] - H0
        e1 = cp['point'] - H1
        res.append(ControlPointResult(
            point=cp['point'], seg_idx=cp['seg_idx'], cp_idx=cp['cp_idx'], radius=0.05,
            start_link=cp['start_link'], end_link=cp['end_link'],
            distance=float(np.linalg.norm(dv)) - 0.05, direction=dv / np.linalg.norm(dv),
            closest_obstacle_point=H0, closest_pixel=(1, 1), cluster_id=0,
            extras=[ObstacleHit(float(np.linalg.norm(e1)) - 0.05, H1, e1 / np.linalg.norm(e1), 1, 1.5)],
            range_m=1.5))
    _, mld = build_cp_messages(res, 400, Time(), 'fr3_link0', cfg['robot']['segment_links'],
                               {'min_thresh': 0.08, 'max_thresh': 0.7}, 2.0, {})
    return [lb for lb, _ in labelled_links(mld)]


expected_bug = ['fr3_link6#0', 'fr3_link6#0.1', 'fr3_link6#0.2', 'fr3_link6#0.3']
for title, tfx in (('Pinocchio FK', tf),
                   ('fr3_link6 = fr3_link5 + R5 @ 0 (TF-style)',
                    {**tf, 'fr3_link6': (tf['fr3_link6'][0], tf['fr3_link5'][1] + tf['fr3_link5'][0] @ np.zeros(3))})):
    cps = define_control_points(tfx, cfg['robot'], cfg['distance'])
    R.fact(f'[{title}] fr3_link6#0 and #1 positions exactly equal', np.array_equal(cps[4]['point'], cps[5]['point']))
    got = labels_for(cps)
    R.info(f'[{title}] labels: {got}')
    l6 = [x for x in got if x.startswith('fr3_link6')]
    R.fact(f'[{title}] F-2-10 reproduced: fr3_link6 labels are {expected_bug}', l6 == expected_bug, str(l6))

# Control: break the exact equality by 1 mm; the labels must then be the intended ones.
cps = define_control_points(tf, cfg['robot'], cfg['distance'])
cps[5] = dict(cps[5], point=cps[5]['point'] + np.array([0.001, 0, 0]))
l6 = [x for x in labels_for(cps) if x.startswith('fr3_link6')]
R.fact('control (duplicate shifted by 1 mm): intended labels', l6 == ['fr3_link6#0', 'fr3_link6#0.1',
                                                                    'fr3_link6#1', 'fr3_link6#1.1'], str(l6))
sys.exit(R.finish())
