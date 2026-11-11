#!/usr/bin/env python3
"""F-2-11: a held distance entry is published valid with closest_point_human = (0, 0, 0)
(preserved from the Stage 2 `hold.py`).

During the bounded LPF hold, `DistanceEngine._lpf_pass` emits a ControlPointResult with
`closest_obstacle_point=None` and the previous direction. This script feeds exactly such a result
(home-pose fingertip, held direction (-1, 0, 0), d = 0.16 m) to the baseline `build_cp_messages`,
reads it back with `labelled_links`, and evaluates the normal the safety filter computes from the
message, n = (pr - ph) / |pr - ph| (cbf_state_rows.py around L2509).

What this verifies: the MESSAGE content (valid flag, ph = 0) and the angle between that normal and
the published direction. What it does NOT verify: what the QP then does with the row. That is
Q-2-1 (Stage 3), and is the runtime / filter-side part of F-2-11.

Needs: the ROS 2 environment (franka_msgs, builtin_interfaces), numpy, PyYAML. Run inside the
`franka_ros2` container (README.md).
"""
import os
import sys

sys.dont_write_bytecode = True   # read-only: never write __pycache__ into the repository
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common as C  # noqa: E402

C.use_source_package()
import numpy as np  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from franka_experiments.utils.distance_engine import ControlPointResult  # noqa: E402
from franka_experiments.utils.perception_msgs import build_cp_messages, labelled_links  # noqa: E402

R = C.Report('check_held_entry (chapter 05 §3.5; F-2-11)')
cfg = C.load_yaml(os.path.join(C.CONFIG, 'fr3_complete.yaml'))
held_dir = np.array([-1.0, 0.0, 0.0], np.float32)
r = ControlPointResult(point=np.array([0.3069, 0.0, 0.4903]), seg_idx=4, cp_idx=2, radius=0.05,
                       start_link='fr3_link8', end_link='fr3_link8', distance=0.16, direction=held_dir,
                       closest_obstacle_point=None, closest_pixel=None, range_m=None)
_, mld = build_cp_messages([r], 400, Time(), 'fr3_link0', cfg['robot']['segment_links'],
                           {'min_thresh': 0.08, 'max_thresh': 0.7}, 2.0, {})
rows = list(labelled_links(mld))
R.fact('one entry published', len(rows) == 1, str([lb for lb, _ in rows]))
lb, ld = rows[0]
ph = np.array([ld.closest_point_human.x, ld.closest_point_human.y, ld.closest_point_human.z])
pr = np.array([ld.closest_point_robot.x, ld.closest_point_robot.y, ld.closest_point_robot.z])
R.fact('held entry is valid', ld.valid is True, f'valid={ld.valid}, distance={ld.distance}')
R.close('closest_point_human of the held entry [m]', ph, [0, 0, 0], 0, kind='FACT')
R.close('published direction', [ld.direction.x, ld.direction.y, ld.direction.z], held_dir, 1e-7, kind='FACT')
n = (pr - ph) / np.linalg.norm(pr - ph)
R.close('normal the filter computes, (pr - ph)/|pr - ph|', n, [0.531, 0, 0.848], 6e-4)
R.close('angle between that normal and the published direction [deg]',
        np.degrees(np.arccos(np.clip(n @ held_dir, -1, 1))), 122.0, 0.06)
sys.exit(R.finish())
