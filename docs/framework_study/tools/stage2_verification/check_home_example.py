#!/usr/bin/env python3
"""The worked example of chapter 04 §6 (preserved from the Stage 2 `home.py`).

At q_home with qd = (0.3, 0, 0, 0.3, 0, 0, 0) rad/s, for the fingertip control point fr3_link8#2:
position, Jacobian (4.8)/(4.4), velocity J qd and drift J-dot qd, compared with the numbers printed
in chapter 04 §6. Also checks column 1 against the closed form a1 x (p - o1) of (4.3).

Needs: the ROS 2 environment (xacro, franka_description), Pinocchio, numpy, PyYAML (container).
"""
import os
import sys

sys.dont_write_bytecode = True   # read-only: never write __pycache__ into the repository
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common as C  # noqa: E402

C.use_source_package()
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand  # noqa: E402

R = C.Report('check_home_example (chapter 04 §6)')
cfg = C.load_yaml(os.path.join(C.CONFIG, 'fr3_complete.yaml'))
p = build_urdf_no_hand()
try:
    m = pin.buildModelFromUrdf(p)
finally:
    os.unlink(p)
kin = CBFKinematics(m)
kin.configure_control_points(cfg['robot'], cfg['distance'])
qd = np.array([0.3, 0, 0, 0.3, 0, 0, 0])
kin.update(C.Q_HOME, qd)
lb = 'fr3_link8#2'
J, Jd = kin.control_point_jacobian(lb)
pos = kin.control_point_position(lb)
R.close('fingertip p [m]', pos, [0.3069, 0, 0.4903], 6e-5)
R.close('flange fr3_link8 origin [m]', kin.data.oMf[m.getFrameId('fr3_link8')].translation,
        [0.3069, 0, 0.5903], 6e-5)
R.close('z8 at home', kin.data.oMf[m.getFrameId('fr3_link8')].rotation[:, 2], [0, 0, -1], 1e-9)
J_reported = np.array([[0, 0.1573, 0, 0.1245, 0, 0.207, 0],
                       [0.3069, 0, 0.3282, 0, 0.207, 0, 0],
                       [0, -0.3069, 0, 0.472, 0, 0.088, 0]])
R.close('fingertip J (3x7)', J, J_reported, 6e-4, note='printed with 4 significant digits')
o1 = kin.data.oMf[m.getFrameId('fr3_link1')].translation
R.close('(4.3) column 1 = z x (p - o1)', J[:, 0], np.cross([0, 0, 1], pos - o1), 1e-12, kind='NUM')
R.close('v = J qd [m/s]', J @ qd, [0.0373, 0.0921, 0.1416], 6e-5)
R.close('J-dot qd [m/s^2]', Jd @ qd, [-0.0701, 0.0224, 0.0112], 6e-5)
R.close('|J-dot qd| [m/s^2]', np.linalg.norm(Jd @ qd), 0.075, 1e-3)
sys.exit(R.finish())
