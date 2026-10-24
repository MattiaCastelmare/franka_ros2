"""Ground-truth geometry: the TRUE control points and the TRUE surface gaps. No rclpy.

The control points are the ones real_time_distance uses, computed by the same
function, ``utils.distance_utils.define_control_points``, with
``fr3_complete.yaml``'s segments, radii and fingertip offset. Only the
transforms differ: real_time_distance reads TF, this uses forward kinematics
of the TRUE joint state (exact in sim) on the run's URDF. Both are rooted at
``base`` = ``fr3_link0`` = the Gazebo world origin.

True surface gap of control point i to sphere j (centre c, radius r):
    gap_ij = ‖cp_i − c_j‖ − r_j − r_cp,i
which is what the engine's ``surface = max(dist − radius − margin, 0)`` estimates
from the visible points (``distance_engine.py:475-531``). The truth is NOT
clamped at 0: a negative gap is penetration, and contact is ``gap < 0``.
"""
from __future__ import annotations

import os

import numpy as np
import yaml

LINKS = [f'fr3_link{i}' for i in range(0, 9)]


def load_perception_config(path: str | None = None) -> dict:
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('franka_experiments'), 'config',
                            'fr3_complete.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


class TruthGeometry:
    def __init__(self, urdf_xml: str, perception_cfg: dict):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()
        self.robot_cfg = perception_cfg['robot']
        self.distance_cfg = perception_cfg['distance']
        self.fids = {n: self.model.getFrameId(n) for n in LINKS if self.model.existFrame(n)}
        self.qidx = [self.model.joints[self.model.getJointId(f'fr3_joint{i}')].idx_q
                     for i in range(1, 8)]

    def transforms(self, q) -> dict:
        qp = self.pin.neutral(self.model)
        qp[self.qidx] = np.asarray(q, float)
        self.pin.framesForwardKinematics(self.model, self.data, qp)
        return {n: (self.data.oMf[f].rotation.copy(), self.data.oMf[f].translation.copy())
                for n, f in self.fids.items()}

    def control_points(self, q) -> list:
        from franka_experiments.utils.distance_utils import define_control_points
        return define_control_points(self.transforms(q), self.robot_cfg, self.distance_cfg)

    def flange(self, q) -> np.ndarray:
        return self.transforms(q)['fr3_link8'][1]


def cp_label(cp: dict) -> str:
    """real_time_distance's label convention: '<end_link>#<cp_idx>'."""
    return f"{cp['end_link']}#{cp['cp_idx']}"


def surface_gaps(cps: list, centres, radii) -> np.ndarray:
    """(n_cp, n_obs) true surface gaps [m] (signed)."""
    P = np.array([c['point'] for c in cps])
    rcp = np.array([c['radius'] for c in cps])
    C = np.asarray(centres, float).reshape(-1, 3)
    R = np.asarray(radii, float).reshape(-1)
    d = np.linalg.norm(P[:, None, :] - C[None, :, :], axis=2)
    return d - R[None, :] - rcp[:, None]


def closest_point_visible(cp, centre, radius, camera) -> bool:
    """Is the sphere point nearest to ``cp`` on the hemisphere facing ``camera``?

    When it is not, the engine (which only sees the visible surface) can only
    report a gap larger than the true one; the Phase 3 test places its spheres
    so that it is.
    """
    n = np.asarray(cp, float) - np.asarray(centre, float)
    n /= np.linalg.norm(n)
    p = np.asarray(centre, float) + radius * n
    return float(n @ (np.asarray(camera, float) - p)) > 0.0
