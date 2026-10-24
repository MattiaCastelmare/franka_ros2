"""cbf_scenarios' emulated perception, as the messages the real chain publishes (Phase 5, column 2).

The middle column of the three-column comparison: the Gazebo plant and the
whole control stack, but the depth camera, distance engine and tracker
replaced by what ``cbf_scenarios.run_scenario`` feeds its filter:

* the TRUE surface gap of every control point to its nearest sphere, clamped
  at 0 (no dilation margin, no depth noise, no field of view);
* normal = unit(CP − centre), obstacle point = centre + r·normal;
* one track (id 1) per obstacle, ``frames_seen`` counted from the obstacle's
  first frame, velocity = truth + N(0, 0.02) per axis, or a draw of
  ``cbf_scenarios.camera_velocity_noise`` with ``cam_noise``; velocity
  covariance 0.05²·I, position covariance 0.01²·I, no acceleration;
* delivered ``t_cam`` (13 ms) after capture, stamped with the capture time;
* no CP within ``max_thresh``: an empty heartbeat, as real_time_distance does.

The messages are built by ``perception_msgs.build_cp_messages``, the real
constructor, from ``ControlPointResult`` objects. Only the track fields are
written here, with cbf_scenarios' values. The CP set is the robot config's
(the one the Gazebo filter and the ground truth use), not cbf_scenarios'
5-segment R_CAP = 0.05 set. That difference is part of the plant/geometry
gap between columns 1 and 2. No rclpy here: the node owns the I/O.
"""
from __future__ import annotations

import os
import sys

import numpy as np

T_CAM = 0.013          # cbf_scenarios.run_scenario(t_cam=0.013)
V_SIGMA = 0.02         # rng.normal(0, 0.02, 3) without cam_noise
VEL_COV = 0.05 ** 2    # vel_cov=(0.05 ** 2) * np.eye(3)
POS_COV = 0.01 ** 2    # pos_cov=(0.01 ** 2) * np.eye(3)


def _cbf_scenarios():
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.join(here, '..', '..', '..', 'franka_experiments', 'scripts'),
                 '/ros2_ws/src/franka_experiments/scripts'):
        if os.path.isfile(os.path.join(cand, 'cbf_scenarios.py')):
            cand = os.path.normpath(cand)
            if cand not in sys.path:
                sys.path.insert(0, cand)
            break
    import cbf_scenarios
    return cbf_scenarios


def nearest(cp_point, centres, radii, cp_radius):
    """(k, gap, n̂, obstacle point) of the nearest sphere by SURFACE gap."""
    d = np.linalg.norm(np.asarray(centres, float) - np.asarray(cp_point, float), axis=1)
    gaps = d - np.asarray(radii, float) - float(cp_radius)
    k = int(np.argmin(gaps))
    c = np.asarray(centres[k], float)
    v = np.asarray(cp_point, float) - c
    n = v / max(float(np.linalg.norm(v)), 1e-9)
    return k, float(gaps[k]), n, c + float(radii[k]) * n


def frame_results(cps, centres, radii):
    """(ControlPointResult per CP, nearest-obstacle index per CP, true gap per CP)."""
    from franka_experiments.utils.distance_engine import ControlPointResult
    out, ks, gaps = [], [], []
    for c in cps:
        k, gap, n, ph = nearest(c['point'], centres, radii, c['radius'])
        out.append(ControlPointResult(
            point=np.asarray(c['point'], float), seg_idx=c['seg_idx'], cp_idx=c['cp_idx'],
            radius=c['radius'], start_link=c['start_link'], end_link=c['end_link'],
            distance=max(gap, 0.0), direction=n, closest_obstacle_point=ph))
        ks.append(k)
        gaps.append(gap)
    return out, ks, np.array(gaps)


class TrackEmulator:
    """frames_seen and the noisy velocity cbf_scenarios attaches to every row."""

    def __init__(self, n_obstacles: int, frame_period: float, *, cam_noise: bool, seed: int):
        self.first_seen = [None] * n_obstacles
        self.frame_period = float(frame_period)
        self.rng = np.random.default_rng(seed)
        self._noise = _cbf_scenarios().camera_velocity_noise if cam_noise else None

    def frames_seen(self, k: int, t_cap: float) -> int:
        if self.first_seen[k] is None:
            self.first_seen[k] = t_cap
        return int((t_cap - self.first_seen[k]) / self.frame_period) + 1

    def velocity(self, v_true) -> np.ndarray:
        v = np.asarray(v_true, float)
        if self._noise is not None:
            return v + self._noise(self.rng)[0]
        return v + self.rng.normal(0.0, V_SIGMA, 3)


def fill_track_fields(mld, results, ks, vels, tracks: TrackEmulator, t_cap: float) -> None:
    """Write cbf_scenarios' track fields into every LinkDistance of ``mld``, in place.

    Each entry is matched to its ControlPointResult by its own
    closest_point_robot, so this does not depend on build_cp_messages' row
    order. track_id is 1 for every obstacle, as in cbf_scenarios; frames_seen
    counts per obstacle. The velocity noise is drawn per entry per frame
    (cbf_scenarios redraws it at each 50 Hz rebuild of the same frame).
    """
    from geometry_msgs.msg import Vector3
    P = np.array([r.point for r in results])
    seen = {}
    for ld in mld.links:
        p = np.array([ld.closest_point_robot.x, ld.closest_point_robot.y, ld.closest_point_robot.z])
        k = ks[int(np.argmin(np.linalg.norm(P - p, axis=1)))]
        if k not in seen:
            seen[k] = tracks.frames_seen(k, t_cap)
        v = tracks.velocity(vels[k])
        ld.track_id = 1
        ld.frames_seen = seen[k]
        ld.obstacle_velocity = Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))
        ld.velocity_covariance = list((VEL_COV * np.eye(3)).ravel())
        ld.position_covariance = list((POS_COV * np.eye(3)).ravel())
        ld.position_velocity_covariance = [0.0] * 9
        ld.obstacle_acceleration = Vector3()
