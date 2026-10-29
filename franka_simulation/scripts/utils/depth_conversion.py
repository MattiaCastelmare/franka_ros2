"""Gazebo depth -> the RealSense depth stream the distance engine reads (plan H2, H11).

Pure numpy, no ROS: the ``depth_sim_adapter`` node calls these.

H2, the encoding (always on). Gazebo's depth camera publishes 32FC1 METRES,
with +inf beyond the far clip, -inf before the near clip, and NaN where nothing
returns. ``distance_engine`` (``_DEPTH_TO_M = 0.001``, ``distance_engine.py:145``)
reads 16UC1 MILLIMETRES, and treats 0 as "no return": it is below
``min_depth_m`` and never reopens a masked pixel (``distance_engine.py:376-402``).
So: metres -> rounded millimetres (the D455's own depth unit), non-finite or
<= 0 -> 0, anything a uint16 cannot hold -> 0 (out of range = no return, which
is what the camera reports).

H11, fidelity (each stage OFF by default, each with its own switch). Gazebo
depth is exact and instantaneous; the stages put back what the rig measured.
The order and semantics follow ``franka_rl/envs/randomization.py::obstacle``:
delay first (the frame is old), then dropout (a lost frame), then range noise.

* range noise: per-pixel Gaussian on z with
  sigma_z(z) = z² / (f_px · baseline_m) · sigma_d_px. This is the model
  ``utils.cbf_state_rows.sensor_range_uncertainty`` prices, with its constants
  read from fr3_control.yaml by the node.
* dropout: the frame is not published. ``real_time_distance`` then has no new
  frame and holds its last result, the same as "hold the last good frame";
  republishing the old frame would be dropped by its duplicate-stamp check
  anyway (``real_time_distance.py:626-630``).
* delay: published ``delay_s`` after capture, with the CAPTURE stamp kept, so
  every latency number downstream is measured exactly as on the rig.
"""
from __future__ import annotations

import collections

import numpy as np

UINT16_MAX_MM = 65535


def metres_to_mm(depth_m: np.ndarray) -> np.ndarray:
    """32FC1 metres -> 16UC1 millimetres; no return (non-finite, <= 0, too far) -> 0."""
    d = np.asarray(depth_m, dtype=np.float32)
    mm = np.rint(d * np.float32(1000.0))
    ok = np.isfinite(mm) & (mm > 0) & (mm <= UINT16_MAX_MM)
    return np.where(ok, mm, 0).astype(np.uint16)


def range_sigma(z_m: np.ndarray, f_px: float, baseline_m: float, sigma_d_px: float) -> np.ndarray:
    """[m] stereo range noise sigma_z(z) = z^2 / (f b) * sigma_d (range_noise_calibration.py)."""
    return (np.asarray(z_m, dtype=np.float64) ** 2) / (f_px * baseline_m) * sigma_d_px


def add_range_noise(depth_m: np.ndarray, rng: np.random.Generator, *, f_px: float,
                    baseline_m: float, sigma_d_px: float) -> np.ndarray:
    """Noisy copy of a metres image; pixels with no return stay without one."""
    d = np.asarray(depth_m, dtype=np.float32)
    valid = np.isfinite(d) & (d > 0)
    out = d.copy()
    z = d[valid]
    out[valid] = (z + rng.standard_normal(z.shape) *
                  range_sigma(z, f_px, baseline_m, sigma_d_px)).astype(np.float32)
    return out


class FidelityStages:
    """Dropout + noise at receipt, then a delay queue keyed on the capture stamp.

    ``push(stamp_s, depth_m, tag)`` -> whether the frame survived dropout;
    ``pop_due(now_s)`` -> list of (stamp_s, tag, uint16 mm image) whose release
    time ``stamp_s + delay_s`` has come. ``tag`` rides along untouched (the
    node passes the original header stamp, so no float round trip can make two
    frames look like duplicates). With every stage off, a push is popped at
    once, unchanged except for the encoding.
    """

    def __init__(self, *, delay_s: float = 0.0, dropout_p: float = 0.0,
                 range_noise: bool = False, f_px: float = 0.0, baseline_m: float = 0.095,
                 sigma_d_px: float = 0.20, seed: int = 0):
        if not 0.0 <= dropout_p < 1.0:
            raise ValueError(f'dropout_p must be in [0, 1), got {dropout_p}')
        if delay_s < 0.0:
            raise ValueError(f'delay_s must be >= 0, got {delay_s}')
        if range_noise and f_px <= 0.0:
            raise ValueError('range_noise needs f_px > 0 (from camera_info)')
        self.delay_s = float(delay_s)
        self.dropout_p = float(dropout_p)
        self.range_noise = bool(range_noise)
        self.f_px, self.baseline_m, self.sigma_d_px = float(f_px), float(baseline_m), float(sigma_d_px)
        self.rng = np.random.default_rng(seed)
        self.queue: collections.deque = collections.deque()
        self.n_in = self.n_dropped = self.n_out = 0

    def push(self, stamp_s: float, depth_m: np.ndarray, tag=None) -> bool:
        self.n_in += 1
        if self.dropout_p > 0.0 and self.rng.random() < self.dropout_p:
            self.n_dropped += 1
            return False
        if self.range_noise:
            depth_m = add_range_noise(depth_m, self.rng, f_px=self.f_px,
                                      baseline_m=self.baseline_m, sigma_d_px=self.sigma_d_px)
        self.queue.append((stamp_s + self.delay_s, stamp_s, tag, metres_to_mm(depth_m)))
        return True

    def pop_due(self, now_s: float) -> list:
        out = []
        while self.queue and self.queue[0][0] <= now_s + 1e-9:
            _, stamp, tag, img = self.queue.popleft()
            out.append((stamp, tag, img))
            self.n_out += 1
        return out
