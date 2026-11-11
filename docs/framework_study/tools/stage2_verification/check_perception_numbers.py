#!/usr/bin/env python3
"""Perception arithmetic of chapter 05: (5.1)-(5.3), (5.6), the LPF weight, sampling density.

  - worked example of 05 §11: an obstacle 0.25 m in +x of the home fingertip, projected with the
    shipped extrinsic and depth K, rounded to a pixel and to 1 mm, back-projected (5.1)-(5.2),
    distance (5.3) with the margin (5.6)
  - F-2-8: the mask margins 12 / 24 px, computed with the integer arithmetic of
    `MaskBuilder.rebuild` (L150-152, L290-292) from fr3_complete.yaml, and the metric bias
    12 Z / f_x at Z = 1.0, 1.5, 2.0 m
  - (5.1)/(5.2) round trip on random points (NUM)
  - the LPF weight alpha = exp(-dt / tau) from the source `rate_scaling.ema_alpha_for`
  - the ROI sampling density quoted in 05 §4

The fingertip position (0.3069, 0, 0.4903) is the chapter 04 §6 value, re-verified in the container
by check_home_example.py. The margin arithmetic is re-implemented here (MaskBuilder imports cv2);
it is a two-line integer formula, cited by line.

Needs: Python 3, numpy, PyYAML. Imports only `franka_experiments.utils.rate_scaling` (pure Python)
from the source tree. Runs on the host.
"""
import os
import sys

sys.dont_write_bytecode = True   # read-only: never write __pycache__ into the repository
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common as C  # noqa: E402

C.use_source_package()
import numpy as np  # noqa: E402
from franka_experiments.utils.rate_scaling import ema_alpha_for, frames_for  # noqa: E402

R = C.Report('check_perception_numbers (chapter 05 §3, §4, §11; F-2-8)')
K = np.array(C.load_yaml(os.path.join(C.CONFIG, 'depth_intrinsics.yaml'))['k'], dtype=float).reshape(3, 3)
R.close('depth K: fx, fy, cx, cy', [K[0, 0], K[1, 1], K[0, 2], K[1, 2]],
        [428.84436035, 428.84436035, 426.06338501, 235.1672821], 1e-6, kind='FACT')
T = C.pose_from_extrinsics(C.load_yaml(os.path.join(C.CONFIG, 'camera_extrinsics.yaml')))
Rb, tb = T[:3, :3], T[:3, 3]
cfg = C.load_yaml(os.path.join(C.CONFIG, 'fr3_complete.yaml'))
fx = K[0, 0]

# ── (5.1)/(5.2) round trip ────────────────────────────────────────────────────────────────
rng = np.random.default_rng(0)
pB = tb + rng.normal(0, 0.5, (1000, 3)) + Rb @ np.array([0, 0, 1.5])
pC = (Rb.T @ (pB - tb).T).T
uv = (K @ pC.T).T
u, v, Z = uv[:, 0] / uv[:, 2], uv[:, 1] / uv[:, 2], pC[:, 2]
back = np.column_stack([(u - K[0, 2]) * Z / K[0, 0], (v - K[1, 2]) * Z / K[1, 1], Z])
R.below('(5.1)+(5.2) project / back-project round trip, 1000 points [m]',
        np.abs((Rb @ back.T).T + tb - pB).max(), 1e-12)

# ── worked example (05 §11) ───────────────────────────────────────────────────────────────
tip = np.array([0.3069, 0.0, 0.4903])
obs = tip + np.array([0.25, 0.0, 0.0])
oc = Rb.T @ (obs - tb)
uvz = K @ oc
u0, v0 = uvz[0] / uvz[2], uvz[1] / uvz[2]
R.close('obstacle in camera frame [m]', oc, [-0.0888, 0.1869, 1.3182], 6e-5)
R.close('obstacle pixel (u, v)', [u0, v0], [397.2, 296.0], 0.06)
ui, vi = int(round(u0)), int(round(v0))
Zq = int(round(oc[2] * 1000)) / 1000.0                    # uint16 millimetres
ocq = np.array([(ui - K[0, 2]) * Zq / K[0, 0], (vi - K[1, 2]) * Zq / K[1, 1], Zq])
ob = Rb @ ocq + tb
R.close('back-projected obstacle, base [m]', ob, [0.5571, -0.0005, 0.4904], 6e-5)
R.close('quantisation error [m]', np.linalg.norm(ob - obs), 0.0005, 6e-5)
dist = np.linalg.norm(ob - tip)

# ── F-2-8 margins, MaskBuilder integer arithmetic ─────────────────────────────────────────
mk = cfg['mask']
ds = int(mk['mask_downsample'])
dil = max(1, int(mk['robot_mask_dilate_px']) // ds)
ee = max(1, int(mk['ee_mask_dilate_px']) // ds)
extra = int(mk['search_exclusion_extra_px']) // ds
m_body, m_ee = dil * ds + extra * ds, ee * ds + extra * ds
R.close('F-2-8 margins (body, EE) [full-resolution px]', [m_body, m_ee], [12, 24], 0, kind='FACT')
margin = m_body * Zq / fx
R.close('worked example: |o - c| [m]', dist, 0.2502, 6e-5)
R.close('worked example: margin 12 Z / fx [m]', margin, 0.0369, 6e-5)
R.close('worked example: published d = |o - c| - r - margin [m]', max(dist - 0.05 - margin, 0.0), 0.1634, 6e-5)
for Zm, rep in ((1.0, 0.028), (1.5, 0.042), (2.0, 0.056)):
    R.close(f'F-2-8 uniform low bias at Z = {Zm} m [m]', m_body * Zm / fx, rep, 6e-4)

# ── sampling density (05 §4) ──────────────────────────────────────────────────────────────
step = int(cfg['distance'].get('pixel_step', 6))
R.close('pixel footprint at Z = 1.318 m [m/px]', Zq / fx, 0.0031, 6e-5)
R.close(f'ROI stride {step} at Z = 1.318 m [m]', step * Zq / fx, 0.0184, 6e-5)

# ── LPF weight and frame counts (05 §3.5, §7.4, §8.1) ─────────────────────────────────────
tau = float(cfg['distance']['lpf_tau_s'])
R.close('alpha = exp(-dt/tau) at 90 Hz', ema_alpha_for(tau, 1 / 90), 0.793, 6e-4)
R.close('alpha = exp(-dt/tau) at 30 Hz', ema_alpha_for(tau, 1 / 30), 0.499, 6e-4)
R.close('ema_alpha_for equals exp(-dt/tau)', ema_alpha_for(tau, 1 / 90), np.exp(-(1 / 90) / tau), 1e-15, kind='NUM')
R.close('approach limiter: largest single-frame drop at 90 Hz, v_max 2 m/s [m]', 2.0 / 90, 0.022, 3e-4,
        kind='REPRO', note='v_max = 2.0 m/s as read in DistanceEngine (05 §3.5)')
trk = cfg['tracking']
for hz, rep in ((90, (3, 5, 72)), (30, (2, 3, 24))):
    w = frames_for(float(trk['confirm_window_s']), hz, minimum=3)
    h = max(2, min(w, int(round(w * float(trk['confirm_hits_frac'])))))     # real_time_distance._track_counts
    c = frames_for(float(trk['max_coast_s']), hz, minimum=2)
    R.close(f'tracker M-of-N and coast at {hz} Hz (frames)', [h, w, c], rep, 0, kind='FACT')
sd = trk['self_detection']
for hz, rep in ((90, (60, 15, 45)), (30, (20, 5, 15))):
    got = [frames_for(float(sd['window_s']), hz, minimum=4), frames_for(float(sd['confirm_s']), hz, minimum=2),
           frames_for(float(sd['release_s']), hz, minimum=3)]
    R.close(f'self-detection window / confirm / release at {hz} Hz (frames)', got, rep, 0, kind='FACT')
sys.exit(R.finish())
