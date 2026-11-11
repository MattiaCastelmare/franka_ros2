#!/usr/bin/env python3
"""Camera calibration consistency (F-2-7, Q-1-3) and the camera geometry of chapter 05 §2.

`camera_link_extrinsics.yaml` says in its header that it was derived from
`camera_extrinsics.yaml` by composing the inverse of a "D435i typical"
camera_link -> camera_color_optical_frame offset, t = (0, 0.015, 0), q = (-0.5, 0.5, -0.5, 0.5).
This script recomposes T_base_optical' = T_base_link · T_link_optical and compares it with
  (1) the HEAD `camera_extrinsics.yaml`, and
  (2) every historical version of `camera_extrinsics.yaml` in git (read-only `git log` / `git show`),
so it finds which calibration the TF file was derived from.
It also reproduces the camera position, optical axis and elevation quoted in 05 §2.

Needs: Python 3, numpy, PyYAML, and git for part (2) (skipped if unavailable). Runs on the host.
"""
import os
import sys

sys.dont_write_bytecode = True   # read-only: never write __pycache__ into the repository
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common as C  # noqa: E402
import numpy as np  # noqa: E402

R = C.Report('check_camera_extrinsics (chapter 05 §2, §5.2; F-2-7)')
CE = 'franka_experiments/config/camera_extrinsics.yaml'
CL = 'franka_experiments/config/camera_link_extrinsics.yaml'
T_ce = C.pose_from_extrinsics(C.load_yaml(os.path.join(C.REPO, CE)))
T_cl = C.pose_from_extrinsics(C.load_yaml(os.path.join(C.REPO, CL)))
L = np.eye(4)
L[:3, :3] = C.quat_to_R(-0.5, 0.5, -0.5, 0.5)
L[:3, 3] = [0, 0.015, 0]
T_re = T_cl @ L          # base -> optical, as reconstructed from the TF file


def mismatch(T):
    return (float(np.linalg.norm(T_re[:3, 3] - T[:3, 3])),
            C.rotation_angle_deg(T[:3, :3].T @ T_re[:3, :3]))


# ── camera geometry (05 §2) ────────────────────────────────────────────────────────────────
R.close('camera position t_b [m]', T_ce[:3, 3], [1.754113, 0.154097, 1.059057], 1e-6)
R.close('optical axis z_C in base', T_ce[:3, 2], [-0.954, -0.053, -0.295], 6e-4)
R.close('optical axis elevation [deg]', np.degrees(np.arcsin(T_ce[2, 2])), -17.2, 0.06)
R.close('camera distance to base origin [m]', np.linalg.norm(T_ce[:3, 3]), 2.055, 6e-4)

# ── (1) HEAD vs the TF file ─────────────────────────────────────────────────────────────────
dt, dr = mismatch(T_ce)
R.close('HEAD camera_extrinsics vs recomposed camera_link_extrinsics: |dt| [m]', dt, 0.276, 6e-4)
R.close('HEAD camera_extrinsics vs recomposed camera_link_extrinsics: angle [deg]', dr, 5.14, 6e-3)

# ── (2) git history ─────────────────────────────────────────────────────────────────────────
log_ce = C.git('log', '--format=%h %ad %s', '--date=short', '--', CE).splitlines()
log_cl = C.git('log', '--format=%h %ad %s', '--date=short', '--', CL).splitlines()
if not log_ce:
    R.info('git history unavailable: part (2) skipped')
    sys.exit(R.finish())
R.info(f'{CL} last changed by: {log_cl[0] if log_cl else "?"}')
matches = []
print('  version of camera_extrinsics.yaml            |dt| [m]   angle [deg]')
for line in log_ce:
    sha = line.split()[0]
    text = C.git('show', f'{sha}:{CE}')
    if not text.strip():
        continue
    try:
        T = C.pose_from_extrinsics(C.load_yaml_text(text))
    except Exception as exc:  # an old format without the expected keys
        print(f'  {line[:44]:44s}  unparsable ({type(exc).__name__})')
        continue
    a, b = mismatch(T)
    print(f'  {line[:44]:44s}  {a:8.4f}   {b:8.3f}')
    if a < 1e-4 and b < 1e-3:
        matches.append(line)
R.fact('exactly one historical calibration matches the TF file', len(matches) == 1,
       '; '.join(matches) or 'none')
if matches and log_cl:
    sha_match = matches[0].split()[0]
    sha_cl = log_cl[0].split()[0]
    newer = C.git('rev-list', f'{sha_match}..HEAD', '--', CE).split()
    R.info(f'calibrations committed after the matching one: {len(newer)}')
    # Which camera_extrinsics.yaml content was current when the TF file was last written?
    at_cl = C.git('show', f'{sha_cl}:{CE}')
    if at_cl.strip():
        a, b = mismatch(C.pose_from_extrinsics(C.load_yaml_text(at_cl)))
        R.fact(f'camera_extrinsics.yaml as of {sha_cl} (TF file commit) matches it', a < 1e-4 and b < 1e-3,
               f'|dt| {a:.4f} m, {b:.3f} deg')
sys.exit(R.finish())
