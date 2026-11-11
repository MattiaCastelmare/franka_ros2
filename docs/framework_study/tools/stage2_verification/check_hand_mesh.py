#!/usr/bin/env python3
"""Hand collision mesh: extent (F-2-4) and the missing -45 deg rotation (F-2-9).

`MaskBuilder` poses `hand.stl` with the TF of `fr3_link8` (the YAML key it is listed under),
while in the URDF the mesh belongs to `fr3_hand`, which sits at fr3_link8 with rpy (0, 0, -pi/4)
(verified in the container by check_kinematics_dynamics.py [A] and check_geometry_and_so3.py).
In the fr3_link8 frame the correct vertex is Rz(-pi/4) v; the one the node uses is v.
This script measures |Rz(-pi/4) v - v| over the mesh vertices and over 3000 area-weighted surface
samples (the node draws 3000 samples per link, `meshes.sample_points_per_link`).

Differences from the Stage 2 run, which did not change the mathematical test:
  - the mesh is read with a numpy STL reader instead of trimesh;
  - the 3000 samples are drawn with numpy from the same area-weighted distribution, with a fixed
    seed. The Stage 2 run used trimesh's unseeded sampler, so the sample statistics agree only
    statistically (the vertex statistics are exact).

Needs: Python 3 and numpy. Reads franka_description/ from the repository. Runs on the host.
"""
import os
import sys

sys.dont_write_bytecode = True   # read-only: never write __pycache__ into the repository
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common as C  # noqa: E402
import numpy as np  # noqa: E402

R = C.Report('check_hand_mesh (chapter 05 §3.2, §4.2; F-2-4, F-2-9)')
tri = C.read_stl_triangles(C.HAND_STL)
v = np.unique(tri.reshape(-1, 3), axis=0)
R.info(f'{os.path.relpath(C.HAND_STL, C.REPO)}: {len(tri)} triangles, {len(v)} unique vertices')
R.close('hand.stl bounding box min [m]', v.min(0), [-0.0316, -0.104, -0.0259], 6e-5)
R.close('hand.stl bounding box max [m]', v.max(0), [0.0316, 0.1004, 0.066], 6e-5)
R.info(f'lateral half-extent of the hand: {max(-v[:, 1].min(), v[:, 1].max()):.4f} m '
       f'(the fingertip sphere radius is 0.05 m)')

c, s = np.cos(-np.pi / 4), np.sin(-np.pi / 4)
Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
d = np.linalg.norm(v @ Rz.T - v, axis=1)
R.close('F-2-9 vertex displacement max [m]', d.max(), 0.0796, 6e-5)
R.close('F-2-9 vertex displacement mean [m]', d.mean(), 0.0685, 2e-3,
        note='Stage 2 counted the 102 trimesh vertices; this counts unique STL vertices')
pts = C.sample_surface(tri, 3000, seed=0)
ds = np.linalg.norm(pts @ Rz.T - pts, axis=1)
R.close('F-2-9 displacement over 3000 surface samples, max [m]', ds.max(), 0.0794, 2e-3,
        note='statistical: different random draws from the same distribution')
R.close('F-2-9 displacement over 3000 surface samples, mean [m]', ds.mean(), 0.0442, 3e-3,
        note='statistical')
R.close('F-2-9 gripper long axis in fr3_link8: correct (Rz y_H)', Rz[:, 1], [0.7071, 0.7071, 0], 1e-4,
        kind='FACT', note='the node uses (0, 1, 0)')
R.close('displacement of a point at radius rho: 2 rho sin(22.5 deg), rho = max radial extent',
        2 * np.max(np.hypot(v[:, 0], v[:, 1])) * np.sin(np.pi / 8), d.max(), 1e-9, kind='NUM')
sys.exit(R.finish())
