"""Shared helpers for the Stage 2 verification scripts.

Everything here is read-only. Paths are derived from this file's location, so the scripts work
from any checkout and inside the `franka_ros2` container (where the repository is mounted at
/ros2_ws/src) without hard-coded absolute paths.

Result vocabulary (printed by `Report`), used to keep three kinds of evidence apart:
  NUM    numerical verification of a mathematical identity (for example an analytic Jacobian
         against a central finite difference). PASS means the identity holds to the stated bound.
  REPRO  reproduction of a number reported in chapter 04 or 05. PASS means the rerun agrees with the
         reported value within the stated tolerance.
  FACT   a code or configuration fact evaluated numerically (for example "these two control points
         coincide"). PASS means the fact holds.
None of these is a runtime validation: nothing here runs ROS nodes, Gazebo or the robot.
"""
from __future__ import annotations

import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
PKG_SRC = os.path.join(REPO, 'franka_experiments')
CONFIG = os.path.join(PKG_SRC, 'config')
DESCRIPTION = os.path.join(REPO, 'franka_description')
HAND_STL = os.path.join(DESCRIPTION, 'meshes', 'robot_ee', 'franka_hand_white', 'collision',
                        'hand.stl')

# q_home used throughout chapters 04 and 05.
Q_HOME = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])


def use_source_package() -> str:
    """Import `franka_experiments` from the repository source, never from an installed copy.

    The container's installed copy was stale at Stage 2 (F-2-2), so every script that imports the
    package goes through this guard. Returns the directory the package was imported from.
    """
    if PKG_SRC not in sys.path:
        sys.path.insert(0, PKG_SRC)
    import franka_experiments  # noqa: E402
    origin = os.path.dirname(os.path.abspath(franka_experiments.__file__))
    expected = os.path.join(PKG_SRC, 'franka_experiments')
    if os.path.realpath(origin) != os.path.realpath(expected):
        raise RuntimeError(f'franka_experiments imported from {origin}, expected the source tree '
                           f'{expected}. Run without a conflicting installed copy first on '
                           f'sys.path.')
    return origin


def load_yaml(path: str):
    """First YAML document of `path` (the CameraInfo dumps contain several)."""
    import yaml
    with open(path) as f:
        return next(yaml.safe_load_all(f))


def load_yaml_text(text: str):
    import yaml
    return next(yaml.safe_load_all(text))


def quat_to_R(x, y, z, w) -> np.ndarray:
    """Rotation matrix of a (normalised) unit quaternion (x, y, z, w)."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_from_extrinsics(d: dict) -> np.ndarray:
    """4x4 transform from a `{translation: {x,y,z}, rotation: {x,y,z,w}}` dict."""
    T = np.eye(4)
    T[:3, :3] = quat_to_R(*[float(d['rotation'][k]) for k in 'xyzw'])
    T[:3, 3] = [float(d['translation'][k]) for k in 'xyz']
    return T


def rotation_angle_deg(R: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))


def git(*args) -> str:
    """Read-only git query in the repository. Returns stdout, or '' when git is unavailable."""
    try:
        r = subprocess.run(['git', '-C', REPO, *args], capture_output=True, text=True, check=False)
    except OSError:
        return ''
    return r.stdout if r.returncode == 0 else ''


def read_stl_triangles(path: str) -> np.ndarray:
    """(n, 3, 3) triangle vertices of a binary or ASCII STL, with numpy only."""
    import struct
    with open(path, 'rb') as f:
        b = f.read()
    if b[:5] == b'solid' and b'facet' in b[:512]:
        v = [list(map(float, ln.split()[1:4])) for ln in b.decode(errors='replace').splitlines()
             if ln.strip().startswith('vertex')]
        return np.asarray(v, dtype=np.float64).reshape(-1, 3, 3)
    n = struct.unpack('<I', b[80:84])[0]
    rec = np.frombuffer(b[84:84 + 50 * n],
                        dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return rec['v'].astype(np.float64).reshape(-1, 3, 3)


def sample_surface(tri: np.ndarray, n: int, seed: int = 0) -> np.ndarray:
    """Area-weighted uniform surface samples (the distribution `trimesh.sample` draws from)."""
    rng = np.random.default_rng(seed)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    idx = rng.choice(len(tri), size=n, p=area / area.sum())
    r1, r2 = rng.random(n), rng.random(n)
    flip = r1 + r2 > 1.0
    r1[flip], r2[flip] = 1.0 - r1[flip], 1.0 - r2[flip]
    return a[idx] + r1[:, None] * (b[idx] - a[idx]) + r2[:, None] * (c[idx] - a[idx])


class Report:
    """Collects PASS/FAIL lines; `finish()` returns the process exit code (0 = all passed)."""

    def __init__(self, title: str):
        self.title = title
        self.n_pass = 0
        self.n_fail = 0
        print(f'=== {title} ===')
        print(f'repository: {REPO}')

    def _emit(self, ok: bool, kind: str, name: str, detail: str) -> bool:
        if ok:
            self.n_pass += 1
        else:
            self.n_fail += 1
        print(f'[{"PASS" if ok else "FAIL"}] {kind:5s} {name}: {detail}')
        return ok

    def below(self, name: str, value: float, bound: float, kind: str = 'NUM', note: str = ''):
        ok = bool(np.isfinite(value) and value <= bound)
        return self._emit(ok, kind, name, f'{value:.3e} <= {bound:.1e}' + (f'  ({note})' if note else ''))

    def close(self, name: str, value, expected, tol: float, kind: str = 'REPRO', note: str = ''):
        v = np.asarray(value, dtype=np.float64)
        e = np.asarray(expected, dtype=np.float64)
        err = float(np.max(np.abs(v - e))) if v.size else float('inf')
        ok = bool(np.isfinite(err) and err <= tol)
        fmt = lambda a: np.array2string(np.round(np.atleast_1d(a), 6), separator=', ') if np.ndim(a) else f'{float(a):.6g}'
        return self._emit(ok, kind, name, f'observed {fmt(v)} vs reported {fmt(e)} (|diff| {err:.2e} <= {tol:.1e})'
                          + (f'  ({note})' if note else ''))

    def fact(self, name: str, ok: bool, detail: str = '', kind: str = 'FACT'):
        return self._emit(bool(ok), kind, name, detail)

    def info(self, text: str):
        print(f'[INFO] {text}')

    def finish(self) -> int:
        print(f'--- {self.title}: {self.n_pass} passed, {self.n_fail} failed ---')
        return 0 if self.n_fail == 0 else 1
