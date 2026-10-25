"""The Gazebo worlds of the CBF experiments, generated from the ``worlds/`` templates.

The shipped SDFs are a lab at its default: 1 kHz physics, g = -9.8, the floor,
the surfaces measured from a real depth frame, and the D455 at the calibrated
pose. ``worlds/fr3_cbf_lab.sdf`` is the irim1 rig, ``fr3_cbf_lab_diag.sdf``
the DIAG robotics lab; ``simexp.sim_config.SCENES`` says which ``scene`` uses
which.
This module edits a copy for one run, so the camera pose and the obstacles can
never disagree with the files they come from:

* the camera ``<pose>`` is recomputed from an extrinsics YAML (H8) holding the
  pose of the frame the depth image is rendered in — labelled
  ``camera_color_optical_frame``, the name the stack publishes. A Gazebo camera
  looks along +x of its link and an optical frame along +z, so
  ``R_link = R_optical · R_ol``;
* the depth sensor gets the D455 depth stream geometry (H8): resolution,
  update rate and a horizontal FOV computed from fx;
* ``scene='bare'`` drops the furniture, whichever template it came from, and
  keeps the floor, the camera and its mount (robot-only scene, H9);
* obstacles and the ScenarioObstacles system are appended when given.

Pure ElementTree + numpy; no ROS import.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Sequence

import numpy as np
import yaml

#: Columns are the optical axes expressed in the Gazebo camera-link frame:
#: optical x (image right) = -y_link, optical y (image down) = -z_link,
#: optical z (forward) = +x_link.
R_LINK_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0],
                                [-1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])

CAMERA_MODEL = 'd455'
DEPTH_SENSOR = 'depth'
#: The rig template's furniture, kept as a name for the tests that pin it.
TABLE_MODELS = ('rig_table_top', 'rig_side_table')
#: What ``scene='bare'`` keeps. Everything else in the template is furniture,
#: so a new world only has to name its models to inherit the bare variant.
BARE_KEEP = ('floor', CAMERA_MODEL, 'd455_mount')


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Same convention as franka_experiments utils.config.load_extrinsics."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """Inverse of R = Rz(yaw) Ry(pitch) Rx(roll) (SDF / URDF convention)."""
    sp = -R[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) < 1.0 - 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                                    # gimbal lock: put it all in yaw
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    return roll, pitch, yaw


def load_optical_extrinsics(path: str) -> tuple[np.ndarray, np.ndarray]:
    """(R, t) of base -> camera_color_optical_frame from the YAML."""
    with open(path) as f:
        d = yaml.safe_load(f)
    r, t = d['rotation'], d['translation']
    return (quat_to_matrix(r['x'], r['y'], r['z'], r['w']),
            np.array([t['x'], t['y'], t['z']], dtype=float))


def camera_link_pose(R_opt: np.ndarray, t_opt: np.ndarray) -> tuple[np.ndarray, tuple]:
    """Pose (xyz, rpy) of the Gazebo camera LINK whose image is that optical frame."""
    R_link = R_opt @ R_LINK_FROM_OPTICAL.T
    return np.asarray(t_opt, float), matrix_to_rpy(R_link)


def hfov_from_fx(width: int, fx: float) -> float:
    """Horizontal FOV of a pinhole with square pixels: 2·atan(W / 2fx)."""
    return 2.0 * math.atan(0.5 * width / fx)


def load_depth_intrinsics(path: str) -> dict:
    with open(path) as f:
        # A `ros2 topic echo` dump: one CameraInfo document, then a '---'.
        d = next(doc for doc in yaml.safe_load_all(f) if doc)
    k = d['k']
    return dict(width=int(d['width']), height=int(d['height']),
                fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]))


def _fmt(v) -> str:
    return ' '.join(f'{float(x):.9g}' for x in v)


def _model(root: ET.Element, name: str) -> ET.Element:
    for m in root.iter('model'):
        if m.get('name') == name:
            return m
    raise KeyError(f'model {name!r} not in the world')


def configure_world(sdf_xml: str, *, extrinsics_path: str | None = None,
                    intrinsics: dict | None = None, depth_rate_hz: float | None = None,
                    scene: str = 'rig', extra_models: Sequence[str] = (),
                    extra_world_plugins: Sequence[str] = (),
                    depth_topic: str | None = None) -> str:
    """Return the world SDF for one run (see the module docstring)."""
    root = ET.fromstring(sdf_xml)
    world = root.find('world')
    cam = _model(root, CAMERA_MODEL)
    if extrinsics_path:
        xyz, rpy = camera_link_pose(*load_optical_extrinsics(extrinsics_path))
        cam.find('pose').text = _fmt(list(xyz) + list(rpy))
    sensor = cam.find(f"link/sensor[@name='{DEPTH_SENSOR}']")
    if intrinsics:
        c = sensor.find('camera')
        c.find('horizontal_fov').text = f"{hfov_from_fx(intrinsics['width'], intrinsics['fx']):.9g}"
        c.find('image/width').text = str(intrinsics['width'])
        c.find('image/height').text = str(intrinsics['height'])
    if depth_rate_hz:
        sensor.find('update_rate').text = f'{float(depth_rate_hz):g}'
    if depth_topic:
        sensor.find('topic').text = depth_topic
    if scene == 'bare':
        for m in [m for m in world.findall('model') if m.get('name') not in BARE_KEEP]:
            world.remove(m)
    elif scene != 'rig':
        raise ValueError(f"scene must be 'rig' or 'bare', got {scene!r}")
    for xml in extra_world_plugins:
        world.append(ET.fromstring(xml))
    for xml in extra_models:
        world.append(ET.fromstring(xml))
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode')


def sphere_model(name: str, radius: float, xyz: Sequence[float], *,
                 static: bool = True, rgba: str = '0.9 0.3 0.3 1') -> str:
    """A visual-only sphere: seen by the depth camera, collides with nothing.

    No ``<collision>``: obstacles must not touch the arm (contact is measured
    geometrically from truth). Gravity off and a tiny inertia so that, when the
    ScenarioObstacles system moves it with WorldPoseCmd, physics adds nothing.
    """
    return (f'<model name="{name}"><static>{"true" if static else "false"}</static>'
            f'<pose>{_fmt(xyz)} 0 0 0</pose>'
            f'<link name="link"><gravity>false</gravity>'
            f'<inertial><mass>0.01</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy>'
            f'<izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            f'<visual name="visual"><geometry><sphere><radius>{float(radius):.6g}</radius>'
            f'</sphere></geometry><material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>'
            f'</material></visual></link></model>')


#: Where an obstacle waits before its episode starts: below the floor, out of
#: the camera's frustum (the floor is at z = -0.88).
PARK_AT = (0.0, 0.0, -20.0)


def scenario_obstacles_plugin(obstacles, *, start_topic='/sim/episode_start',
                              truth_topic='/sim/truth/obstacle_poses', truth_period=0.001,
                              target_model='fr3', target_link='fr3_link7',
                              target_offset=(0.0, 0.0, 0.107)) -> str:
    """The ScenarioObstacles world-system block for these obstacle specs.

    Each spec: ``name, radius, law (static|linear|ballistic|pursuit), p0`` and,
    per law, ``v0``, ``gravity``, ``speed``; ``park`` (default True). The
    pursuit target is the flange: fr3_link7 + 0.107 m along its z, because
    Gazebo lumps the fixed fr3_link8 into fr3_link7.
    """
    rows = []
    for o in obstacles:
        rows.append(
            '<obstacle>'
            f'<model>{o["name"]}</model><law>{o.get("law", "static")}</law>'
            f'<p0>{_fmt(o["p0"])}</p0><v0>{_fmt(o.get("v0", (0, 0, 0)))}</v0>'
            f'<gravity>{_fmt(o.get("gravity", (0, 0, -9.81)))}</gravity>'
            f'<speed>{float(o.get("speed", 0.0)):.9g}</speed>'
            f'<park>{"true" if o.get("park", True) else "false"}</park>'
            f'<park_at>{_fmt(o.get("park_at", PARK_AT))}</park_at>'
            '</obstacle>')
    return ('<plugin filename="franka_scenario_obstacles" '
            'name="franka_simulation::ScenarioObstacles">'
            f'<start_topic>{start_topic}</start_topic><truth_topic>{truth_topic}</truth_topic>'
            f'<truth_period>{truth_period:.9g}</truth_period>'
            f'<target_model>{target_model}</target_model><target_link>{target_link}</target_link>'
            f'<target_offset>{_fmt(target_offset)}</target_offset>'
            + ''.join(rows) + '</plugin>')


def obstacle_models(obstacles) -> list[str]:
    """Visual-only spheres, initially where the system will hold them before start."""
    out = []
    for o in obstacles:
        at = o['p0'] if not o.get('park', True) else o.get('park_at', PARK_AT)
        out.append(sphere_model(o['name'], o['radius'], at, static=False,
                                rgba=o.get('rgba', '0.9 0.3 0.3 1')))
    return out
