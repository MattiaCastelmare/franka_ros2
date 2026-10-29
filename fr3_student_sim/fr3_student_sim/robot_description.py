"""The FR3 URDF that is both simulated by Gazebo and loaded by Pinocchio.

One URDF string serves three readers: Gazebo (``ros_gz_sim create``), the
gz_ros2_control plugin (which builds its KDL gravity model from the
``robot_state_publisher`` copy) and :class:`fr3_student_sim.fr3_model.FR3Model`.
Using the same string everywhere is what makes the students' model match the
simulated plant.

Starting from ``franka_description``'s xacro (``gazebo:=true
gazebo_effort:=true hand:=true``), the changes are the ones worked out for the
CBF Gazebo plant (``franka_simulation/scripts/utils/gz_robot_description.py``),
copied here so this package does not depend on ``franka_simulation``:

* The fingers become FIXED at fully open and their mass is lumped into
  ``fr3_hand``. The plugin's KDL gravity chain runs from the root to
  ``fr3_hand_tcp`` and ignores side branches, so finger mass would otherwise
  be uncompensated. The finger joints also leave ``<ros2_control>``.
* Each arm joint gets its reflected rotor inertia (armature) as a massless disk
  on its child link. Without it joint 7 is too light for 1 kHz torque control
  (it oscillates and runs into its stop).
* A massless ``fr3_fingertip`` frame at the tip of the fingers (the contact point).
* The ``initial_value`` of each arm joint is the requested start pose.
* The plugin gets the controller YAML of this package as extra ``<parameters>``.
* By default (``joint_friction=False``) the ``<dynamics>`` friction/damping of
  the joints is removed, so the plant is exactly the rigid-body model.
* Optionally (``mass_error > 0``) every link's mass and inertia are scaled by a
  random factor in [1 - mass_error, 1 + mass_error]. Used for the GAZEBO copy
  only: the students' model keeps the nominal values, as with a real robot whose
  URDF is not exact.

Pure ElementTree; :func:`process_xacro` needs ``xacro`` and
``franka_description`` installed.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Iterable, Sequence

import numpy as np

ARM_JOINTS = tuple(f'fr3_joint{i}' for i in range(1, 8))
FINGER_JOINTS = ('fr3_finger_joint1', 'fr3_finger_joint2')
FINGER_LINKS = ('fr3_leftfinger', 'fr3_rightfinger')
HAND_LINK = 'fr3_hand'
PLUGIN_FILENAME = 'franka_ign_ros2_control-system'
FINGER_OPEN_M = 0.04
#: [m] the finger pads end this far beyond fr3_hand_tcp along its z axis
#: (finger collision boxes: hand z 0.0584 + 0.0545 = 0.1129, TCP at 0.1034).
FINGERTIP_OFFSET = 0.0095
#: [kg m^2] reflected rotor inertia per joint (MuJoCo Menagerie FR3 values).
ARMATURE = (0.195, 0.195, 0.195, 0.195, 0.074, 0.074, 0.074)


def process_xacro(arm_id: str = 'fr3') -> str:
    """Run franka_description's xacro with the Gazebo effort mappings."""
    import os
    import xacro
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('franka_description'),
                        'robots', arm_id, f'{arm_id}.urdf.xacro')
    return xacro.process_file(path, mappings={
        'arm_id': arm_id, 'hand': 'true', 'ros2_control': 'true', 'gazebo': 'true',
        'gazebo_effort': 'true', 'ee_id': 'franka_hand'}).toxml()


def _floats(s, n=3):
    return np.array([float(v) for v in s.split()]) if s else np.zeros(n)


def _rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _fmt(v: Iterable[float]) -> str:
    return ' '.join(repr(float(x)) for x in v)


def _by_name(root, tag, name):
    for e in root.findall(tag):
        if e.get('name') == name:
            return e
    return None


def _inertial(link):
    """(mass, com, inertia about com) in the link frame, or None."""
    ine = link.find('inertial')
    if ine is None:
        return None
    o = ine.find('origin')
    xyz = _floats(o.get('xyz')) if o is not None else np.zeros(3)
    rpy = _floats(o.get('rpy')) if o is not None else np.zeros(3)
    m = float(ine.find('mass').get('value'))
    i = ine.find('inertia')
    g = {k: float(i.get(k, 0.0)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
    I_loc = np.array([[g['ixx'], g['ixy'], g['ixz']],
                      [g['ixy'], g['iyy'], g['iyz']],
                      [g['ixz'], g['iyz'], g['izz']]])
    R = _rpy(rpy)
    return m, xyz, R @ I_loc @ R.T


def _lump(bodies):
    m = sum(b[0] for b in bodies)
    c = sum(b[0] * b[1] for b in bodies) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in bodies:
        d = ci - c
        I += Ii + mi * (float(d @ d) * np.eye(3) - np.outer(d, d))
    return m, c, I


def _set_inertial(link, m, com, I):
    for old in link.findall('inertial'):
        link.remove(old)
    ine = ET.SubElement(link, 'inertial')
    ET.SubElement(ine, 'origin', rpy='0 0 0', xyz=_fmt(com))
    ET.SubElement(ine, 'mass', value=repr(float(m)))
    ET.SubElement(ine, 'inertia', ixx=repr(I[0, 0]), ixy=repr(I[0, 1]), ixz=repr(I[0, 2]),
                  iyy=repr(I[1, 1]), iyz=repr(I[1, 2]), izz=repr(I[2, 2]))


def _rigid_open_fingers(root):
    hand = _by_name(root, 'link', HAND_LINK)
    bodies = [_inertial(hand)]
    for jname, lname in zip(FINGER_JOINTS, FINGER_LINKS):
        j = _by_name(root, 'joint', jname)
        o = j.find('origin')
        xyz, R = _floats(o.get('xyz')), _rpy(_floats(o.get('rpy')))
        xyz_open = xyz + R @ (_floats(j.find('axis').get('xyz')) * FINGER_OPEN_M)
        o.set('xyz', _fmt(xyz_open))
        j.set('type', 'fixed')
        for tag in ('axis', 'limit', 'mimic', 'dynamics', 'safety_controller'):
            for e in j.findall(tag):
                j.remove(e)
        link = _by_name(root, 'link', lname)
        fi = _inertial(link)
        if fi is not None:
            m, c, I = fi
            bodies.append((m, xyz_open + R @ c, R @ I @ R.T))
            for e in link.findall('inertial'):
                link.remove(e)
    _set_inertial(hand, *_lump(bodies))


def _drop_finger_control(root):
    for rc in root.findall('ros2_control'):
        for j in list(rc.findall('joint')):
            if j.get('name') in FINGER_JOINTS:
                rc.remove(j)
    for t in list(root.findall('transmission')):
        if {j.get('name') for j in t.iter('joint')} & set(FINGER_JOINTS):
            root.remove(t)


def _add_armature(root, armature):
    for k, jn in enumerate(ARM_JOINTS):
        j = _by_name(root, 'joint', jn)
        link = _by_name(root, 'link', j.find('child').get('link'))
        m, c, I = _inertial(link)
        z = _floats(j.find('axis').get('xyz'))
        z = z / np.linalg.norm(z)
        disk = armature[k] * (0.5 * (np.eye(3) - np.outer(z, z)) + np.outer(z, z))
        _set_inertial(link, *_lump([(m, c, I), (0.0, c, disk)]))


def _set_initial_q(root, q0):
    if len(q0) != 7:
        raise ValueError(f'initial_q needs 7 values, got {len(q0)}')
    for rc in root.findall('ros2_control'):
        for j in rc.findall('joint'):
            if j.get('name') not in ARM_JOINTS:
                continue
            for si in j.findall('state_interface'):
                if si.get('name') != 'position':
                    continue
                p = si.find("param[@name='initial_value']")
                if p is None:
                    p = ET.SubElement(si, 'param', name='initial_value')
                p.text = repr(float(q0[ARM_JOINTS.index(j.get('name'))]))


def _add_fingertip(root):
    """Massless frame ``fr3_fingertip`` at the very tip of the fingers: the point
    that touches first when the tool points at a surface."""
    ET.SubElement(root, 'link', name='fr3_fingertip')
    j = ET.SubElement(root, 'joint', name='fr3_fingertip_joint', type='fixed')
    ET.SubElement(j, 'origin', rpy='0 0 0', xyz=f'0 0 {FINGERTIP_OFFSET}')
    ET.SubElement(j, 'parent', link='fr3_hand_tcp')
    ET.SubElement(j, 'child', link='fr3_fingertip')


def _strip_joint_dynamics(root):
    for jn in ARM_JOINTS:
        for e in _by_name(root, 'joint', jn).findall('dynamics'):
            e.set('damping', '0.0')
            e.set('friction', '0.0')


def _add_plugin_params(root, param_files):
    plugins = [p for g in root.findall('gazebo') for p in g.findall('plugin')
               if p.get('filename') == PLUGIN_FILENAME]
    if len(plugins) != 1:
        raise ValueError(f'expected one {PLUGIN_FILENAME} plugin, found {len(plugins)}')
    for path in param_files:
        ET.SubElement(plugins[0], 'parameters').text = str(path)


def _perturb_masses(root, mass_error, seed):
    rng = np.random.default_rng(seed)
    for link in root.findall('link'):
        ine = _inertial(link)
        if ine is None or ine[0] <= 0.0:
            continue
        m, c, I = ine
        s = 1.0 + rng.uniform(-mass_error, mass_error)
        _set_inertial(link, s * m, c, s * I)


def build_urdf(urdf_xml: str, *, initial_q: Sequence[float] = (0, 0, 0, -1.5, 0, 1.5, 0.785),
               param_files: Sequence[str] = (), joint_friction: bool = False,
               mass_error: float = 0.0, seed: int = 0, armature=ARMATURE) -> str:
    """The simulated FR3 URDF (rooted at ``world``) from the xacro output."""
    root = ET.fromstring(urdf_xml)
    if mass_error > 0.0:
        _perturb_masses(root, mass_error, seed)      # before the armature: rotors stay exact
    if armature is not None:
        _add_armature(root, armature)
    _rigid_open_fingers(root)
    _drop_finger_control(root)
    _add_fingertip(root)
    _set_initial_q(root, initial_q)
    if not joint_friction:
        _strip_joint_dynamics(root)
    _add_plugin_params(root, param_files)
    return ET.tostring(root, encoding='unicode')


def kdl_tip_link(urdf_xml: str) -> str:
    """The tip the plugin's ``findTipLink`` picks: the last leaf in BFS order
    (children sorted by joint name, as urdfdom stores them)."""
    root = ET.fromstring(urdf_xml)
    children, has_parent = {}, set()
    for j in root.findall('joint'):
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        children.setdefault(p, []).append((j.get('name'), c))
        has_parent.add(c)
    queue = [lk.get('name') for lk in root.findall('link') if lk.get('name') not in has_parent]
    tip = None
    while queue:
        cur = queue.pop(0)
        kids = sorted(children.get(cur, []))
        if not kids:
            tip = cur
        queue.extend(c for _, c in kids)
    return tip
