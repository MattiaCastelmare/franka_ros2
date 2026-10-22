"""The FR3 robot description for the Gazebo plant of the CBF experiments.

``franka_description``'s xacro with ``gazebo:=true gazebo_effort:=true
hand:=true`` is not, by itself, a plant that produces pipeline A's contract
(docs/gz_cbf_experiments_plan.md §3.1). This module turns its output into two
URDF strings, without touching ``franka_description``:

* the **Gazebo** copy, spawned by ``ros_gz_sim create``. Its root is ``world``,
  fixed to ``base``, fixed to ``fr3_link0``. Gazebo needs a ``world`` link to
  hold a model still, and the ``base`` frame is what the stack measures in.
* the **robot_state_publisher** copy, rooted at ``base`` as on hardware. The
  stack's ``world_to_robot_base_tf`` then stays the only publisher of
  ``world -> base``, as with the bringup. The gz_ros2_control plugin also reads
  its URDF from this node (``<ns>/robot_state_publisher``), so the KDL gravity
  model is built from this copy.

The changes, each tied to a hazard:

H14  ``world -> fr3_link0`` becomes ``world -> base -> fr3_link0``, with the
     joint named ``fr3_base_joint`` as in the bringup URDF.
H3/H15  The fingers become FIXED at fully open, and their mass is lumped into
     ``fr3_hand``. Two reasons. The plugin's KDL gravity chain runs from the
     root to the last BFS leaf, ``fr3_hand_tcp``, and leaves out side branches,
     so finger mass would be uncompensated. And every joint with an effort
     interface starts in EFFORT mode, where the plugin reads
     ``joint_efforts[7]`` out of a 7-element array. The finger therefore also
     leaves ``<ros2_control>``.
H16  The ``initial_value`` of each arm joint is the requested start pose.
H19  Each arm joint gets its reflected rotor inertia (armature) as a disk on
     its child link (see :func:`_add_armature`): without it the shipped 1 kHz
     damping is unstable on joint 7.
H4/H17  The plugin gets extra ``<parameters>`` files (the generated
     rt_torque_controller YAML) and a ``<ros>`` block: the namespace, plus the
     ``joint_states -> franka/joint_states`` remap that franka.launch.py
     applies to ros2_control_node.

Pure ElementTree; no ROS import (pytest-reachable). ``process_xacro`` needs
``xacro`` and ``franka_description`` installed.
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
#: Fully open: franka_hand.xacro limits the finger joint to [0, 0.04] m.
FINGER_OPEN_M = 0.04


def process_xacro(arm_id: str = 'fr3', hand: bool = True) -> str:
    """Run franka_description's xacro with the Gazebo effort mappings."""
    import os
    import xacro
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('franka_description'),
                        'robots', arm_id, f'{arm_id}.urdf.xacro')
    return xacro.process_file(path, mappings={
        'arm_id': arm_id, 'hand': 'true' if hand else 'false',
        'ros2_control': 'true', 'gazebo': 'true', 'gazebo_effort': 'true',
        'ee_id': 'franka_hand'}).toxml()


# ── small geometry helpers ───────────────────────────────────────────────────

def _floats(s: str | None, n: int = 3) -> np.ndarray:
    if not s:
        return np.zeros(n)
    return np.array([float(v) for v in s.split()], dtype=float)


def rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw: R = Rz(y) Ry(p) Rx(r)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _fmt(v: Iterable[float]) -> str:
    return ' '.join(repr(float(x)) for x in v)


def _inertial(link: ET.Element):
    """(mass, com (3,), inertia about com (3,3)) in the LINK frame, or None."""
    ine = link.find('inertial')
    if ine is None:
        return None
    o = ine.find('origin')
    xyz = _floats(o.get('xyz')) if o is not None else np.zeros(3)
    rpy = _floats(o.get('rpy')) if o is not None else np.zeros(3)
    m = float(ine.find('mass').get('value'))
    i = ine.find('inertia')
    g = {k: float(i.get(k, 0.0)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
    I_local = np.array([[g['ixx'], g['ixy'], g['ixz']],
                        [g['ixy'], g['iyy'], g['iyz']],
                        [g['ixz'], g['iyz'], g['izz']]])
    R = rpy_to_matrix(rpy)
    return m, xyz, R @ I_local @ R.T


def lump_inertials(bodies):
    """Combine ``[(mass, com, I_com)]`` (same frame) into one (mass, com, I_com)."""
    m = sum(b[0] for b in bodies)
    c = sum(b[0] * b[1] for b in bodies) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in bodies:
        d = ci - c
        I += Ii + mi * (float(d @ d) * np.eye(3) - np.outer(d, d))
    return m, c, I


def _set_inertial(link: ET.Element, m: float, com: np.ndarray, I: np.ndarray) -> None:
    for old in link.findall('inertial'):
        link.remove(old)
    ine = ET.SubElement(link, 'inertial')
    ET.SubElement(ine, 'origin', rpy='0 0 0', xyz=_fmt(com))
    ET.SubElement(ine, 'mass', value=repr(float(m)))
    ET.SubElement(ine, 'inertia', ixx=repr(I[0, 0]), ixy=repr(I[0, 1]), ixz=repr(I[0, 2]),
                  iyy=repr(I[1, 1]), iyz=repr(I[1, 2]), izz=repr(I[2, 2]))


# ── the transformation ───────────────────────────────────────────────────────

def _by_name(root: ET.Element, tag: str, name: str) -> ET.Element | None:
    for e in root.findall(tag):
        if e.get('name') == name:
            return e
    return None


def _rigid_open_fingers(root: ET.Element, opening: float) -> None:
    hand = _by_name(root, 'link', HAND_LINK)
    if hand is None or _inertial(hand) is None:
        raise ValueError(f'no {HAND_LINK} link with an inertial: was the hand loaded?')
    bodies = [_inertial(hand)]
    for jname, lname in zip(FINGER_JOINTS, FINGER_LINKS):
        j = _by_name(root, 'joint', jname)
        if j is None:
            raise ValueError(f'{jname} not in the URDF: was the hand loaded?')
        o = j.find('origin')
        xyz = _floats(o.get('xyz'))
        rpy = _floats(o.get('rpy'))
        R = rpy_to_matrix(rpy)
        axis = _floats(j.find('axis').get('xyz'))
        # Prismatic child frame = origin * Trans(axis * q), axis in the joint frame.
        xyz_open = xyz + R @ (axis * opening)
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
    _set_inertial(hand, *lump_inertials(bodies))


def _drop_finger_control(root: ET.Element) -> None:
    for rc in root.findall('ros2_control'):
        for j in list(rc.findall('joint')):
            if j.get('name') in FINGER_JOINTS:
                rc.remove(j)
    for t in list(root.findall('transmission')):
        names = {j.get('name') for j in t.iter('joint')}
        if names & set(FINGER_JOINTS):
            root.remove(t)


def _set_initial_q(root: ET.Element, q0: Sequence[float]) -> None:
    if len(q0) != len(ARM_JOINTS):
        raise ValueError(f'initial_q needs {len(ARM_JOINTS)} values, got {len(q0)}')
    for rc in root.findall('ros2_control'):
        for j in rc.findall('joint'):
            if j.get('name') not in ARM_JOINTS:
                continue
            k = ARM_JOINTS.index(j.get('name'))
            for si in j.findall('state_interface'):
                if si.get('name') != 'position':
                    continue
                p = si.find("param[@name='initial_value']")
                if p is None:
                    p = ET.SubElement(si, 'param', name='initial_value')
                p.text = repr(float(q0[k]))


def _insert_base(root: ET.Element) -> None:
    wj = _by_name(root, 'joint', 'world_joint')
    if wj is None:
        raise ValueError('no world_joint: was the xacro run with gazebo:=true?')
    wj.set('name', 'fr3_base_joint')
    wj.find('parent').set('link', 'base')
    root.append(ET.Element('link', name='base'))
    j = ET.SubElement(root, 'joint', name='world_to_base', type='fixed')
    ET.SubElement(j, 'origin', rpy='0 0 0', xyz='0 0 0')
    ET.SubElement(j, 'parent', link='world')
    ET.SubElement(j, 'child', link='base')


def _configure_plugin(root: ET.Element, namespace: str, extra_params: Sequence[str],
                      joint_states_remap: str | None) -> None:
    plugins = [p for g in root.findall('gazebo') for p in g.findall('plugin')
               if p.get('filename') == PLUGIN_FILENAME]
    if len(plugins) != 1:
        raise ValueError(f'expected one {PLUGIN_FILENAME} plugin, found {len(plugins)}')
    plugin = plugins[0]
    for path in extra_params:
        ET.SubElement(plugin, 'parameters').text = str(path)
    ros = ET.SubElement(plugin, 'ros')
    ns = '/' + namespace.strip('/') if namespace.strip('/') else '/'
    ET.SubElement(ros, 'namespace').text = ns
    if joint_states_remap:
        ET.SubElement(ros, 'remapping').text = f'joint_states:={joint_states_remap}'


#: [kg·m²] reflected rotor inertia per arm joint, as identified in the MuJoCo
#: Menagerie FR3 that franka_sim trains on (franka_sim/assets/franka_fr3/fr3.xml:
#: joint class default 0.195, joints 5-7 0.074). franka_description's URDF has
#: none, and URDF/SDF cannot express it.
ARMATURE = (0.195, 0.195, 0.195, 0.195, 0.074, 0.074, 0.074)


def _add_armature(root: ET.Element, armature) -> None:
    """Put each joint's rotor inertia on its child link, as a disk about the joint axis.

    Why (plan H19): without it joint 7, the lightest, is 0.0023 kg·m² (measured:
    0.02 N·m -> 8.7 rad/s²). RtTorqueController's 1 kHz damping Kd7 = 5 N·m·s/rad
    gives Kd·T/I = 2.2, past the discrete stability limit of 2. MEASURED in the
    Phase 2 pentagon run: q̇7 flipped sign every tick, the Kd term saturated,
    and j7 ran 4 rad into its stop. On the robot the rotor inertia behind the
    gearbox (~0.07 reflected on the wrist) is what keeps the same gains stable.

    The disk (I_a about the axis, I_a/2 transverse, no mass) is the only
    rotor-like inertia a URDF link can carry. It leaves masses, COMs and
    therefore g(q) exactly as they were, so KDL's gravity compensation stays
    exact. It is an approximation in one respect, stated rather than hidden:
    the transverse part rotates with the child link, not with the stator. It
    therefore adds up to I_a/2 to upstream diagonal terms of M(q), as a real
    rotor does too, from the parent side.
    """
    for k, jn in enumerate(ARM_JOINTS):
        j = _by_name(root, 'joint', jn)
        link = _by_name(root, 'link', j.find('child').get('link'))
        m, c, I = _inertial(link)
        z = _floats(j.find('axis').get('xyz'))
        z = z / np.linalg.norm(z)
        disk = armature[k] * (0.5 * (np.eye(3) - np.outer(z, z)) + np.outer(z, z))
        _set_inertial(link, *lump_inertials([(m, c, I), (0.0, c, disk)]))


def _strip_world(root: ET.Element) -> None:
    for tag, name in (('joint', 'world_to_base'), ('link', 'world')):
        e = _by_name(root, tag, name)
        if e is not None:
            root.remove(e)


def build_descriptions(urdf_xml: str, *, initial_q: Sequence[float], namespace: str = 'NS_1',
                       extra_param_files: Sequence[str] = (),
                       joint_states_remap: str | None = 'franka/joint_states',
                       finger_opening: float = FINGER_OPEN_M,
                       armature=ARMATURE) -> tuple[str, str]:
    """(gazebo_urdf, rsp_urdf) from the xacro output; see the module docstring.

    ``armature=None`` leaves the link inertias as franka_description has them.
    """
    root = ET.fromstring(urdf_xml)
    _insert_base(root)
    if armature is not None:
        _add_armature(root, armature)
    _rigid_open_fingers(root, finger_opening)
    _drop_finger_control(root)
    _set_initial_q(root, initial_q)
    _configure_plugin(root, namespace, extra_param_files, joint_states_remap)
    gazebo = ET.tostring(root, encoding='unicode')
    _strip_world(root)
    rsp = ET.tostring(root, encoding='unicode')
    return gazebo, rsp


def kdl_tip_link(urdf_xml: str) -> str:
    """The tip ``findTipLink`` picks: the LAST leaf in BFS order from the root.

    urdfdom adds a parent's children in joint-NAME order (``joints_`` is a
    std::map), so the BFS here sorts children the same way. Used by the tests
    to pin H3: if this ever stops being ``fr3_hand_tcp``, the KDL gravity chain
    stops covering the hand.
    """
    root = ET.fromstring(urdf_xml)
    children: dict[str, list[tuple[str, str]]] = {}
    has_parent = set()
    for j in root.findall('joint'):
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        children.setdefault(p, []).append((j.get('name'), c))
        has_parent.add(c)
    roots = [lk.get('name') for lk in root.findall('link') if lk.get('name') not in has_parent]
    if len(roots) != 1:
        raise ValueError(f'URDF has {len(roots)} roots: {roots}')
    queue, tip = [roots[0]], None
    while queue:
        cur = queue.pop(0)
        kids = sorted(children.get(cur, []))
        if not kids:
            tip = cur
        queue.extend(c for _, c in kids)
    return tip
