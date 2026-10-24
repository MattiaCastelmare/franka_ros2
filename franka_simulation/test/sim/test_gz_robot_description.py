"""The Gazebo FR3 description: what it changes, and that it changes nothing else.

Pins plan §3.1 (H3, H4, H14-H17) on the real xacro output. Needs xacro +
franka_description (the container has both); Pinocchio for the dynamics checks.
"""
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from utils import gz_robot_description as G

Q0 = [0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70]
EXTRA = '/tmp/some/rt_controllers.yaml'


@pytest.fixture(scope='module')
def raw():
    return G.process_xacro('fr3', hand=True)


@pytest.fixture(scope='module')
def built(raw):
    return G.build_descriptions(raw, initial_q=Q0, namespace='NS_1', extra_param_files=[EXTRA])


def _joints(xml):
    r = ET.fromstring(xml)
    return {j.get('name'): j for j in r.findall('joint')}, {lk.get('name'): lk for lk in r.findall('link')}, r


def _parent_child(j):
    return j.find('parent').get('link'), j.find('child').get('link')


def test_gazebo_copy_is_world_base_link0_and_rsp_copy_is_base_rooted(built):
    gz, rsp = built
    J, L, _ = _joints(gz)
    assert _parent_child(J['world_to_base']) == ('world', 'base')
    assert _parent_child(J['fr3_base_joint']) == ('base', 'fr3_link0')
    assert 'world_joint' not in J
    J, L, _ = _joints(rsp)
    assert 'world' not in L and 'world_to_base' not in J
    assert _parent_child(J['fr3_base_joint']) == ('base', 'fr3_link0')


def test_fingers_are_fixed_fully_open_and_out_of_ros2_control(raw, built):
    gz, _ = built
    J0, _, _ = _joints(raw)
    J, L, root = _joints(gz)
    for jn in G.FINGER_JOINTS:
        assert J[jn].get('type') == 'fixed'
        for tag in ('axis', 'limit', 'mimic', 'dynamics'):
            assert J[jn].find(tag) is None
        o0 = J0[jn].find('origin')
        R = G.rpy_to_matrix(G._floats(o0.get('rpy')))
        axis = G._floats(J0[jn].find('axis').get('xyz'))
        expect = G._floats(o0.get('xyz')) + R @ axis * G.FINGER_OPEN_M
        np.testing.assert_allclose(G._floats(J[jn].find('origin').get('xyz')), expect, atol=1e-12)
    # left at +y, right at -y: they open AWAY from each other
    yl = G._floats(J['fr3_finger_joint1'].find('origin').get('xyz'))[1]
    yr = G._floats(J['fr3_finger_joint2'].find('origin').get('xyz'))[1]
    assert yl == pytest.approx(0.04) and yr == pytest.approx(-0.04)
    rc_joints = {j.get('name') for rc in root.findall('ros2_control') for j in rc.findall('joint')}
    assert rc_joints == set(G.ARM_JOINTS)                     # H15: no finger, no UB index 7
    assert not [t for t in root.findall('transmission')
                if {j.get('name') for j in t.iter('joint')} & set(G.FINGER_JOINTS)]


def test_finger_mass_is_lumped_into_the_hand_and_total_mass_is_unchanged(raw, built):
    def masses(xml):
        return {lk.get('name'): float(lk.find('inertial/mass').get('value'))
                for lk in ET.fromstring(xml).findall('link') if lk.find('inertial') is not None}
    m0, m1 = masses(raw), masses(built[1])
    assert sum(m1.values()) == pytest.approx(sum(m0.values()), abs=1e-12)
    assert m1['fr3_hand'] == pytest.approx(m0['fr3_hand'] + m0['fr3_leftfinger']
                                           + m0['fr3_rightfinger'], abs=1e-12)
    assert 'fr3_leftfinger' not in m1 and 'fr3_rightfinger' not in m1


def test_lumped_model_has_the_same_gravity_as_the_open_hand(raw, built):
    """H3: g(q) of the lumped 7-DoF model == g(q) of the original with fingers at 0.04."""
    pin = pytest.importorskip('pinocchio')
    full = pin.buildModelFromXML(raw)
    lump = pin.buildModelFromXML(built[1])
    assert lump.nq == 7
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = np.asarray(Q0) + rng.uniform(-0.6, 0.6, 7)
        qf = pin.neutral(full)
        for k, jn in enumerate(G.ARM_JOINTS):
            qf[full.joints[full.getJointId(jn)].idx_q] = q[k]
        for jn in G.FINGER_JOINTS:
            qf[full.joints[full.getJointId(jn)].idx_q] = G.FINGER_OPEN_M
        g_full = pin.computeGeneralizedGravity(full, full.createData(), qf)
        g_arm = np.array([g_full[full.joints[full.getJointId(jn)].idx_v] for jn in G.ARM_JOINTS])
        g_lump = pin.computeGeneralizedGravity(lump, lump.createData(), q)
        np.testing.assert_allclose(g_lump, g_arm, atol=1e-9)


def test_kdl_gravity_chain_ends_at_the_tcp_through_the_hand(built):
    """H3: findTipLink's choice. The lumped hand must be ON the chain it builds."""
    _, rsp = built
    assert G.kdl_tip_link(rsp) == 'fr3_hand_tcp'
    J, _, _ = _joints(rsp)
    child_to_parent = {j.find('child').get('link'): j.find('parent').get('link') for j in J.values()}
    chain, cur = [], 'fr3_hand_tcp'
    while cur in child_to_parent:
        chain.append(cur)
        cur = child_to_parent[cur]
    assert cur == 'base' and 'fr3_hand' in chain and 'fr3_link8' in chain


def test_kdl_tip_on_the_untouched_xacro_is_also_the_tcp(raw):
    """Why lumping is needed: on the original URDF the tip is the TCP too, and the
    fingers hang off fr3_hand on side branches the chain does not contain."""
    assert G.kdl_tip_link(raw) == 'fr3_hand_tcp'


def test_initial_values_are_the_requested_pose(built):
    gz, _ = built
    root = ET.fromstring(gz)
    got = {}
    for j in root.find('ros2_control').findall('joint'):
        for si in j.findall('state_interface'):
            if si.get('name') == 'position':
                got[j.get('name')] = float(si.find("param[@name='initial_value']").text)
    assert [got[j] for j in G.ARM_JOINTS] == pytest.approx(Q0, abs=0)


def test_effort_command_interfaces_survive_on_the_seven_arm_joints(built):
    root = ET.fromstring(built[0])
    for j in root.find('ros2_control').findall('joint'):
        assert 'effort' in {c.get('name') for c in j.findall('command_interface')}, j.get('name')


def test_plugin_gets_the_extra_parameters_after_its_own_and_the_namespace(built):
    gz, rsp = built
    for xml in (gz, rsp):
        root = ET.fromstring(xml)
        plugins = [p for g in root.findall('gazebo') for p in g.findall('plugin')
                   if p.get('filename') == G.PLUGIN_FILENAME]
        assert len(plugins) == 1
        params = [e.text for e in plugins[0].findall('parameters')]
        assert params[0].endswith('franka_gazebo_controllers.yaml')   # H4: kept, not edited
        assert params[1:] == [EXTRA]
        assert plugins[0].find('ros/namespace').text == '/NS_1'
        assert plugins[0].find('ros/remapping').text == 'joint_states:=franka/joint_states'


def test_rpy_to_matrix_matches_the_urdf_convention():
    r, p, y = 0.3, -0.4, 1.1
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    np.testing.assert_allclose(G.rpy_to_matrix((r, p, y)), Rz @ Ry @ Rx, atol=1e-15)


def test_bad_inputs_are_refused(raw):
    with pytest.raises(ValueError):
        G.build_descriptions(raw, initial_q=Q0[:6])
    no_hand = G.process_xacro('fr3', hand=False)
    with pytest.raises(ValueError, match='hand'):
        G.build_descriptions(no_hand, initial_q=Q0)


def test_armature_adds_exactly_the_rotor_inertia_on_joint_7_and_leaves_gravity(raw, built):
    """H19: M_77 gains exactly I_a(7); masses, COMs and g(q) are untouched."""
    pin = pytest.importorskip('pinocchio')
    bare, _ = G.build_descriptions(raw, initial_q=Q0, armature=None)
    m0 = pin.buildModelFromXML(ET.tostring(_strip(bare), encoding='unicode'))
    m1 = pin.buildModelFromXML(built[1])
    rng = np.random.default_rng(3)
    for _ in range(10):
        q = np.asarray(Q0) + rng.uniform(-0.5, 0.5, 7)
        M0 = pin.crba(m0, m0.createData(), q)
        M1 = pin.crba(m1, m1.createData(), q)
        assert M1[6, 6] - M0[6, 6] == pytest.approx(G.ARMATURE[6], abs=1e-12)
        # every diagonal term grows by at least its own rotor inertia
        assert np.all(np.diag(M1) - np.diag(M0) >= np.asarray(G.ARMATURE) - 1e-12)
        np.testing.assert_allclose(pin.computeGeneralizedGravity(m1, m1.createData(), q),
                                   pin.computeGeneralizedGravity(m0, m0.createData(), q),
                                   atol=1e-12)


def test_link_inertias_stay_physical(built):
    """Triangle inequality on every principal moment (DART refuses otherwise)."""
    for lk in ET.fromstring(built[0]).findall('link'):
        fi = G._inertial(lk)
        if fi is None or fi[0] == 0:
            continue
        ev = np.linalg.eigvalsh(fi[2])
        assert ev.min() > 0
        assert ev[0] + ev[1] >= ev[2] - 1e-12, lk.get('name')


def _strip(gz_xml):
    root = ET.fromstring(gz_xml)
    G._strip_world(root)
    return root
