"""The Gazebo world: physics rate, gravity, camera pose, scene variants, obstacles."""
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from utils import gz_world as W
from utils.gz_robot_description import rpy_to_matrix

PKG = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..')
WORLD = os.path.join(PKG, 'worlds', 'fr3_cbf_lab.sdf')
EXT_SIM = os.path.join(PKG, 'config', 'sim', 'camera_extrinsics_sim.yaml')
WORLD_DIAG = os.path.join(PKG, 'worlds', 'fr3_cbf_lab_diag.sdf')
EXT_DIAG = os.path.join(PKG, 'config', 'sim', 'camera_extrinsics_diag.yaml')
INTR_DIAG = os.path.join(PKG, 'config', 'sim', 'depth_intrinsics_diag.yaml')


@pytest.fixture(scope='module')
def sdf():
    with open(WORLD) as f:
        return f.read()


@pytest.fixture(scope='module')
def sdf_diag():
    with open(WORLD_DIAG) as f:
        return f.read()


def _pose(model):
    return np.array([float(v) for v in model.find('pose').text.split()])


def test_physics_is_1khz_with_the_gravity_kdl_compensates(sdf):
    w = ET.fromstring(sdf).find('world')
    ph = w.find('physics')
    assert float(ph.find('max_step_size').text) == 0.001
    assert float(ph.find('real_time_update_rate').text) == 1000
    # ign_system.cpp:644 compensates with exactly 9.8 (plan H3)
    assert [float(v) for v in w.find('gravity').text.split()] == [0.0, 0.0, -9.8]


def test_shipped_camera_pose_is_the_one_derived_from_the_sim_extrinsics(sdf):
    cam = W._model(ET.fromstring(sdf), W.CAMERA_MODEL)
    xyz, rpy = W.camera_link_pose(*W.load_optical_extrinsics(EXT_SIM))
    np.testing.assert_allclose(_pose(cam), np.r_[xyz, rpy], atol=1e-8)


def test_camera_link_pose_reproduces_the_optical_frame():
    """Round trip: the link pose, composed with the optical->link rotation,
    gives back exactly the calibrated base -> optical rotation (H8)."""
    R_opt, t = W.load_optical_extrinsics(EXT_SIM)
    xyz, rpy = W.camera_link_pose(R_opt, t)
    R_back = rpy_to_matrix(rpy) @ W.R_LINK_FROM_OPTICAL
    # The calibrated quaternion is unit-norm to its 6 printed decimals only, so
    # R_opt (built exactly like utils.config.load_extrinsics, unnormalised) is
    # orthogonal to ~1e-6 and no rotation reproduces it closer than that.
    np.testing.assert_allclose(R_back, R_opt, atol=5e-6)
    np.testing.assert_allclose(xyz, t)
    # The optical axis (z) is the link's +x.
    np.testing.assert_allclose(rpy_to_matrix(rpy)[:, 0], R_opt[:, 2], atol=5e-6)


def test_rpy_inversion_on_random_rotations():
    rng = np.random.default_rng(1)
    for _ in range(200):
        rpy = rng.uniform([-np.pi, -1.5, -np.pi], [np.pi, 1.5, np.pi])
        R = rpy_to_matrix(rpy)
        np.testing.assert_allclose(rpy_to_matrix(W.matrix_to_rpy(R)), R, atol=1e-9)


def test_depth_sensor_is_the_d455_depth_stream(sdf):
    s = W._model(ET.fromstring(sdf), W.CAMERA_MODEL).find(f"link/sensor[@name='{W.DEPTH_SENSOR}']")
    assert s.get('type') == 'depth_camera'
    assert float(s.find('update_rate').text) == 90
    c = s.find('camera')
    assert (int(c.find('image/width').text), int(c.find('image/height').text)) == (848, 480)
    assert c.find('image/format').text == 'R_FLOAT32'


def test_configure_world_rewrites_pose_fov_rate_and_scene(sdf, tmp_path):
    ext = tmp_path / 'ext.yaml'
    ext.write_text('translation: {x: 1.0, y: 2.0, z: 3.0}\n'
                   'rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}\n')
    out = W.configure_world(sdf, extrinsics_path=str(ext),
                            intrinsics=dict(width=640, height=480, fx=400.0, fy=400.0,
                                            cx=320.0, cy=240.0),
                            depth_rate_hz=30, scene='bare',
                            extra_models=[W.sphere_model('ball', 0.05, [0.5, 0, 0.4])])
    root = ET.fromstring(out)
    cam = W._model(root, W.CAMERA_MODEL)
    np.testing.assert_allclose(_pose(cam)[:3], [1.0, 2.0, 3.0])
    s = cam.find("link/sensor[@name='depth']")
    assert float(s.find('camera/horizontal_fov').text) == pytest.approx(2 * np.arctan(320 / 400.0))
    assert float(s.find('update_rate').text) == 30
    names = {m.get('name') for m in root.iter('model')}
    assert not names & set(W.TABLE_MODELS) and 'ball' in names and 'floor' in names


def test_rig_scene_keeps_the_measured_tables(sdf):
    names = {m.get('name') for m in ET.fromstring(W.configure_world(sdf)).iter('model')}
    assert set(W.TABLE_MODELS) <= names
    with pytest.raises(ValueError):
        W.configure_world(sdf, scene='kitchen')


def test_obstacle_spheres_cannot_touch_the_arm():
    m = ET.fromstring(W.sphere_model('o', 0.06, [0, 0, 0], static=False))
    assert m.find('.//collision') is None
    assert m.find('link/gravity').text == 'false'
    assert float(m.find('.//sphere/radius').text) == 0.06


def test_hfov_from_fx_inverts_the_pinhole():
    W_px, fx = 848, 428.8443603515625
    h = W.hfov_from_fx(W_px, fx)
    assert (W_px / 2) / np.tan(h / 2) == pytest.approx(fx, rel=1e-12)


def test_scenario_obstacles_block_carries_every_law_parameter():
    obs = [dict(name='a', radius=0.05, law='linear', p0=[1, 2, 3], v0=[0, -3, 0]),
           dict(name='b', radius=0.08, law='pursuit', p0=[0, 0.5, 0.5], speed=0.3, park=False),
           dict(name='c', radius=0.06, law='oscillate', p0=[0.4, 0, 0.45], axis=[1, 0, 0],
                amplitude=0.9, period=4.0)]
    pl = ET.fromstring(W.scenario_obstacles_plugin(obs))
    assert pl.get('filename') == 'franka_scenario_obstacles'
    rows = pl.findall('obstacle')
    assert [r.find('model').text for r in rows] == ['a', 'b', 'c']
    assert rows[0].find('v0').text.split() == ['0', '-3', '0']
    assert rows[1].find('law').text == 'pursuit' and float(rows[1].find('speed').text) == 0.3
    assert rows[1].find('park').text == 'false'
    assert rows[2].find('law').text == 'oscillate'
    assert rows[2].find('axis').text.split() == ['1', '0', '0']
    assert float(rows[2].find('amplitude').text) == 0.9
    assert float(rows[2].find('period').text) == 4.0
    models = [ET.fromstring(m) for m in W.obstacle_models(obs)]
    # parked obstacles wait out of view, unparked ones start at p0
    assert [float(v) for v in models[0].find('pose').text.split()[:3]] == list(W.PARK_AT)
    assert [float(v) for v in models[1].find('pose').text.split()[:3]] == [0, 0.5, 0.5]
    assert all(m.find('.//collision') is None for m in models)


# ── the diag lab (worlds/fr3_cbf_lab_diag.sdf) ──────────────────────────────

def test_diag_world_has_the_same_physics_and_gravity(sdf_diag):
    """It is the same plant with different furniture: H3 and H4 still hold."""
    w = ET.fromstring(sdf_diag).find('world')
    assert float(w.find('physics/max_step_size').text) == 0.001
    assert float(w.find('physics/real_time_update_rate').text) == 1000
    assert [float(v) for v in w.find('gravity').text.split()] == [0.0, 0.0, -9.8]


def test_diag_camera_pose_is_derived_from_its_own_extrinsics(sdf_diag):
    cam = W._model(ET.fromstring(sdf_diag), W.CAMERA_MODEL)
    xyz, rpy = W.camera_link_pose(*W.load_optical_extrinsics(EXT_DIAG))
    np.testing.assert_allclose(_pose(cam), np.r_[xyz, rpy], atol=1e-8)


def test_diag_camera_is_the_one_the_bag_recorded():
    """The pose is base -> camera_depth_optical_frame of
    datasets/experiments_irim2.zip /tf_static: base -> camera_link
    [1.993009, 0.032024, 1.080394] q [-0.157767, -0.007306, 0.98732, 0.016008],
    composed with the RealSense optical rotation. Re-derive it from the bag
    before changing these numbers."""
    R, t = W.load_optical_extrinsics(EXT_DIAG)
    np.testing.assert_allclose(t, [1.993009, 0.032024, 1.080394], atol=1e-9)
    # It is NOT the rig calibration: the camera was moved between the two
    # sessions (24 cm in x, 6 cm in y). A sim file, so the rig file is untouched.
    _, t_rig = W.load_optical_extrinsics(EXT_SIM)
    assert np.linalg.norm(t - t_rig) > 0.2


def test_diag_depth_sensor_matches_its_own_intrinsics(sdf_diag):
    I = W.load_depth_intrinsics(INTR_DIAG)
    s = W._model(ET.fromstring(sdf_diag),
                 W.CAMERA_MODEL).find(f"link/sensor[@name='{W.DEPTH_SENSOR}']")
    assert s.get('type') == 'depth_camera'
    assert float(s.find('update_rate').text) == 90
    c = s.find('camera')
    assert (int(c.find('image/width').text), int(c.find('image/height').text)) == \
        (I['width'], I['height'])
    assert float(c.find('horizontal_fov').text) == pytest.approx(
        W.hfov_from_fx(I['width'], I['fx']), abs=1e-8)


def test_diag_surfaces_are_the_measured_ones(sdf_diag):
    """The numbers measured from the bag's depth frames: the floor 0.904 m below
    the robot's mounting surface, its table top at z = 0, the +y desk bank at
    z = -0.139, and the room yawed -0.127013 rad about z (the back-wall fit)."""
    root = ET.fromstring(sdf_diag)
    assert _pose(W._model(root, 'floor'))[2] == pytest.approx(-0.904)

    def top(name):
        m = W._model(root, name)
        return _pose(m)[2] + float(m.find('link/visual/geometry/box/size').text.split()[2]) / 2

    assert top('diag_robot_table') == pytest.approx(0.0, abs=1e-9)
    assert top('diag_desk_right') == pytest.approx(-0.139, abs=1e-9)
    assert top('diag_desk_corner') == pytest.approx(-0.139, abs=1e-9)
    assert top('diag_cell_bench') == pytest.approx(-0.022, abs=1e-9)
    # the robot's own table is square to the robot; the room is not
    assert _pose(W._model(root, 'diag_robot_table'))[5] == 0.0
    for name in ('diag_wall_back', 'diag_desk_right', 'diag_desk_corner',
                 'diag_cell_bench'):
        assert _pose(W._model(root, name))[5] == pytest.approx(-0.127013)


def test_diag_laptop_and_monitor_stand_on_the_right_desk(sdf_diag):
    """The two things that stand up off the +y desk, fitted to the same bag's
    depth returns: the open laptop's lid (top 0.236 m above the desk top,
    0.346 m wide, 16.1 deg off vertical) and the monitor's panel (top 0.373 m,
    0.455 m wide, vertical). Everything they are made of rests on the desk top
    at z = -0.139 and nothing of either pokes through it."""
    root = ET.fromstring(sdf_diag)
    desk = -0.139

    def part(model, name):
        m = W._model(root, model)
        g = m.find(f"link/collision[@name='{name}']")
        pose = np.array([float(v) for v in g.find('pose').text.split()])
        size = np.array([float(v) for v in g.find('geometry/box/size').text.split()])
        return pose, size

    def corners(pose, size):
        r, p_, y = pose[3:]
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        c = np.array([[sx, sy, sz] for sx in (-.5, .5) for sy in (-.5, .5) for sz in (-.5, .5)])
        return (c * size) @ (Rz @ Rx).T + pose[:3]

    lid = corners(*part('diag_laptop', 'lid'))
    base = corners(*part('diag_laptop', 'base'))
    panel = corners(*part('diag_desk_monitor', 'panel'))
    foot = corners(*part('diag_desk_monitor', 'foot'))

    assert lid[:, 2].max() - desk == pytest.approx(0.236, abs=0.005)
    assert panel[:, 2].max() - desk == pytest.approx(0.373, abs=0.005)
    assert part('diag_laptop', 'lid')[1][0] == pytest.approx(0.346, abs=0.01)
    assert part('diag_desk_monitor', 'panel')[1][0] == pytest.approx(0.455, abs=0.01)
    # the lid leans back off vertical, the panel does not
    assert np.degrees(part('diag_laptop', 'lid')[0][3]) == pytest.approx(16.1, abs=0.2)
    assert part('diag_desk_monitor', 'panel')[0][3] == 0.0
    # both sit ON the desk, on the robot's right (+y), and nothing dips below it
    for pts in (lid, base, panel, foot):
        assert pts[:, 2].min() >= desk - 1e-6
        assert pts[:, 1].min() > 0.3
    assert base[:, 2].min() == pytest.approx(desk, abs=1e-6)
    assert foot[:, 2].min() == pytest.approx(desk, abs=1e-6)


def test_diag_laptop_and_monitor_are_scene_not_obstacles(sdf_diag):
    """They are furniture: static, inside the desk's footprint, and dropped by
    `bare` like the rest of it (so a robot-only run is unchanged)."""
    root = ET.fromstring(sdf_diag)
    deskm = W._model(root, 'diag_desk_right')
    dpose = np.array([float(v) for v in deskm.find('pose').text.split()])
    dsize = np.array([float(v) for v in deskm.find('link/collision/geometry/box/size').text.split()])
    for name in ('diag_laptop', 'diag_desk_monitor'):
        m = W._model(root, name)
        assert m.find('static').text == 'true'
        for g in m.findall('link/collision'):
            xy = np.array([float(v) for v in g.find('pose').text.split()[:2]]) - dpose[:2]
            yaw = -dpose[5]
            c, s_ = np.cos(yaw), np.sin(yaw)
            local = np.array([c * xy[0] - s_ * xy[1], s_ * xy[0] + c * xy[1]])
            assert (np.abs(local) < dsize[:2] / 2).all(), (name, g.get('name'), local)
    bare = {m.get('name') for m in
            ET.fromstring(W.configure_world(sdf_diag, scene='bare')).iter('model')}
    assert 'diag_laptop' not in bare and 'diag_desk_monitor' not in bare


def test_diag_world_ships_without_obstacles(sdf_diag):
    """Obstacles come from the scenario, never from the scene."""
    root = ET.fromstring(sdf_diag)
    assert root.find(".//plugin[@filename='franka_scenario_obstacles']") is None
    assert not [m for m in root.iter('model') if m.find('.//sphere') is not None]


def test_bare_strips_the_furniture_of_either_world(sdf, sdf_diag):
    """H9 is a property of the variant, not of one template: whatever furniture
    a world ships, `bare` leaves the floor, the camera and its mount."""
    for xml in (sdf, sdf_diag):
        names = {m.get('name') for m in
                 ET.fromstring(W.configure_world(xml, scene='bare')).iter('model')}
        assert names == set(W.BARE_KEEP)
