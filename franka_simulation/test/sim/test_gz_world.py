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


@pytest.fixture(scope='module')
def sdf():
    with open(WORLD) as f:
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
           dict(name='b', radius=0.08, law='pursuit', p0=[0, 0.5, 0.5], speed=0.3, park=False)]
    pl = ET.fromstring(W.scenario_obstacles_plugin(obs))
    assert pl.get('filename') == 'franka_scenario_obstacles'
    rows = pl.findall('obstacle')
    assert [r.find('model').text for r in rows] == ['a', 'b']
    assert rows[0].find('v0').text.split() == ['0', '-3', '0']
    assert rows[1].find('law').text == 'pursuit' and float(rows[1].find('speed').text) == 0.3
    assert rows[1].find('park').text == 'false'
    models = [ET.fromstring(m) for m in W.obstacle_models(obs)]
    # parked obstacles wait out of view, unparked ones start at p0
    assert [float(v) for v in models[0].find('pose').text.split()[:3]] == list(W.PARK_AT)
    assert [float(v) for v in models[1].find('pose').text.split()[:3]] == [0, 0.5, 0.5]
    assert all(m.find('.//collision') is None for m in models)
