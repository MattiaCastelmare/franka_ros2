"""Sim-only files that must not drift from the hardware files they derive from."""
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from utils import gz_world as W

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
REAL_EXT = os.path.join(REPO, 'franka_experiments', 'config', 'camera_extrinsics.yaml')
SIM_EXT = os.path.join(REPO, 'franka_simulation', 'config', 'sim', 'camera_extrinsics_sim.yaml')
INTR = os.path.join(REPO, 'franka_experiments', 'config', 'depth_intrinsics.yaml')
WORLD = os.path.join(REPO, 'franka_simulation', 'worlds', 'fr3_cbf_lab.sdf')


def test_sim_extrinsics_default_to_the_real_calibration():
    """Plan H8: the default simulated camera IS the calibrated one. Moving it
    is done in the sim file, on purpose; a changed real calibration fails here."""
    with open(REAL_EXT) as f:
        real = yaml.safe_load(f)
    with open(SIM_EXT) as f:
        sim = yaml.safe_load(f)
    for k in ('parent_frame', 'child_frame', 'translation', 'rotation'):
        assert sim[k] == real[k], k


def test_world_depth_sensor_has_the_d455_depth_fov_and_resolution():
    I = W.load_depth_intrinsics(INTR)
    s = W._model(ET.fromstring(open(WORLD).read()), W.CAMERA_MODEL).find("link/sensor[@name='depth']")
    assert float(s.find('camera/horizontal_fov').text) == pytest.approx(
        W.hfov_from_fx(I['width'], I['fx']), abs=1e-8)
    assert int(s.find('camera/image/width').text) == I['width']
    assert int(s.find('camera/image/height').text) == I['height']


def test_perception_sim_overrides_are_exactly_the_justified_ones():
    """Ground rule 2. Every key here is justified in that file; adding one
    means adding it to this list, with its reason there."""
    p = os.path.join(REPO, 'franka_simulation', 'config', 'sim', 'perception_sim.yaml')
    with open(p) as f:
        assert (yaml.safe_load(f) or {}) == {'booleans': {'visualize': False}}


def test_deep_merge_overrides_nested_keys_only():
    from simexp.sim_config import deep_merge
    a = {'tracking': {'enabled': False, 'imm_enabled': True, 'q_jerk': 2.0}, 'x': 1}
    deep_merge(a, {'tracking': {'enabled': True, 'prediction': {'enabled': True}}})
    assert a == {'tracking': {'enabled': True, 'imm_enabled': True, 'q_jerk': 2.0,
                              'prediction': {'enabled': True}}, 'x': 1}


def test_run_dirs_never_land_in_franka_logs(tmp_path, monkeypatch):
    from simexp import sim_config
    monkeypatch.delenv('FRANKA_SIM_RUNS_DIR', raising=False)
    root = sim_config.sim_runs_root()
    assert 'franka_logs' not in root and 'rosbag' not in root
    assert root.endswith(os.path.join('franka_simulation_experiments', 'sim_runs')) or \
        root.endswith('franka_sim_runs')


def test_depth_adapter_noise_constants_are_the_cbf_sensor_model():
    """H11: the adapter's range-noise constants are fr3_control.yaml's sensor_range_*."""
    with open(os.path.join(REPO, 'franka_experiments', 'config', 'fr3_control.yaml')) as f:
        P = yaml.safe_load(f)['params']
    with open(os.path.join(REPO, 'franka_simulation', 'config', 'sim', 'depth_adapter.yaml')) as f:
        A = yaml.safe_load(f)['/**']['ros__parameters']
    assert A['baseline_m'] == P['sensor_range_baseline_m']
    assert A['sigma_d_px'] == P['sensor_range_sigma_d_px']
    # every fidelity stage ships OFF
    assert (A['delay_s'], A['dropout_p'], A['range_noise']) == (0.0, 0.0, False)


# ── scenes (sim_config.SCENES) ───────────────────────────────────────────────

def _scene_paths(scene):
    """scene_paths against the SOURCE tree, so the test needs no install."""
    from simexp import sim_config
    fs = os.path.join(REPO, 'franka_simulation')
    paths = {
        'world_template': WORLD,
        'world_template_diag': os.path.join(fs, 'worlds', 'fr3_cbf_lab_diag.sdf'),
        'camera_extrinsics_sim': SIM_EXT,
        'camera_extrinsics_diag': os.path.join(fs, 'config', 'sim',
                                                'camera_extrinsics_diag.yaml'),
        'depth_intrinsics': INTR,
        'depth_intrinsics_diag': os.path.join(fs, 'config', 'sim',
                                               'depth_intrinsics_diag.yaml'),
    }
    return sim_config.scene_paths(scene, paths)


def test_every_scene_names_files_that_exist():
    from simexp import sim_config
    for scene in sim_config.SCENES:
        S = _scene_paths(scene)
        for key in ('template', 'extrinsics', 'intrinsics'):
            assert os.path.isfile(S[key]), (scene, key, S[key])
        assert S['furniture'] in ('rig', 'bare')


def test_an_unknown_scene_is_refused_before_a_world_is_written():
    with pytest.raises(ValueError, match='kitchen'):
        _scene_paths('kitchen')


def test_the_two_labs_never_share_a_calibration():
    """The camera moved and the depth profile changed between the recordings:
    a run on one scene must not be able to pick up the other's numbers."""
    rig, diag = _scene_paths('rig'), _scene_paths('diag')
    for key in ('template', 'extrinsics', 'intrinsics'):
        assert rig[key] != diag[key], key
    assert _scene_paths('bare')['extrinsics'] == rig['extrinsics']
    assert _scene_paths('diag_bare')['extrinsics'] == diag['extrinsics']


def test_diag_extrinsics_are_the_pose_the_bag_recorded():
    """base -> camera_depth_optical_frame of datasets/experiments_irim2.zip
    (/tf_static, 2026-10-09). This is a SIM file: unlike `rig` it is NOT tied
    to franka_experiments/config/camera_extrinsics.yaml, because the rig's own
    calibration still carries the irim1 pose."""
    with open(_scene_paths('diag')['extrinsics']) as f:
        d = yaml.safe_load(f)
    assert d['parent_frame'] == 'base'
    assert d['child_frame'] == 'camera_color_optical_frame'   # the frame the stack publishes
    assert (d['translation']['x'], d['translation']['y'], d['translation']['z']) == \
        (1.993009, 0.032024, 1.080394)
    np.testing.assert_allclose(
        [d['rotation'][k] for k in 'xyzw'],
        [0.576894, 0.568193, -0.403120, -0.426433], atol=1e-6)


def test_diag_world_depth_sensor_has_the_diag_fov_and_resolution():
    S = _scene_paths('diag')
    I = W.load_depth_intrinsics(S['intrinsics'])
    assert (I['width'], I['height']) == (848, 480)
    s = W._model(ET.fromstring(open(S['template']).read()),
                 W.CAMERA_MODEL).find("link/sensor[@name='depth']")
    assert float(s.find('camera/horizontal_fov').text) == pytest.approx(
        W.hfov_from_fx(I['width'], I['fx']), abs=1e-8)
    assert int(s.find('camera/image/width').text) == I['width']
    assert int(s.find('camera/image/height').text) == I['height']
