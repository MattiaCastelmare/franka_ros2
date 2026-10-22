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
