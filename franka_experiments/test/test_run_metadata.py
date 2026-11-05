"""Runtime provenance must work with the installed ROS API before SIM starts."""
import json

import numpy as np
import rclpy
from rclpy.context import Context

from franka_experiments.utils.run_metadata import write_runtime_metadata


def test_metadata_captures_real_ros_parameters_effective_values_and_hashes(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '154')
    monkeypatch.setenv('FRANKA_RUN_METADATA_DIR', str(tmp_path))
    ctx = Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node('metadata_probe', context=ctx)
    try:
        node.declare_parameter('governor_rate_limit', 2.)
        node.declare_parameter('limits', [1., 2.])
        source = tmp_path / 'config.yaml'
        source.write_text('gain: 2\n')
        write_runtime_metadata(node, effective={'a': np.array([3., 4.])}, files=[source])
        data = json.loads((tmp_path / 'metadata_probe_runtime.json').read_text())
        assert data['ros_parameters']['governor_rate_limit'] == 2.
        assert data['ros_parameters']['limits'] == [1., 2.]
        assert data['effective']['a'] == [3., 4.]
        assert len(data['file_sha256'][str(source)]) == 64
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
