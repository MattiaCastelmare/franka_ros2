"""Robust YAML loader for ``sensor_msgs/CameraInfo`` dumps.

The config files ``rgb_intrinsics.yaml`` and ``depth_intrinsics.yaml``
are raw dumps of ``sensor_msgs/CameraInfo`` messages.  They may contain
YAML multi-document separators (``---``) which cause ``yaml.safe_load()``
to fail with *"expected a single document in the stream"*.

This module provides a single helper that handles this correctly.
"""

# TODO[LEGACY]: compatibility shim: load_camera_info_yaml now lives in utils/config.py | confidence: high | superseded-by: utils/config.py | flagged: 2026-09-01

# MOVED to utils/config.py (Phase 2). Re-exported under the original name so
# every existing call site keeps working unchanged.
from franka_experiments.utils.config import (  # noqa: F401
    load_camera_info_yaml,
)
