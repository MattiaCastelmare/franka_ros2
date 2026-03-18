"""Robust YAML loader for ``sensor_msgs/CameraInfo`` dumps.

The config files ``rgb_intrinsics.yaml`` and ``depth_intrinsics.yaml``
are raw dumps of ``sensor_msgs/CameraInfo`` messages.  They may contain
YAML multi-document separators (``---``) which cause ``yaml.safe_load()``
to fail with *"expected a single document in the stream"*.

This module provides a single helper that handles this correctly.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

import yaml


def load_camera_info_yaml(
    path: str,
    required_keys: tuple[str, ...] = ("k",),
) -> Optional[Dict[str, Any]]:
    """Load a CameraInfo-style YAML file, handling multi-document streams.

    Parameters
    ----------
    path : str
        Absolute path to the YAML file.
    required_keys : tuple[str, ...]
        Keys that must be present in the returned document for it to be
        considered valid.  Defaults to ``("k",)``.

    Returns
    -------
    dict or None
        The first YAML document that is a non-empty ``dict`` containing all
        *required_keys*, or ``None`` if no such document is found.
    """
    with open(path, "r") as fh:
        for doc in yaml.safe_load_all(fh):
            if not isinstance(doc, dict):
                continue
            if all(key in doc for key in required_keys):
                return doc
    return None
