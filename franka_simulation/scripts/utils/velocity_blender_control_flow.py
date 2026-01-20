"""Compatibility shim (deprecated).

The velocity blender control-flow helpers were merged into
`utils.velocity_blender_ros_helpers` to keep the project from accumulating
one-off helper modules.

This file remains so that any external scripts importing it do not break.
"""

from __future__ import annotations

# Re-export from the canonical location.
from .velocity_blender_ros_helpers import (  # noqa: F401
    build_name_to_index,
    handle_emergency_override,
    handle_no_trajectory_mode,
    handle_pause_mode,
    update_joint_positions_inplace,
    update_lpf,
)
