"""Compatibility shim (deprecated).

Diagnostics bookkeeping for velocity_control_blender now lives in
`utils.velocity_blender_ros_helpers.VelocityBlenderDiagnostics`.
"""

from __future__ import annotations

from .velocity_blender_ros_helpers import VelocityBlenderDiagnostics  # noqa: F401
