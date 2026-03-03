"""Shared constants for franka_experiments."""

from __future__ import annotations

NUM_JOINTS: int = 7
"""Number of arm joints on the FR3."""

FR3_JOINT_NAMES: list = [f'fr3_joint{i}' for i in range(1, NUM_JOINTS + 1)]
"""Ordered list of FR3 arm joint names as they appear in the URDF."""

TRACKING_TOPIC_SUFFIX: str = 'tracking_qdot'
"""Default suffix for the tracking velocity topic."""

CONTROLLER_NAME: str = 'fr3_forward_velocity_controller'
"""Default forward-velocity controller name."""

AUTO_SENTINEL: str = '__auto__'
"""Sentinel value that triggers auto-detection (e.g. joint_state_topic)."""
