"""Bridge imports from franka_simulation avoidance utilities.

The avoidance utilities (CBF-QP math, Pinocchio helpers, RViz markers,
publishers) live in franka_simulation's ``scripts/utils/`` directory and
are installed alongside the simulation scripts at build time under
``lib/franka_simulation/utils/``.

This module resolves their installation path at runtime so that
franka_experiments nodes can reuse them without code duplication.

.. note::
   ``franka_simulation`` must be built and sourced before any module
   that imports from here.
"""

# TODO[LEGACY]: import shim with a single consumer (capsule_overlay_node) | confidence: medium | superseded-by: none | flagged: 2026-09-01

from __future__ import annotations

import os
import sys

_SIM_LIB_DIR: str | None = None


def _ensure_simulation_utils_on_path() -> None:
    """Add ``franka_simulation`` lib directory to *sys.path* (idempotent)."""
    global _SIM_LIB_DIR
    if _SIM_LIB_DIR is not None:
        return

    from ament_index_python.packages import get_package_prefix

    prefix = get_package_prefix("franka_simulation")
    lib_dir = os.path.join(prefix, "lib", "franka_simulation")
    if lib_dir not in sys.path:
        sys.path.insert(0, lib_dir)
    _SIM_LIB_DIR = lib_dir


try:
    _ensure_simulation_utils_on_path()
except Exception as _exc:
    raise ImportError(
        "franka_simulation must be built and sourced before "
        "franka_experiments can use the avoidance utilities. "
        f"Original error: {_exc}"
    ) from _exc

# ── Re-exported symbols from franka_simulation/scripts/utils/ ─────────

from utils.avoidance_math import (  # noqa: E402
    build_joint_to_joint_capsules,
    build_reduced_pinocchio_model_from_urdf,
    capsule_risk_weight,
    check_all_halfspaces_feasible,
    compute_closest_constraint_jacobian,
    pocs_project_halfspaces_with_box,
    solve_cbf_qp_1constraint_box,
)
from utils.closest_constraint import (  # noqa: E402
    format_closest_label,
)
from utils.ros_publishers import (  # noqa: E402
    PublishersBundle,
    publish_minimal_avoidance_outputs,
    publish_not_ready_outputs,
)
from utils.ros_setup import (  # noqa: E402
    init_pinocchio_and_capsules,
    make_joint_state_callback,
)
from utils.avoidance_core import (  # noqa: E402
    iter_world_capsule_segments,
)
from utils.rviz_markers import (  # noqa: E402
    build_marker_array,
)

__all__ = [
    "build_joint_to_joint_capsules",
    "build_reduced_pinocchio_model_from_urdf",
    "capsule_risk_weight",
    "check_all_halfspaces_feasible",
    "compute_closest_constraint_jacobian",
    "pocs_project_halfspaces_with_box",
    "solve_cbf_qp_1constraint_box",
    "format_closest_label",
    "PublishersBundle",
    "publish_minimal_avoidance_outputs",
    "publish_not_ready_outputs",
    "init_pinocchio_and_capsules",
    "make_joint_state_callback",
    "iter_world_capsule_segments",
    "build_marker_array",
]
