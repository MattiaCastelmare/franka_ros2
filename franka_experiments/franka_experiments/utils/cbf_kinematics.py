"""Deprecated shim: :class:`CBFKinematics` now lives in ``utils.kinematics``.

MOVED in Phase 2 — point Jacobians are part of the FR3 robot model, not a
separate concern. Re-exported here so every existing import keeps working.
"""

# TODO[LEGACY]: compatibility shim: CBFKinematics now lives in utils/kinematics.py | confidence: high | superseded-by: utils/kinematics.py | flagged: 2026-09-01

from franka_experiments.utils.kinematics import CBFKinematics  # noqa: F401
