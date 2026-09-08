"""ROS helpers — COMPATIBILITY FACADE.

Phase 2 split this module by concern:

* ``utils.node_runtime``   — node lifecycle + topic-name resolution
* ``utils.launch_support`` — launch-description composition
* ``utils.config``         — YAML/defaults loading

Every symbol that ever lived here is re-exported below, so existing imports
(including all five launch files) keep working unchanged. New code should
import from the module that owns the concept.
"""

# TODO[LEGACY]: compatibility facade: split into utils/node_runtime.py, utils/launch_support.py, utils/config.py | confidence: high | superseded-by: utils/node_runtime.py + utils/launch_support.py + utils/config.py | flagged: 2026-09-01

from __future__ import annotations

import os
from typing import Optional

import yaml

import rclpy
import rclpy.executors
from rclpy.node import Node

from .constants import AVOIDANCE_TOPIC_SUFFIX, TRACKING_TOPIC_SUFFIX  # noqa: F401
from .node_runtime import (  # noqa: F401
    build_namespaced_topic,
    get_namespace_from_config,
    resolve_avoidance_topic,
    resolve_topic_with_deprecated_alias,
    resolve_tracking_topic,
    run_node_main,
    teardown,
)
from .launch_support import (  # noqa: F401
    _ensure_urdf_cached,
    _generate_cbf_base_yaml,
    _generate_torque_yaml,
    build_wrapper_log_actions,
    declare_robot_args,
    declare_rt_blender_args,
    declare_rt_torque_args,
    generate_rt_controllers_yaml,
    pick_controllers_yaml,
    resolve_controller_manager_name,
    write_temp_yaml,
)


# ---------------------------------------------------------------------------
# Namespace / topic resolution
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Common main() patterns
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Franka bringup configuration
# ---------------------------------------------------------------------------

# MOVED to utils/config.py (Phase 2): these are file loaders, not launch
# composition. Re-exported under the original names so every existing call
# site — including all five launch files — keeps working unchanged.
from franka_experiments.utils.config import (  # noqa: F401,E402
    load_franka_config_defaults,
    load_launch_defaults,
)


# ---------------------------------------------------------------------------
# Controller manager
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Controllers YAML generation & selection
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Launch argument declarations
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Launch logging helpers
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Self-check (moved from selfcheck.py)
# ---------------------------------------------------------------------------

def selfcheck_run():
    """Import all utils modules and run basic sanity checks.

    Returns 0 on success, 1 on failure.

    Usage::

        python3 -c "from franka_experiments.utils.ros import selfcheck_run; selfcheck_run()"
    """
    import pathlib
    import py_compile
    import sys

    errors = []

    # ---- constants ----
    try:
        from franka_experiments.utils.constants import (
            NUM_JOINTS, FR3_JOINT_NAMES, TRACKING_TOPIC_SUFFIX,
            AUTO_SENTINEL,
        )
        assert NUM_JOINTS == 7
        assert len(FR3_JOINT_NAMES) == 7
        assert FR3_JOINT_NAMES[0] == 'fr3_joint1'
        assert isinstance(TRACKING_TOPIC_SUFFIX, str)
        assert isinstance(AUTO_SENTINEL, str)
        print('[OK] constants')
    except Exception as e:
        errors.append(f'constants: {e}')
        print(f'[FAIL] constants: {e}')

    # ---- math_utils ----
    try:
        from franka_experiments.utils.math_utils import (
            min_jerk, cosine_ramp, clamp_joints, lpf,
        )
        import numpy as np

        s, sd = min_jerk(0.5)
        assert 0.4 < s < 0.6, f's={s}'
        assert sd > 0.0, f'sdot={sd}'

        s0, sd0 = min_jerk(0.0)
        assert s0 == 0.0 and sd0 == 0.0
        s1, sd1 = min_jerk(1.0)
        assert abs(s1 - 1.0) < 1e-12 and abs(sd1) < 1e-12

        assert cosine_ramp(0.0, 1.0) == 0.0
        assert cosine_ramp(1.0, 1.0) == 1.0
        assert cosine_ramp(0.5, 0.0) == 1.0
        assert 0.0 < cosine_ramp(0.5, 1.0) < 1.0

        v = clamp_joints(np.array([1.0, -1.0, 0.1]), 0.5)
        assert np.allclose(v, [0.5, -0.5, 0.1])

        f = lpf(np.array([1.0]), np.array([0.0]), 0.5)
        assert np.allclose(f, [0.5])

        print('[OK] math_utils')
    except Exception as e:
        errors.append(f'math_utils: {e}')
        print(f'[FAIL] math_utils: {e}')

    # ---- trajectory ----
    try:
        from franka_experiments.utils.trajectory import (
            PentagonTrajectory, sample_waypoints, sample_single_waypoint,
        )
        import numpy as np

        traj = PentagonTrajectory(
            center=np.array([0.4, 0.0, 0.4]),
            radius=0.03, plane='front', cycle_time=15.0,
        )
        p, v = traj.evaluate(0.0)
        assert p.shape == (3,)
        assert v.shape == (3,)

        rng = np.random.default_rng(42)
        pts = sample_waypoints(
            5, [0.2, 0.5, -0.2, 0.2, 0.1, 0.5], 0.01, rng)
        assert len(pts) == 5

        wp = sample_single_waypoint(
            [0.2, 0.5, -0.2, 0.2, 0.1, 0.5], 0.01, rng)
        assert wp.shape == (3,)

        print('[OK] trajectory')
    except Exception as e:
        errors.append(f'trajectory: {e}')
        print(f'[FAIL] trajectory: {e}')

    # ---- logging_utils ----
    try:
        from franka_experiments.utils.logging_utils import (
            ThrottledLogger, vec_to_str,
        )
        import numpy as np

        assert vec_to_str(np.array([1.0, 2.0])) == '1.0000, 2.0000'
        assert vec_to_str(None) == '?'
        print('[OK] logging_utils')
    except Exception as e:
        errors.append(f'logging_utils: {e}')
        print(f'[FAIL] logging_utils: {e}')

    # ---- ros (import only — no rclpy.init needed) ----
    try:
        from franka_experiments.utils import ros  # noqa: F401
        print('[OK] ros (import)')
    except Exception as e:
        errors.append(f'ros: {e}')
        print(f'[FAIL] ros: {e}')

    # ---- kinematics (import only — pinocchio may not be installed) ----
    try:
        from franka_experiments.utils import kinematics  # noqa: F401
        # JointStateManager is now in kinematics (merged from joint_state)
        from franka_experiments.utils.kinematics import JointStateManager  # noqa: F401
        print('[OK] kinematics (import, includes JointStateManager)')
    except ImportError:
        print('[SKIP] kinematics (pinocchio not installed)')
    except Exception as e:
        errors.append(f'kinematics: {e}')
        print(f'[FAIL] kinematics: {e}')

    # ---- py_compile all .py files ----
    pkg_root = pathlib.Path(__file__).resolve().parent.parent
    py_files = sorted(pkg_root.rglob('*.py'))
    compile_ok = 0
    compile_fail = 0
    for pf in py_files:
        if '__pycache__' in str(pf):
            continue
        try:
            py_compile.compile(str(pf), doraise=True)
            compile_ok += 1
        except py_compile.PyCompileError as e:
            compile_fail += 1
            errors.append(f'py_compile {pf.name}: {e}')
            print(f'[FAIL] py_compile {pf.relative_to(pkg_root)}: {e}')
    print(f'[{"OK" if compile_fail == 0 else "FAIL"}] '
          f'py_compile: {compile_ok} ok, {compile_fail} failed '
          f'(out of {compile_ok + compile_fail} files)')

    print()
    if errors:
        print(f'FAILED ({len(errors)} error(s)):')
        for e in errors:
            print(f'  - {e}')
        return 1
    print('All checks passed.')
    return 0
