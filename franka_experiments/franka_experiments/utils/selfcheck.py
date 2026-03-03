#!/usr/bin/env python3
"""Smoke test: import all modules and validate key functions.

Run with::

    python -m franka_experiments.utils.selfcheck

or equivalently::

    python3 -c "from franka_experiments.utils.selfcheck import run; run()"
"""

from __future__ import annotations

import sys


def run() -> int:
    """Import all utils modules and run basic sanity checks.

    Returns 0 on success, 1 on failure.
    """
    errors = []

    # ---- constants ----
    try:
        from franka_experiments.utils.constants import (
            NUM_JOINTS, FR3_JOINT_NAMES, TRACKING_TOPIC_SUFFIX,
            CONTROLLER_NAME, AUTO_SENTINEL,
        )
        assert NUM_JOINTS == 7
        assert len(FR3_JOINT_NAMES) == 7
        assert FR3_JOINT_NAMES[0] == 'fr3_joint1'
        assert isinstance(TRACKING_TOPIC_SUFFIX, str)
        assert isinstance(CONTROLLER_NAME, str)
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

        # min_jerk at midpoint
        s, sd = min_jerk(0.5)
        assert 0.4 < s < 0.6, f's={s}'
        assert sd > 0.0, f'sdot={sd}'

        # min_jerk boundary conditions
        s0, sd0 = min_jerk(0.0)
        assert s0 == 0.0 and sd0 == 0.0
        s1, sd1 = min_jerk(1.0)
        assert abs(s1 - 1.0) < 1e-12 and abs(sd1) < 1e-12

        # cosine_ramp
        assert cosine_ramp(0.0, 1.0) == 0.0
        assert cosine_ramp(1.0, 1.0) == 1.0
        assert cosine_ramp(0.5, 0.0) == 1.0
        assert 0.0 < cosine_ramp(0.5, 1.0) < 1.0

        # clamp_joints
        v = clamp_joints(np.array([1.0, -1.0, 0.1]), 0.5)
        assert np.allclose(v, [0.5, -0.5, 0.1])

        # lpf
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
        print('[OK] kinematics (import)')
    except ImportError:
        print('[SKIP] kinematics (pinocchio not installed)')
    except Exception as e:
        errors.append(f'kinematics: {e}')
        print(f'[FAIL] kinematics: {e}')

    # ---- joint_state (import only) ----
    try:
        from franka_experiments.utils import joint_state  # noqa: F401
        print('[OK] joint_state (import)')
    except ImportError:
        print('[SKIP] joint_state (pinocchio not installed)')
    except Exception as e:
        errors.append(f'joint_state: {e}')
        print(f'[FAIL] joint_state: {e}')

    # ---- py_compile all .py files ----
    import py_compile
    import pathlib

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

    # ---- Summary ----
    print()
    if errors:
        print(f'FAILED ({len(errors)} error(s)):')
        for e in errors:
            print(f'  - {e}')
        return 1
    print('All checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(run())
