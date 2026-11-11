"""Shared helpers for the Stage 6 verification scripts (chapter 12). Read-only.

Re-uses `../stage5_verification/_common5.py` (paths, the source-import guard of F-2-2, the PASS/FAIL
`Report`, `C3.effective_params`, `src`, `stub`). Adds the two simulation source trees to `sys.path`
the way `franka_simulation_experiments/test/conftest.py` does:

* `franka_simulation_experiments/scripts` (`simexp`, the sim nodes);
* `franka_simulation/scripts` (`utils.gz_robot_description`, `utils.gz_world`,
  `utils.depth_conversion`);

and refuses an installed copy of `simexp` (the same guard as `use_source_package`).
"""
import os
import sys

sys.dont_write_bytecode = True
HERE6 = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE6, '..', 'stage5_verification'))
import _common5 as C5  # noqa: E402

C, C3, src, stub = C5.C, C5.C3, C5.src, C5.stub
SIMEXP_SCRIPTS = os.path.join(C.REPO, 'franka_simulation_experiments', 'scripts')
FS_SCRIPTS = os.path.join(C.REPO, 'franka_simulation', 'scripts')
SIM_RUNS = os.path.join(C.REPO, 'franka_simulation_experiments', 'sim_runs')
BASELINE = 'ba68077'
for _p in (FS_SCRIPTS, SIMEXP_SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import simexp  # noqa: E402

if os.path.realpath(os.path.dirname(simexp.__file__)) != os.path.realpath(os.path.join(SIMEXP_SCRIPTS, 'simexp')):
    raise RuntimeError(f'simexp imported from {simexp.__file__}, expected the source tree')
