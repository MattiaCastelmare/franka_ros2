#!/usr/bin/env python3
"""F-8-41 harness (scripts_h3, C-078): r2gz_resolve.py with the scenario as the 3rd argument (C6).
Usage: python3 -B - <default|multi> <seed> <scenario> < h3_resolve.py

R2 Gazebo static validation (C-078): copy of scripts_h2/e3_resolve.py. Arms: default (NO --set; the
shipped default must resolve to single, from the installed conditions.yaml AND fr3_control.yaml) and multi
(--set executor_mode=multi). The other checks are unchanged.

Original docstring:
E3: resolve the cell exactly as gz_cbf_stack will, from the INSTALLED harness (simexp and
conditions.yaml), with the arm's explicit --set executor_mode. No ROS graph, no launch, no bytecode.

Usage (container, ROS sourced):  python3 -B - <multi|single> <seed> < e3_resolve.py

Prints one JSON document to stdout. Exit 0 only if:
  * both arms resolve,
  * the arm under test carries executor_mode == <mode> in cbf_params and in the override string,
  * the two arms differ in EXACTLY the three executor_mode keys and in nothing else,
  * every other stack argument and the perception block are identical,
  * the cell name is the expected one.
Any other outcome: a message on stderr and exit 1.
"""
import json
import os
import sys

sys.dont_write_bytecode = True

from ament_index_python.packages import get_package_prefix  # noqa: E402

LIB = os.path.join(get_package_prefix('franka_simulation_experiments'), 'lib',
                   'franka_simulation_experiments')
sys.path.insert(0, LIB)
from simexp import batch  # noqa: E402
from simexp import conditions as C  # noqa: E402
from simexp import scenario_catalog as SC  # noqa: E402

CONDITION = 'full'
EXPECTED_DIFF = ['cbf_params.executor_mode', 'stack_args.cbf_param_overrides',
                 'toggles.cbf_params.executor_mode']


def die(msg):
    print(f'e3_resolve: {msg}', file=sys.stderr)
    sys.exit(1)


def flat(d, p=''):
    out = {}
    for k, v in d.items():
        out.update(flat(v, f'{p}{k}.') if isinstance(v, dict) else {p + k: v})
    return out


def jsonable(x):
    return json.loads(json.dumps(x, sort_keys=True,
                                 default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)))


if len(sys.argv) != 4 or sys.argv[1] not in ('default', 'multi') or sys.argv[3] not in (
        'pentagon_static_baseline_bare', 'pentagon_wander_close'):
    die('usage: python3 -B - <default|multi> <seed> <scenario> < h3_resolve.py')
arm_label, seed, SCENARIO = sys.argv[1], int(sys.argv[2]), sys.argv[3]
SETS = {'default': {}, 'multi': {'executor_mode': 'multi'}}
mode = 'single' if arm_label == 'default' else 'multi'
other_label = 'multi' if arm_label == 'default' else 'default'
import yaml  # noqa: E402
FR3 = os.path.join(get_package_prefix('franka_experiments'), 'share', 'franka_experiments', 'config',
                   'fr3_control.yaml')
with open(FR3) as f:
    shipped = yaml.safe_load(f)['params']['executor_mode']
if shipped != 'single':
    die(f'installed fr3_control.yaml executor_mode is {shipped!r}, not the R2 default single')

arm = jsonable(C.resolve(CONDITION, SETS[arm_label]))
alt = jsonable(C.resolve(CONDITION, SETS[other_label]))
if arm['cbf_params'].get('executor_mode') != mode:
    die(f'cbf_params.executor_mode is {arm["cbf_params"].get("executor_mode")!r}, not {mode!r}')
if f'executor_mode={mode}' not in arm['stack_args'].get('cbf_param_overrides', '').split(';'):
    die(f'cbf_param_overrides does not carry executor_mode={mode}: '
        f'{arm["stack_args"].get("cbf_param_overrides")!r}')
fa, fb = flat(arm), flat(alt)
diff = sorted(k for k in set(fa) | set(fb) if fa.get(k) != fb.get(k))
if diff != EXPECTED_DIFF:
    die(f'the arms differ in {diff}, expected exactly {EXPECTED_DIFF}')
strip = {k: v for k, v in arm['stack_args'].items() if k != 'cbf_param_overrides'}
if strip != {k: v for k, v in alt['stack_args'].items() if k != 'cbf_param_overrides'}:
    die('stack arguments other than cbf_param_overrides differ between the arms')
if arm['perception'] != alt['perception']:
    die('perception blocks differ between the arms')

cell = batch.cell_name(dict(scenario=SCENARIO, condition=CONDITION, seed=seed, perception='true',
                            sets=SETS[arm_label]))
expected_cell = (f'{SCENARIO}__{CONDITION}__gz__s{seed}' if arm_label == 'default'
                 else f'{SCENARIO}__{CONDITION}-executor_mode=multi__gz__s{seed}')
if cell != expected_cell:
    die(f'cell name {cell!r} != expected {expected_cell!r}')

json.dump({'arm': arm_label, 'mode': mode, 'shipped_executor_mode': shipped, 'fr3_control_yaml': FR3,
           'seed': seed, 'cell': cell, 'condition': arm,
           'arm_diff_keys': diff, 'n_keys_compared': len(set(fa) | set(fb)),
           'scenario': jsonable(SC.resolve(SCENARIO, seed=seed)),
           'files': {'conditions_module': C.__file__, 'conditions_yaml': C.default_path(),
                     'scenario_module': SC.__file__}},
          sys.stdout, indent=1, sort_keys=True)
print()
