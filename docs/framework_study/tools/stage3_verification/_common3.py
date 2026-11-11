"""Shared helpers for the Stage 3 verification scripts (chapters 06 and 07). Read-only.

Re-uses `../stage2_verification/_common.py` (paths, the source-import guard, the PASS/FAIL report),
and adds one thing: the EFFECTIVE parameter set of `cbf_safety_filter` for a given launch
configuration, computed with the launch file's own code instead of a hand copy.

How the effective parameters are obtained (03 §2 precedence model):
  1. base layer: `franka_experiments/config/fr3_control.yaml -> params` (the source copy; the node
     reads the installed copy, which equals it after a rebuild);
  2. launch arguments: `launch_defaults.yaml`, overridden for simulation by
     `simexp.conditions.resolve(name)['stack_args']` plus the fixed `camera_depth_profile`
     that `gz_cbf_stack.launch.py` passes;
  3. ROS overrides: the functions `_cbf_parameters`, `_speed_ceiling_overrides`,
     `_cbf_param_overrides`, `_as_bool` and `_profile_fps`, extracted with `ast` from
     `torque_control_stack.launch.py` and executed on (2). The launch file itself cannot be
     imported without the ROS `launch` package, so only these pure functions are compiled.

Keys that `_cbf_parameters` emits but the node never declares (`robot_config_path`, F-1-12) are
dropped, as rclpy would drop them.
"""
import ast
import os
import sys
from types import SimpleNamespace

sys.dont_write_bytecode = True
HERE3 = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE3, '..', 'stage2_verification'))
import _common as C  # noqa: E402,F401  (re-exported)

import yaml  # noqa: E402

LAUNCH_TORQUE = os.path.join(C.PKG_SRC, 'launch', 'torque_control_stack.launch.py')
SIM_SCRIPTS = os.path.join(C.REPO, 'franka_simulation_experiments', 'scripts')
LAUNCH_FNS = ('_as_bool', '_cbf_param_overrides', '_speed_ceiling_overrides', '_profile_fps',
              '_cbf_parameters')
GZ_FIXED_ARGS = {'camera_depth_profile': '848x480x90'}   # gz_cbf_stack.launch.py, stack_args.update


def launch_functions() -> dict:
    """The five pure helper functions of torque_control_stack.launch.py, compiled from source."""
    src = open(LAUNCH_TORQUE).read()
    tree = ast.parse(src, LAUNCH_TORQUE)
    body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in LAUNCH_FNS]
    missing = set(LAUNCH_FNS) - {n.name for n in body}
    if missing:
        raise RuntimeError(f'launch helpers not found: {sorted(missing)}')
    ns = {'yaml': yaml}
    exec(compile(ast.Module(body=body, type_ignores=[]), LAUNCH_TORQUE, 'exec'), ns)
    return {k: ns[k] for k in LAUNCH_FNS}


def yaml_params() -> dict:
    return dict(C.load_yaml(os.path.join(C.CONFIG, 'fr3_control.yaml'))['params'])


def launch_args(condition: str | None = None) -> dict:
    """Launch-argument values: launch_defaults.yaml, or a simulation condition's stack_args."""
    args = {k: str(v) for k, v in C.load_yaml(os.path.join(C.CONFIG, 'launch_defaults.yaml')).items()}
    args.setdefault('robot_config_yaml', '')
    if condition is not None:
        if SIM_SCRIPTS not in sys.path:
            sys.path.insert(0, SIM_SCRIPTS)
        from simexp.conditions import resolve
        args.update(resolve(condition)['stack_args'])
        args.update(GZ_FIXED_ARGS)
    return args


def effective_params(config: str = 'hw_default') -> dict:
    """`cbf_safety_filter` parameters for one configuration.

    config: 'yaml' (no launch overrides), 'hw_default' (torque_control_stack with launch_defaults),
            or 'sim:<condition>' (gz_cbf_stack with that conditions.yaml condition).
    """
    base = yaml_params()
    if config == 'yaml':
        return base
    cond = config.split(':', 1)[1] if config.startswith('sim:') else None
    if config != 'hw_default' and cond is None:
        raise ValueError(config)
    fns = launch_functions()
    p = dict(base)
    for d in fns['_cbf_parameters'](launch_args(cond)):
        for k, v in d.items():
            if k in base:                      # undeclared keys are silently ignored by rclpy
                p[k] = v
            elif k != 'robot_config_path':
                raise KeyError(f'override {k!r} is not a fr3_control.yaml parameter')
    return p


def params_namespace(config: str = 'hw_default', **over) -> SimpleNamespace:
    p = effective_params(config)
    p.update(over)
    return SimpleNamespace(**p)
