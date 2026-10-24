"""Expand torque_control_stack.launch.py into a comparable snapshot, without launching anything.

The launch file's behaviour lives in ``_launch_all(context)``, an OpaqueFunction:
which nodes start, with which parameters, after which delay. This module
evaluates it against a ``LaunchContext`` holding the declared defaults (plus
overrides), then serialises the action tree into plain JSON-able data:

* every ``Substitution`` performed in the context;
* temp files the launch writes (the controllers YAML, the perception-config
  copy) replaced by ``<tmp:sha256-of-content>``, so a snapshot pins what the
  file SAYS and not where ``mkstemp`` put it;
* objects with no stable value (event-handler callables) reduced to their type;
* the workspace's install prefix written as ``<install>/``, so the snapshot does
  not depend on where the workspace lives.

Used by test_torque_stack_external_plant_off_identical.py. The golden snapshot it
compares against was produced from the file at 94cd7b0 (see that test).
"""
from __future__ import annotations

import functools
import hashlib
import importlib.util
import os
import re
import types

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitution import Substitution
from launch.utilities import perform_substitutions

_TMP = re.compile(r'(/tmp/(?:franka_rt_blender_[^/\s\'"]+\.yaml|fr3_complete_launch_\d+\.yaml))')


_INSTALL = re.compile(r'[^\s\'"=:]*/install/(?=[A-Za-z_])')


def _norm_str(s: str) -> str:
    s = _INSTALL.sub('<install>/', s)

    def _sub(m):
        path = m.group(1)
        try:
            with open(path, 'rb') as f:
                return '<tmp:' + hashlib.sha256(f.read()).hexdigest()[:16] + '>'
        except OSError:
            return '<tmp:missing>'
    return _TMP.sub(_sub, s)


def _is_subst_list(x) -> bool:
    return isinstance(x, (list, tuple)) and x and all(
        isinstance(e, (Substitution, str)) for e in x) and any(
        isinstance(e, Substitution) for e in x)


def _perform(ctx, subs) -> str:
    """Perform in ``ctx``; a substitution that only resolves at execute time
    (``LocalSubstitution('ros_specific_arguments...')``) is kept by name."""
    out = []
    for sub in subs:
        if isinstance(sub, str):
            out.append(sub)
            continue
        try:
            out.append(sub.perform(ctx))
        except Exception:                                   # noqa: BLE001
            out.append(f'<{sub.describe()}>')
    return _norm_str(''.join(out))


def serialize(obj, ctx, depth=0):
    if depth > 30:
        return '<deep>'
    if isinstance(obj, Substitution):
        return _perform(ctx, [obj])
    if _is_subst_list(obj):
        return _perform(ctx, list(obj))
    if isinstance(obj, str):
        return _norm_str(obj)
    if isinstance(obj, (int, float, bool)) or obj is None:
        return obj
    if isinstance(obj, dict):
        return {str(serialize(k, ctx, depth + 1)): serialize(v, ctx, depth + 1)
                for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [serialize(v, ctx, depth + 1) for v in obj]
    if isinstance(obj, (types.FunctionType, types.MethodType, types.BuiltinFunctionType,
                        functools.partial)):
        return '<callable>'
    if (type(obj).__module__ or '').split('.')[0] in ('launch', 'launch_ros'):
        out = {'__type__': type(obj).__name__}
        for k, v in sorted(vars(obj).items()):
            short = k.split('__')[-1].lstrip('_')
            if short not in _SKIP:
                out[short] = serialize(v, ctx, depth + 1)
        return out
    return f'<{type(obj).__name__}>'


#: Attributes that carry identity, runtime state or callables, never configuration.
_SKIP = {
    'logger', 'process_event_args', 'subprocess_transport', 'subprocess_protocol',
    'completed_future', 'shutdown_future', 'stdin_lock', 'stdout_buffer',
    'stderr_buffer', 'shutdown_lock', 'sigterm_timer', 'sigkill_timer',
    'exit_future', 'timer_future', 'canceled_future', 'context', 'condition',
    'on_exit', 'on_stdin', 'on_stdout', 'on_stderr', 'matcher', 'handler',
    'entities', 'target_action', 'ld', 'launch_description',
    'expanded_node_namespace', 'final_node_name', 'substitutions_performed',
    'expanded_parameter_arguments', 'expanded_remappings', 'emulate_tty',
    'event_handlers', 'respawn_retries', 'id', 'final_cmd', 'final_name',
    'final_cwd', 'final_env',
}


def load_launch_module(path: str, name: str = 'torque_control_stack_snapshot'):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def snapshot(path: str, **overrides) -> list:
    """Serialised action list of ``_launch_all`` with the declared defaults + overrides."""
    mod = load_launch_module(path)
    ld = mod.generate_launch_description()
    ctx = LaunchContext()
    for ent in ld.entities:
        if isinstance(ent, DeclareLaunchArgument):
            val = overrides.get(ent.name)
            if val is None:
                val = perform_substitutions(ctx, ent.default_value)
            ctx.launch_configurations[ent.name] = str(val)
    unknown = set(overrides) - set(ctx.launch_configurations)
    if unknown:
        raise KeyError(f'not declared by {os.path.basename(path)}: {sorted(unknown)}')
    return [serialize(a, ctx) for a in mod._launch_all(ctx)]
