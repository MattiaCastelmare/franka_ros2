"""Resolve packaged controllers and load optional external controller files.

Controller selection is shared by launch, runner and batch execution. Legacy
selectors are aliases only; the former ``students/`` tree is not required.
"""
from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path
import sys


_ALIASES = {
    'baseline': 'baseline.py',
    'baseline.py': 'baseline.py',
    'controllers/baseline.py': 'baseline.py',
    'controller_baseline.py': 'baseline.py',
    'circle_tracking/controller_baseline.py': 'baseline.py',
    'baseline_perp': 'baseline_perp.py',
    'baseline_perp.py': 'baseline_perp.py',
    'controllers/baseline_perp.py': 'baseline_perp.py',
    'optimized_hqp': 'optimized_hqp.py',
    'optimized_hqp.py': 'optimized_hqp.py',
    'controllers/optimized_hqp.py': 'optimized_hqp.py',
    'optimized_hqp_perp': 'optimized_hqp_perp.py',
    'optimized_hqp_perp.py': 'optimized_hqp_perp.py',
    'controllers/optimized_hqp_perp.py': 'optimized_hqp_perp.py',
    'impedance_hqp': 'optimized_hqp.py',
    'impedance_hqp.py': 'optimized_hqp.py',
    'controllers/impedance_hqp.py': 'optimized_hqp.py',
    'controller_impedance_hqp.py': 'optimized_hqp.py',
    'circle_tracking/controller_impedance_hqp.py': 'optimized_hqp.py',
}
_TASKS = {
    'baseline.py': 'circle_tracking_baseline.yaml',
    'baseline_perp.py': 'baseline_perp.yaml',
    'optimized_hqp.py': 'circle_tracking_optimized_hqp.yaml',
    'optimized_hqp_perp.py': 'optimized_hqp_perp.yaml',
}


def resolve_controller_path(selector: str) -> str:
    """Return an existing built-in file or an explicitly absolute custom file."""
    path = Path(selector).expanduser()
    if not path.is_absolute():
        normalized = str(selector).replace('\\', '/').removeprefix('./')
        filename = _ALIASES.get(normalized)
        if filename is None:
            raise FileNotFoundError(
                f'Unknown controller {selector!r}; use baseline.py, '
                'baseline_perp.py, optimized_hqp.py, optimized_hqp_perp.py '
                'or an absolute Python file path')
        path = Path(__file__).resolve().parent / filename
    if not path.is_file():
        raise FileNotFoundError(f'Controller file does not exist: {path}')
    return str(path)


def default_task_config(selector: str) -> str:
    """Select the matching YAML; custom files retain the baseline default."""
    name = str(selector).replace('\\', '/').rsplit('/', 1)[-1]
    canonical = _ALIASES.get(name, name)
    return _TASKS.get(canonical, _TASKS['baseline.py'])


def load_controller_class(path: str):
    """Load and validate ``Controller(StudentController)`` without ROS state."""
    from fr3_sim.controllers.base import StudentController

    resolved = Path(resolve_controller_path(path)).resolve()
    packaged = Path(__file__).resolve().parent
    if resolved.parent == packaged and resolved.name in _TASKS:
        module = importlib.import_module(f'fr3_sim.controllers.{resolved.stem}')
    else:
        # Preserve sibling helper imports for externally supplied controllers.
        folder = str(resolved.parent)
        if folder in sys.path:
            sys.path.remove(folder)
        sys.path.insert(0, folder)
        spec = importlib.util.spec_from_file_location('fr3_sim_external_controller', resolved)
        if spec is None or spec.loader is None:
            raise ImportError(f'Cannot load controller module: {resolved}')
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
    cls = getattr(module, 'Controller', None)
    if not isinstance(cls, type) or not issubclass(cls, StudentController):
        raise TypeError(f'{resolved} must define class Controller(StudentController)')
    return cls
