"""Freeze effective Python sources and configuration files for each SIM run."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def snapshot(run_dir, files, stack_args):
    import franka_experiments
    try:
        from franka_experiments.utils.cbf_diagnostics import STATUS_FIELDS, STATUS_SCHEMA_VERSION
    except ImportError:
        # An intentionally selected pre-correction controller has the v1 wire.
        STATUS_SCHEMA_VERSION = 1
        STATUS_FIELDS = ('n_rows', 'slack_max', 'fault_braking', 'n_violated',
                         'd_obstacle_min', 'ssm_sp_min', 'ssm_vcap_min',
                         'v_closing_max', 'iso_stop_latched', 'v_obs_conditioned', 'v_obs_hdot')
    root = Path(franka_experiments.__file__).resolve().parent
    dest = Path(run_dir) / 'provenance'
    dest.mkdir(parents=True, exist_ok=True)
    source_files = {}
    # The actual import path matters: PYTHONPATH may select a baseline copy.
    for path in sorted(root.rglob('*.py')):
        rel = path.relative_to(root)
        target = dest / 'python' / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        source_files[str(rel)] = hashlib.sha256(path.read_bytes()).hexdigest()
    configs = {}
    for key, name in files.items():
        if name and Path(name).is_file():
            path = Path(name)
            target = dest / f'{key}{path.suffix}'
            shutil.copy2(path, target)
            configs[key] = dict(original=str(path.resolve()), copy=str(target),
                                sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    # Preserve the complete tracked patch, including work already in progress.
    patch = subprocess.run(['git', 'diff', 'HEAD', '--binary'],
                           cwd=Path(__file__).resolve().parent,
                           capture_output=True, timeout=15, check=False)
    (dest / 'working_tree.patch').write_bytes(patch.stdout)
    data = dict(python_root=str(root), source_sha256=source_files, files=configs,
                stack_args=stack_args, status_schema_version=STATUS_SCHEMA_VERSION,
                status_fields=list(STATUS_FIELDS))
    (dest / 'snapshot.json').write_text(json.dumps(data, indent=2, sort_keys=True))
    return str(dest / 'snapshot.json')
