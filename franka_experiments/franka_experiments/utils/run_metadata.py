"""Optional per-node runtime provenance, enabled by the experiment runner."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import numpy as np


def write_runtime_metadata(node, *, effective=None, files=()):
    directory = os.environ.get('FRANKA_RUN_METADATA_DIR', '')
    if not directory:
        return
    parameters = {name: p.value for name, p in node.get_parameters_by_prefix('').items()}
    sources = {}
    for name in files:
        if name and Path(name).is_file():
            path = Path(name).resolve()
            sources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    data = dict(node=node.get_fully_qualified_name(), ros_parameters=parameters,
                effective=effective, file_sha256=sources)

    def encode(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        return str(value)

    target = Path(directory) / f'{node.get_name()}_runtime.json'
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=2, sort_keys=True, default=encode))
    tmp.replace(target)
