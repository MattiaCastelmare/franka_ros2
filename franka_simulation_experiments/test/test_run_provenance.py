import hashlib
import json
from pathlib import Path

import franka_experiments

from simexp.provenance import snapshot


def test_snapshot_keeps_actual_sources_configs_hashes_and_wire_schema(tmp_path, monkeypatch):
    package = tmp_path / 'controller'
    package.mkdir()
    (package / '__init__.py').write_text('')
    code = package / 'controller.py'
    code.write_text('gain = 2\n')
    config = tmp_path / 'input.yaml'
    config.write_text('gain: 2\n')
    monkeypatch.setattr(franka_experiments, '__file__', str(package / '__init__.py'))
    output = Path(snapshot(tmp_path / 'run', {'control': str(config)}, {'motion': 'pentagon'}))
    saved = json.loads(output.read_text())
    assert saved['source_sha256']['controller.py'] == hashlib.sha256(code.read_bytes()).hexdigest()
    assert saved['files']['control']['sha256'] == hashlib.sha256(config.read_bytes()).hexdigest()
    assert saved['stack_args'] == {'motion': 'pentagon'}
    assert 'fallback_level' in saved['status_fields']
    code.write_text('gain = 9\n')
    config.write_text('gain: 9\n')
    assert (output.parent / 'python/controller.py').read_text() == 'gain = 2\n'
    assert Path(saved['files']['control']['copy']).read_text() == 'gain: 2\n'
