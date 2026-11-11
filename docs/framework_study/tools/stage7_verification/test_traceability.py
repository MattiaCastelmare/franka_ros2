#!/usr/bin/env python3
"""Static test-to-component traceability (chapter 14 §7, REFACTORING_READINESS §6–7). Host.

Read-only, nothing is executed: every test file is parsed with `ast`. For each one it records

* the number of test functions (`test_*` at module level or in `Test*` classes), and pytest markers
  (`slow`, `skip`, `skipif`, `xfail`);
* the **infrastructure** it needs, from its imports and string literals:
  - `ros`      : `rclpy`, `launch`, `launch_ros`, `launch_testing`, `*_msgs`, `ament_index_python`;
  - `pinocchio`: `pinocchio` (or `hppfcl`/`coal`);
  - `gazebo`   : the `slow` marker, or `ros2 launch`/`gz`/`ign` command strings;
  - `ml`       : `torch`, `onnxruntime`, `stable_baselines3`, `gymnasium`, `mujoco`;
  - `data`     : paths into `sim_runs/`, `franka_logs/`, bags (`.db3`, `.mcap`);
  - `textual`  : it reads production source as TEXT (`open(...py)`, `read_text`, `inspect.getsource`)
    — such a test pins wording, not behaviour;
* the **repository modules** it imports (resolved like `inventory.py`) and, for `from M import a, b`,
  whether every imported name is still defined at the top level of M (`def`, `class`, assignment, or
  re-export). A missing name means the test cannot even be collected at the baseline: **stale**.

Writes `results/test_traceability.json` and `results/test_traceability.md` (component → tests and
test → components). Prints a summary. No PASS/FAIL: this script produces evidence, it does not
decide; `check_docs.py` asserts the facts the chapters state from it.
Usage: `python3 -I -B docs/framework_study/tools/stage7_verification/test_traceability.py`
"""
from __future__ import annotations

import ast
import glob
import json
import os
import re
import sys
from collections import defaultdict

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import inventory as INV  # noqa: E402

REPO = INV.REPO
ROS = {'rclpy', 'launch', 'launch_ros', 'launch_testing', 'ament_index_python', 'rosbag2_py', 'tf2_ros',
       'cv_bridge', 'rcl_interfaces', 'builtin_interfaces', 'controller_manager_msgs'}
PIN = {'pinocchio', 'hppfcl', 'coal'}
ML = {'torch', 'onnxruntime', 'stable_baselines3', 'gymnasium', 'mujoco'}


def top_level_names(path):
    try:
        tree = ast.parse(open(os.path.join(REPO, path)).read())
    except (SyntaxError, FileNotFoundError):
        return None
    names = set()
    for n in tree.body:
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(n.name)
        elif isinstance(n, (ast.Assign, ast.AnnAssign)):
            for t in (n.targets if isinstance(n, ast.Assign) else [n.target]):
                for x in ast.walk(t):
                    if isinstance(x, ast.Name):
                        names.add(x.id)
        elif isinstance(n, (ast.Import, ast.ImportFrom)):
            for a in n.names:
                names.add((a.asname or a.name).split('.')[0])
        elif isinstance(n, (ast.If, ast.Try)):
            for x in ast.walk(n):
                if isinstance(x, (ast.FunctionDef, ast.ClassDef)):
                    names.add(x.name)
                elif isinstance(x, ast.Assign):
                    for t in x.targets:
                        if isinstance(t, ast.Name):
                            names.add(t.id)
                elif isinstance(x, (ast.Import, ast.ImportFrom)):
                    for a in x.names:
                        names.add((a.asname or a.name).split('.')[0])
    return names


def analyse(path, mods, smaps):
    src = open(os.path.join(REPO, path), errors='replace').read()
    tree = ast.parse(src)
    n_tests = 0
    for n in tree.body:
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name.startswith('test'):
            n_tests += 1
        elif isinstance(n, ast.ClassDef) and n.name.startswith('Test'):
            n_tests += sum(1 for b in n.body if isinstance(b, (ast.FunctionDef, ast.AsyncFunctionDef))
                           and b.name.startswith('test'))
    markers = sorted(set(re.findall(r'pytest\.mark\.(\w+)', src)))
    top = set()
    repo_imports = defaultdict(set)
    sdir = os.path.dirname(path)
    smap = {}
    if path.startswith('franka_simulation_experiments/test'):
        smap = {**smaps[os.path.join(INV.FSE, 'scripts')], **smaps.get(os.path.join(INV.FSE, 'test', 'slow'), {})}
    elif path.startswith('franka_experiments/test'):
        smap = smaps[os.path.join(INV.FE, 'scripts')]
    # test-local helper modules (e.g. `_cbf_builder_harness`, `_launch_snapshot`), resolved transitively
    smap = {**smap, **{os.path.basename(q)[:-3]: os.path.relpath(q, REPO)
                       for q in glob.glob(os.path.join(REPO, sdir, '*.py'))}}
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            for a in n.names:
                top.add(a.name.split('.')[0])
                if a.name in mods:
                    repo_imports[mods[a.name]]
                elif a.name.split('.')[0] in smap:
                    repo_imports[smap[a.name.split('.')[0]]]
        elif isinstance(n, ast.ImportFrom) and n.module:
            top.add(n.module.split('.')[0])
            target = mods.get(n.module) or (smap.get(n.module.split('.')[0]) if n.module.split('.')[0] in smap else None)
            if target:
                for a in n.names:
                    sub = f'{n.module}.{a.name}'
                    if sub in mods:                       # `from pkg.utils import module`
                        repo_imports[mods[sub]]
                    else:
                        repo_imports[target].add(a.name)
            elif n.module.endswith('_msgs') or n.module.split('.')[0].endswith('_msgs'):
                top.add('ros_msgs')
    infra = set()
    if top & ROS or 'ros_msgs' in top or any(t.endswith('_msgs') for t in top):
        infra.add('ros')
    if top & PIN:
        infra.add('pinocchio')
    if top & ML:
        infra.add('ml')
    if 'slow' in markers or re.search(r"['\"]ros2['\"],\s*['\"]launch|ign gazebo|gz sim", src):
        infra.add('gazebo')
    if re.search(r'sim_runs|franka_logs|\.db3|\.mcap', src):
        infra.add('data')
    if re.search(r"inspect\.getsource|read_text\(|open\([^)]*\.(py|yaml|launch\.py)", src) or \
            re.search(r"\.py['\"]\)?\s*\)?\.read\(\)", src):
        infra.add('textual')
    stale = {}
    for mpath, names in repo_imports.items():
        defined = top_level_names(mpath)
        if defined is None:
            stale[mpath] = ['<module missing>']
            continue
        miss = sorted(x for x in names if x != '*' and x not in defined)
        if miss:
            stale[mpath] = miss
    if re.search(r'[Mm]irror of|reproduced here|[Ii]nline the function|copied (?:verbatim )?from|re-?implement', src):
        infra.add('replica')
    if re.search(r"config['\"],\s*['\"]\w+\.yaml|fr3_control\.yaml|conditions\.yaml|scenarios\.yaml|launch_defaults\.yaml", src):
        infra.add('config')
    doc = (ast.get_docstring(tree) or '').strip().split('\n')[0][:140]
    return {'n_tests': n_tests, 'markers': markers, 'infra': sorted(infra),
            'imports': sorted(repo_imports), 'stale': stale, 'doc': doc}


def main():
    mods, smaps = INV.module_map(), INV.script_maps()
    ledger = {e['path']: e for e in INV.load_ledger()}
    tests = sorted(p for p in ledger if p.endswith('.py') and '/test/' in p
                   and os.path.basename(p) not in ('__init__.py',))
    out = {}
    for p in tests:
        try:
            out[p] = analyse(p, mods, smaps)
        except SyntaxError as e:
            out[p] = {'error': str(e)}
        out[p]['status'] = ledger[p].get('status', 'pending')
    helpers = {}
    for p, d in out.items():
        if os.path.basename(p).startswith('_') or os.path.basename(p) == 'conftest.py':
            helpers[p] = d.get('imports', [])
    for p, d in out.items():
        seen, todo = set(d.get('imports', [])), list(d.get('imports', []))
        while todo:
            q = todo.pop()
            if q not in out and q.endswith('.py') and '/test/' in q:
                try:
                    out_q = analyse(q, mods, smaps)
                except SyntaxError:
                    continue
                helpers[q] = out_q['imports']
            for r in helpers.get(q, []):
                if r not in seen:
                    seen.add(r)
                    todo.append(r)
        d['imports_transitive'] = sorted(x for x in seen if '/test/' not in x)
    comp = defaultdict(list)
    for p, d in out.items():
        for m in d.get('imports_transitive', []):
            comp[m].append(p)
    res = os.path.join(HERE, 'results')
    os.makedirs(res, exist_ok=True)
    with open(os.path.join(res, 'test_traceability.json'), 'w') as f:
        json.dump({'tests': out, 'component_to_tests': comp}, f, indent=1, sort_keys=True)
    lines = ['# Test traceability (generated by test_traceability.py; static evidence)', '',
             '## Test → components', '',
             '| Test file | Tests | Infra | Markers | Imports (repo) | Stale imports |', '|---|---:|---|---|---|---|']
    for p, d in out.items():
        lines.append(f"| `{p}` | {d.get('n_tests', '?')} | {','.join(d.get('infra', [])) or 'pure'} | "
                     f"{','.join(d.get('markers', []))} | "
                     f"{', '.join(os.path.basename(m) for m in d.get('imports_transitive', []))} | "
                     f"{'; '.join(f'{os.path.basename(k)}: {v}' for k, v in d.get('stale', {}).items())} |")
    lines += ['', '## Component → tests', '', '| Component | # | Tests |', '|---|---:|---|']
    for m in sorted(comp, key=lambda x: (-len(comp[x]), x)):
        lines.append(f"| `{m}` | {len(comp[m])} | {', '.join(os.path.basename(t) for t in comp[m])} |")
    with open(os.path.join(res, 'test_traceability.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    n = len(out)
    nt = sum(d.get('n_tests', 0) for d in out.values())
    infra = defaultdict(int)
    for d in out.values():
        for i in d.get('infra', []) or ['pure']:
            infra[i] += 1
    stale = {p: d['stale'] for p, d in out.items() if d.get('stale')}
    print(f'{n} test files, {nt} test functions; infra: {dict(infra)}')
    none = [p for p, d in out.items() if not d.get('imports_transitive')]
    print(f'{len(none)} files reach no production module (replica, config-only, textual or helper):')
    for p in none:
        print(f"  {p}: {out[p].get('infra')}")
    print(f"replica: {[os.path.basename(p) for p, d in out.items() if 'replica' in d.get('infra', [])]}")
    print(f'{len(stale)} files with stale imports:')
    for p, s in stale.items():
        print(f'  {p}: {s}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
