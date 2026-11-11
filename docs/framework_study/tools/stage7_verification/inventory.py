#!/usr/bin/env python3
"""Static evidence worksheet for every in-scope file (Stage 7 coverage audit, chapters 14–15). Host.

Read-only. For every file of the coverage ledger it collects the evidence that decides whether the
file runs and whether it has been documented:

* Python files (parsed with ``ast``, never imported):
  - the first docstring line, top-level classes and functions (with the methods of classes);
  - **static imports** resolved to repository files: `franka_experiments.*`, relative imports,
    `simexp.*`, and bare script-to-script imports inside `franka_experiments/scripts` and
    `franka_simulation_experiments/scripts` (the `sys.path` shims those directories use);
  - **string references**: string literals that name another in-scope Python file (`'x.py'`), a
    dotted module (`'franka_experiments.utils.x'`), or a console entry point. This catches
    `importlib`, subprocess calls of scripts, and launch `executable=` arguments;
* the **entry points**: `setup.py` `console_scripts` (franka_experiments) and the
  `install(PROGRAMS ...)` glob of `franka_simulation_experiments/CMakeLists.txt` (`scripts/*.py`
  installed under their stem);
* **launch references**: an entry point or a program name that appears as a string literal in a
  launch file of franka_experiments, franka_simulation_experiments or franka_simulation;
* **tests** that import the file or name its entry point;
* **chapter citations**: for each top-level definition, whether its name appears in a chapter
  (`docs/framework_study/[0-9][0-9]_*.md`). A citation is evidence that a chapter *mentions* the
  definition, not that it was reviewed: the ledger status is decided by hand (Stage 7 rule).

Writes `results/inventory.json` and `results/inventory.md` (both under this directory only).
Usage: `python3 -I -B docs/framework_study/tools/stage7_verification/inventory.py`
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
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
DOCS = os.path.join(REPO, 'docs', 'framework_study')
FE = 'franka_experiments'
FSE = 'franka_simulation_experiments'


def rel(p):
    return os.path.relpath(p, REPO)


def load_ledger():
    with open(os.path.join(DOCS, 'ledger', 'ledger.json')) as f:
        return json.load(f)['ledger']


def module_map():
    """dotted module name -> repo-relative path, for every importable Python file in scope."""
    m = {}
    for p in glob.glob(os.path.join(REPO, FE, FE, '**', '*.py'), recursive=True):
        r = rel(p)
        mod = r[len(FE) + 1:-3].replace(os.sep, '.')
        if mod.endswith('.__init__'):
            mod = mod[:-9]
        m[mod] = r
    for p in glob.glob(os.path.join(REPO, FSE, 'scripts', 'simexp', '*.py')):
        r = rel(p)
        mod = 'simexp.' + os.path.basename(p)[:-3]
        m['simexp' if mod == 'simexp.__init__' else mod] = r
    return m


def script_maps():
    """bare module name -> path, per scripts directory (sys.path shims)."""
    out = {}
    for d in (os.path.join(FE, 'scripts'), os.path.join(FSE, 'scripts'), os.path.join(FSE, 'test', 'slow')):
        out[d] = {os.path.basename(p)[:-3]: rel(p) for p in glob.glob(os.path.join(REPO, d, '*.py'))}
    return out


def entry_points():
    eps = {}
    src = open(os.path.join(REPO, FE, 'setup.py')).read()
    for name, mod in re.findall(r"'([\w]+)\s*=\s*([\w.]+):main'", src):
        eps[name] = os.path.join(FE, mod.replace('.', os.sep) + '.py')
    for p in glob.glob(os.path.join(REPO, FSE, 'scripts', '*.py')):
        eps[os.path.basename(p)[:-3]] = rel(p)
    return eps


def launch_files():
    pats = [os.path.join(REPO, FE, 'launch', '*.launch.py'), os.path.join(REPO, FSE, 'launch', '*.launch.py'),
            os.path.join(REPO, 'franka_simulation', 'launch', '*.launch.py'),
            os.path.join(REPO, FE, 'test', 'launch', '*.launch.py'), os.path.join(REPO, FSE, 'test', 'launch', '*.py')]
    return sorted(set(p for pat in pats for p in glob.glob(pat)))


def strings_of(tree):
    for n in ast.walk(tree):
        if isinstance(n, ast.Constant) and isinstance(n.value, str):
            yield n.value
        elif isinstance(n, ast.JoinedStr):
            for v in n.values:
                if isinstance(v, ast.Constant) and isinstance(v.value, str):
                    yield v.value


def analyse_py(path, mods, smaps, eps, basenames):
    src = open(os.path.join(REPO, path), errors='replace').read()
    try:
        tree = ast.parse(src)
    except SyntaxError as e:
        return {'error': f'SyntaxError: {e}'}
    doc = (ast.get_docstring(tree) or '').strip().split('\n')[0][:160]
    defs = []
    for n in tree.body:
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
            defs.append(n.name)
        elif isinstance(n, ast.ClassDef):
            defs.append(n.name)
            defs += [f'{n.name}.{b.name}' for b in n.body if isinstance(b, (ast.FunctionDef, ast.AsyncFunctionDef))]
    pkg_mod = None
    if path.startswith(os.path.join(FE, FE)):
        pkg_mod = path[len(FE) + 1:-3].replace(os.sep, '.')
    elif path.startswith(os.path.join(FSE, 'scripts', 'simexp')):
        pkg_mod = 'simexp.' + os.path.basename(path)[:-3]
    imports = set()
    sdir = os.path.dirname(path)
    smap = smaps.get(sdir, {})
    if sdir.startswith(os.path.join(FSE, 'test')):
        smap = {**smaps[os.path.join(FSE, 'scripts')], **smaps.get(os.path.join(FSE, 'test', 'slow'), {})}
    if sdir.startswith(os.path.join(FE, 'test')):
        smap = smaps[os.path.join(FE, 'scripts')]
    for n in ast.walk(tree):
        names = []
        if isinstance(n, ast.Import):
            names = [a.name for a in n.names]
        elif isinstance(n, ast.ImportFrom):
            base = n.module or ''
            if n.level and pkg_mod:
                parts = pkg_mod.split('.')[:-n.level]
                base = '.'.join(parts + ([base] if base else []))
            names = [base] + [f'{base}.{a.name}' for a in n.names]
        for nm in names:
            if nm in mods:
                imports.add(mods[nm])
            elif nm.split('.')[0] in smap:
                imports.add(smap[nm.split('.')[0]])
    strrefs = set()
    for s in strings_of(tree):
        s2 = s.strip()
        b = os.path.basename(s2)
        if b in basenames and b.endswith('.py'):
            strrefs.update(basenames[b])
        if s2 in mods:
            strrefs.add(mods[s2])
        if s2 in eps:
            strrefs.add(eps[s2])
    strrefs.discard(path)
    imports.discard(path)
    return {'doc': doc, 'defs': defs, 'imports': sorted(imports), 'strrefs': sorted(strrefs)}


def chapter_text():
    txt = {}
    for p in sorted(glob.glob(os.path.join(DOCS, '[0-9][0-9]_*.md'))):
        txt[os.path.basename(p)[:2]] = open(p).read()
    return txt


def main():
    ledger = load_ledger()
    paths = [e['path'] for e in ledger]
    mods, smaps, eps = module_map(), script_maps(), entry_points()
    basenames = defaultdict(set)
    for p in paths:
        if p.endswith('.py'):
            basenames[os.path.basename(p)].add(p)
    info = {}
    for e in ledger:
        p = e['path']
        d = {'status': e.get('status', 'pending'), 'ch': e['ch'], 'role': e['role'], 'lines': e['lines']}
        if p.endswith('.py') and os.path.isfile(os.path.join(REPO, p)):
            d.update(analyse_py(p, mods, smaps, eps, basenames))
        info[p] = d
    # launch references
    ep_by_path = defaultdict(list)
    for name, p in eps.items():
        ep_by_path[p].append(name)
    launch_refs = defaultdict(set)
    for lf in launch_files():
        try:
            strs = set(strings_of(ast.parse(open(lf).read())))
        except SyntaxError:
            continue
        for name, p in eps.items():
            if name in strs:
                launch_refs[p].add(rel(lf))
        for s in strs:
            b = os.path.basename(s.strip())
            for p in basenames.get(b, ()):
                launch_refs[p].add(rel(lf))
    # reverse edges
    importers, strref_by = defaultdict(set), defaultdict(set)
    for p, d in info.items():
        for q in d.get('imports', []):
            importers[q].add(p)
        for q in d.get('strrefs', []):
            strref_by[q].add(p)
    is_test = lambda p: '/test/' in p or os.path.basename(p).startswith('test_')  # noqa: E731
    ch = chapter_text()
    alltxt = '\n'.join(ch.values())
    for p, d in info.items():
        imp = importers.get(p, set())
        d['importers_prod'] = sorted(q for q in imp if not is_test(q))
        d['importers_test'] = sorted(q for q in imp if is_test(q))
        d['strref_by'] = sorted(strref_by.get(p, set()))
        d['entry_points'] = sorted(ep_by_path.get(p, []))
        d['launch'] = sorted(launch_refs.get(p, set()))
        tests = set(d['importers_test']) | {q for q in d['strref_by'] if is_test(q)}
        d['tests'] = sorted(tests)
        if 'defs' in d:
            cited = []
            for nm in d['defs']:
                short = nm.split('.')[-1]
                if len(short) > 3 and re.search(r'(?<![\w])' + re.escape(short) + r'(?![\w])', alltxt):
                    cited.append(nm)
            d['defs_cited'] = cited
        d['path_cited_in'] = sorted(k for k, t in ch.items() if os.path.basename(p) in t or p in t)
    out = os.path.join(HERE, 'results')
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'inventory.json'), 'w') as f:
        json.dump(info, f, indent=1, sort_keys=True)
    rows = ['| File | Status | Ch | Entry/launch | Prod importers | Tests | Defs cited / total | Cited in |',
            '|---|---|---|---|---|---|---|---|']
    for p in sorted(info):
        d = info[p]
        if not p.endswith('.py'):
            continue
        rows.append(f"| `{p}` | {d['status']} | {d['ch']} | {len(d['entry_points'])}/{len(d['launch'])} | "
                    f"{len(d['importers_prod']) + len([q for q in d['strref_by'] if '/test/' not in q])} | "
                    f"{len(d['tests'])} | {len(d.get('defs_cited', []))}/{len(d.get('defs', []))} | "
                    f"{','.join(d['path_cited_in'])} |")
    with open(os.path.join(out, 'inventory.md'), 'w') as f:
        f.write('# Stage 7 static inventory (generated by inventory.py; evidence, not a status)\n\n'
                + '\n'.join(rows) + '\n')
    n_py = sum(1 for p in info if p.endswith('.py'))
    print(f'inventory: {len(info)} ledger files, {n_py} Python; {len(eps)} entry points; '
          f'{len(launch_files())} launch files scanned -> results/inventory.{{json,md}}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
