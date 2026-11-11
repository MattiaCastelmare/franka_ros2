#!/usr/bin/env python3
"""Which Git revision is the container's install tree a build of? (chapter 14 §5, F-6-3, F-7-4). Host.

Read-only. Streams the two packages' install trees out of the container with
`docker exec <c> tar -cf - ...` (nothing is written in the container or in the repository), maps
every installed file back to its source path, and compares its bytes with `git cat-file` blobs of
each candidate revision:

* `franka_experiments`:
  - `share/franka_experiments/{config,launch,scripts}/*` ← `franka_experiments/{config,launch,scripts}/*`
    (copied by `setup.py` `data_files`);
  - `share/franka_experiments/test/{launch,config}/*` ← `franka_experiments/test/...`;
  - `lib/python3.10/site-packages/franka_experiments/**.py` ← `franka_experiments/franka_experiments/**.py`;
* `franka_simulation_experiments` (`CMakeLists.txt`):
  - `share/.../{launch,config}/**` ← `franka_simulation_experiments/{launch,config}/**`;
  - `lib/franka_simulation_experiments/<stem>` ← `scripts/<stem>.py` (`install(PROGRAMS … RENAME)`);
  - `lib/franka_simulation_experiments/simexp/*.py` ← `scripts/simexp/*.py`.

Revisions compared (default): the documented baseline `ba68077` and every local branch tip. The
result is the set of revisions with which **every** mapped file is byte-identical.

Checks:
1. FACT: the install tree was read and at least 100 files were mapped.
2. FACT: the install is NOT a build of the baseline (some mapped file differs). Written to pass while
   F-6-3 holds; after an approved rebuild it FAILs, which is the expected change (re-assess F-6-3).
3. FACT: no revision matches every file (a mixed tree); the best match is `42df80e`
   (`fr3-student-sim`), and most files that differ from it do not exist in that revision at all:
   they were left by earlier builds of other revisions, because a copy-mode (`symlink_install=False`)
   colcon install adds and overwrites but never deletes (F-7-4).

Environment: FS_CONTAINER (default franka_ros2), FS_INSTALL (default /ros2_ws/install).
Exit 2 (skip) when the container is not running.
"""
import io
import os
import subprocess
import sys
import tarfile

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'stage2_verification'))
import _common as C  # noqa: E402

CONTAINER = os.environ.get('FS_CONTAINER', 'franka_ros2')
INSTALL = os.environ.get('FS_INSTALL', '/ros2_ws/install')
BASELINE = 'ba68077'
EXPECTED = '42df80e'


def git(*a):
    return subprocess.run(['git', '-C', C.REPO, *a], capture_output=True, check=True).stdout


def map_install(name):
    """installed path (relative to INSTALL) -> repository source path, or None."""
    parts = name.split('/')
    if parts[0] == 'franka_experiments':
        if parts[1:3] == ['share', 'franka_experiments'] and len(parts) >= 5 and parts[3] in ('config', 'launch', 'scripts'):
            return '/'.join(['franka_experiments'] + parts[3:])
        if parts[1:3] == ['share', 'franka_experiments'] and len(parts) >= 6 and parts[3] == 'test':
            return '/'.join(['franka_experiments'] + parts[3:])
        if len(parts) >= 5 and parts[1] == 'lib' and parts[3] == 'site-packages' and parts[4] == 'franka_experiments' \
                and name.endswith('.py'):
            return '/'.join(['franka_experiments'] + parts[4:])
    if parts[0] == 'franka_simulation_experiments':
        if parts[1:3] == ['share', 'franka_simulation_experiments'] and len(parts) >= 5 and parts[3] in ('config', 'launch'):
            return '/'.join(['franka_simulation_experiments'] + parts[3:])
        if parts[1:3] == ['lib', 'franka_simulation_experiments'] and len(parts) == 4:
            return f'franka_simulation_experiments/scripts/{parts[3]}.py'
        if parts[1:4] == ['lib', 'franka_simulation_experiments', 'simexp'] and name.endswith('.py'):
            return '/'.join(['franka_simulation_experiments', 'scripts'] + parts[3:])
    return None


def blobs(rev, paths):
    """{path: bytes or None} via one `git cat-file --batch` call."""
    req = ''.join(f'{rev}:{p}\n' for p in paths).encode()
    out = subprocess.run(['git', '-C', C.REPO, 'cat-file', '--batch'], input=req, capture_output=True,
                         check=True).stdout
    res, i = {}, 0
    for p in paths:
        j = out.index(b'\n', i)
        header = out[i:j].decode()
        if header.endswith('missing'):
            res[p], i = None, j + 1
            continue
        size = int(header.split()[2])
        res[p] = out[j + 1:j + 1 + size]
        i = j + 1 + size + 1
    return res


def main():
    R = C.Report('Stage 7 — install-tree provenance (ch. 14 §5; F-6-3, F-7-4)')
    up = subprocess.run(['docker', 'inspect', '-f', '{{.State.Running}}', CONTAINER], capture_output=True, text=True)
    if up.stdout.strip() != 'true':
        R.info(f'container {CONTAINER!r} not running: skipped')
        return 2
    tar_bytes = subprocess.run(['docker', 'exec', CONTAINER, 'tar', '-C', INSTALL, '-cf', '-',
                                'franka_experiments', 'franka_simulation_experiments'],
                               capture_output=True, check=True).stdout
    installed = {}
    with tarfile.open(fileobj=io.BytesIO(tar_bytes)) as tf:
        for m in tf.getmembers():
            if not m.isfile() or '__pycache__' in m.name:
                continue
            src = map_install(m.name)
            if src:
                installed[src] = tf.extractfile(m).read()
    R.fact('install tree read and mapped', len(installed) >= 100, f'{len(installed)} installed files mapped to source paths')
    branches = git('for-each-ref', '--format=%(refname:short) %(objectname:short)', 'refs/heads').decode().split('\n')
    revs = {BASELINE: 'baseline'}
    for line in filter(None, branches):
        b, h = line.split()
        revs.setdefault(h[:7], b)
    paths = sorted(installed)
    match, detail = {}, {}
    for rev, label in revs.items():
        bl = blobs(rev, paths)
        same = [p for p in paths if bl[p] == installed[p]]
        missing = [p for p in paths if bl[p] is None]
        differ = [p for p in paths if bl[p] is not None and bl[p] != installed[p]]
        match[rev], detail[rev] = len(same), (missing, differ)
        R.info(f'{rev} ({label}): {len(same)}/{len(paths)} identical, {len(differ)} differ, '
               f'{len(missing)} not in that revision')
    base_same = match[BASELINE]
    R.fact('install is NOT a build of the baseline (F-6-3 holds)', base_same < len(paths),
           f'{len(paths) - base_same} of {len(paths)} files differ from or are absent in {BASELINE}')
    full = [r for r, n in match.items() if n == len(paths)]
    best = max(match, key=match.get)
    missing, differ = detail[best]
    R.fact('the install is a MIXED tree: no revision matches every file; the best match is 42df80e '
           '(fr3-student-sim) and most of its exceptions are files that revision does not contain '
           '(left behind by earlier builds: a copy-mode install never deletes)',
           not full and best.startswith(EXPECTED) and len(missing) > len(differ),
           f'best {best}: {match[best]}/{len(paths)} identical; {len(missing)} absent from it, '
           f'{len(differ)} differing')
    for p in missing:
        R.info(f'  stale (absent from {best}): {p}')
    for p in differ:
        R.info(f'  differs from {best}: {p}')
    if EXPECTED not in revs:
        R.info(f'{EXPECTED} is not a local branch tip any more; compare it explicitly')
    return R.finish()


if __name__ == '__main__':
    sys.exit(main())
