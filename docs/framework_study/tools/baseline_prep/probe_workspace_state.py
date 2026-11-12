#!/usr/bin/env python3
"""Which code would a Gazebo run execute today? Read-only probe of the container workspace. Host.

Baseline preparation (2026-11-12), before any approved rebuild. Extends
`stage7_verification/check_install_provenance.py` (F-7-4) to the in-scope dependencies, and works on
a **stopped** container: every read goes through `docker cp <c>:<path> -`, which streams a tar to
stdout and neither starts nor changes the container. Nothing is written in the repository or the
container; bytecode caching is disabled.

Sections (all INFO/FACT; there is no pass/fail, the output is evidence):
1. container: state, image, bind mounts;
2. colcon history: the last build command that selected each in-scope package (`log/build_*`);
3. install mode per package: regular files (copies) vs symlinks, and where the symlinks point
   (`/ros2_ws/src` = follows the checked-out tree; `/ros2_ws/build` = whatever was last compiled);
4. every **copied** install file of the in-scope packages, byte-compared with `git cat-file` blobs at
   the baseline `ba68077` and at `42df80e` (`fr3-student-sim`);
5. `franka_msgs`: field names and order of the generated IDL in `build/` against the baseline `.msg`;
6. compiled plugins in `build/`: mtime, the commit checked out at that time (`git reflog`), and
   whether that commit's package sources equal the baseline's (dirty trees cannot be excluded).

Environment: FS_CONTAINER (default franka_ros2).
Usage: python3 -I -B docs/framework_study/tools/baseline_prep/probe_workspace_state.py
"""
import collections
import io
import json
import os
import re
import subprocess
import sys
import tarfile
import time
from datetime import datetime, timezone

sys.dont_write_bytecode = True
REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..'))
C = os.environ.get('FS_CONTAINER', 'franka_ros2')
BASE, OTHER = 'ba68077', '42df80e'
PKGS = ['franka_msgs', 'franka_rt_controllers', 'franka_simulation', 'franka_experiments',
        'franka_simulation_experiments', 'franka_ign_ros2_control']
PKGDIR = {'franka_msgs': 'franka_msgs', 'franka_rt_controllers': 'franka_rt_controllers',
          'franka_simulation': 'franka_simulation', 'franka_experiments': 'franka_experiments',
          'franka_simulation_experiments': 'franka_simulation_experiments',
          'franka_ign_ros2_control': 'franka_gazebo/franka_ign_ros2_control'}
PLUGINS = [('franka_rt_controllers', 'libfranka_rt_controllers.so'),
           ('franka_rt_controllers', 'libcbf_torque_controller.so'),
           ('franka_simulation', 'libfranka_scenario_obstacles.so'),
           ('franka_ign_ros2_control', 'libfranka_ign_ros2_control-system.so')]


def sh(*a, inp=None):
    return subprocess.run(a, capture_output=True, input=inp)


def git(*a):
    return sh('git', '-C', REPO, *a).stdout.decode()


def blob(rev, path):
    r = sh('git', '-C', REPO, 'cat-file', 'blob', f'{rev}:{path}')
    return r.stdout if r.returncode == 0 else None


def dcp(path):
    """tarfile of `docker cp C:path -` (in memory), or None."""
    r = sh('docker', 'cp', f'{C}:{path}', '-')
    return tarfile.open(fileobj=io.BytesIO(r.stdout)) if r.returncode == 0 else None


def utc(ts):
    return datetime.fromtimestamp(ts, timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')


def section(t):
    print(f'\n== {t}')


def main():
    section('1. container')
    r = sh('docker', 'inspect', C)
    if r.returncode:
        print(f'INFO container {C!r} not found: nothing to probe'); return 2
    info = json.loads(r.stdout)[0]
    st = info['State']
    print(f"FACT name={C} status={st['Status']} exit={st['ExitCode']} started={st['StartedAt'][:19]} "
          f"finished={st['FinishedAt'][:19]} image={info['Image'][7:19]}")
    for m in info['Mounts']:
        print(f"FACT mount {m['Source']} -> {m['Destination']} rw={m['RW']}")

    section('2. colcon history (last build that selected each package)')
    logs = dcp('/ros2_ws/log')
    cmds = {}
    if logs:
        for m in logs.getmembers():
            if m.name.endswith('/logger_all.log') and '/build_' in m.name:
                txt = logs.extractfile(m).read().decode(errors='replace')
                g = re.search(r"Command line arguments: (\[.*?\])", txt)
                if g:
                    cmds[m.name.split('/')[1]] = g.group(1)
    builds = sorted(cmds)
    print(f'FACT {len(builds)} builds, last {builds[-1] if builds else None}')
    for p in PKGS:
        last = None
        for b in builds:
            a = cmds[b]
            if f"'{p}'" in a or ('packages-select' not in a and 'packages-up-to' not in a):
                last = b
        print(f'FACT {p:32s} {last}: {cmds.get(last, "")[:150]}')

    section('3. install mode per package')
    inst = dcp('/ros2_ws/install')
    members = inst.getmembers()
    for p in PKGS:
        c = collections.Counter()
        for m in members:
            if m.name.startswith(f'install/{p}/'):
                if m.isfile():
                    c['copy'] += 1
                elif m.issym():
                    t = m.linkname
                    c['sym->src' if t.startswith('/ros2_ws/src') else
                      'sym->build' if t.startswith('/ros2_ws/build') else 'sym->other'] += 1
        print(f'FACT {p:32s} {dict(c)}')

    section(f'4. copied install files vs {BASE} (baseline) and {OTHER} (fr3-student-sim)')
    skip = ('__pycache__', '.pyc', '.dsv', '.ps1', '.sh', '.bash', '.zsh', '.pth', 'colcon-core',
            'egg-info', 'ament_index', 'hook/', '/cmake/', '.so')
    res = collections.defaultdict(list)
    for m in members:
        parts = m.name.split('/')
        if not m.isfile() or len(parts) < 3 or parts[1] not in PKGDIR or parts[1] == 'franka_msgs':
            continue
        pkg, rel = parts[1], '/'.join(parts[2:])
        if any(s in rel for s in skip):
            continue
        d, q = PKGDIR[pkg], rel.split('/')
        cands = []
        if q[0] == 'share' and len(q) > 2 and q[1] == pkg:
            cands = [f'{d}/' + '/'.join(q[2:])]
        elif q[0] == 'lib' and len(q) >= 3 and q[1] == pkg:
            r_ = '/'.join(q[2:])
            cands = [f'{d}/scripts/{r_}.py', f'{d}/scripts/{r_}']
        elif 'site-packages' in q:
            cands = [f'{d}/' + '/'.join(q[q.index('site-packages') + 1:])]
        data = inst.extractfile(m).read()
        hit = None
        for cnd in cands:
            b = (blob(BASE, cnd), blob(OTHER, cnd))
            if b != (None, None):
                hit = (cnd, b)
                break
        if hit is None:
            res[(pkg, 'no source in either revision (entry-point wrapper or leftover)')].append(rel)
            continue
        cnd, (bb, bo) = hit
        tag = ('=base' if bb == data else 'absent@base' if bb is None else '!=base') + ' ' + \
              ('=42df' if bo == data else 'absent@42df' if bo is None else '!=42df')
        res[(pkg, tag)].append(cnd)
    for (pkg, tag), v in sorted(res.items()):
        show = '' if tag.startswith('=base =42df') else ' ' + ', '.join(os.path.basename(x) for x in v)
        print(f'FACT {pkg:30s} {tag:22s} {len(v):4d}{show}')

    section('5. franka_msgs: generated IDL (build/) vs baseline .msg, field names and order')
    idl = dcp('/ros2_ws/build/franka_msgs/rosidl_adapter/franka_msgs/msg')
    n = bad = 0
    for m in (idl.getmembers() if idl else []):
        if not m.name.endswith('.idl'):
            continue
        name = os.path.basename(m.name)[:-4]
        src = blob(BASE, f'franka_msgs/msg/{name}.msg')
        n += 1
        if src is None:
            print(f'FACT {name}: not in the baseline'); bad += 1; continue
        mf = []
        for line in src.decode().splitlines():
            line = line.split('#')[0].strip()
            tok = line.split()
            if len(tok) >= 2 and '=' not in line:
                mf.append(tok[1])
        body = idl.extractfile(m).read().decode()
        body = re.sub(r'@verbatim\s*\(.*?\)\s*', '', body, flags=re.S)
        body = re.sub(r'^\s*typedef .*$', '', body, flags=re.M)
        fields = re.findall(r'^\s*[\w:<>, ]+?\s+(\w+)(?:\[\d+\])?;\s*$', body, flags=re.M)
        if fields != mf:
            bad += 1
            print(f'FACT {name}: DIFFERS\n  msg {mf}\n  idl {fields}')
    print(f'FACT {n} generated messages compared, {bad} differ from the baseline')

    section('6. compiled plugins in build/ (cannot be byte-compared with source)')
    reflog = []
    for line in git('reflog', '--date=unix', '--format=%gd %H').splitlines():
        g = re.match(r'HEAD@\{(\d+)\} (\w+)', line)
        if g:
            reflog.append((int(g.group(1)), g.group(2)))
    for pkg, so in PLUGINS:
        t = dcp(f'/ros2_ws/build/{pkg}/{so}')
        if not t:
            print(f'FACT {so}: not found'); continue
        mt = t.getmembers()[0].mtime
        head = next((h for ts, h in reflog if ts <= mt), None)
        same = None
        if head:
            same = sh('git', '-C', REPO, 'diff', '--quiet', head, BASE, '--', PKGDIR[pkg]).returncode == 0
        print(f'FACT {so:40s} built {utc(mt)}; HEAD then {head[:7] if head else "?"}; '
              f'{PKGDIR[pkg]} at that HEAD == baseline: {same} (an uncommitted tree cannot be excluded)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
