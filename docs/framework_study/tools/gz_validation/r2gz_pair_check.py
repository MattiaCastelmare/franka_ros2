#!/usr/bin/env python3
"""R2 Gazebo static validation (C-078): pair-identity check. Host, read-only.

Usage:  python3 -I r2gz_pair_check.py <run_dir_A> <run_dir_B> <new_out_json>

Verifies, from what each run actually used (not from what it was asked to use), that two recorded runs
differ ONLY in the executor selection:
  P1 launch parameter files (every /tmp/launch_params_* the launch wrote, copied into run/container_tmp):
     equal as a multiset after normalising (a) the run's own cell directory, (b) random /tmp names and
     (c) the value of an `executor_mode:` key; exactly one file per run carries that key;
  P2 the cell's own configuration files (top-level *.yaml / *.urdf in the cell directory: controller,
     robot and description files the stack loads) equal after the same normalisation;
  P3 manifest: scenario_resolved (obstacle geometry, q0, T, scene), seed, perception, episode_start,
     motion and plant equal; condition equal except the executor keys; git sha equal and dirty false;
  P4 the filter's start-up configuration lines in launch.log equal except the executor line;
  P5 commander parameters read by the H2 supervisor equal (h2_metrics.commander_params);
  P6 the executor each run started (start-up line) is recorded; the arms must differ there.
PASS only if P1-P5 hold and P6 shows two different executors. Writes one JSON (refuses to overwrite).
"""
import collections
import glob
import json
import os
import re
import sys

sys.dont_write_bytecode = True


def die(msg):
    sys.exit(f'r2gz_pair_check: {msg}')


if len(sys.argv) != 4:
    die('usage: r2gz_pair_check.py <run_dir_A> <run_dir_B> <new_out_json>')
RA, RB, OUT = sys.argv[1:]
if os.path.exists(OUT):
    die(f'{OUT} exists')

EXEC_KEY = re.compile(r'^(\s*executor_mode:\s*)\S+\s*$', re.M)


def cell_of(run):
    c = sorted(glob.glob(os.path.join(run, 'run', 'batch', 'cells', '*')))
    if len(c) != 1:
        die(f'{run}: expected one cell, found {len(c)}')
    return c[0]


def container_cell_path(run, cell):
    with open(os.path.join(cell, 'run_manifest_sim.json')) as f:
        rd = json.load(f).get('run_dir')
    return rd


def norm(text, cpath):
    if cpath:
        text = text.replace(cpath, '<CELL>')
    text = re.sub(r'/home/user/[A-Za-z0-9_]+_2026\d{4}/[0-9]{2}_s[012]_[a-z]+/batch/cells/[A-Za-z0-9_=.\-]+',
                  '<CELL>', text)
    text = re.sub(r'/tmp/[A-Za-z0-9_.\-]+', '/tmp/<TMP>', text)
    return EXEC_KEY.sub(r'\1<EXECUTOR>', text)


def run_info(run):
    cell = cell_of(run)
    cpath = container_cell_path(run, cell)
    lp = {}
    for p in sorted(glob.glob(os.path.join(run, 'run', 'container_tmp', 'tmp__launch_params_*'))):
        with open(p, errors='replace') as f:
            lp[os.path.basename(p)] = f.read()
    carriers = {k: EXEC_KEY.findall(v) for k, v in lp.items() if EXEC_KEY.search(v)}
    exec_vals = sorted(re.findall(r'^\s*executor_mode:\s*(\S+)\s*$', '\n'.join(lp.values()), re.M))
    cfg = {}
    for p in sorted(glob.glob(os.path.join(cell, '*.yaml')) + glob.glob(os.path.join(cell, '*.urdf'))):
        with open(p, errors='replace') as f:
            cfg[os.path.basename(p)] = norm(f.read(), cpath)
    with open(os.path.join(cell, 'run_manifest_sim.json')) as f:
        man = json.load(f)
    with open(os.path.join(cell, 'launch.log'), errors='replace') as f:
        log = f.read().splitlines()
    startup = [re.sub(r'^\[[^\]]*\]\s*', '', re.sub(r'\[\d+\.\d+\]', '', x)).strip()
               for x in log if '[cbf_safety_filter' in x and ('d_safe=' in x or 'horizon' in x or 'qp_rate' in x)]
    execline = sorted(set(re.findall(r'executor_mode=(\w+) \((\w+)\)', '\n'.join(log))))
    h2 = os.path.join(run, 'analysis', 'h2_metrics.json')
    cmd = None
    if os.path.exists(h2):
        with open(h2) as f:
            cmd = json.load(f).get('commander_params')
    return dict(cell=cell, cpath=cpath, lp=collections.Counter(norm(v, cpath) for v in lp.values()),
                n_lp=len(lp), carriers=carriers, exec_vals=exec_vals, cfg=cfg, man=man,
                startup=startup, execline=execline, commander=cmd)


A, B = run_info(RA), run_info(RB)
checks = []


def check(cid, ok, detail):
    checks.append(dict(id=cid, verdict='PASS' if ok else 'FAIL', detail=detail))


onlyA, onlyB = A['lp'] - B['lp'], B['lp'] - A['lp']
check('P1', not onlyA and not onlyB and len(A['carriers']) == 1 and len(B['carriers']) == 1,
      dict(files=[A['n_lp'], B['n_lp']], only_A=[t[:300] for t in onlyA], only_B=[t[:300] for t in onlyB],
           executor_values=[A['exec_vals'], B['exec_vals']],
           carriers=[list(A['carriers']), list(B['carriers'])]))
names = sorted(set(A['cfg']) | set(B['cfg']))
diff_cfg = [n for n in names if A['cfg'].get(n) != B['cfg'].get(n)]
check('P2', bool(names) and not diff_cfg, dict(files=names, differing=diff_cfg))

ma, mb = A['man'], B['man']
keys = ['scenario_resolved', 'seed', 'perception', 'perception_mode', 'episode_start', 'motion', 'plant',
        'scene', 'scenario', 'emulated_fps', 'duration_s']
mdiff = [k for k in keys if ma.get(k) != mb.get(k)]


def strip_exec(c):
    c = json.loads(json.dumps(c))
    for path in (('cbf_params',), ('toggles', 'cbf_params')):
        d = c
        for k in path:
            d = d.get(k, {}) if isinstance(d, dict) else {}
        if isinstance(d, dict):
            d.pop('executor_mode', None)
    sa = c.get('stack_args', {})
    if isinstance(sa, dict) and 'cbf_param_overrides' in sa:
        sa['cbf_param_overrides'] = ';'.join(x for x in sa['cbf_param_overrides'].split(';')
                                             if not x.startswith('executor_mode='))
    c.pop('name', None)
    return c


cond_eq = strip_exec(ma.get('condition', {})) == strip_exec(mb.get('condition', {}))
git_ok = (ma.get('git', {}).get('sha') == mb.get('git', {}).get('sha') and ma.get('git', {}).get('dirty') is False
          and mb.get('git', {}).get('dirty') is False)
check('P3', not mdiff and cond_eq and git_ok,
      dict(manifest_keys_differing=mdiff, condition_equal_except_executor=cond_eq,
           git=[ma.get('git'), mb.get('git')],
           obstacle_p0=[[o.get('p0') for o in ma.get('scenario_resolved', {}).get('obstacles', [])],
                        [o.get('p0') for o in mb.get('scenario_resolved', {}).get('obstacles', [])]],
           q0=[ma.get('scenario_resolved', {}).get('q0'), mb.get('scenario_resolved', {}).get('q0')]))
check('P4', bool(A['startup']) and A['startup'] == B['startup'],
      dict(n=[len(A['startup']), len(B['startup'])],
           differing=[x for x in A['startup'] if x not in B['startup']] + [x for x in B['startup'] if x not in A['startup']]))
check('P5', A['commander'] is not None and A['commander'] == B['commander'],
      dict(present=[A['commander'] is not None, B['commander'] is not None]))
check('P6', len(A['execline']) == 1 and len(B['execline']) == 1 and A['execline'] != B['execline'],
      dict(A=A['execline'], B=B['execline']))

verdict = 'PASS' if all(c['verdict'] == 'PASS' for c in checks) else 'FAIL'
doc = dict(verdict=verdict, runs=[RA, RB], checks=checks)
with open(OUT, 'x') as f:
    json.dump(doc, f, indent=1, default=str)
print(f'r2gz_pair_check {verdict}: ' + '; '.join(f"{c['id']} {c['verdict']}" for c in checks))
sys.exit(0 if verdict == 'PASS' else 1)
