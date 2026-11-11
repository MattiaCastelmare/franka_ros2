#!/usr/bin/env python3
"""Stage 0 extraction: read-only static analysis of franka_experiments and
franka_simulation_experiments.  Writes Markdown ledgers into OUT_DIR only.

Usage: python3 -I stage0_extract.py <repo_root> <out_dir>
"""
from __future__ import annotations

import ast
import collections
import os
import re
import subprocess
import sys

REPO = os.path.abspath(sys.argv[1])
OUT = os.path.abspath(sys.argv[2])
os.makedirs(os.path.join(OUT, 'ledger'), exist_ok=True)

PKGS = ['franka_experiments', 'franka_simulation_experiments']
EXCLUDE = re.compile(
    r'(/sim_runs/|/experiment_logs/|/franka_logs/|/paper_draft/|/Papers/|/Videos/|'
    r'/\.pytest_cache/|__pycache__)')

# Executed-path files of workspace dependencies (documented in full, Stage 4-6).
EXTERNAL_EXECUTED = [
    'franka_rt_controllers/src/rt_torque_controller.cpp',
    'franka_rt_controllers/src/cbf_torque_controller.cpp',
    'franka_rt_controllers/src/rt_velocity_blender_controller.cpp',
    'franka_rt_controllers/config/cbf_torque_controller.yaml',
    'franka_rt_controllers/config/rt_velocity_blender.yaml',
    'franka_rt_controllers/franka_rt_controllers.xml',
    'franka_rt_controllers/rt_velocity_blender_controller.xml',
    'franka_simulation/launch/gz_fr3_plant.launch.py',
    'franka_msgs/msg/MultiLinkDistance.msg',
    'franka_msgs/msg/LinkDistance.msg',
    'franka_msgs/msg/MultiDistance.msg',
    'franka_msgs/msg/HumanRobotDistance.msg',
]


def git_ls(paths):
    out = subprocess.run(['git', '-C', REPO, 'ls-files', *paths],
                         capture_output=True, text=True, check=True).stdout
    return [p for p in out.splitlines() if p]


def nlines(path):
    try:
        with open(os.path.join(REPO, path), 'rb') as f:
            return sum(1 for _ in f)
    except OSError:
        return 0


# ── chapter / role assignment ─────────────────────────────────────────────
CH = {
    '01': 'Architecture', '02': 'ROS graph & data flow', '03': 'Configuration',
    '04': 'Math foundations', '05': 'Perception & distance',
    '06': 'Nominal motion', '07': 'CBF obstacle barrier',
    '08': 'Constraints & QP', '09': 'Torque realization',
    '10': 'Alternative controllers', '11': 'ISO safety layer',
    '12': 'Simulation', '13': 'Observability & tools',
    '14': 'Build/launch/test', '16': 'Issues & discrepancies (lead docs)',
}
STAGE_OF = {'01': 1, '02': 1, '03': 1, '04': 2, '05': 2, '06': 3, '07': 3,
            '08': 4, '09': 4, '10': 5, '11': 5, '13': 5, '12': 6, '14': 7,
            '16': 7}

U = 'franka_experiments/franka_experiments/utils/'
N = 'franka_experiments/franka_experiments/nodes/'
UTIL_CH = {
    '04': ['kinematics', 'math_utils', 'constants', 'cbf_kinematics'],
    '05': ['distance_engine', 'distance_utils', 'tf_manager', 'mask_builder',
           'self_detection', 'self_shadow', 'obstacle_tracker',
           'obstacle_clusters', 'obstacle_track_pipeline', 'obstacle_sim',
           'perception_msgs', 'surface_velocity', 'ballistic_prediction',
           'static_scene', 'rtd_debug', 'handeye_solver', 'calibration_check',
           'camera_yaml'],
    '06': ['trajectory', 'timing_law', 'avoidance', 'avoidance_math',
           'evasion_direction', 'go_around', 'livelock', 'rl_policy'],
    '07': ['cbf_state_rows', 'cbf_constraints', 'cbf_zones', 'cbf_evasion',
           'cbf_utils'],
    '08': ['cbf_qp_assembly', 'cbf_qp', 'cbf_hard_limits', 'cbf_singularity',
           'singularity_recovery', 'self_collision', 'state_governor',
           'rate_scaling'],
    '11': ['iso_ssm'],
    '03': ['config', 'params', 'launch_support', 'ros', 'ros_setup',
           'node_runtime', 'node_utils', 'run_metadata', '__init__'],
    '13': ['logging_utils', 'visualization', 'trajectory_overlay',
           'trajectory_trace', 'cbf_diagnostics', 'simulation_imports'],
}
NODE_CH = {
    '05': ['real_time_distance', 'handeye_calibration_node',
           'handeye_eye_in_hand_node', 'image_rectifier_node', 'frame_grabber'],
    '06': ['pentagon_qddot_commander', 'rl_policy_commander',
           'ee_circle_velocity_commander', 'ee_pentagon_velocity_commander',
           'ee_random_waypoints_velocity_commander'],
    '08': ['cbf_safety_filter'],
    '09': ['qddot_to_torque'],
    '10': ['cbf_velocity_filter', 'cbf_OSCBF_filter', 'pentagon_torque_commander'],
    '11': ['iso_safety_monitor', 'iso_evidence_logger'],
    '13': ['experiment_logger', 'capsule_overlay_node', 'trajectory_overlay_node',
           'trajectory_visualization_node', 'distance_visualization_node',
           'tag_overlay_node', '__init__'],
}
TEST_KW = [  # (regex on test basename, chapter)
    (r'iso', '11'), (r'rl_policy|smoke_rl', '06'),
    (r'handeye|calibration', '05'),
    (r'obstacle|mask|self_detection|self_reject|sensor_range|distance_surface|'
     r'ssb_perception|link_distance|track_|range_noise|cp_label|ballistic|'
     r'self_shadow', '05'),
    (r'trajectory|go_around|livelock|evasion_direction|avoidance|'
     r'phase_governor|directional_fade|fr3_velocity_envelope', '06'),
    (r'cbf_velocity_filter|torque_stack_external', '10'),
    (r'hard_constraints|qp_fallback|row_l1|row_padding|singularity|'
     r'state_governor|rate_scaling|accel_box|brake_slew|safety_failure|'
     r'multi_cp_qp|risk_weighted|smoke_cbf|cbf_builder|retreat_excess|'
     r'shipped_smoothness|ssb_qp', '08'),
    (r'cbf_|vobs|multi_obstacle', '07'),
    (r'experiment_log|log_header|control_point_deriv|kinematics', '13'),
]


def assign(path):
    """Return (role, chapter, note)."""
    b = os.path.basename(path)
    stem = os.path.splitext(b)[0]
    if path.startswith(U):
        for ch, names in UTIL_CH.items():
            if stem in names:
                return 'util module', ch, ''
        return 'util module', '15', 'UNASSIGNED'
    if path.startswith(N):
        for ch, names in NODE_CH.items():
            if stem in names:
                return 'ROS node' if stem != '__init__' else 'package init', ch, ''
        if path.endswith('.py'):
            return 'ROS node', '15', 'UNASSIGNED'
    if path == 'franka_experiments/franka_experiments/__init__.py':
        return 'package init', '03', ''
    if '/launch/' in path and path.endswith('.launch.py') and '/test/' not in path:
        return 'launch file', ('12' if path.startswith('franka_simulation_exp') else '01'), ''
    if path.startswith('franka_experiments/config/'):
        if 'camera' in b or 'intrinsics' in b or 'extrinsic' in b or 'handeye' in b or 'ee_tag' in b:
            return 'config (calibration)', '05', ''
        if b.endswith('.rviz'):
            return 'config (rviz)', '13', ''
        if b.endswith('.md'):
            return 'doc (lead)', '11', ''
        return 'config (parameters)', '03', ''
    if path.startswith('franka_simulation_experiments/config/'):
        return 'config (sim experiment axes)', '12', ''
    if path.startswith('franka_experiments/scripts/'):
        if b.startswith('iso_'):
            return 'offline/aux script', '11', ''
        if b.startswith('check_control_derivatives'):
            return 'offline/aux script', '04', ''
        if b.endswith('.sh'):
            return 'shell script', '09', 'RT pinning'
        return 'offline/aux script', '13', ''
    if path.startswith('franka_simulation_experiments/scripts/simexp/'):
        return 'sim library (simexp)', '12', ''
    if path.startswith('franka_simulation_experiments/scripts/'):
        return 'sim node / tool', '12', ''
    if '/test/' in path:
        if path.startswith('franka_simulation_experiments/'):
            return 'test', '12', ''
        for rx, ch in TEST_KW:
            if re.search(rx, b):
                return 'test', ch, ''
        return 'test', '14', ''
    if path.startswith('franka_experiments/docs/'):
        if b.endswith('.png') or b.endswith('.svg'):
            return 'doc asset', '16', ''
        if b.endswith('.py'):
            return 'doc tool', '13', ''
        return 'doc (lead)', '16', ''
    if b in ('README.md', 'LEGACY.md', 'SAFETY.md', 'next_step.md'):
        return 'doc (lead)', '16', ''
    if b in ('setup.py', 'setup.cfg', 'package.xml', 'CMakeLists.txt',
             'MANIFEST.in', 'pytest.ini', '.gitignore') or path.endswith('/resource/franka_experiments'):
        return 'build metadata', '14', ''
    return 'other', '15', 'UNASSIGNED'


def assign_external(path):
    if path.endswith('.cpp') or path.endswith('.xml') or 'franka_rt_controllers/config' in path:
        ch = '10' if 'velocity_blender' in path else '09'
        return 'C++ controller (external, executed)', ch
    if path.endswith('.msg'):
        return 'message definition (external)', '02'
    return 'launch file (external, executed)', '12'


# ── AST helpers ───────────────────────────────────────────────────────────
def parse(path):
    try:
        with open(os.path.join(REPO, path), encoding='utf-8') as f:
            return ast.parse(f.read(), filename=path)
    except (SyntaxError, UnicodeDecodeError) as e:
        print('PARSE FAIL', path, e, file=sys.stderr)
        return None


def short(node, n=90):
    try:
        s = ast.unparse(node)
    except Exception:  # noqa: BLE001
        s = '?'
    s = ' '.join(s.split())
    return s if len(s) <= n else s[:n - 1] + '…'


def enclosing_map(tree):
    """node -> qualified enclosing def name."""
    owner = {}

    def walk(node, stack):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                walk(child, stack + [child.name])
            else:
                owner[child] = '.'.join(stack) or '<module>'
                walk(child, stack)
    walk(tree, [])
    return owner


ROS_CALLS = {
    'create_publisher': 'pub', 'create_subscription': 'sub',
    'create_service': 'srv', 'create_client': 'cli', 'create_timer': 'timer',
    'declare_parameter': 'param', 'declare_parameters': 'params',
    'ActionClient': 'action_cli', 'ActionServer': 'action_srv',
    'TransformListener': 'tf_listen', 'TransformBroadcaster': 'tf_bcast',
    'StaticTransformBroadcaster': 'tf_static', 'lookup_transform': 'tf_lookup',
    'sendTransform': 'tf_send', 'create_rate': 'rate',
    'MultiThreadedExecutor': 'executor', 'ReentrantCallbackGroup': 'cbgroup',
    'MutuallyExclusiveCallbackGroup': 'cbgroup',
}


def call_name(call):
    f = call.func
    if isinstance(f, ast.Attribute):
        return f.attr
    if isinstance(f, ast.Name):
        return f.id
    return None


def ros_iface(path, tree):
    owner = enclosing_map(tree)
    rows = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            name = call_name(node)
            if name in ROS_CALLS:
                args = [short(a, 70) for a in node.args[:3]]
                kws = [f'{k.arg}={short(k.value, 40)}' for k in node.keywords
                       if k.arg in ('qos_profile', 'callback_group', 'timeout_sec')]
                rows.append((ROS_CALLS[name], node.lineno, owner.get(node, '?'),
                             ', '.join(args + kws)))
    rows.sort(key=lambda r: r[1])
    return rows


def defs_of(tree):
    classes, funcs = [], []
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            meths = [n.name for n in node.body
                     if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]
            classes.append((node.name, node.lineno, len(meths)))
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            funcs.append((node.name, node.lineno))
    return classes, funcs


STDLIB = set(sys.stdlib_module_names) if hasattr(sys, 'stdlib_module_names') else set()


def imports_of(path, tree):
    """Return list of (module_string, lineno, toplevel?)."""
    out = []
    top = set(id(n) for n in tree.body)
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                out.append((a.name, node.lineno, id(node) in top))
        elif isinstance(node, ast.ImportFrom):
            mod = node.module or ''
            if node.level:
                # relative import -> resolve against path's package
                pkgdir = os.path.dirname(path)
                for _ in range(node.level - 1):
                    pkgdir = os.path.dirname(pkgdir)
                base = pkgdir.replace('/', '.')
                mod = base + ('.' + mod if mod else '')
                # also each imported name may be a submodule
                for a in node.names:
                    out.append((mod + '.' + a.name, node.lineno, id(node) in top))
            else:
                out.append((mod, node.lineno, id(node) in top))
                for a in node.names:
                    out.append((mod + '.' + a.name, node.lineno, id(node) in top))
    return out


# ── module resolution ─────────────────────────────────────────────────────
def module_index(py_files):
    """Map dotted names to repo paths for the internal resolvable modules."""
    idx = {}
    for p in py_files:
        if p.startswith('franka_experiments/franka_experiments/'):
            rel = p[len('franka_experiments/'):-3]
            dotted = rel.replace('/', '.')
            if dotted.endswith('.__init__'):
                dotted = dotted[:-9]
            idx[dotted] = p
        elif p.startswith('franka_simulation_experiments/scripts/'):
            rel = p[len('franka_simulation_experiments/scripts/'):-3]
            dotted = rel.replace('/', '.')
            if dotted.endswith('.__init__'):
                dotted = dotted[:-9]
            idx[dotted] = p
    return idx


def resolve(mod, src_path, idx):
    # repo-relative path from a relative import resolution
    if mod.startswith('franka_experiments.franka_experiments'):
        mod = mod[len('franka_experiments.'):]
    if mod.startswith('franka_simulation_experiments.scripts.'):
        mod = mod[len('franka_simulation_experiments.scripts.'):]
    parts = mod.split('.')
    for k in range(len(parts), 0, -1):
        cand = '.'.join(parts[:k])
        if cand in idx:
            return idx[cand]
    return None


# ── launch scanning ───────────────────────────────────────────────────────
def launch_scan(path, tree):
    nodes, includes, args = [], [], []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = call_name(node)
        kw = {k.arg: k.value for k in node.keywords if k.arg}
        if name in ('Node', 'ComposableNode', 'LifecycleNode'):
            nodes.append((node.lineno, short(kw.get('package', ast.Constant('?')), 40),
                          short(kw.get('executable', kw.get('plugin', ast.Constant('?'))), 50),
                          short(kw.get('name', ast.Constant('')), 40),
                          'yes' if 'condition' in kw else ''))
        elif name in ('IncludeLaunchDescription',):
            includes.append((node.lineno, short(node.args[0] if node.args else ast.Constant('?'), 110)))
        elif name == 'DeclareLaunchArgument':
            a0 = short(node.args[0], 40) if node.args else short(kw.get('name', ast.Constant('?')), 40)
            args.append((node.lineno, a0, short(kw.get('default_value', ast.Constant('—')), 50)))
        elif name == 'ExecuteProcess':
            nodes.append((node.lineno, '(ExecuteProcess)', short(kw.get('cmd', ast.Constant('?')), 60), '',
                          'yes' if 'condition' in kw else ''))
    return sorted(nodes), sorted(includes), sorted(args)


# ── main ──────────────────────────────────────────────────────────────────
all_files = [p for p in git_ls(PKGS) if not EXCLUDE.search('/' + p)]
excluded = [p for p in git_ls(PKGS) if EXCLUDE.search('/' + p)]
ext_files = [p for p in EXTERNAL_EXECUTED if os.path.exists(os.path.join(REPO, p))]

py_files = [p for p in all_files if p.endswith('.py')]
trees = {p: parse(p) for p in py_files}
for p in ext_files:
    if p.endswith('.py'):
        trees[p] = parse(p)
idx = module_index(py_files)

# entry points
setup_src = open(os.path.join(REPO, 'franka_experiments/setup.py')).read()
entry = dict(re.findall(r"'([a-z_0-9]+) = franka_experiments\.([a-zA-Z_.0-9]+):main'", setup_src))
entry_paths = {k: 'franka_experiments/franka_experiments/' + v.replace('.', '/') + '.py'
               for k, v in entry.items()}
sim_exec = {os.path.splitext(os.path.basename(p))[0]: p for p in py_files
            if p.startswith('franka_simulation_experiments/scripts/')
            and '/simexp/' not in p}

# import graph
graph = collections.defaultdict(set)
ext_imports = collections.defaultdict(set)
for p, t in trees.items():
    if t is None or not p.endswith('.py'):
        continue
    for mod, ln, top in imports_of(p, t):
        r = resolve(mod, p, idx)
        if r and r != p:
            graph[p].add(r)
        elif not r:
            root = mod.split('.')[0]
            if root and root not in ('franka_experiments', 'simexp', '__future__'):
                ext_imports[p].add(root)
# bare `utils.*` in franka_experiments refers to franka_simulation/scripts/utils
# (sys.path shim); record separately.


def closure(starts):
    seen, stack = set(), list(starts)
    while stack:
        x = stack.pop()
        if x in seen:
            continue
        seen.add(x)
        stack.extend(graph.get(x, ()))
    return seen


launch_files = [p for p in all_files if p.endswith('.launch.py')]
launch_info = {p: launch_scan(p, trees[p]) for p in launch_files if trees.get(p)}
launched_execs = set()        # by production launch files (not test/launch)
test_launched_execs = set()   # by test/launch/*.launch.py only
for p, (nodes, _, _) in launch_info.items():
    for _, pkg, exe, _, _ in nodes:
        (test_launched_execs if '/test/' in p else launched_execs).add(exe.strip("'\""))

reach_entry = closure(entry_paths.values())
reach_sim = closure(sim_exec.values())
scripts_fe = [p for p in py_files if p.startswith('franka_experiments/scripts/')]
reach_scripts = closure(scripts_fe)
tests = [p for p in py_files if '/test/' in p]
reach_tests = closure(tests)
reach_launch = closure(launch_files)
launched_entry = {k for k in entry if k in launched_execs}

# ── assemble ledger ───────────────────────────────────────────────────────
ledger = []
for p in all_files:
    role, ch, note = assign(p)
    reach = []
    if p in reach_entry:
        reach.append('entry')
    if p in reach_sim:
        reach.append('sim-exec')
    if p in reach_scripts and p not in scripts_fe:
        reach.append('script')
    if p in reach_launch and not p.endswith('.launch.py'):
        reach.append('launch')
    if p in reach_tests and '/test/' not in p:
        reach.append('test')
    ledger.append(dict(path=p, lines=nlines(p), role=role, ch=ch, note=note,
                       reach=','.join(reach), ext=False))
for p in ext_files:
    role, ch = assign_external(p)
    ledger.append(dict(path=p, lines=nlines(p), role=role, ch=ch, note='',
                       reach='', ext=True))

# ── write coverage ledger ─────────────────────────────────────────────────
def w(name, text):
    with open(os.path.join(OUT, name), 'w') as f:
        f.write(text)


GROUPS = [
    ('franka_experiments — nodes', lambda p: p.startswith(N)),
    ('franka_experiments — utils', lambda p: p.startswith(U) or p == 'franka_experiments/franka_experiments/__init__.py'),
    ('franka_experiments — launch', lambda p: p.startswith('franka_experiments/launch/')),
    ('franka_experiments — config', lambda p: p.startswith('franka_experiments/config/')),
    ('franka_experiments — scripts', lambda p: p.startswith('franka_experiments/scripts/')),
    ('franka_experiments — tests', lambda p: p.startswith('franka_experiments/test/')),
    ('franka_experiments — docs & package metadata',
     lambda p: p.startswith('franka_experiments/') and not p.startswith(('franka_experiments/franka_experiments/', 'franka_experiments/launch/', 'franka_experiments/config/', 'franka_experiments/scripts/', 'franka_experiments/test/'))),
    ('franka_simulation_experiments — launch', lambda p: p.startswith('franka_simulation_experiments/launch/')),
    ('franka_simulation_experiments — config', lambda p: p.startswith('franka_simulation_experiments/config/')),
    ('franka_simulation_experiments — sim nodes & tools', lambda p: p.startswith('franka_simulation_experiments/scripts/') and '/simexp/' not in p),
    ('franka_simulation_experiments — simexp library', lambda p: p.startswith('franka_simulation_experiments/scripts/simexp/')),
    ('franka_simulation_experiments — tests', lambda p: p.startswith('franka_simulation_experiments/test/')),
    ('franka_simulation_experiments — docs & package metadata',
     lambda p: p.startswith('franka_simulation_experiments/') and not p.startswith(('franka_simulation_experiments/launch/', 'franka_simulation_experiments/config/', 'franka_simulation_experiments/scripts/', 'franka_simulation_experiments/test/'))),
]

L = []
L.append('# Coverage ledger\n')
L.append('> Generated by Stage 0 static extraction (read-only `git ls-files` + `ast`). '
         'Chapter assignments are **provisional** and are corrected by hand as stages progress. '
         'Status values: `pending` → `read` (source read line-by-line) → `documented` (covered in its chapter, reviewed).\n')
L.append('**Reach** column (static import closure, see [import_graph.md](import_graph.md)): '
         '`entry` = imported (transitively) by a `setup.py` console-script node; '
         '`sim-exec` = by a `franka_simulation_experiments/scripts/*.py` executable; '
         '`launch` = by a launch file (launch-time import); `script` = by a `franka_experiments/scripts/*.py` tool; `test` = by a test. '
         'Empty reach on a library module = candidate for "unused" (to be confirmed in Stage 7).\n')
tot = collections.Counter()
for title, pred in GROUPS:
    rows = [r for r in ledger if not r['ext'] and pred(r['path'])]
    if not rows:
        continue
    L.append(f'\n## {title} ({len(rows)} files, {sum(r["lines"] for r in rows)} lines)\n')
    L.append('| File | Lines | Role | Ch. | Stage | Reach | Status |')
    L.append('|---|---:|---|---|---|---|---|')
    for r in sorted(rows, key=lambda r: r['path']):
        st = STAGE_OF.get(r['ch'], 7)
        L.append(f"| `{r['path']}` | {r['lines']} | {r['role']}{' ⚠ '+r['note'] if r['note'] else ''} "
                 f"| {r['ch']} | {st} | {r['reach']} | pending |")
        tot[title] += 1
ext_rows = [r for r in ledger if r['ext']]
L.append(f'\n## Workspace dependencies — executed path ({len(ext_rows)} files)\n')
L.append('Documented in full because they run inside the pipeline; the rest of these packages is interface-only.\n')
L.append('| File | Lines | Role | Ch. | Stage | Status |')
L.append('|---|---:|---|---|---|---|')
for r in ext_rows:
    L.append(f"| `{r['path']}` | {r['lines']} | {r['role']} | {r['ch']} | {STAGE_OF.get(r['ch'], 7)} | pending |")
L.append('\n## Interface-only dependencies (not line-read)\n')
L.append('- `franka_simulation/scripts/utils/*` — imported at runtime via the `sys.path` shim in '
         '`utils/simulation_imports.py` and by `simexp/sim_config.py`, `simexp/emulation.py`, '
         '`simexp/estimator_metrics.py`; which symbols are used is listed in [import_graph.md](import_graph.md). '
         'The modules actually called on the executed path will be promoted to the table above in Stage 5/6.')
L.append('- `franka_rl/` (on disk) a.k.a. `franka_sim` (in code) — located at runtime by `utils/rl_policy.py`; interface only.')
L.append('- `franka_description`, `franka_bringup`, `franka_fr3_moveit_config`, `franka_gripper`, `franka_hardware`: launch-time interfaces only.')
L.append('\n## Excluded as data (not documented line-by-line)\n')
exc_by = collections.Counter()
for p in excluded:
    m = EXCLUDE.search('/' + p)
    exc_by[m.group(1) if m else '?'] += 1
for k, v in sorted(exc_by.items()):
    L.append(f'- `{k.strip("/")}` — {v} tracked files')
L.append('\n## Totals\n')
inscope = [r for r in ledger]
L.append(f'- In-scope files: **{len(inscope)}** ({sum(r["lines"] for r in inscope)} lines), '
         f'of which {len(ext_rows)} are executed-path dependency files.')
L.append(f'- Status: pending {len(inscope)} · read 0 · documented 0.')
unassigned = [r['path'] for r in ledger if r['note'] == 'UNASSIGNED']
L.append(f'- Unassigned by the rules: {len(unassigned)}' + (': ' + ', '.join(f'`{u}`' for u in unassigned) if unassigned else ''))
by_ch = collections.Counter(r['ch'] for r in ledger)
L.append('\n| Chapter | Files |\n|---|---:|')
for ch in sorted(by_ch):
    L.append(f'| {ch} {CH.get(ch, "Code status")} | {by_ch[ch]} |')
w('ledger/coverage_ledger.md', '\n'.join(L) + '\n')

# ── ROS interfaces ────────────────────────────────────────────────────────
R = ['# ROS interfaces — extracted, NOT yet verified\n',
     '> Every call site of `create_publisher/create_subscription/create_service/create_client/'
     'create_timer/declare_parameter(s)`, action clients/servers, TF listeners/broadcasters/lookups, '
     'executors and callback groups, found by `ast` in every Python file in scope. '
     'Arguments are shown **as written** (often a variable or `self.<attr>`, not the resolved topic name); '
     'resolving them to concrete names, types, rates and QoS is Stage 1 work (chapter 02). '
     'Helpers that create interfaces on behalf of a node (e.g. in `utils/ros*.py`, `utils/node_runtime.py`) '
     'are listed under their own file.\n']
summary = []
for p in sorted(py_files):
    t = trees.get(p)
    if t is None or '/test/' in p:
        continue
    rows = ros_iface(p, t)
    if not rows:
        continue
    c = collections.Counter(r[0] for r in rows)
    summary.append((p, c))
R.append('## Summary (counts per file)\n')
kinds = ['pub', 'sub', 'srv', 'cli', 'action_cli', 'action_srv', 'timer', 'param', 'params',
         'tf_listen', 'tf_lookup', 'tf_bcast', 'tf_static', 'tf_send', 'executor', 'cbgroup', 'rate']
R.append('| File | ' + ' | '.join(kinds) + ' |')
R.append('|---|' + '---:|' * len(kinds))
for p, c in summary:
    R.append(f'| `{p.split("/", 1)[1]}` | ' + ' | '.join(str(c.get(k, '')) for k in kinds) + ' |')
R.append('\n## Details\n')
for p, _ in summary:
    rows = ros_iface(p, trees[p])
    R.append(f'### `{p}`\n')
    R.append('| Kind | Line | Enclosing def | Arguments (as written) |')
    R.append('|---|---:|---|---|')
    for k, ln, own, a in rows:
        if k == 'param':
            continue
        aa = a.replace("|", "\\|"); R.append(f"| {k} | {ln} | `{own}` | `{aa}` |")
    params = [r for r in rows if r[0] == 'param']
    if params:
        names = [r[3].split(',')[0] for r in params]
        R.append(f'\n**declare_parameter** ×{len(params)}: ' + ', '.join(f'`{n}`' for n in names[:400]) + '\n')
    R.append('')
w('ledger/ros_interfaces.md', '\n'.join(R) + '\n')

# ── import graph ──────────────────────────────────────────────────────────
G = ['# Import graph and reachability — static, NOT yet verified\n',
     '> Built from every `import`/`from … import` statement (including function-local and '
     'conditional imports) in the Python files in scope. Static analysis cannot see `importlib` '
     'string imports or `sys.path` manipulation targets; those are listed by hand below.\n']
G.append('## Entry points (`franka_experiments/setup.py`)\n')
G.append('| Console script | Module | Started by a production launch file? | Started by a test launch file? | Internal modules in closure |')
G.append('|---|---|---|---|---:|')
for k, p in sorted(entry_paths.items()):
    G.append(f'| `{k}` | `{p.split("/", 1)[1]}` | {"yes" if k in launched_entry else "**no**"} | {"yes" if k in test_launched_execs else ""} | {len(closure([p])) - 1} |')
G.append('\n## Executables of `franka_simulation_experiments` (CMake installs every `scripts/*.py`)\n')
G.append('| Executable | Started by an in-package launch file? | Internal modules in closure |')
G.append('|---|---|---:|')
for k, p in sorted(sim_exec.items()):
    G.append(f'| `{k}` | {"yes" if k in launched_execs else "no (CLI tool / batch)"} | {len(closure([p])) - 1} |')

G.append('\n## Per-module internal dependencies (fan-out → / fan-in ←)\n')
fanin = collections.defaultdict(set)
for a, bs in graph.items():
    for b in bs:
        fanin[b].add(a)
lib_mods = sorted(p for p in py_files if (p.startswith(U) or p.startswith(N) or '/simexp/' in p))
G.append('| Module | Imports (internal) | Imported by (non-test) | Imported by tests |')
G.append('|---|---|---|---:|')
for p in lib_mods:
    outs = ', '.join(sorted(os.path.splitext(os.path.basename(x))[0] for x in graph.get(p, ())))
    ins = fanin.get(p, set())
    ins_nt = sorted(os.path.splitext(os.path.basename(x))[0] for x in ins if '/test/' not in x)
    ins_t = sum(1 for x in ins if '/test/' in x)
    G.append(f'| `{p.split("/")[-2]}/{os.path.basename(p)}` | {outs} | {", ".join(ins_nt)} | {ins_t} |')

G.append('\n## Reachability classes for library modules\n')
G.append('| Module | entry | sim-exec | launch-file | script | tests only | none |')
G.append('|---|:-:|:-:|:-:|:-:|:-:|:-:|')
cand_unused = []
for p in lib_mods:
    e = p in reach_entry
    s = p in reach_sim
    sc = p in reach_scripts and p not in scripts_fe
    la = p in reach_launch
    te = p in reach_tests
    none = not (e or s or sc or la)
    if none:
        cand_unused.append((p, te))
    G.append(f'| `{p.split("/")[-2]}/{os.path.basename(p)}` | {"●" if e else ""} | {"●" if s else ""} | '
             f'{"●" if la else ""} | {"●" if sc else ""} | {"●" if (te and none) else ""} | {"●" if (none and not te) else ""} |')
G.append('\n**Library modules reached by no executable (candidates only — Stage 7 confirms):** ' +
         (', '.join(f'`{os.path.basename(p)}`' + (' (tests)' if te else '') for p, te in cand_unused) or 'none'))

G.append('\n## External (non-internal) imports per file\n')
G.append('| File | Top-level packages imported |')
G.append('|---|---|')
for p in sorted(ext_imports):
    if '/test/' in p:
        continue
    pk = sorted(x for x in ext_imports[p] if x not in STDLIB)
    if pk:
        G.append(f'| `{p.split("/", 1)[1]}` | {", ".join(pk)} |')
ext_count = collections.Counter()
for p, s in ext_imports.items():
    for x in s:
        if x not in STDLIB:
            ext_count[x] += 1
G.append('\n**Third-party / ROS packages by number of importing files:** ' +
         ', '.join(f'{k} ({v})' for k, v in ext_count.most_common()))
w('ledger/import_graph.md', '\n'.join(G) + '\n')

# ── launch inventory ──────────────────────────────────────────────────────
LI = ['# Launch inventory — extracted, NOT yet verified\n',
      '> `Node(...)`, `ExecuteProcess(...)`, `IncludeLaunchDescription(...)` and '
      '`DeclareLaunchArgument(...)` call sites per launch file. Values are shown as written; '
      'which nodes actually start depends on conditions and `OpaqueFunction` logic resolved in Stage 1.\n']
for p in sorted(launch_info):
    nodes, incs, args = launch_info[p]
    LI.append(f'## `{p}` ({nlines(p)} lines)\n')
    if nodes:
        LI.append('| Line | Package | Executable | Name | Conditional |')
        LI.append('|---:|---|---|---|---|')
        for ln, pkg, exe, nm, cond in nodes:
            LI.append(f'| {ln} | `{pkg}` | `{exe}` | `{nm}` | {cond} |')
    if incs:
        LI.append('\n**Includes:**\n')
        for ln, s in incs:
            LI.append(f'- L{ln}: `{s}`')
    if args:
        LI.append(f'\n**Launch arguments ({len(args)}):** ' +
                  ', '.join(f'`{a.strip(chr(39))}`' for _, a, _ in args))
    LI.append('')
w('ledger/launch_inventory.md', '\n'.join(LI) + '\n')

# ── module definitions (classes/functions) ────────────────────────────────
D = ['# Module definitions — top-level classes and functions\n',
     '> Top-level `class`/`def` per module (methods counted, not listed). Used as the checklist '
     'of symbols each chapter must account for.\n']
nsym = 0
for p in lib_mods + sorted(scripts_fe) + sorted(sim_exec.values()):
    t = trees.get(p)
    if t is None:
        continue
    cl, fn = defs_of(t)
    if not cl and not fn:
        continue
    nsym += len(cl) + len(fn)
    D.append(f'### `{p}`\n')
    if cl:
        D.append('Classes: ' + ', '.join(f'`{n}` (L{ln}, {m} methods)' for n, ln, m in cl))
    if fn:
        D.append('\nFunctions: ' + ', '.join(f'`{n}` (L{ln})' for n, ln in fn))
    D.append('')
D.insert(2, f'**Total top-level symbols: {nsym}.**\n')
w('ledger/module_definitions.md', '\n'.join(D) + '\n')

# ── machine-readable dump for later stages ───────────────────────────────
import json  # noqa: E402
with open(os.path.join(OUT, 'ledger', 'ledger.json'), 'w') as f:
    json.dump(dict(ledger=ledger,
                   entry=entry_paths, sim_exec=sim_exec,
                   launched_execs=sorted(launched_execs),
                   test_launched_execs=sorted(test_launched_execs),
                   candidates_unused=[p for p, _ in cand_unused],
                   graph={k: sorted(v) for k, v in graph.items()}), f, indent=1)

print('files in scope', len(ledger), 'excluded', len(excluded))
print('unassigned', unassigned)
print('candidates unused', [os.path.basename(p) for p, _ in cand_unused])
print('entry not launched', sorted(set(entry) - launched_entry))
print('symbols', nsym)
