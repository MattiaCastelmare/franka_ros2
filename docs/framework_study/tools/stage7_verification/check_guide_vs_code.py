#!/usr/bin/env python3
"""The old technical guide against the documented baseline source (chapter 16, Part E). Host.

`franka_experiments/docs/FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md` (last changed in `ce80f36`, F-0-9)
is compared, mechanically, with the files of the baseline commit (`git show <baseline>:<path>`, never
the working tree or an install). Nothing is imported from the packages and nothing starts ROS.

Every check is a FACT. The script PASSES when it could evaluate the comparison; the discrepancies
it finds are printed as data, and the counts are the evidence quoted in 16 Part E. (A discrepancy is
the expected outcome for a stale guide, not a failure of the script.)

1. §15.2 "Complete configuration values": every ```yaml block that follows a `#### <file>` heading
   with a `[Source](../config/...)` link is flattened and compared key by key with the baseline YAML
   (changed values, keys only in the guide, keys only in the baseline).
2. §15.3 "Node-local ROS parameter declaration index": each listed parameter must still be declared
   in that node at the baseline (the quoted name appears in the file); the listed line is compared
   with the line of the first occurrence. Baseline `declare_parameter('<literal>'` names that the
   guide does not list are counted.
3. §15.6 and §15.11.5 "source identity": every row with a repository path and a full 64-hex SHA-256 is
   compared with the SHA-256 of the baseline blob.
4. Relative Markdown links of the guide: each target must exist in the baseline tree.
5. Backticked identifiers in the "Implementation" column of §15.1 and the "Entry points" of §15.4:
   each `Name`, `Class.method` or `module.py` must be defined (def/class/file) somewhere in the
   baseline tree of the four documented packages.
6. Launch defaults: the guide's `launch_defaults.yaml` block against the baseline file is part of 1;
   additionally, the `d_safe` values the guide states anywhere are listed against the baseline YAML.

Usage: python3 -I -B check_guide_vs_code.py [--verbose]
"""
import hashlib
import os
import re
import subprocess
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'stage2_verification'))
import _common as C  # noqa: E402

import yaml  # noqa: E402

BASELINE = 'ba68077052e8d0fb3d4606c05dff553b75baa641'
GUIDE = 'franka_experiments/docs/FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md'
GUIDE_DIR = 'franka_experiments/docs'
PKGS = ('franka_experiments/', 'franka_simulation_experiments/', 'franka_rt_controllers/',
        'franka_msgs/')
VERBOSE = '--verbose' in sys.argv


def git_bytes(*args) -> bytes:
    return subprocess.run(['git', '-C', C.REPO, *args], check=True, capture_output=True).stdout


def baseline_files() -> set:
    out = git_bytes('ls-tree', '-r', '--name-only', BASELINE).decode()
    return set(out.splitlines())


def show(path: str) -> bytes:
    return git_bytes('show', f'{BASELINE}:{path}')


def norm(path: str) -> str:
    return os.path.normpath(path).replace(os.sep, '/')


def section(lines, start_pat, stop_pat):
    """Lines from the heading matching start_pat up to (excluding) the next matching stop_pat."""
    out, on = [], False
    for i, ln in enumerate(lines):
        if re.match(start_pat, ln):
            on = True
            continue
        if on and re.match(stop_pat, ln):
            break
        if on:
            out.append((i + 1, ln))
    return out


def flatten(obj, prefix=''):
    flat = {}
    if isinstance(obj, dict):
        for k, v in obj.items():
            flat.update(flatten(v, f'{prefix}.{k}' if prefix else str(k)))
    else:
        flat[prefix] = obj
    return flat


def same(a, b) -> bool:
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool) \
            and not isinstance(b, bool):
        return abs(float(a) - float(b)) <= 1e-12 * max(1.0, abs(float(a)))
    return a == b


def show_list(R, title, items, limit=None):
    items = list(items)
    lim = len(items) if (VERBOSE or limit is None) else limit
    R.info(f'{title} ({len(items)}):')
    for it in items[:lim]:
        R.info(f'    {it}')
    if len(items) > lim:
        R.info(f'    … {len(items) - lim} more (--verbose)')


def check_yaml(R, lines, files):
    body = section(lines, r'^### 15\.2 ', r'^### 15\.3 ')
    blocks, cur, src, inblock, buf = [], None, None, False, []
    for _, ln in body:
        m = re.match(r'^#### (\S+)', ln)
        if m:
            cur, src = m.group(1), None
            continue
        m = re.match(r'^\[Source\]\(([^)]+)\)', ln)
        if m and cur:
            src = m.group(1)
            continue
        if ln.startswith('```yaml') and cur and src:
            inblock, buf = True, []
            continue
        if inblock and ln.startswith('```'):
            blocks.append((cur, src, '\n'.join(buf)))
            inblock, cur, src = False, None, None
            continue
        if inblock:
            buf.append(ln)
    R.fact('15.2: YAML blocks with a source link parsed', len(blocks) >= 5,
           f'{len(blocks)} blocks: ' + ', '.join(b[0] for b in blocks))
    totals = {}
    for name, link, text in blocks:
        path = norm(os.path.join(GUIDE_DIR, link))
        if path not in files:
            R.info(f'  {name}: {path} NOT in the baseline tree (file removed or moved)')
            totals[name] = ('missing', 0, 0, 0)
            continue
        g = flatten(yaml.safe_load(text) or {})
        docs = [d for d in yaml.safe_load_all(show(path).decode()) if d is not None]
        b = flatten(docs[0] if docs else {})
        changed = sorted(k for k in g if k in b and not same(g[k], b[k]))
        only_g = sorted(k for k in g if k not in b)
        only_b = sorted(k for k in b if k not in g)
        totals[name] = ('ok', len(changed), len(only_g), len(only_b))
        R.info(f'  {name} ({path}): guide keys {len(g)}, baseline keys {len(b)}; changed '
               f'{len(changed)}, only in guide {len(only_g)}, only in baseline {len(only_b)}')
        show_list(R, '    changed value (guide → baseline)',
                  (f'{k}: {g[k]!r} → {b[k]!r}' for k in changed), limit=60)
        show_list(R, '    only in guide (removed or renamed)', only_g, limit=40)
        show_list(R, '    only in baseline (added since the guide)', only_b, limit=60)
    n = sum(t[1] + t[2] + t[3] for t in totals.values() if t[0] == 'ok')
    R.fact('15.2: comparison evaluated for every parsed block', len(totals) == len(blocks),
           f'{n} key-level differences in total')
    return totals


def check_params(R, lines, files):
    body = section(lines, r'^### 15\.3 ', r'^### 15\.4 ')
    nodes, cur = {}, None
    for _, ln in body:
        m = re.match(r'^#### (\S+)', ln)
        if m:
            cur = m.group(1)
            nodes[cur] = {'src': None, 'rows': []}
            continue
        m = re.match(r'^\[Source\]\(([^)]+)\)', ln)
        if m and cur:
            nodes[cur]['src'] = norm(os.path.join(GUIDE_DIR, m.group(1)))
            continue
        m = re.match(r'^\| `([^`]+)` \|.*\| (\d+) \|\s*$', ln)
        if m and cur:
            nodes[cur]['rows'].append((m.group(1), int(m.group(2))))
    with_rows = {k: v for k, v in nodes.items() if v['src'] and v['rows']}
    R.fact('15.3: node parameter tables parsed', len(with_rows) >= 10,
           f'{len(with_rows)} nodes, {sum(len(v["rows"]) for v in with_rows.values())} rows')
    tot_missing = tot_shift = tot_new = 0
    for node, v in sorted(with_rows.items()):
        if v['src'] not in files:
            R.info(f'  {node}: {v["src"]} not in the baseline')
            continue
        text = show(v['src']).decode().splitlines()
        missing, shifted = [], []
        for pname, gline in v['rows']:
            hits = [i + 1 for i, t in enumerate(text)
                    if f"'{pname}'" in t or f'"{pname}"' in t]
            if not hits:
                missing.append(pname)
            elif gline not in hits:
                shifted.append(f'{pname}: guide L{gline} → baseline L{hits[0]}')
        declared = set()
        for t in text:
            for mm in re.finditer(r"declare_parameter\(\s*['\"]([A-Za-z0-9_.]+)['\"]", t):
                declared.add(mm.group(1))
        listed = {p for p, _ in v['rows']}
        new = sorted(declared - listed)
        tot_missing += len(missing)
        tot_shift += len(shifted)
        tot_new += len(new)
        if missing or new or VERBOSE:
            R.info(f'  {node}: {len(v["rows"])} listed, {len(missing)} not found at baseline, '
                   f'{len(shifted)} line moved, {len(new)} literal declarations not listed')
            if missing:
                show_list(R, '    listed but absent', missing)
            if new:
                show_list(R, '    declared but not listed', new)
            if VERBOSE and shifted:
                show_list(R, '    line moved', shifted)
    R.fact('15.3: parameter rows checked', True,
           f'{tot_missing} absent at baseline, {tot_shift} at another line, '
           f'{tot_new} literal declarations not listed')
    return tot_missing, tot_shift, tot_new


def check_hashes(R, lines, files):
    rows = []
    for i, ln in enumerate(lines):
        for m in re.finditer(r'`((?:franka_|analysis/)[^`]+)`[^`]*?`([0-9a-f]{64})`', ln):
            rows.append((i + 1, m.group(1), m.group(2)))
    R.fact('15.6/15.11.5: path + full SHA-256 rows found', len(rows) >= 10, f'{len(rows)} rows')
    equal = differ = absent = 0
    last = {}
    for ln, path, dig in rows:
        last[path] = (ln, dig)          # the latest digest the guide gives for a path
    eq_paths, diff_paths, abs_paths = [], [], []
    for path, (ln, dig) in sorted(last.items()):
        if path not in files:
            absent += 1
            abs_paths.append(f'{path} (guide L{ln})')
            continue
        cur = hashlib.sha256(show(path)).hexdigest()
        if cur == dig:
            equal += 1
            eq_paths.append(path)
        else:
            differ += 1
            diff_paths.append(f'{path} (guide L{ln})')
    R.info(f'  latest guide digest per path: {len(last)} paths; equal to the baseline blob '
           f'{equal}, different {differ}, path absent {absent}')
    show_list(R, '    equal', eq_paths)
    show_list(R, '    different', diff_paths)
    show_list(R, '    absent at baseline', abs_paths)
    R.fact('15.6/15.11.5: digests compared', equal + differ + absent == len(last),
           f'{equal}/{len(last)} still identify the baseline file')
    return equal, differ, absent


def check_links(R, lines, files):
    dirs = {os.path.dirname(f) for f in files}
    while True:
        more = {os.path.dirname(d) for d in dirs} - dirs
        if not more:
            break
        dirs |= more
    bad, n = [], 0
    for i, ln in enumerate(lines):
        for m in re.finditer(r'\]\(([^)#\s]+)(?:#[^)]*)?\)', ln):
            tgt = m.group(1)
            if re.match(r'^[a-z]+:', tgt):
                continue
            n += 1
            p = norm(os.path.join(GUIDE_DIR, tgt))
            if p not in files and p not in dirs:
                bad.append(f'L{i + 1}: {tgt}')
    show_list(R, '  relative links whose target is absent at baseline', bad)
    R.fact('links: relative targets checked', n > 50, f'{n} links, {len(bad)} absent at baseline')
    return n, bad


def baseline_symbols(files):
    defs, mods = set(), set()
    for f in files:
        if not f.startswith(PKGS):
            continue
        mods.add(os.path.basename(f))
        if f.endswith('.py'):
            text = show(f).decode(errors='replace')
            cls = None
            for t in text.splitlines():
                m = re.match(r'^class (\w+)', t)
                if m:
                    cls = m.group(1)
                    defs.add(cls)
                    continue
                m = re.match(r'^(\s*)def (\w+)', t)
                if m:
                    defs.add(m.group(2))
                    if m.group(1) and cls:
                        defs.add(f'{cls}.{m.group(2)}')
                    if not m.group(1):
                        cls = None
                m = re.match(r'^(\w+)\s*(?::[^=]+)?=', t)   # module-level constants / aliases
                if m:
                    defs.add(m.group(1))
        elif f.endswith(('.cpp', '.hpp', '.h')):
            text = show(f).decode(errors='replace')
            for m in re.finditer(r'\b(\w+)::(\w+)\s*\(', text):
                defs.add(f'{m.group(1)}.{m.group(2)}')
                defs.add(m.group(2))
            for m in re.finditer(r'\bclass (\w+)', text):
                defs.add(m.group(1))
    return defs, mods


def check_symbols(R, lines, files):
    defs, mods = baseline_symbols(files)
    refs = []
    in15_1 = section(lines, r'^### 15\.1 ', r'^### 15\.2 ')
    for ln_no, ln in in15_1:
        cells = [c.strip() for c in ln.split('|')]
        if len(cells) >= 6 and cells[1].startswith('$'):
            for m in re.finditer(r'`([^`]+)`', cells[4]):
                refs.append((ln_no, m.group(1), '15.1'))
    in15_4 = section(lines, r'^### 15\.4 ', r'^### 15\.5 ')
    for ln_no, ln in in15_4:
        if 'Entry points:' in ln:
            tail = ln.split('Entry points:', 1)[1]
            for m in re.finditer(r'`([^`]+)`', tail):
                refs.append((ln_no, m.group(1), '15.4'))
    found, missing = 0, []
    for ln_no, ref, sec in refs:
        tok = ref.split('(')[0].strip().rstrip('()').replace('::', '.')
        names = [t for t in re.split(r'[\s,/]+', tok) if t]
        ok = False
        for n in names:
            n = n.rstrip('.')
            if n in defs or n in mods or n.split('.')[-1] in defs:
                ok = True
                break
        if ok:
            found += 1
        else:
            missing.append(f'§{sec} L{ln_no}: `{ref}`')
    show_list(R, '  identifiers with no baseline definition (removed, renamed, or a non-code label)',
              missing)
    R.fact('15.1/15.4: implementation identifiers checked', len(refs) > 50,
           f'{len(refs)} identifiers, {found} defined at baseline, {len(missing)} not found')
    return len(refs), missing


def check_dsafe(R, lines):
    base = yaml.safe_load(show('franka_experiments/config/fr3_control.yaml').decode())
    p = base.get('params', {})
    R.info(f'  baseline fr3_control.yaml params: d_safe={p.get("d_safe")!r}, '
           f'd_safe_default={p.get("d_safe_default")!r}')
    hits = [(i + 1, ln.strip()) for i, ln in enumerate(lines)
            if re.search(r'd_safe\b[^_]', ln) and re.search(r'0\.\d+', ln)]
    show_list(R, '  guide lines stating a d_safe value', (f'L{a}: {b[:150]}' for a, b in hits), 25)
    R.fact('d_safe: baseline value read', 'd_safe' in p, f'{len(hits)} guide lines quote a value')


def check_claims(R, lines):
    """Specific guide statements re-evaluated at the guide's revision and at the baseline."""
    guide_rev = 'ce80f36'

    def at(rev, path):
        return git_bytes('show', f'{rev}:{path}').decode(errors='replace')

    csr = 'franka_experiments/franka_experiments/utils/cbf_state_rows.py'
    flt = 'franka_experiments/franka_experiments/nodes/cbf_safety_filter.py'
    ctl = 'franka_experiments/config/fr3_control.yaml'
    cpl = 'franka_experiments/config/fr3_complete.yaml'

    def nslack(rev):
        m = re.search(r'^N_SLACK\s*=\s*(\d+)', at(rev, csr), re.M)
        return int(m.group(1)) if m else None
    g_ns, b_ns = nslack(guide_rev), nslack(BASELINE)
    says13 = any('mathbb R^{13}' in ln for ln in lines)
    R.fact('E1 (1.1)/(9.1): guide x ∈ R^13 (6 family slacks) — guide rev N_SLACK=6, baseline 7',
           says13 and g_ns == 6 and b_ns == 7, f'N_SLACK {guide_rev}={g_ns}, baseline={b_ns}; '
           f'baseline NX = 7 + {b_ns} = {7 + (b_ns or 0)} (family mode)')

    base_ctl = yaml.safe_load(at(BASELINE, ctl))['params']
    pol = base_ctl.get('qp_failure_policy')
    g_has = 'qp_failure_policy' in at(guide_rev, flt)
    b_has = "qp_failure_policy', 'brake'" in at(BASELINE, flt)
    R.fact('E2 §9.4: guide "box-only QP first, then brake"; baseline default policy = brake',
           pol == 'brake' and not g_has and b_has,
           f'fr3_control.yaml qp_failure_policy={pol!r}; policy absent at {guide_rev}: {not g_has}')

    mode = base_ctl.get('obstacle_slack_mode')
    R.fact('E3 §9.1: per-row ℓ1 obstacle slack exists at baseline (guide: family only); shipped mode',
           mode == 'family' and 'row_l1' in at(BASELINE, csr) and 'row_l1' not in at(guide_rev, csr),
           f'obstacle_slack_mode={mode!r}, slots={base_ctl.get("obstacle_row_slack_slots")}')

    prio = base_ctl.get('qp_thread_fifo_prio')
    R.fact('E4 §10.3: guide "SCHED_FIFO 50 for the QP thread"; baseline qp_thread_fifo_prio',
           prio == 0, f'qp_thread_fifo_prio={prio!r} (0 = no per-thread elevation)')

    flags = {k: base_ctl.get(k) for k in ('enable_curvature_term', 'enable_evidence_margin',
                                          'enable_obstacle_memory', 'ssb_rows_enabled')}
    absent = [k for k in flags if k not in at(guide_rev, ctl)]
    R.fact('E5 mechanisms added after the guide: all four flags absent at the guide rev, off at '
           'baseline', all(v is False for v in flags.values()) and len(absent) == 4,
           ', '.join(f'{k}={v}' for k, v in flags.items()))

    lg = 'franka_experiments/franka_experiments/nodes/experiment_logger.py'
    R.fact('E6 §12.7/§13.1: guide "ExperimentLogger no longer declares truth_topic" — superseded',
           'declare_parameter("truth_topic"' in at(BASELINE, lg)
           and 'truth_topic' not in at(guide_rev, lg),
           'declared at baseline, absent at the guide rev (restored by 7da68ac)')

    cfg = yaml.safe_load(at(BASELINE, cpl))
    coast = cfg['tracking']['max_coast_s']
    test = at(BASELINE, 'franka_experiments/test/test_rate_scaling.py')
    asserts_0167 = "assert trk['max_coast_s'] == pytest.approx(0.167)" in test
    R.fact('E7 §13.1: test_rate_scaling::test_the_shipped_config_states_durations asserts 0.167 '
           'against the shipped coast (still true at baseline: the test must fail)',
           asserts_0167 and abs(coast - 0.167) > 1e-3, f'shipped tracking.max_coast_s={coast}')

    jl = yaml.safe_load(at(BASELINE, ctl)).get('joint_limits', {})
    eff = sorted({v[4] for v in jl.values()})
    R.fact('E8 §13.1: fr3_control.yaml joint_limits effort column (guide: 500 for all joints)',
           eff == [500.0], f'effort column values {eff}')

    hn = at(BASELINE, 'franka_experiments/franka_experiments/nodes/handeye_eye_in_hand_node.py')
    R.fact('E9 §13.1: handeye_eye_in_hand_node reads the private rclpy `self._parameters` '
           '(still at baseline)', 'for name in self._parameters' in hn,
           'the test outcome the guide reports was not re-run (pytest needs approval)')

    sr = at(BASELINE, 'franka_experiments/franka_experiments/utils/singularity_recovery.py')
    R.fact('E10 §6.7 (6.24): guide already states q̇(T)=v₀ — the formula is unchanged at baseline',
           sr == at(guide_rev, 'franka_experiments/franka_experiments/utils/singularity_recovery.py'),
           'file identical at the guide rev and the baseline; F-7-1 quantifies the divergence')

    # The guide describes DIRTY working trees (its §15.11.5), so the commit ce80f36 is not its
    # exact subject: the launch file below differs between ce80f36 and the baseline, yet the
    # digest the guide recorded for it equals the baseline blob. Identity is therefore taken from
    # the guide's own digests, not from the commit.
    digests = {}
    for ln in lines:
        for m in re.finditer(r'`((?:franka_)[^`]+)`[^`]*?`([0-9a-f]{64})`', ln):
            digests[m.group(1)] = m.group(2)
    launch = ['franka_experiments/config/launch_defaults.yaml',
              'franka_experiments/launch/torque_control_stack.launch.py',
              'franka_experiments/launch/minimal.launch.py',
              'franka_experiments/launch/velocity_cbf_control_stack.launch.py']
    same_digest = [p for p in launch
                   if digests.get(p) == hashlib.sha256(show(p)).hexdigest()]
    moved = at(guide_rev, launch[1]) != at(BASELINE, launch[1])
    R.fact('E11 launch defaults: the guide\'s digests of launch_defaults.yaml and the three stack '
           'launch files equal the baseline blobs', len(same_digest) == len(launch),
           f'{len(same_digest)}/{len(launch)} equal; torque_control_stack differs between the '
           f'commit {guide_rev} and the baseline: {moved} (the guide hashed a dirty tree)')


def main():
    R = C.Report('Stage 7 — the old technical guide against the baseline (ch. 16 Part E)')
    files = baseline_files()
    if GUIDE not in files:
        R.fact('guide present at baseline', False, GUIDE)
        return R.finish()
    text = show(GUIDE).decode()
    lines = text.splitlines()
    last = subprocess.run(['git', '-C', C.REPO, 'log', '-1', '--format=%h %cs', BASELINE, '--',
                           GUIDE], capture_output=True, text=True).stdout.strip()
    R.info(f'baseline {BASELINE[:12]}; guide {len(lines)} lines, last changed {last}')
    n_since = subprocess.run(['git', '-C', C.REPO, 'rev-list', '--count',
                              f'{last.split()[0]}..{BASELINE}', '--', 'franka_experiments',
                              'franka_simulation_experiments', 'franka_rt_controllers'],
                             capture_output=True, text=True).stdout.strip()
    R.fact('commits touching the packages since the guide', n_since.isdigit(), n_since)
    R.info('— 1. §15.2 configuration values —')
    check_yaml(R, lines, files)
    R.info('— 2. §15.3 node parameter index —')
    check_params(R, lines, files)
    R.info('— 3. §15.6 / §15.11.5 source identity —')
    check_hashes(R, lines, files)
    R.info('— 4. links —')
    check_links(R, lines, files)
    R.info('— 5. §15.1 / §15.4 identifiers —')
    check_symbols(R, lines, files)
    R.info('— 6. d_safe statements —')
    check_dsafe(R, lines)
    R.info('— 7. specific guide statements, at the guide revision and at the baseline —')
    check_claims(R, lines)
    return R.finish()


if __name__ == '__main__':
    sys.exit(main())
