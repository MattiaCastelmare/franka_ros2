#!/usr/bin/env python3
"""Read-only consistency check of the framework-study documentation state.

Run from the repository root at the start of every session and before every checkpoint:
    python3 -I docs/framework_study/tools/check_state.py

It never writes anything.

Key idea: the *documented source baseline* (PROGRESS.md §0, history in §8) is the commit whose ROS 2
source the chapters describe. It is NOT the current HEAD. Commits that only touch the documentation
(DOC_PATHS) move HEAD but leave the baseline valid. Any change to source since the baseline, whether
committed, uncommitted or untracked, is reported as REVIEW, with the chapters it affects.

Checks:
  1. the baseline exists, is an ancestor of HEAD, and §0 agrees with the last row of §8;
  2. every path changed between the baseline and the working tree is classified as:
       doc       DOC_PATHS                                          -> ignored
       source    documented scope (ledger, the two packages,
                 executed dependency files)                         -> REVIEW
       interface interface-only workspace dependencies              -> NOTE (check relevance)
       other     everything else                                    -> NOTE
  3. the ledger totals agree between ledger.json, coverage_ledger.md and PROGRESS.md §2;
  4. every relative Markdown link in docs/framework_study/*.md and CLAUDE.md resolves;
  5. which stage PROGRESS.md §1 records as next.

Exit code: 0 = consistent, no review needed
           1 = integrity PROBLEM (fix the documentation state first)
           2 = consistent, but the source changed since the baseline: REVIEW needed (PROGRESS.md §8)
"""
from __future__ import annotations

import collections
import glob
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(DOCS))
DOCS_REL = os.path.relpath(DOCS, REPO) + '/'

# Paths whose changes never invalidate the baseline (documentation-only commits).
DOC_PATHS = (DOCS_REL, 'CLAUDE.md')
# Documented scope, besides every file listed in the ledger (00_INDEX.md §6).
SOURCE_PREFIXES = ('franka_experiments/', 'franka_simulation_experiments/',
                   'franka_rt_controllers/', 'franka_msgs/msg/')
# Data directories excluded from scope (same pattern as tools/stage0_extract.py).
DATA_RE = re.compile(r'(/sim_runs/|/experiment_logs/|/franka_logs/|/paper_draft/|/Papers/|/Videos/|'
                     r'/\.pytest_cache/|__pycache__)')
# Interface-only workspace dependencies (00_INDEX.md §6): changes may matter, check by hand.
INTERFACE_PREFIXES = ('franka_simulation/', 'franka_rl/', 'franka_description/', 'franka_bringup/',
                      'franka_fr3_moveit_config/', 'franka_hardware/', 'franka_gripper/',
                      'franka_msgs/', 'libfranka/')
STATUSES = ('pending', 'partial', 'read', 'documented')

problems, reviews, notes = [], [], []


def git(*args, check=False):
    r = subprocess.run(['git', '-C', REPO, *args], capture_output=True, text=True)
    return r.stdout if not check else (r.returncode, r.stdout)


def problem(msg):
    problems.append(msg)
    print('PROBLEM:', msg)


def note(msg):
    notes.append(msg)
    print('NOTE:', msg)


def is_doc(p):
    return p.startswith(DOC_PATHS)


def classify(p, ledger_paths):
    if is_doc(p):
        return 'doc'
    if p in ledger_paths or (p.startswith(SOURCE_PREFIXES) and not DATA_RE.search('/' + p)):
        return 'source'
    if p.startswith(INTERFACE_PREFIXES):
        return 'interface'
    return 'other'


def chapters_mentioning(p, chapter_texts):
    base = os.path.basename(p)
    return sorted(ch for ch, txt in chapter_texts.items() if p in txt or base in txt)


def main():
    progress = open(os.path.join(DOCS, 'PROGRESS.md')).read()
    data = json.load(open(os.path.join(DOCS, 'ledger', 'ledger.json')))
    ledger = {r['path']: r for r in data['ledger']}
    chapter_texts = {os.path.basename(f)[:2]: open(f).read()
                     for f in sorted(glob.glob(os.path.join(DOCS, '[0-9][0-9]_*.md')))}

    # 1. git position and baseline
    branch = git('branch', '--show-current').strip()
    head = git('rev-parse', 'HEAD').strip()
    print(f'git: branch {branch}, HEAD {head[:12]}, stashes {len(git("stash", "list").splitlines())}')
    m_b = re.search(r'Pinned branch:\*\*\s*`([^`]+)`', progress)
    m_s = re.search(r'Documented source baseline:\*\*\s*`([0-9a-f]{7,40})`', progress)
    hist = re.findall(r'^\| B-\d+ \| `([0-9a-f]{7,40})` \|', progress, re.M)
    if m_b and m_b.group(1) != branch:
        note(f'branch is {branch}, PROGRESS.md §0 pins {m_b.group(1)} (the baseline check below still applies)')
    if not m_s:
        problem('PROGRESS.md §0 lacks the "Documented source baseline" line')
        return finish()
    base = m_s.group(1)
    rc, full = git('rev-parse', '--verify', '--quiet', base + '^{commit}', check=True)
    if rc:
        problem(f'baseline {base[:12]} is not a commit in this repository')
        return finish()
    base = full.strip()
    if not hist or not base.startswith(hist[-1]) and not hist[-1].startswith(base):
        problem(f'§0 baseline {base[:12]} != last row of PROGRESS.md §8 ({hist[-1][:12] if hist else "none"})')
    rc, _ = git('merge-base', '--is-ancestor', base, 'HEAD', check=True)
    if rc:
        problem(f'baseline {base[:12]} is not an ancestor of HEAD (branch switch, rebase or reset?): '
                f'the documented revision is not in the current history')
        return finish()
    commits = git('rev-list', '--reverse', f'{base}..HEAD').split()
    doc_only = [c for c in commits
                if all(is_doc(p) for p in git('diff-tree', '--no-commit-id', '--name-only', '-r',
                                               '--root', c).split())]
    print(f'baseline {base[:12]}; HEAD is {len(commits)} commit(s) ahead '
          f'({len(doc_only)} documentation-only, {len(commits) - len(doc_only)} touching other paths)')

    # 2. what changed since the baseline: committed, uncommitted (tracked) and untracked
    changed = {}
    for line in git('diff', '--no-renames', '--name-status', base, 'HEAD').splitlines():
        st, p = line.split('\t', 1)
        changed[p] = f'committed {st}'
    for line in git('status', '--porcelain', '--untracked-files=all', '--no-renames').splitlines():
        code, p = line[:2], line[3:]
        tag = 'untracked' if code == '??' else f'uncommitted {code.strip()}'
        changed[p] = changed[p] + ' + ' + tag if p in changed else tag
    by_class = collections.defaultdict(list)
    for p, how in sorted(changed.items()):
        by_class[classify(p, ledger)].append((p, how))
    print(f'changes since baseline: {len(by_class["doc"])} documentation path(s) (ignored)')
    for p, how in by_class['source']:
        primary = ledger[p]['ch'] if p in ledger else 'not in ledger'
        mention = chapters_mentioning(p, chapter_texts)
        msg = f'{p} [{how}] primary ch. {primary}; mentioned in written ch. {", ".join(mention) or "none"}'
        reviews.append(msg)
        print('REVIEW:', msg)
    for p, how in by_class['interface']:
        note(f'interface-only dependency changed: {p} [{how}]; '
             f'mentioned in written ch. {", ".join(chapters_mentioning(p, chapter_texts)) or "none"}')
    for p, how in by_class['other']:
        note(f'path outside the documented scope changed: {p} [{how}]')

    # 3. ledger totals
    cnt = collections.Counter(r.get('status', 'pending') for r in data['ledger'])
    js = {s: cnt.get(s, 0) for s in STATUSES}
    print(f'ledger.json: {len(data["ledger"])} files, ' + ', '.join(f'{s} {js[s]}' for s in STATUSES))
    md = open(os.path.join(DOCS, 'ledger', 'coverage_ledger.md')).read()
    m = re.search(r'- Status: (.*)\.', md)
    md_cnt = dict((k, int(v)) for k, v in re.findall(r'(\w+) (\d+)', m.group(1))) if m else {}
    if md_cnt != js:
        problem(f'coverage_ledger.md totals {md_cnt} != ledger.json {js}')
    rows = collections.Counter(re.findall(r'^\| `[^`]+` \|.*\| (\w+) \|$', md, re.M))
    if {s: rows.get(s, 0) for s in STATUSES} != js:
        problem(f'coverage_ledger.md row statuses {dict(rows)} != ledger.json {js}')
    for s in STATUSES:
        m = re.search(r'^\| ' + s + r' \| (\d+) \|', progress, re.M)
        if not m or int(m.group(1)) != js[s]:
            problem(f'PROGRESS.md §2 "{s}" = {m.group(1) if m else "missing"}, ledger.json = {js[s]}')

    # 4. relative links
    files = sorted(glob.glob(os.path.join(DOCS, '**', '*.md'), recursive=True))
    claude_md = os.path.join(REPO, 'CLAUDE.md')
    if os.path.exists(claude_md):
        files.append(claude_md)
    else:
        problem('CLAUDE.md missing at the repository root')
    n_links = 0
    for f in files:
        text = re.sub(r'```.*?```', '', open(f).read(), flags=re.S)
        for target in re.findall(r'\]\(([^)\s]+)\)', text):
            if re.match(r'[a-z]+:', target) or target.startswith('#'):
                continue
            n_links += 1
            if not os.path.exists(os.path.normpath(os.path.join(os.path.dirname(f),
                                                                target.split('#')[0]))):
                problem(f'broken link in {os.path.relpath(f, REPO)}: {target}')
    print(f'links: {n_links} relative links checked in {len(files)} files')

    # 5. stage table
    for line in progress.splitlines():
        m = re.match(r'\| (\d) — [^|]+\|[^|]+\|[^|]+\| ([^|]+) \|', line)
        if m and not re.match(r'\**done\b', m.group(2).strip()):
            print(f'next stage by §1: Stage {m.group(1)} ({m.group(2).strip()})')
            break
    return finish()


def finish():
    if problems:
        print(f'RESULT: {len(problems)} problem(s)')
        return 1
    if reviews:
        print(f'RESULT: consistent; REVIEW NEEDED: {len(reviews)} source path(s) changed since the '
              f'documented baseline (procedure: PROGRESS.md §8)')
        return 2
    print('RESULT: consistent' + (f' ({len(notes)} note(s))' if notes else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
