#!/usr/bin/env python3
"""Generate appendix A (`A_file_index.md`) from the ledger, the static inventory and chapter 15. Host.

Prints Markdown on stdout; it writes nothing itself. Regenerate with
    python3 -I -B docs/framework_study/tools/stage7_verification/gen_file_index.py \
        > docs/framework_study/A_file_index.md
after any ledger change (`tools/ledger_status.py`) or a chapter-15 edit.

Inputs (all in the repository):
- `ledger/ledger.json`: path, role, primary chapter, status, stage of every in-scope file (346);
- `tools/stage7_verification/results/inventory.json`: production importers, launch references,
  test importers, chapters citing the path;
- `15_code_status.md`: the code class (CURRENT, ALTERNATIVE, EXPERIMENTAL, LEGACY,
  UNUSED CANDIDATE, UNKNOWN) of the first table row that names the file's basename, or the class
  in the heading of that row's section. A file chapter 15 does not name gets "—" (tests, data and
  build metadata are classified by kind in ch. 14/15, not per file).
"""
import ast
import json
import os
import re
import sys
from collections import defaultdict

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.abspath(os.path.join(HERE, '..', '..'))

CHAPTERS = {
    '01': ('Architecture', '01_architecture.md'),
    '02': ('ROS graph and data flow', '02_ros_graph_and_dataflow.md'),
    '03': ('Configuration', '03_configuration.md'),
    '04': ('Mathematical foundations', '04_math_foundations.md'),
    '05': ('Perception and distance', '05_perception_and_distance.md'),
    '06': ('Nominal motion', '06_nominal_motion.md'),
    '07': ('CBF obstacle barrier', '07_cbf_obstacle_barrier.md'),
    '08': ('Constraints and QP', '08_constraints_and_qp.md'),
    '09': ('Torque realization', '09_torque_realization.md'),
    '10': ('Alternative controllers', '10_alternative_controllers.md'),
    '11': ('ISO safety layer', '11_iso_safety_layer.md'),
    '12': ('Simulation', '12_simulation.md'),
    '13': ('Observability and tools', '13_observability_and_tools.md'),
    '14': ('Build, launch and test', '14_build_launch_test.md'),
    '15': ('Code status', '15_code_status.md'),
    '16': ('Issues and discrepancies', '16_issues_and_discrepancies.md'),
}
CLASSES = ['UNUSED CANDIDATE', 'CURRENT', 'ALTERNATIVE', 'EXPERIMENTAL', 'LEGACY', 'UNKNOWN']


def as_list(v):
    if isinstance(v, list):
        return v
    try:
        x = ast.literal_eval(v) if isinstance(v, str) and v.startswith('[') else []
        return x if isinstance(x, list) else []
    except (ValueError, SyntaxError):
        return []


def class_map():
    text = open(os.path.join(DOCS, '15_code_status.md'), encoding='utf-8').read().splitlines()
    out, sec_cls = {}, None
    for ln in text:
        if ln.startswith('#'):
            sec_cls = next((c for c in CLASSES if c in ln), None)
            continue
        if not ln.startswith('| '):
            continue
        cells = [c.strip() for c in ln.strip('|').split('|')]
        names = re.findall(r'`([^`]+)`', cells[0])
        row_cls = None
        for c in cells[1:]:
            found = sorted((c.index(k), k) for k in CLASSES if k in c)
            found = [k for _, k in found if not (k == 'CURRENT' and 'UNUSED CANDIDATE' in c
                                                  and c.index('CURRENT') > c.index('UNUSED'))]
            if found:
                row_cls = ' / '.join(dict.fromkeys(found))
                break
        cls = row_cls or sec_cls
        if not cls:
            continue
        for n in names:
            for part in re.split(r'[,\s]+', n):
                base = os.path.basename(part.strip())
                if base and base not in out:
                    out[base] = cls
    # §6.1 classifies the launch files in prose: "CURRENT (`a`, `b`, …) and ALTERNATIVE (`c`, …)".
    joined = ' '.join(text)
    i = joined.find('### 6.1')
    para = joined[i:joined.find('###', i + 5)] if i >= 0 else ''
    for m in re.finditer(r'(CURRENT|ALTERNATIVE)\s*\(([^)]*)\)', para):
        for n in re.findall(r'`([^`]+)`', m.group(2)):
            out.setdefault(n + '.launch.py', m.group(1))
    return out


def main():
    led = json.load(open(os.path.join(DOCS, 'ledger', 'ledger.json'), encoding='utf-8'))['ledger']
    inv = json.load(open(os.path.join(HERE, 'results', 'inventory.json'), encoding='utf-8'))
    inv = inv.get('files', inv)
    cls = class_map()
    for f in led:
        f.setdefault('status', 'pending')     # the ledger omits the key for never-touched files
    by_ch = defaultdict(list)
    for f in led:
        by_ch[f['ch']].append(f)
    status_tot = defaultdict(int)
    for f in led:
        status_tot[f['status']] += 1

    p = print
    p('# Appendix A — File index')
    p()
    p('> **Generated** (Stage 7) by `tools/stage7_verification/gen_file_index.py` from '
      '[ledger/ledger.json](ledger/ledger.json), `tools/stage7_verification/results/inventory.json` and '
      '[15_code_status.md](15_code_status.md). Do not edit by hand: change the ledger '
      '(`tools/ledger_status.py`) or chapter 15 and regenerate. Baseline `ba68077`.')
    p()
    p('Each in-scope file appears once, under its **primary** chapter. Columns:')
    p('- **Status** (coverage): `documented` = covered by its chapter; `read` = every function read; '
      '`partial` = the parts named in PROGRESS §2 were read (for tests: static traceability only, '
      'C-043); `pending` = inventoried only, not read.')
    p('- **Class** (ch. 15): CURRENT / ALTERNATIVE / EXPERIMENTAL / LEGACY / UNUSED CANDIDATE / UNKNOWN; '
      '"—" where ch. 15 classifies by kind (tests, data, build files) and not per file.')
    p('- **Prod / Launch / Tests**: production importers, launch files that reference it, test files '
      'that import it (static, `inventory.py`). **Cited in**: chapters that name the path.')
    p()
    p('| Status | Files |')
    p('|---|---:|')
    for s in ('documented', 'read', 'partial', 'pending'):
        p(f'| {s} | {status_tot[s]} |')
    p(f'| **total** | **{len(led)}** |')
    p()
    for ch in sorted(by_ch):
        title, fn = CHAPTERS.get(ch, (f'chapter {ch}', None))
        files = sorted(by_ch[ch], key=lambda f: f['path'])
        cnt = defaultdict(int)
        for f in files:
            cnt[f['status']] += 1
        link = f'[{fn}]({fn})' if fn and os.path.exists(os.path.join(DOCS, fn)) else '(no file)'
        p(f'## {ch} — {title} → {link}')
        p()
        p(f'{len(files)} files, {sum(int(f["lines"]) for f in files)} lines; ' +
          ', '.join(f'{k} {cnt[k]}' for k in ('documented', 'read', 'partial', 'pending') if cnt[k]) + '.')
        p()
        p('| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |')
        p('|---|---|---:|---|---|---:|---:|---:|---|')
        for f in files:
            i = inv.get(f['path'], {})
            prod = len(as_list(i.get('importers_prod', [])))
            lau = len(as_list(i.get('launch', [])))
            tst = len(as_list(i.get('tests', []))) or len(as_list(i.get('importers_test', [])))
            cited = ', '.join(sorted(as_list(i.get('path_cited_in', []))))
            c = cls.get(os.path.basename(f['path']), '—')
            p(f'| `{f["path"]}` | {f["role"]} | {f["lines"]} | {f["status"]} | {c} | {prod} | {lau} | '
              f'{tst} | {cited} |')
        p()
    return 0


if __name__ == '__main__':
    sys.exit(main())
