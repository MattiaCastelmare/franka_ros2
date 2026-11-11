#!/usr/bin/env python3
"""Documentation consistency of `docs/framework_study/` (Stage 7). Host, read-only.

Validates the documentation, not the code. Every check is a FACT and must PASS for Stage 7 to be
complete:

1. Findings: every `F-<s>-<n>` defined in PROGRESS §4 has exactly one row in chapter 16 Part B
   (F-4-9 is allowed a second, pointer row in B.6, as ch. 16 states), and every ID with a ch. 16
   row is defined in PROGRESS §4. The count matches the "Total: N findings" line of ch. 16.
2. Every finding ID cited anywhere in the documentation is defined in PROGRESS §4.
3. Every question ID `Q-<s>-<n>` cited anywhere is defined in a PROGRESS §5 table.
4. Chapter 15 names every non-test Python, launch and configuration file of both packages (by
   basename), as ch. 15 §8 promises.
5. Equation references `(C.N)` with C in 4…13, outside chapter 16 Part E (which cites the OLD
   guide's numbering), resolve to a tag that chapter C defines.
6. Every numbered chapter 01–16 has a "Consequences of modification" section (CLAUDE.md rule).
7. Every relative Markdown link of the documentation resolves to an existing file (anchors not
   checked).
8. The generated appendices are current: regenerating A and B gives the committed files byte for
   byte (they are stdout generators, run in memory here).
9. Every stage verification directory 2–7 has a README, a `run_all.sh` and a recorded result.
10. The resume card names the newest checkpoint (resume safety).
11. No document outside chapter 16 Part E states the superseded QP size (six slack families, ℝ¹³).

Usage: python3 -I -B check_docs.py
"""
import io
import os
import re
import subprocess
import sys
from contextlib import redirect_stdout

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.abspath(os.path.join(HERE, '..', '..'))
REPO = os.path.abspath(os.path.join(DOCS, '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'stage2_verification'))
import _common as C  # noqa: E402

BASELINE = 'ba68077052e8d0fb3d4606c05dff553b75baa641'


def rd(fn):
    return open(os.path.join(DOCS, fn), encoding='utf-8').read()


def doc_files():
    out = []
    for root, dirs, files in os.walk(DOCS):
        dirs[:] = [d for d in dirs if d not in ('__pycache__',)]
        for f in files:
            if f.endswith('.md'):
                out.append(os.path.relpath(os.path.join(root, f), DOCS))
    return sorted(out)


def main():
    R = C.Report('Stage 7 — documentation consistency (check_docs.py)')
    prog = rd('PROGRESS.md')
    ch16 = rd('16_issues_and_discrepancies.md')
    docs = {f: rd(f) for f in doc_files()}

    # 1 — findings ↔ ch. 16
    s4 = prog[prog.index('## 4. Findings log'):prog.index('## 5. Decisions')]
    defined = re.findall(r'^\| (F-\d-\d+) \|', s4, re.M)
    dup_def = sorted({x for x in defined if defined.count(x) > 1})
    partB = ch16[ch16.index('## Part B'):ch16.index('## Part C')]
    rows = re.findall(r'^\| (F-\d-\d+) \|', partB, re.M)
    multi = sorted({x for x in rows if rows.count(x) > 1} - {'F-4-9'})
    missing = sorted(set(defined) - set(rows))
    extra = sorted(set(rows) - set(defined))
    m = re.search(r'Total: (\d+) findings', ch16)
    total = int(m.group(1)) if m else -1
    R.fact('1a: each PROGRESS finding defined once', not dup_def, f'{len(defined)} defined; '
           f'duplicates {dup_def}')
    R.fact('1b: each finding has exactly one ch. 16 Part B row (F-4-9 pointer allowed)',
           not missing and not multi and not extra,
           f'missing {missing}, repeated {multi}, unknown {extra}')
    R.fact('1c: ch. 16 total matches', total == len(set(defined)), f'ch. 16 says {total}, '
           f'PROGRESS defines {len(set(defined))}')

    # 2 — finding citations
    cited = set()
    for f, t in docs.items():
        if f.startswith('tools/') or f == 'CHECKPOINTS.md':
            continue
        cited |= set(re.findall(r'\bF-\d-\d+\b', t))
    undef = sorted(cited - set(defined))
    R.fact('2: every cited finding ID is defined', not undef, f'{len(cited)} cited; undefined {undef}')

    # 3 — questions
    s5 = prog[prog.index('## 5. Decisions'):prog.index('## 6. Checkpoint protocol')]
    qdef = set(re.findall(r'^\| (Q-\d-\d+)\b', s5, re.M))
    qcited = set()
    for f, t in docs.items():
        if f.startswith('tools/'):
            continue
        for a, b, c in re.findall(r'\bQ-(\d)-(\d+)(?:…Q-\d-(\d+))?', t):
            qcited.add(f'Q-{a}-{b}')
            if c:
                for k in range(int(b), int(c) + 1):
                    qcited.add(f'Q-{a}-{k}')
    qundef = sorted(qcited - qdef)
    R.fact('3: every cited question ID is defined in PROGRESS §5', not qundef,
           f'{len(qdef)} defined, {len(qcited)} cited; undefined {qundef}')

    # 4 — ch. 15 names every production file
    ch15 = rd('15_code_status.md')
    tree = subprocess.run(['git', '-C', REPO, 'ls-tree', '-r', '--name-only', BASELINE],
                          capture_output=True, text=True, check=True).stdout.splitlines()
    prod = [p for p in tree
            if p.startswith(('franka_experiments/', 'franka_simulation_experiments/'))
            and '/test/' not in p and '/sim_runs/' not in p and '/docs/' not in p
            and p.endswith(('.py', '.yaml', '.launch.py', '.rviz', '.sh'))
            and os.path.basename(p) not in ('setup.py', '__init__.py')]
    # expand brace groups such as `simexp/{analysis, batch, …}` into `simexp/analysis` …
    for m in re.finditer(r'([\w/]*)\{([^}`]+)\}', ch15):
        ch15 += ' ' + ' '.join(f'`{m.group(1)}{x.strip()}` `{x.strip()}`' for x in m.group(2).split(','))

    def named(p):
        b = os.path.basename(p)
        stem = b[:-len('.launch.py')] if b.endswith('.launch.py') else b.rsplit('.', 1)[0]
        glob = re.sub(r'_[^_]*$', '_*', stem)          # `throw_*.py` names throw_lead.py
        return b in ch15 or f'`{stem}`' in ch15 or f'{glob}.py' in ch15
    unnamed = sorted(p for p in prod if not named(p))
    R.fact('4: ch. 15 names every non-test production file (by basename)', not unnamed,
           f'{len(prod)} files; not named: {unnamed[:12]}{" …" if len(unnamed) > 12 else ""}')

    # 5 — equation references
    tags = {}
    for c in range(4, 14):
        fn = [f for f in docs if re.match(r'%02d_' % c, f)][0]
        tags[c] = set(re.findall(r'\((%d\.\d+[a-z]?)\)' % c, docs[fn]))
    bad = []
    for f, t in docs.items():
        if f.startswith('tools/') or f in ('B_equation_index.md', 'CHECKPOINTS.md'):
            continue
        if f == '16_issues_and_discrepancies.md':
            t = t[:t.index('## Part E')]
        for m in re.finditer(r'\((\d{1,2})\.(\d+[a-z]?)\)', t):
            c = int(m.group(1))
            if 4 <= c <= 13 and f'{c}.{m.group(2)}' not in tags[c]:
                line = t[:m.start()].count('\n') + 1
                bad.append(f'{f}:{line} ({c}.{m.group(2)})')
    R.fact('5: equation references resolve', not bad, f'unresolved: {bad[:15]}')

    # 6 — consequences sections
    nocons = []
    for f in docs:
        if re.match(r'(0[1-9]|1[0-6])_', f) and 'Consequences' not in docs[f]:
            nocons.append(f)
    R.fact('6: every chapter 01–16 has a "Consequences of modification" section', not nocons,
           f'missing in {nocons}')

    # 7 — links
    broken, n = [], 0
    for f, t in docs.items():
        base = os.path.dirname(os.path.join(DOCS, f))
        t = re.sub(r'`[^`\n]*`', '', t)                 # links never sit inside inline code
        for m in re.finditer(r'\]\(([^)\s#]+)(?:#[^)]*)?\)', t):
            tgt = m.group(1)
            if re.match(r'^[a-z]+:', tgt):
                continue
            n += 1
            if not os.path.exists(os.path.normpath(os.path.join(base, tgt))):
                broken.append(f'{f}: {tgt}')
    R.fact('7: relative links resolve', not broken, f'{n} links; broken {broken[:10]}')

    # 8 — generated appendices are current
    for gen, target in (('gen_file_index.py', 'A_file_index.md'),
                        ('gen_equation_index.py', 'B_equation_index.md')):
        buf = io.StringIO()
        ns = {'__name__': 'gen', '__file__': os.path.join(HERE, gen)}
        code = compile(open(os.path.join(HERE, gen), encoding='utf-8').read(), gen, 'exec')
        with redirect_stdout(buf):
            exec(code, ns)
            ns['main']()
        R.fact(f'8: {target} equals a fresh run of {gen}', buf.getvalue() == rd(target),
               'regenerate with the command in the generator docstring if not')

    # 9 — verification directories
    lack = []
    for st in range(2, 8):
        d = os.path.join(DOCS, 'tools', f'stage{st}_verification')
        for need in ('README.md', 'run_all.sh'):
            if not os.path.exists(os.path.join(d, need)):
                lack.append(f'stage{st}/{need}')
        res = os.path.join(d, 'results')
        if not (os.path.isdir(res) and any(x.startswith('observed') for x in os.listdir(res))):
            lack.append(f'stage{st}/results/observed_*')
    R.fact('9: each stage 2–7 has README, run_all.sh and recorded results', not lack, f'missing {lack}')

    # 11 — superseded statements must not reappear (ch. 16 Part E, E-1 and E-3)
    stale = []
    for f, t in docs.items():
        if f.startswith('tools/') or f in ('CHECKPOINTS.md', 'PROGRESS.md'):
            continue
        if f == '16_issues_and_discrepancies.md':
            t = t[:t.index('## Part E')]
        for pat in (r'six (?:slack )?famil', r'ℝ¹³', r'13 variables', r'R\^\{?13'):
            for m in re.finditer(pat, t):
                stale.append(f'{f}:{t[:m.start()].count(chr(10)) + 1}')
    R.fact('11: no chapter states the old QP size (six families, ℝ¹³) outside 16 Part E', not stale,
           f'{stale}')

    # 10 — resume safety
    last = re.findall(r'^## (C-\d+) ', rd('CHECKPOINTS.md'), re.M)[-1]
    card = prog[prog.index('## 0. Resume card'):prog.index('## 1. Stages')]
    R.fact('10: the resume card names the newest checkpoint', last in card, f'newest {last}')
    return R.finish()


if __name__ == '__main__':
    sys.exit(main())
