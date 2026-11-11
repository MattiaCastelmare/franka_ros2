#!/usr/bin/env python3
"""Update the status (and optionally the chapter) of files in the coverage ledger.

Touches ONLY docs/framework_study/ledger/ledger.json and ledger/coverage_ledger.md.

Usage (from the repository root):
    python3 -I docs/framework_study/tools/ledger_status.py STATUS path [path ...] [--ch NN] [--stage N]

STATUS is one of: pending, partial, read, documented.
  partial    = the parts relevant to the current stage were read; the rest is pending
  read       = every function in the file was read
  documented = covered by its chapter and reviewed
--ch/--stage reassign the file's primary chapter / stage (both tables are rewritten).
Prints the new totals.
"""
from __future__ import annotations

import json
import os
import re
import sys

STATUSES = ('pending', 'partial', 'read', 'documented')
HERE = os.path.dirname(os.path.abspath(__file__))
LEDGER_DIR = os.path.join(os.path.dirname(HERE), 'ledger')
JSON = os.path.join(LEDGER_DIR, 'ledger.json')
MD = os.path.join(LEDGER_DIR, 'coverage_ledger.md')


def main(argv):
    if len(argv) < 2 or argv[0] not in STATUSES:
        print(__doc__)
        return 2
    status, rest = argv[0], argv[1:]
    ch = stage = None
    paths = []
    it = iter(rest)
    for a in it:
        if a == '--ch':
            ch = next(it)
        elif a == '--stage':
            stage = next(it)
        else:
            paths.append(a)
    data = json.load(open(JSON))
    by_path = {r['path']: r for r in data['ledger']}
    missing = [p for p in paths if p not in by_path]
    if missing:
        print('not in ledger:', missing)
        return 1
    md = open(MD).read()
    for p in paths:
        r = by_path[p]
        r['status'] = status
        if ch:
            r['ch'] = ch
        if stage:
            r['stage'] = int(stage)
        # rewrite the markdown row: | `path` | lines | role | ch | stage | [reach |] status |
        pat = re.compile(r'^(\| `' + re.escape(p) + r'` \|.*)$', re.M)
        m = pat.search(md)
        if not m:
            print('row not found in md:', p)
            continue
        cells = m.group(1).split(' | ')
        cells[-1] = status + ' |'
        if ch:
            cells[3] = ch
        if stage:
            cells[4] = str(stage)
        md = md[:m.start()] + ' | '.join(cells) + md[m.end():]
    # totals line
    cnt = {s: sum(1 for r in data['ledger'] if r.get('status', 'pending') == s) for s in STATUSES}
    md = re.sub(r'- Status: .*', '- Status: ' + ' · '.join(f'{s} {cnt[s]}' for s in STATUSES) + '.', md)
    json.dump(data, open(JSON, 'w'), indent=1)
    open(MD, 'w').write(md)
    print(cnt)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
