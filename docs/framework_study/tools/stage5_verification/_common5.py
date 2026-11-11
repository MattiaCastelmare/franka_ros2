"""Shared helpers for the Stage 5 verification scripts (chapters 10, 11, 13). Read-only.

Re-uses `../stage4_verification/_common4.py` (which re-uses Stages 2 and 3): repository paths, the
source-import guard (F-2-2), the PASS/FAIL `Report`, the effective parameters per launch
configuration (`C3.effective_params`) and `C4.static_limits`. Adds `src(path)` (read a repository
file as text, optionally stripping comments) and `stub(**kw)` (a stand-in `self` with a no-op logger,
used to call unbound production methods without a ROS graph).
"""
import os
import re
import sys
from types import SimpleNamespace

sys.dont_write_bytecode = True
HERE5 = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE5, '..', 'stage4_verification'))
import _common4 as C4  # noqa: E402

C, C3 = C4.C, C4.C3


def src(rel, strip=None):
    s = open(os.path.join(C.REPO, rel)).read()
    if strip == 'py':
        s = '\n'.join(x for x in s.splitlines() if not x.strip().startswith('#'))
    elif strip == 'cpp':
        s = re.sub(r'/\*.*?\*/', '', s, flags=re.S)
        s = re.sub(r'//[^\n]*', '', s)
    return s


class _Log:
    def __init__(self):
        self.lines = []

    def _r(self, lvl, m, **_):
        self.lines.append((lvl, str(m)))

    def info(self, m, **k):
        self._r('info', m, **k)

    def warn(self, m, **k):
        self._r('warn', m, **k)

    warning = warn

    def error(self, m, **k):
        self._r('error', m, **k)


def stub(**kw):
    log = _Log()
    return SimpleNamespace(get_logger=lambda: log, _log=log, **kw)
