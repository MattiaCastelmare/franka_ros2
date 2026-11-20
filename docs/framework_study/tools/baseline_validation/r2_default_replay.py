#!/usr/bin/env python3
"""R2 performance check, the ``default`` arm: the replay with NO executor override anywhere.

Approved by the user on 2026-11-20 (R2 STEP 6). Protocol: ``refactor_r2/04_perf/protocol/R2_PERF_PROTOCOL.md``.

  python3 -B r2_default_replay.py <cell_dir> <out_prefix> <rate> <params_file>...

The executor must come from the INSTALLED ``fr3_control.yaml``, as in ``cbf_safety_filter.main``. Two
things would otherwise set it:
* ``r1_executor_replay.py`` always appends ``-p executor_mode:=<mode>`` to the node's ``--ros-args``;
* the E3-05 parameter file of the ``full`` cell (``launch_params_xa00zx9n``) pins ``executor_mode: multi``.

So this wrapper:
1. refuses to start if any parameter file sets an ``executor_mode`` key (the harness passes a copy of
   ``launch_params_xa00zx9n`` with that one line removed);
2. replaces ``rclpy.init`` for this process only: it removes exactly the pair
   ``['-p', 'executor_mode:=single']`` that ``r1_executor_replay.py`` appends, and refuses if that pair is
   not there exactly once or if any other ``executor_mode:=`` override remains;
3. records the arguments actually passed to ``rclpy.init``, the removed pair and the parameter files
   with their SHA-256 in ``<out_prefix>_r2.json``.

Everything else is ``m1_executor_replay.py`` (arm ``single``) and ``r1_executor_replay.py``, run unchanged
with ``runpy``: the same node, callbacks, instrumentation, ``ros2 bag play`` and outputs. The ``single``
arm underneath only selects M1's validity expectation (``SingleThreadedExecutor``), which is the R2
claim. Its ``executor_mode_param`` field in ``_m1.json`` names the override it asked for, which this
wrapper removed. The effective values are in ``_r2.json``, and the node's resolved mode is
``node_executor_mode`` in ``_m1.json``.
"""
import hashlib
import json
import os
import re
import runpy
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
M1 = os.path.join(HERE, 'm1_executor_replay.py')
UNDERLYING_ARM = 'single'
APPENDED = ['-p', f'executor_mode:={UNDERLYING_ARM}']      # what r1_executor_replay.py appends
# A parameter KEY, not any mention: the E3-05 robot_config_path contains the cell directory name
# '...__full-executor_mode=multi__...', which sets nothing.
PARAM_KEY = re.compile(rb'^\s*executor_mode\s*:', re.M)


def sets_executor(params_text):
    return bool(PARAM_KEY.search(params_text))


def strip_override(args):
    """``args`` without the one appended executor override; ValueError if the shape is not the expected one."""
    args = list(args)
    idx = [i for i in range(len(args) - 1) if args[i:i + 2] == APPENDED]
    if len(idx) != 1:
        raise ValueError(f'expected the pair {APPENDED} exactly once in the rclpy.init args, found {len(idx)}')
    out = args[:idx[0]] + args[idx[0] + 2:]
    left = [a for a in out if a.startswith('executor_mode:=')]
    if left:
        raise ValueError(f'executor_mode still set after removing the appended pair: {left}')
    return out


def main(argv):
    cd, outp, rate, pfiles = argv[0], argv[1], argv[2], argv[3:]
    rec = {'arm': 'default', 'underlying_m1_arm': UNDERLYING_ARM, 'rate': float(rate), 'cell_dir': cd,
           'params_files': {}, 'removed_pair': APPENDED}
    for p in pfiles:
        with open(p, 'rb') as f:
            txt = f.read()
        if sets_executor(txt):
            sys.exit(f'{p}: sets executor_mode; the default arm needs parameter files without it')
        rec['params_files'][p] = hashlib.sha256(txt).hexdigest()

    import rclpy
    real_init = rclpy.init

    def init(*a, args=None, **k):
        new = strip_override(args or [])
        rec['rclpy_init_args'] = new
        with open(outp + '_r2.json', 'w') as f:
            json.dump(rec, f, indent=1)
        return real_init(*a, args=new, **k)

    rclpy.init = init
    sys.argv = [M1, UNDERLYING_ARM, cd, outp, rate, *pfiles]
    runpy.run_path(M1, run_name='__main__')


if __name__ == '__main__':
    main(sys.argv[1:])
