#!/usr/bin/env python3
"""QP-performance study, phase 3 — does a native call release the GIL? (isolated microbenchmark)

  python3 -B qpperf_gil_probe.py <out_json>

Method: a pure-Python "probe" thread increments a counter as fast as it can (it needs the GIL for
every bytecode). A "worker" thread runs one of the candidate operations in a tight loop. If the
operation releases the GIL while it computes, the probe keeps most of its solo rate; if it holds the
GIL, the probe gets roughly the share the interpreter's switch interval (5 ms) grants it.
Operations, at the filter's problem size:
  * osqp 0.6 `solve()` on a random QP with nx=13, m=30 (slightly smaller than the filter's: 7 joints +
    N_SLACK 7 + per-row slots >= 14 decision variables; rows p50 26 + box), and on a
    300-variable dense QP (same Python overhead, ~1000x the C work) to isolate the release;
  * Pinocchio `computeJointJacobians` + `framesForwardKinematics` on the FR3 URDF;
  * a small-array NumPy chain (norm / cross / clip on 3-vectors), the rebuild's typical pattern;
  * `time.sleep(0.5 ms)` as a reference for "releases" and a pure-Python loop for "holds".
Reports probe throughput relative to solo, per operation.
"""
import json
import sys
import threading
import time

import numpy as np

sys.dont_write_bytecode = True
OUT = sys.argv[1]


def probe_rate(work, dur=1.5):
    stop = threading.Event()
    cnt = [0]

    def probe():
        n = 0
        while not stop.is_set():
            n += 1
        cnt[0] = n

    def worker():
        while not stop.is_set():
            work()

    tp = threading.Thread(target=probe)
    tw = threading.Thread(target=worker) if work else None
    tp.start()
    if tw:
        tw.start()
    time.sleep(dur)
    stop.set()
    tp.join()
    if tw:
        tw.join()
    return cnt[0] / dur


# ---- candidate operations
import osqp  # noqa: E402
import scipy.sparse as sp  # noqa: E402

rng = np.random.default_rng(0)
nx, m = 13, 30
P = sp.csc_matrix(np.eye(nx))
A = sp.csc_matrix(np.vstack([rng.normal(size=(m - nx, nx)), np.eye(nx)]))
lo = -np.ones(m) * 5
hi = np.ones(m) * 5
q = rng.normal(size=nx)
prob = osqp.OSQP()
prob.setup(P=P, q=q, A=A, l=lo, u=hi, verbose=False, warm_start=False, max_iter=4000,
           eps_abs=1e-7, eps_rel=1e-7, polish=True)


def osqp_solve():
    prob.update(q=rng.normal(size=nx))
    prob.solve()


NL, ML = 300, 600
PL = sp.csc_matrix(np.eye(NL))
AL = sp.csc_matrix(rng.normal(size=(ML, NL)))
probL = osqp.OSQP()
probL.setup(P=PL, q=rng.normal(size=NL), A=AL, l=-np.ones(ML), u=np.ones(ML), verbose=False,
            warm_start=False, max_iter=200, eps_abs=1e-9, eps_rel=1e-9, polish=False)


def osqp_solve_large():
    probL.solve()          # same Python overhead, ~1000x the C work: isolates the release


try:
    import pinocchio as pin
    from ament_index_python.packages import get_package_share_directory  # noqa: F401
    import glob
    urdf = sorted(glob.glob('/home/user/s6b_20261112/batch/cells/*/fr3_rsp.urdf'))[0]
    model = pin.buildModelFromXML(open(urdf).read())
    data = model.createData()
    qpin = pin.neutral(model)

    def pin_fk():
        pin.computeJointJacobians(model, data, qpin)
        pin.framesForwardKinematics(model, data, qpin)
except Exception as exc:  # pragma: no cover
    pin_fk = None
    PIN_ERR = repr(exc)

a3, b3 = rng.normal(size=3), rng.normal(size=3)


def np_small():
    for _ in range(20):
        np.linalg.norm(np.cross(a3, b3))
        np.clip(a3, -1, 1)


def py_loop():
    s = 0
    for i in range(2000):
        s += i


def sleep0():
    time.sleep(0.0005)


# time per call of each op (single thread)
def per_call(f, n=300):
    t0 = time.perf_counter()
    for _ in range(n):
        f()
    return 1e3 * (time.perf_counter() - t0) / n


R = {'switchinterval_s': sys.getswitchinterval()}
solo = np.median([probe_rate(None) for _ in range(3)])
R['probe_solo_per_s'] = float(solo)
for name, f in (('sleep_0.5ms(releases)', sleep0), ('pure_python(holds)', py_loop),
                ('osqp_solve', osqp_solve), ('osqp_solve_large', osqp_solve_large), ('pinocchio_fk_jac', pin_fk), ('numpy_small', np_small)):
    if f is None:
        R[name] = {'error': PIN_ERR}
        continue
    r = np.median([probe_rate(f) for _ in range(3)])
    R[name] = dict(ms_per_call=per_call(f), probe_rel_to_solo=float(r / solo))
json.dump(R, open(OUT, 'w'), indent=1)
print(json.dumps(R, indent=1))
