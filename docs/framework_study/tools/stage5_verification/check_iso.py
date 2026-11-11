#!/usr/bin/env python3
"""The ISO-inspired layer: SSM math, configuration gate, monitor and preflight (chapter 11).

Production code: `utils/iso_ssm.py` (all four functions), `CBFSafetyFilter._iso_configure`
(unbound, on a stub `self`), `iso_safety_monitor.trip_ticks`, and `scripts/iso_preflight_check.py`
(run as a read-only subprocess on the source YAML). `iso_pfl_speed.ANNEX_M_ROWS` and
`FR3_MASS_KG` are read with `ast` (the script is not executed).

  1. (11.4) ssm_speed_cap is the exact inverse of protective_separation in v_r (2000 random cases);
     cap = 0 at or inside the floor C + Z_d + Z_r                                              (NUM)
  2. (11.6) pfl_speed: hands-and-fingers, no payload, reproduces iso_v_pfl = 0.68; the other
     regions of the script's own table give lower speeds (minimum = chest)                (REPRO/NUM)
  3. what v_h does: S_p at the shipped constants with the MEASURED approach speed (0 when untracked,
     as the monitor and SSM rows use) vs the normative iso_v_human = 2.0                       (NUM)
  4. _iso_configure: inert with the flag off; with iso_enabled and the shipped values it RAISES
     (d_safe = 0.10 < C+Z_d+Z_r = 0.92); with d_safe ≥ 0.92, link 0.68, cap 0.60 it passes and turns
     the standoff on with t = T_r + v_h/a_s, max = min(v_h·t, 2 m)                             (NUM)
  5. monitor trip debounce: trip_ticks = round(iso_monitor_ticks·qp_rate_hz/100)               (NUM)
  6. the launch preflight reads only the source/installed YAML: with iso_enabled false there it
     exits 0 without checking anything (command-line iso_enabled is invisible to it)          (FACT)
  7. the monitor's S_h uses the raw track velocity (no frames_seen gate), not iso_v_human     (FACT)
  8. F-5-17 stop-latch deadlock: the production monitor (_tick, _on_status, _on_reset on a stub
     `self`) closed with the filter's VERIFIED reaction to a latch (ISO-stop branch publishes
     fault_braking = 1.0): once latched, the reset is refused and auto-resume never happens, after
     the scene itself has cleared                                                    (FACT, closed loop
     with the filter side transcribed from the verified source)
  9. F-5-18 the row builder's S_p diagnostic is protective_separation(cap, v_app), which equals
     the gap itself wherever the cap is not saturated, so cbf_status[5] means a different thing
     from the monitor's S_p; the SSM speed row bounds ‖ṗ‖, the monitor the closing component (NUM/FACT)
 10. F-5-19 the monitor latches STALE trip_ticks ticks after start if any input is missing, and
     keeps the last cbf_status fault flag forever (no age check)                               (FACT)
 11. chapter 11 §7 worked example (d = 1.2 m, closing 0.2 m/s; v_h = 0 and 2.0; shipped constants) (NUM)

Needs: the ROS 2 container (the filter module imports osqp/rclpy/pinocchio). Read-only.
"""
import ast
import re
import os
import subprocess
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common5 as C5  # noqa: E402

C, C3 = C5.C, C5.C3
import numpy as np  # noqa: E402
from franka_experiments.utils.iso_ssm import (  # noqa: E402
    pfl_speed, protective_separation, ssm_speed_cap, stopping_distance)

R = C.Report('check_iso (chapter 11)')
y = C3.yaml_params()
K = dict(t_r=y['iso_t_reaction'], a_s=y['iso_a_stop'], c=y['iso_c_intrusion'], z_d=y['iso_z_depth'],
         z_r=y['iso_z_robot'])
floor = K['c'] + K['z_d'] + K['z_r']
R.info(f"shipped constants: T_r {K['t_r']} s, a_s {K['a_s']} m/s², C {K['c']} m, Z_d {K['z_d']}, Z_r {K['z_r']} "
       f"→ floor C+Z_d+Z_r = {floor:.2f} m; v_h {y['iso_v_human']}; v_PFL {y['iso_v_pfl']}")

# 1
rng = np.random.default_rng(15)
err, n_ok = 0.0, 0
for _ in range(2000):
    d = rng.uniform(floor, floor + 3.0)
    vh = rng.uniform(0, 2.5)
    v = ssm_speed_cap(d, vh, v_max=1e9, **K)
    if v > 0:
        err = max(err, abs(protective_separation(v, vh, **K) - d))
        n_ok += 1
R.below(f'1. protective_separation(ssm_speed_cap(d, v_h), v_h) = d ({n_ok} cases with a positive cap) [m]', err, 1e-9)
R.fact('1. cap = 0 at d = floor and inside it', ssm_speed_cap(floor, 0.0, v_max=9, **K) == 0.0
       and ssm_speed_cap(floor - 0.1, 0.0, v_max=9, **K) == 0.0)
R.close('1. stopping_distance = v T_r + v²/(2a_s) at v = 0.5', stopping_distance(0.5, t_r=K['t_r'], a_s=K['a_s']),
        0.5 * K['t_r'] + 0.25 / (2 * K['a_s']), 1e-12, kind='NUM')

# 2
tree = ast.parse(C5.src('franka_experiments/scripts/iso_pfl_speed.py'))
consts = {}
for nd in tree.body:
    if isinstance(nd, ast.Assign) and isinstance(nd.targets[0], ast.Name) and nd.targets[0].id in ('ANNEX_M_ROWS', 'FR3_MASS_KG'):
        consts[nd.targets[0].id] = ast.literal_eval(nd.value)
rows, m_r = consts['ANNEX_M_ROWS'], consts['FR3_MASS_KG'] / 2.0
v = {reg: pfl_speed(f, k * 1000.0, m_r, mh) for reg, (f, k, mh) in rows.items()}
R.close('2. v_PFL hands-and-fingers (F 140 N, k 75 N/mm, m_H 0.6 kg, m_R = 17.8/2) = iso_v_pfl', v['hands-and-fingers'],
        y['iso_v_pfl'], 5e-3)
low = min(v, key=v.get)
R.info('2. v_PFL per region (script table, no payload): ' + ', '.join(f'{r} {x:.3f}' for r, x in sorted(v.items(), key=lambda t: t[1])))
R.fact(f'2. iso_v_pfl is the hands-and-fingers value; the binding (minimum) region is {low} at {v[low]:.3f} m/s',
       v[low] < y['iso_v_pfl'])

# 3
for vr in (0.25, 0.5):
    s0 = protective_separation(vr, 0.0, **K)
    s2 = protective_separation(vr, y['iso_v_human'], **K)
    R.info(f'3. v_r = {vr}: S_p with measured v_h = 0 → {s0:.3f} m; with normative v_h = 2.0 → {s2:.3f} m '
           f'(S_h = {s2 - s0:.3f} m)')
c0 = ssm_speed_cap(1.5, 0.0, v_max=9, **K)
c2 = ssm_speed_cap(1.5, y['iso_v_human'], v_max=9, **K)
R.fact(f'3. at d = 1.5 m the SSM speed cap is {c0:.3f} m/s with v_h = 0 and {c2:.3f} m/s with v_h = 2.0', c0 > c2)

# 4
from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter as F  # noqa: E402
P = C3.params_namespace('hw_default')
F._iso_configure(C5.stub(), P)
R.fact('4. flag off: _iso_configure returns without touching P', P.enable_velocity_standoff is True
       and P.velocity_standoff_time_s == 0.25)
P1 = C3.params_namespace('hw_default', iso_enabled=True, config_path='fr3_control.yaml')
try:
    F._iso_configure(C5.stub(), P1)
    raised = None
except ValueError as e:
    raised = str(e).splitlines()[0]
R.fact('4. shipped values + iso_enabled → ValueError (fail closed)', raised is not None, raised or '')
P2 = C3.params_namespace('hw_default', iso_enabled=True, d_safe=0.95, link_speed_max=0.68, retreat_cap_max_speed=0.60, config_path='fr3_control.yaml')
F._iso_configure(C5.stub(), P2)
t_exp = K['t_r'] + y['iso_v_human'] / K['a_s']
R.close('4. with d_safe 0.95 / link 0.68 / cap 0.60: standoff time = T_r + v_h/a_s [s], max = min(v_h·t, 2) [m]',
        [P2.velocity_standoff_time_s, P2.velocity_standoff_max], [min(t_exp, 5.0), min(y['iso_v_human'] * t_exp, 2.0)],
        1e-12, kind='NUM')
R.fact('4. d_safe must exceed the FR3 reach (0.855 m, iso_evidence_logger manifest) for the gate to pass',
       floor > 0.855, f'floor {floor:.2f} m — the D1 "structural" deviation of iso_evidence_report')

# 5
from franka_experiments.nodes.iso_safety_monitor import trip_ticks  # noqa: E402
R.close('5. trip_ticks at qp_rate_hz 300 (iso_monitor_ticks 3) → 9 ticks = 30 ms', trip_ticks(P), 9, 0, kind='NUM')

# 6
r = subprocess.run([sys.executable, '-B', os.path.join(C.PKG_SRC, 'scripts', 'iso_preflight_check.py'),
                    '--config', os.path.join(C.CONFIG, 'fr3_control.yaml')], capture_output=True, text=True)
R.fact('6. preflight on the shipped YAML: exit 0 "nothing to check" (it never sees a command-line iso_enabled)',
       r.returncode == 0 and 'nothing to check' in r.stdout, r.stdout.strip()[:90])
L = C5.src('franka_experiments/launch/torque_control_stack.launch.py', 'py')
R.fact("6. the launch passes only --config <share>/config/fr3_control.yaml to the preflight",
       "'scripts', 'iso_preflight_check.py']),\n                 '--config'," in L)

# 7
M = C5.src('franka_experiments/franka_experiments/nodes/iso_safety_monitor.py', 'py')
R.fact('7. monitor: v_app = max(n̂·obstacle_velocity, 0) fed to ssm_speed_cap/protective_separation; '
       'iso_v_human appears only in a log line', 'v_app = max(float(n @ np.array([ld.obstacle_velocity.x,' in M
       and M.count('iso_v_human') == 1 and 'frames_seen' not in M)

# 8–10: the monitor's latch, driven through its production methods on a stub
import types  # noqa: E402
from franka_experiments.nodes.iso_safety_monitor import (  # noqa: E402
    ISOSafetyMonitor as MON, REASON_FAULT, REASON_INSIDE, REASON_NONE, REASON_SPEED, REASON_STALE)


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


def _monitor(requires_reset):
    m = C5.stub()
    m.P = C3.params_namespace('hw_default', iso_stop_requires_reset=requires_reset)
    m._floor = m.P.iso_c_intrusion + m.P.iso_z_depth + m.P.iso_z_robot
    m._reduced, m._tcp_link = False, 'fr3_link8'
    m._q, m._qdot = np.zeros(7), np.zeros(7)
    m._js_stamp = m._dist_stamp = 100.0
    m._dist = object()
    m._fault, m._latched, m._reason, m._live = 0.0, False, REASON_NONE, REASON_NONE
    m._streak = {REASON_SPEED: 0, REASON_INSIDE: 0, REASON_FAULT: 0, REASON_STALE: 0}
    m._diag = (0.0, float('inf'), 0.0)
    m._pub, m._msg = _Pub(), types.SimpleNamespace(data=[0.0] * 5)
    m._now = lambda: 100.0
    return m


FS = C5.src('franka_experiments/franka_experiments/nodes/cbf_safety_filter.py', 'py')
iso_branch = FS[FS.index('if self._iso_stop_active(now):'):FS.index('if self._iso_was_latched:\n')]
R.fact('8. filter (verified): while _iso_stop_active the tick brakes and publishes status with fault=1.0 '
       '(data[2] = fault_braking, the field the monitor reads as a chain fault)',
       'self._publish_status(0, 0.0, fault=1.0, n_act=0)' in iso_branch
       and "'n_rows', 'slack_max', 'fault_braking'" in C5.src('franka_experiments/franka_experiments/utils/cbf_diagnostics.py'))
for req in (True, False):
    m = _monitor(req)
    m._evaluate = lambda: (0.5, 0.1, 0.4, REASON_SPEED)      # closing faster than the cap
    filt_fault = 0.0
    for k in range(40):
        MON._on_status(m, types.SimpleNamespace(data=[0.0, 0.0, filt_fault]))
        MON._tick(m)
        filt_fault = 1.0 if m._pub.sent[-1][0] >= 1.0 else 0.0   # the filter's verified reaction
    latched_at = next(i for i, r in enumerate(m._pub.sent) if r[0] >= 1.0)
    m._evaluate = lambda: (0.0, float('inf'), 0.0, REASON_NONE)  # the scene clears
    for k in range(3000):                                          # 10 s at 300 Hz
        MON._on_status(m, types.SimpleNamespace(data=[0.0, 0.0, filt_fault]))
        MON._tick(m)
        filt_fault = 1.0 if m._pub.sent[-1][0] >= 1.0 else 0.0
    resp = MON._on_reset(m, None, types.SimpleNamespace(success=None, message=''))
    R.fact(f'8. iso_stop_requires_reset={req}: latched at tick {latched_at + 1}; 10 s after the scene cleared it is '
           f'still latched = {m._latched}; /safety_reset success = {resp.success}',
           m._latched and resp.success is False,
           resp.message[:70])

# 9
K9 = dict(t_r=y['iso_t_reaction'], a_s=y['iso_a_stop'], c=y['iso_c_intrusion'], z_d=y['iso_z_depth'],
          z_r=y['iso_z_robot'])
vmax = 0.68
gaps = np.linspace(K9['c'] + K9['z_d'] + K9['z_r'] + 1e-3, 3.0, 400)
eq = []
for g in gaps:
    cap = ssm_speed_cap(g, 0.0, v_max=vmax, **K9)
    if cap < vmax - 1e-9:
        eq.append(abs(protective_separation(cap, 0.0, **K9) - g))
R.below(f'9. builder diag S_p = protective_separation(cap(d), v_app) − d, over {len(eq)} unsaturated gaps [m]',
        max(eq), 1e-9, note='the "largest S_p this rebuild" is the largest unsaturated gap, not a demand')
d9, v9 = 1.2, 0.2
R.info(f'9. same scene, d = {d9} m, closing at {v9} m/s, v_app = 0: builder S_p = '
       f'{protective_separation(ssm_speed_cap(d9, 0.0, v_max=vmax, **K9), 0.0, **K9):.3f} m, monitor S_p = '
       f'{protective_separation(v9, 0.0, **K9):.3f} m')
SR = C5.src('franka_experiments/franka_experiments/utils/cbf_state_rows.py', 'py')
R.fact('9. builder: sp_i = protective_separation(v_obst, v_app_i, …) with v_obst = the cap',
       re.search(r'sp_i = protective_separation\(\s*v_obst, v_app_i', SR) is not None)
R.fact('9. the SSM speed row bounds ‖Jp q̇‖ along v̂ = Jp q̇/‖Jp q̇‖ (link_speed_row), the monitor trips on −n̂ᵀJp q̇',
       'speed = float(np.linalg.norm(v))' in SR and 'v_closing = max(-float(n @ v_cp), 0.0)' in M)

# 10
m = _monitor(True)
m._q = None
for k in range(trip_ticks(m.P)):
    MON._tick(m)
R.fact(f'10. no joint state yet: latched with reason {m._pub.sent[-1][1]:.0f} (4 = stale) after '
       f'{trip_ticks(m.P)} ticks ({trip_ticks(m.P) / m.P.qp_rate_hz * 1000:.0f} ms)',
       m._latched and m._pub.sent[-1][1] == REASON_STALE)
on_status = M[M.index('def _on_status'):M.index('def _tick')]
R.fact('10. _on_status stores data[2] with no time stamp: the last fault flag is kept forever, and a filter '
       'that stops publishing is invisible to the monitor', 'stamp' not in on_status and '_now' not in on_status)

# 11: the worked example of chapter 11 §7 (shipped constants, no v_max clip)
K11 = dict(t_r=0.10, a_s=1.0, c=0.85, z_d=0.06, z_r=0.01)
R.fact('11. the example uses the shipped YAML constants',
       all(abs(float(y[k]) - v) < 1e-12 for k, v in (('iso_t_reaction', 0.10), ('iso_a_stop', 1.0),
                                                       ('iso_c_intrusion', 0.85), ('iso_z_depth', 0.06),
                                                       ('iso_z_robot', 0.01))))
for lbl, got, exp in (
        ('S_p(v_r = 0.2, v_h = 0) [m]', protective_separation(0.2, 0.0, **K11), 0.96),
        ('S_p(v_r = 0.2, v_h = 2.0) [m]', protective_separation(0.2, 2.0, **K11), 1.56),
        ('cap(d = 1.2, v_h = 0) [m/s]', ssm_speed_cap(1.2, 0.0, v_max=10.0, **K11), np.sqrt(0.57) - 0.1),
        ('cap(d = 1.2, v_h = 2.0) [m/s]', ssm_speed_cap(1.2, 2.0, v_max=10.0, **K11), np.sqrt(4.57) - 2.1),
        ('stopping_distance(0.68) [m]', stopping_distance(0.68, t_r=0.10, a_s=1.0), 0.068 + 0.68 ** 2 / 2)):
    R.close('11. ' + lbl, got, exp, 1e-12, kind='NUM')
sys.exit(R.finish())
