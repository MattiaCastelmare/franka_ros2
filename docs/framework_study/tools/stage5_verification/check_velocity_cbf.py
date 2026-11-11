#!/usr/bin/env python3
"""Pipeline B: the velocity-level CBF filter and its executor (chapter 10 §2).

Production code: `CbfVelocityFilter._solve_cbf_qp` (called unbound on a stub `self`, through the
deployed qpsolvers/OSQP) and `cbf_utils.select_gamma`. Source facts by text match.

  1. (10.3) shared-slack QP: s = (b − aᵀq̇_n)/(1 + ρ‖a‖²), q̇ = q̇_n + λa — the ZCBF row is a
     penalty, like F-4-1                                                                        (NUM)
  2. (10.5) the post-QP per-joint rate clamp (`_control_loop` L559–562, transcribed) can undo the
     QP: with q̇_prev = q̇_n (already approaching at 0.3 m/s) the published q̇ violates aᵀq̇ ≥ −γh
     even though the QP output met it                                                          (FACT)
  3. select_gamma: γ = conf·(2 + 10·e^{−18(d−d_safe)⁺}), conf clipped to [0.5, 1]              (NUM)
  4. `bypass_cbf` launch default is 'false' (QP active) although the launch docstring says Phase 1
     (bypass) is the default; stale/missing distances → clip(q̇_nom) pass-through; QP failure →
     0.5·clip(q̇_nom); the row uses the PERCEPTION direction                                   (FACT)
  5. RtVelocityExecutorController: timeout_threshold_s / timeout_ramp_s read but unused in
     update(); no command watchdog                                                             (FACT)
  6. F-5-16: `rt_velocity_blender.yaml` (0.4 rad/s, 4.0 rad/s²) is loaded only by
     franka_rt_controllers' own launch, not by the velocity stack; `build_wrapper_log_actions`
     (which labels the timeouts [ACTIVE]) is called by no launch file; the commander docstring
     names the executor as its consumer                                                        (FACT)

Needs: the ROS 2 container (qpsolvers, osqp, rclpy importable for the node module). Read-only.
"""
import os
import re
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common5 as C5  # noqa: E402

C, C3 = C5.C, C5.C3
import numpy as np  # noqa: E402
from franka_experiments.nodes.cbf_velocity_filter import CbfVelocityFilter as VF  # noqa: E402
from franka_experiments.utils.cbf_utils import select_gamma  # noqa: E402

R = C.Report('check_velocity_cbf (chapter 10 §2)')
y = C3.yaml_params()
lim = C.load_yaml(os.path.join(C.CONFIG, 'fr3_control.yaml'))['joint_limits']
keys = [f'joint{i}' for i in range(1, 8)]
qd_max = np.array([lim[k][2] for k in keys])
S = C5.stub(model=type('M', (), {'nv': 7})(), params={'rho_slack': y['rho_slack'], 'qp_solver': y['qp_solver']},
            qdot_min=-qd_max, qdot_max=qd_max)
a = np.array([0.0, -0.18, 0.0, 0.47, 0.0, 0.18, 0.0])
d_safe = float(y['d_safe'])
h = 0.02
gam = select_gamma('warning', 1.0, d=d_safe + h, d_safe=d_safe)
qn = -0.3 * a / (a @ a)                    # nominal closes the gap at 0.3 m/s along n̂
b = -gam * h
x = VF._solve_cbf_qp(S, qn.copy(), [(a, b, 'fr3_link8', d_safe + h)])
rho = float(y['rho_slack'])
s_cf = (b - a @ qn) / (1 + rho * (a @ a))
lam = (b - a @ qn) / (a @ a + 1 / rho)
# the identity itself, on the same problem, with an independent tight solver
Pd = np.eye(8); Pd[7, 7] = rho
G = np.concatenate([-a, [-1.0]])[None, :]
xi, _ = C5.C4.independent_qp(Pd, np.concatenate([-qn, [0.0]]), G, np.array([-b]),
                             np.concatenate([-qd_max, [0.0]]), np.concatenate([qd_max, [1e6]]))
R.close('1. (10.3) closed form q̇ = q̇_n + λa (independent tight solve)', xi[:7], qn + lam * a, 1e-8, kind='NUM')
R.close('1. (10.3) residual violation b − aᵀq̇ = (b − aᵀq̇_n)/(1 + ρ‖a‖²) [m/s]', b - a @ xi[:7], s_cf, 1e-9, kind='NUM',
        note=f'γ = {gam:.3f}, h = {h} m: the shared slack makes the ZCBF row a penalty (cf. F-4-1)')
R.below('1. production _solve_cbf_qp (qpsolvers → OSQP, its default tolerance) vs the exact optimum, max |Δq̇|',
        float(np.abs(x - xi[:7]).max()), 5e-3, note='OSQP at ~1e-3 accuracy through qpsolvers')

# 2) post-QP rate clamp (transcription of _control_loop L559-562 with __init__'s qddot_max)
jerk = np.array([7500., 3750., 5000., 6250., 7500., 10000., 10000.])
qdd_max = np.minimum(np.array([lim[k][3] for k in keys]), jerk * 0.001 * 0.80 / 2.0)
dt = 1.0 / float(y['control_rate_hz'])
prev = qn.copy()                           # the arm was already moving at the nominal (approach) velocity
x_ex = xi[:7]                              # the QP's answer (exact optimum)
cmd = prev + np.clip(x_ex - prev, -qdd_max * dt, qdd_max * dt)
R.info(f'2. qddot_max (min of decel limit and jerk·1ms·0.8/2) = {qdd_max.tolist()} rad/s²; dt = {dt} s')
R.fact(f'2. from q̇_prev = q̇_n (approaching at 0.3 m/s): QP output has aᵀq̇ − b = {a @ x_ex - b:+.4f} '
       f'(penalty residue only), the rate-clamped published command has aᵀq̇ − b = {a @ cmd - b:+.4f}: '
       'post-QP processing violates the barrier this tick',
       a @ cmd - b < -0.05)
R.info(f'2. closing speed along n̂: allowed −γh = {b:+.4f} m/s, QP output {a @ x_ex:+.4f} m/s, '
       f'published {a @ cmd:+.4f} m/s (‖a‖ = {np.linalg.norm(a):.3f})')

# 3) select_gamma
for dd, cf in ((0.0, 1.0), (0.05, 1.0), (0.10, 1.0), (0.0, 0.3)):
    g = select_gamma('x', cf, d=d_safe + dd, d_safe=d_safe)
    exp = max(0.5, min(1.0, cf)) * (2.0 + 10.0 * np.exp(-18.0 * dd))
    R.close(f'3. γ(d − d_safe = {dd}, conf = {cf})', g, exp, 1e-12, kind='NUM')

# 4) source facts
L = C5.src('franka_experiments/launch/velocity_cbf_control_stack.launch.py')
R.fact("4. launch: DeclareLaunchArgument('bypass_cbf', default_value='false') — the CBF QP is ACTIVE by default",
       re.search(r"'bypass_cbf',\s*\n\s*default_value='false'", L) is not None)
R.fact("4. launch docstring nevertheless says 'Phase 1  (bypass_cbf:=true, default)'",
       'Phase 1  (bypass_cbf:=true, default)' in L)
F = C5.src('franka_experiments/franka_experiments/nodes/cbf_velocity_filter.py', 'py')
R.fact('4. stale or never-received distances → target = clip(q̇_nom) (pass-through, no braking)',
       re.search(r'if age > self\._dist_timeout:.*?target = np\.clip\(self\.qdot_nom', F, flags=re.S) is not None
       and 'self._last_distance_stamp : float    = 0.0' in F)
R.fact('4. QP failure → 0.5·clip(q̇_nom)', 'return np.clip(qdot_nom, self.qdot_min, self.qdot_max) * 0.5' in F)
R.fact('4. the row normal is the PERCEPTION direction (ld.direction), not pr − ph',
       "'direction': np.array([" in F and "n_world   = item['direction'].copy()" in F)
R.fact("4. joint state from topics.joint_states_topic (the 30 Hz /NS_1/joint_states); no age check",
       "self.topics_ctr['joint_states_topic']" in F and 'js_age' not in F and 'joint_state_timeout' not in F)
cpp = C5.src('franka_rt_controllers/src/rt_velocity_blender_controller.cpp', 'cpp')
upd = cpp[cpp.index('RtVelocityExecutorController::update('):cpp.index('RtVelocityExecutorController::on_init')]
R.fact('5. executor update() never reads timeout_threshold_s_ / timeout_ramp_s_ (no watchdog)',
       'timeout' not in upd and 'timeout_threshold_s_ =' in cpp)
hpp = C5.src('franka_rt_controllers/include/franka_rt_controllers/rt_velocity_blender_controller.hpp')
R.fact('5. header: "reserved – not used in RT path"', hpp.count('reserved – not used in RT path') == 2)
ld = C.load_yaml(os.path.join(C.CONFIG, 'launch_defaults.yaml'))
R.info(f"5. launch_defaults nonetheless set timeout_threshold_s = {ld.get('timeout_threshold_s')}, "
       f"timeout_ramp_s = {ld.get('timeout_ramp_s')}, qdot_max = {ld.get('qdot_max')}, max_accel = {ld.get('max_accel')}")

# 6) F-5-16: stale configuration and diagnostics around the executor
import glob  # noqa: E402
by = C.load_yaml(os.path.join(C.REPO, 'franka_rt_controllers/config/rt_velocity_blender.yaml'))
exe = by['/**']['rt_velocity_executor_controller']['ros__parameters'] if '/**' in by else None
if exe is None:  # multi-document YAML: fall back to a text match
    txt = C5.src('franka_rt_controllers/config/rt_velocity_blender.yaml')
    q04 = re.search(r'qdot_max:\s*0\.4\b', txt) is not None and re.search(r'max_accel:\s*4\.0\b', txt) is not None
else:
    q04 = exe.get('qdot_max') == 0.4 and exe.get('max_accel') == 4.0
users = [f for f in glob.glob(os.path.join(C.REPO, '**', '*.launch.py'), recursive=True)
         if 'rt_velocity_blender.yaml' in open(f).read()]
users = sorted(os.path.relpath(f, C.REPO) for f in users)
R.fact('6. rt_velocity_blender.yaml sets qdot_max 0.4 / max_accel 4.0 and is loaded only by '
       f'franka_rt_controllers/launch/rt_velocity_blender.launch.py: {users}',
       q04 and users == ['franka_rt_controllers/launch/rt_velocity_blender.launch.py'])
callers = [f for f in glob.glob(os.path.join(C.REPO, 'franka_*', '**', '*.launch.py'), recursive=True)
           if 'build_wrapper_log_actions' in open(f).read()]
R.fact('6. build_wrapper_log_actions (prints the timeouts as [ACTIVE]) is called by no launch file',
       callers == [] and "'  [ACTIVE]' if (float(p['timeout_threshold_s']) > 0" in
       C5.src('franka_experiments/franka_experiments/utils/launch_support.py'))
R.fact('6. ee_pentagon_velocity_commander docstring names rt_velocity_blender_controller as the consumer '
       '(the consumer is cbf_velocity_filter via /NS_1/tracking_qdot)',
       'consumed by the **rt_velocity_blender_controller**' in
       C5.src('franka_experiments/franka_experiments/nodes/ee_pentagon_velocity_commander.py'))
sys.exit(R.finish())
