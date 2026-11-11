#!/usr/bin/env python3
"""Source-level facts about the torque realisation chain, plus a transcribed 1 kHz loop (chapter 09 §3–§5).

The C++ controllers cannot be imported, so (1)–(7) are FACT checks on the source text (regular
expressions over comment-stripped C++/Python) and (8) is a TRANSCRIPTION of
`RtTorqueController::update` (L54–202) for one joint, labelled as such.

  1. RtTorqueController writes τ = ffScale·τ_ff + K_p p + K_d e and nothing else: no clamp of the
     effort command, no low-pass filter, no gravity term                                     (FACT)
  2. its plugin description (rt_velocity_blender_controller.xml) says it "applies an optional
     low-pass filter, clips to per-joint limits"                                               (FACT)
  3. `lpf_alpha`, `tau_max_scale`, `urdf_path` are written to its YAML but never declared (F-1-8) (FACT)
  4. the generator writes zero P/D gains and update_rate 100 when is_real is false; the Gazebo stack
     calls it with is_real=True, gazebo='true'                                                  (FACT)
  5. Gazebo plugin `IgnitionSystem::write` adds the KDL gravity torque to every effort command (FACT)
  6. CBFTorqueController: while /q_des_state is fresh, τ = RNEA(q, q̇, sp.qddot) − g with
     sp.qddot = msg.effort; /qddot_safe is used only when /q_des_state is stale               (FACT)
  7. pentagon_qddot_commander publishes ITS OWN q̈ (`_q_ddot`) in /q_des_state.effort            (FACT)
  8. transcribed loop, joint 7 at q_home with the Gazebo armature: velocity error after 0.3 s of a
     constant q̈_safe with and without the PD terms                                (illustrative)

Needs: Python 3, numpy. Host. Read-only.
"""
import os
import re
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common4 as C4  # noqa: E402

C = C4.C
import numpy as np  # noqa: E402

R = C.Report('check_controller_sources (chapter 09 §3-§5)')
REPO = C.REPO


def cpp(path):
    s = open(os.path.join(REPO, path)).read()
    s = re.sub(r'/\*.*?\*/', '', s, flags=re.S)
    return re.sub(r'//[^\n]*', '', s)


rt = cpp('franka_rt_controllers/src/rt_torque_controller.cpp')
upd = rt[rt.index('RtTorqueController::update('):rt.index('RtTorqueController::qdotCeiling')]
sets = re.findall(r'command_interfaces_\[i\]\.set_value\((.*?)\);', upd, flags=re.S)
sets = [' '.join(x.split()) for x in sets]
R.info('1. effort writes in update(): ' + ' | '.join(sets))
R.fact('1. the main write is ffScale(i, τ_ff)·τ_ff + p_gains·p + d_gains·e',
       any(re.fullmatch(r'ffScale\(i, input\.tau\[i\]\) \* input\.tau\[i\] \+ p_gains_\[i\] \* p \+ d_gains_\[i\] \* e', x)
           for x in sets))
R.fact('1. no clamp / saturation / filter / gravity applied to the effort command in update()',
       not re.search(r'clamp|effort_max|tau_max|lpf|alpha|gravity', upd))
xml = ' '.join(open(os.path.join(REPO, 'franka_rt_controllers/rt_velocity_blender_controller.xml')).read().split())
R.fact('2. plugin XML claims an optional low-pass filter and per-joint clipping',
       'low-pass filter' in xml and 'clips to per-joint limits' in xml)
decl = set(re.findall(r'auto_declare<[^>]+>\(\s*"(\w+)"', rt))
R.fact('3. lpf_alpha, tau_max_scale, urdf_path not declared by the controller',
       not ({'lpf_alpha', 'tau_max_scale', 'urdf_path'} & decl), f'declared: {sorted(decl)}')
ls = open(os.path.join(C.PKG_SRC, 'franka_experiments/utils/launch_support.py')).read()
gen = ls[ls.index('def _generate_torque_yaml'):ls.index('def write_temp_yaml')]
gen = '\n'.join(x for x in gen.splitlines() if not x.strip().startswith('#'))
R.fact('4. not is_real ⇒ d_gains = p_gains = 0; update_rate 1000 if is_real else 100',
       re.search(r'if not is_real:\s*\n\s*d_gains = \[0\.0\] \* 7\s*\n\s*p_gains = \[0\.0\] \* 7', gen) is not None
       and "update_rate: {1000 if is_real else 100}" in gen)
sc = open(os.path.join(REPO, 'franka_simulation_experiments/scripts/simexp/sim_config.py')).read()
R.fact("4. Gazebo: rt_controllers_yaml calls the generator with is_real=True, gazebo='true'",
       re.search(r"generate_rt_controllers_yaml\(\s*is_real=True.*?gazebo='true'", sc, flags=re.S) is not None)
ign = cpp('franka_gazebo/franka_ign_ros2_control/src/ign_system.cpp')
w = ign[ign.index('IgnitionSystem::write('):]
R.fact('5. Gazebo plugin: joint_efforts = kdl_model_.gravity(q, g) added to JointForceCmd in EFFORT mode',
       'kdl_model_.gravity(q, gravity_earth)' in w and 'jointEffortCmd->Data()[0] += joint_efforts[i]' in w)
cb = cpp('franka_rt_controllers/src/cbf_torque_controller.cpp')
R.fact('6. CBFTorqueController reads the setpoint acceleration from JointState.effort',
       'sp.qddot(i) = msg->effort[idx]' in cb)
u = cb[cb.index('CBFTorqueController::update('):]
i_sp, i_q, i_hold = u.index('if (!sp_stale)'), u.index('else if (!qddot_stale)'), u.rindex('} else {')
R.fact('6. update(): the sp-fresh branch uses sp.qddot; /qddot_safe (qddot_buf_) only in the next branch',
       'qddot_pin_[arm_v_idx_[i]] = sp.qddot(i)' in u[i_sp:i_q] and 'qddot_buf_' not in u[i_sp:i_q]
       and 'qddot_buf_.readFromRT()' in u[i_q:i_hold])
cm = open(os.path.join(C.PKG_SRC, 'franka_experiments/nodes/pentagon_qddot_commander.py')).read()
R.fact('7. the commander publishes its own q̈ (_q_ddot, the NOMINAL) in /q_des_state.effort',
       'self._sp_msg.effort[i]   = float(self._q_ddot[i])' in cm
       and "self.declare_parameter('q_des_topic',       '/NS_1/q_des_state')" in cm)


# 8) transcription of RtTorqueController::update for one joint
def loop(m_model, m_true, kp, kd, qdd_cmd=1.0, T=0.3, e_max=1.0, p_max=0.15):
    dt_c, dt_q = 1e-3, 1.0 / 300.0
    q = qd = q_des = qd_des = 0.0
    tau_ff, t_next = 0.0, 0.0
    for k in range(int(T / dt_c)):
        t = k * dt_c
        if t >= t_next - 1e-12:                    # qddot_to_torque runs per /qddot_safe message (300 Hz)
            tau_ff = m_model * qdd_cmd             # zero-order hold of τ_ff between messages
            t_next += dt_q
        qd_des += qdd_cmd * dt_c                   # q̇_des integrates the latest q̈_safe (also held)
        e = qd_des - qd
        if abs(e) > e_max:
            e = np.copysign(e_max, e)
            qd_des = qd + e
        q_des += qd_des * dt_c
        p = q_des - q
        if abs(p) > p_max:
            p = np.copysign(p_max, p)
            q_des = q + p
        tau = tau_ff + kp * p + kd * e             # ffScale = 1 away from the velocity envelope
        qd += tau / m_true * dt_c                  # plant: m_true q̈ = τ (gravity cancelled exactly)
        q += qd * dt_c
    return qd, qdd_cmd * T


m7, arm7 = 0.0023, 0.074                         # joint 7 at q_home: M77 (model) and ARMATURE[6]
for kp, kd, tag in ((0.0, 0.0, 'feedforward only'), (20.0, 5.0, 'shipped p_gains[6]=20, d_gains[6]=5')):
    v, v_ref = loop(m7, m7 + arm7, kp, kd)
    R.info(f'8. joint 7, {tag}: realised q̇ after 0.3 s = {v:.4f} rad/s vs ∫q̈_safe = {v_ref:.4f} '
           f'({100 * v / v_ref:.0f} %)')
v0, vr = loop(m7, m7 + arm7, 0.0, 0.0)
v1, _ = loop(m7, m7 + arm7, 20.0, 5.0)
R.fact('8. the PD on the integrated reference recovers most of the armature gap (transcribed, illustrative)',
       v1 / vr > 0.8 and v0 / vr < 0.1, f'{v0 / vr:.2f} → {v1 / vr:.2f}')
sys.exit(R.finish())
