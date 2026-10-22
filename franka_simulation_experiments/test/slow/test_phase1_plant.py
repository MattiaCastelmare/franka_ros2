"""Phase 1: the FR3 + RtTorqueController in Gazebo (plan §8, steps 1.3-1.4).

SLOW: one Gazebo for the module, ~1.5 min. ``python3 -m pytest test -m slow -k phase1``

1. tau_cmd = 0 hold, 10 s at Q0 (H3). The shipped RtTorqueController runs and
   receives tau_ff = 0 with no q̈_safe, so it commands exactly 0
   (rt_torque_controller.cpp: accel not ok -> e = p = 0). The plant then gets
   0 + g_KDL(q) from the gz_ros2_control plugin.

   Bound: |q(t) - q(t0)| < 1e-3 rad per joint over 10 s. Why 1 mrad:
     * if H3 were false (no gravity added), joint 4 would carry its full
       22.5 N·m gravity torque at Q0 and fall by tenths of a radian within the
       first second. Anything below ~10 mrad already refutes that;
     * if H3 holds, the lumped model and the KDL chain have the same g(q) to 1e-9
       (test/sim/test_gz_robot_description.py), and URDF Coulomb friction
       (0.2 N·m per joint) holds any numerical residual. So the expected drift
       is zero, and 1 mrad (0.85 mm at the flange, below one depth pixel at
       1.8 m) leaves room only for integrator noise.
2. A small q̈ step on /NS_1/qddot_safe, converted by the SHIPPED qddot_to_torque
   and executed by the SHIPPED RtTorqueController, is tracked.
"""
import json
import math
import os
import signal
import subprocess
import time

import numpy as np
import pytest

pytestmark = pytest.mark.slow

from _rosprobe import ARM, Probe, Spinner, init_ros, wait_sim, wait_until  # noqa: E402

Q0 = np.array([0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70])
RESULTS = {}


def _simexp():
    import sys
    from ament_index_python.packages import get_package_prefix
    lib = os.path.join(get_package_prefix('franka_simulation_experiments'), 'lib',
                       'franka_simulation_experiments')
    sys.path.insert(0, lib)
    from simexp import procs, sim_config
    return procs, sim_config


@pytest.fixture(scope='module')
def plant():
    procs, sc = _simexp()
    init_ros()
    run_dir = sc.make_run_dir(name='phase1_plant')
    L = procs.Launch(['franka_simulation_experiments', 'gz_plant.launch.py',
                      'spawn_rt_controller:=true', f'run_dir:={run_dir}'],
                     os.path.join(run_dir, 'launch.log'))
    L.start()
    probe = Probe()
    with Spinner(probe):
        try:
            assert wait_until(lambda: probe.sim_now() > 6.0, 120), \
                'no /NS_1/franka/joint_states from Gazebo within 120 s'
            yield dict(run_dir=run_dir, probe=probe, launch=L)
        finally:
            L.stop()
            left = procs.leftovers(L.partition)
            with open(os.path.join(run_dir, 'phase1_results.json'), 'w') as f:
                json.dump(RESULTS, f, indent=1, default=float)
            assert not left, f'processes survived the teardown: {left}'


def _pub_node(name, use_sim_time):
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    return Node(name, parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])


def test_controller_runs_on_sim_time(plant):
    """H6: the CM copies use_sim_time into the controller; otherwise its
    staleness checks (now() vs update(time)) would mix wall and sim clocks."""
    from rcl_interfaces.srv import GetParameters
    n = _pub_node('sim_test_param_client', False)
    cli = n.create_client(GetParameters, '/NS_1/rt_torque_controller/get_parameters')
    import rclpy
    assert cli.wait_for_service(timeout_sec=20)
    fut = cli.call_async(GetParameters.Request(names=['use_sim_time', 'gazebo']))
    rclpy.spin_until_future_complete(n, fut, timeout_sec=20)
    vals = [v.bool_value for v in fut.result().values]
    RESULTS['rt_torque_controller'] = {'use_sim_time': vals[0], 'gazebo': vals[1]}
    assert vals == [True, True]
    n.destroy_node()


def test_tau_zero_hold_at_q0_drifts_less_than_1_mrad(plant):
    from std_msgs.msg import Float64MultiArray
    probe = plant['probe']
    n = _pub_node('sim_test_tau_zero', False)
    pub = n.create_publisher(Float64MultiArray, '/NS_1/torque_cmd', 10)
    msg = Float64MultiArray(data=[0.0] * 7)
    n.create_timer(1.0 / 300.0, lambda: pub.publish(msg))
    with Spinner(n):
        wait_sim(probe, 1.0)                       # tau_cmd fresh, transients gone
        t0 = probe.sim_now()
        wait_sim(probe, 10.0)
        t, q, qd, eff = probe.snapshot(t0, t0 + 10.0)
    n.destroy_node()
    assert t[-1] - t[0] > 9.9 and len(t) > 9000, (len(t), t[-1] - t[0])
    drift = np.abs(q - q[0]).max(axis=0)
    RESULTS['hold'] = dict(samples=len(t), duration_s=t[-1] - t[0],
                           drift_rad=drift.tolist(), q_start_minus_Q0=(q[0] - Q0).tolist(),
                           qdot_abs_max=np.abs(qd).max(axis=0).tolist(),
                           effort_mean=eff.mean(axis=0).tolist(), q_mean=q.mean(axis=0).tolist())
    assert drift.max() < 1e-3, f'drift per joint [rad]: {drift}'


def test_effort_field_is_the_applied_torque(plant):
    """H18: what JSB reports as effort in Gazebo. With tau_cmd = 0 and the arm
    still, the applied torque is g_KDL(q). The check uses Pinocchio's g(q) on
    the same (lumped) URDF at g = 9.8."""
    import pinocchio as pin
    h = RESULTS.get('hold')
    if h is None:
        pytest.skip('needs the hold test')
    with open(os.path.join(plant['run_dir'], 'fr3_rsp.urdf')) as f:
        m = pin.buildModelFromXML(f.read())
    m.gravity.linear[:] = [0.0, 0.0, -9.8]
    g = pin.computeGeneralizedGravity(m, m.createData(), np.array(h['q_mean']))
    res = np.array(h['effort_mean']) - g
    RESULTS['effort_minus_g'] = res.tolist()
    RESULTS['g_pinocchio'] = g.tolist()
    assert np.abs(res).max() < 0.25, f'effort - g(q) [N·m]: {res}'


def test_qddot_step_through_qddot_safe_is_tracked(plant):
    from std_msgs.msg import Float64MultiArray
    probe, run_dir = plant['probe'], plant['run_dir']
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    q2t = subprocess.Popen(
        ['ros2', 'run', 'franka_experiments', 'qddot_to_torque', '--ros-args',
         '-p', 'use_sim_time:=true'],
        stdout=open(os.path.join(run_dir, 'qddot_to_torque.log'), 'w'),
        stderr=subprocess.STDOUT, env=env, start_new_session=True)
    try:
        # (joint index, start, duration, acceleration): accelerate, decelerate, stop.
        # Joint 7 is the lightest: it is where DART stick-slip once zeroed the
        # reported velocity (plan H19), so it is in the profile on purpose.
        segs = [(0, 0.5, 0.5, +1.0), (0, 1.0, 0.5, -1.0),
                (5, 2.0, 0.5, +2.0), (5, 2.5, 0.5, -2.0),
                (6, 3.5, 0.4, +3.0), (6, 3.9, 0.4, -3.0)]
        T_END = 5.5
        n = _pub_node('sim_test_qddot_step', True)
        pub = n.create_publisher(Float64MultiArray, '/NS_1/qddot_safe', 10)
        sent = []
        state = {'t0': None}

        def tick():
            now = n.get_clock().now().nanoseconds * 1e-9
            if state['t0'] is None:
                state['t0'] = now
            tau = now - state['t0']
            a = np.zeros(7)
            for j, s0, d, acc in segs:
                if s0 <= tau < s0 + d:
                    a[j] = acc
            pub.publish(Float64MultiArray(data=a.tolist()))
            sent.append((now, a))
        with Spinner(n):
            time.sleep(3.0)                        # qddot_to_torque up (Pinocchio load)
            n.create_timer(1.0 / 300.0, tick)
            assert wait_until(lambda: state['t0'] is not None, 30)
            assert wait_until(lambda: probe.sim_now() > state['t0'] + T_END, 120)
        n.destroy_node()
    finally:
        os.killpg(q2t.pid, signal.SIGINT)
        try:
            q2t.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(q2t.pid, signal.SIGKILL)
    t0 = state['t0']
    t, q, qd, _ = probe.snapshot(t0, t0 + T_END)
    ts = np.array([s[0] for s in sent])
    A = np.stack([s[1] for s in sent])
    # Reference velocity: the integral of what was actually published, zero-order hold.
    dts = np.diff(np.r_[ts, ts[-1]])
    v_ref_at_pub = np.r_[np.zeros((1, 7)), np.cumsum(A[:-1] * dts[:-1, None], axis=0)]
    v_ref = np.stack([np.interp(t, ts, v_ref_at_pub[:, j]) for j in range(7)], axis=1)
    v0 = qd[0]
    err = (qd - v0) - v_ref
    out = {}
    MOVED = (0, 5, 6)
    # Velocity integrity: the reported q̇ must be the derivative of q (H19).
    dq_num = np.gradient(q, t, axis=0)
    for j in MOVED:
        rel = float(np.sqrt(np.mean((qd[:, j] - dq_num[:, j]) ** 2))
                    / np.sqrt(np.mean(dq_num[:, j] ** 2)))
        out.setdefault('velocity_rel_err', {})[ARM[j]] = rel
    for j in MOVED:
        peak = np.abs(v_ref[:, j]).max()
        rms_rel = float(np.sqrt(np.mean(err[:, j] ** 2)) / peak)
        peak_ratio = float(np.abs(qd[:, j] - v0[j]).max() / peak)
        out[ARM[j]] = dict(peak_ref=peak, rms_rel=rms_rel, peak_ratio=peak_ratio,
                           dq_final=float(q[-1, j] - q[0, j]),
                           dq_ref=float(np.trapz(v_ref[:, j], t)))
    others = [j for j in range(7) if j not in MOVED]
    out['others_qdot_abs_max'] = float(np.abs(qd[:, others] - v0[others]).max())
    RESULTS['step'] = out
    for j in MOVED:
        assert out['velocity_rel_err'][ARM[j]] < 0.05, out
    for j in MOVED:
        assert out[ARM[j]]['rms_rel'] < 0.10, out
        assert 0.90 < out[ARM[j]]['peak_ratio'] < 1.10, out
        assert abs(out[ARM[j]]['dq_final'] - out[ARM[j]]['dq_ref']) < 0.1 * abs(out[ARM[j]]['dq_ref']), out
    assert out['others_qdot_abs_max'] < 0.05, out
