"""launch_testing: the Gazebo graph reproduces the Phase 0 contract on the safety path.

SLOW (Gazebo, ~1 min). Not collected by pytest; run it with launch_test:

    ROS_DOMAIN_ID=77 launch_test \\
        src/franka_simulation_experiments/test/launch/launch_test_graph_contract.py

Starts gz_cbf_stack (condition kf, pentagon, perception on) and captures the
graph with the same code as the Phase 0 capture. Then it diffs the result
against docs/sim_reference_contract/fake_hw on the safety-path nodes and topics
(simexp.graph_contract): types, endpoint reliability and durability, and the TF
tree. Rates are checked separately, against the HARDWARE numbers: the reference
was fake hardware at 100 Hz, the robot runs at 1 kHz. They are counted over a
SIM-time window: every node runs on sim time, so a wall-time count scales with
the RTF of the host instead of measuring the stack (it read 196 Hz for the QP at
RTF ~0.9 on a loaded host, the same stack that reads 298 Hz at RTF 1).

The filter is pinned to an isolated core (``filter_cpu:=2``), as the robot's RT
loop is (rt_pin_cpu): the check is about the stack, not about the host's other
tenants. Measured on this host at load ~20, in one running stack: 245 Hz of
sim time unpinned, 300 Hz pinned. Core 2, not 3: 3 is the robot's RT core.
"""
import json
import os
import sys
import time
import unittest

import launch_testing
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

_LIB = os.path.join(get_package_prefix('franka_simulation_experiments'), 'lib',
                    'franka_simulation_experiments')
sys.path.insert(0, _LIB)
from simexp import graph_contract, procs, sim_config  # noqa: E402

RUN_DIR = sim_config.make_run_dir(name='graph_contract')
PARTITION = f'contract_{os.getpid()}'


@pytest.mark.launch_test
def generate_test_description():
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('franka_simulation_experiments'), 'launch',
            'gz_cbf_stack.launch.py')),
        launch_arguments={'condition': 'kf', 'motion': 'pentagon', 'perception': 'true',
                          'run_dir': RUN_DIR, 'record_bag': 'false',
                          'filter_cpu': '2'}.items())
    return LaunchDescription([
        SetEnvironmentVariable('IGN_PARTITION', PARTITION),
        stack,
        launch_testing.actions.ReadyToTest(),
    ])


class TestGraphContract(unittest.TestCase):

    def test_safety_path_matches_the_reference(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        rclpy.init()
        node = Node('_contract_capture', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        try:
            # Everything up: the commander starts at control_spawner_delay_s + 2.
            t_end = time.time() + 120
            while time.time() < t_end:
                rclpy.spin_once(node, timeout_sec=0.2)
                names = {('/' + n if ns == '/' else ns + '/' + n)
                         for n, ns in node.get_node_names_and_namespaces()}
                if {'/NS_1/pentagon_qddot_commander', '/experiment_logger',
                        '/NS_1/rt_torque_controller'} <= names:
                    break
            time.sleep(5.0)
            sim_g = graph_contract.capture_graph(node)
            sim_tf = graph_contract.capture_tf(node)
            rates, rtf = _rates(node, {'/NS_1/franka/joint_states': 1000.0,
                                       '/NS_1/qddot_safe': 300.0, '/NS_1/qddot_nom': 100.0,
                                       '/NS_1/joint_states': 30.0})
        finally:
            node.destroy_node()
            rclpy.shutdown()
        ref_g, ref_tf = graph_contract.load_reference()
        problems = graph_contract.diff(ref_g, sim_g, ref_tf, sim_tf)
        with open(os.path.join(RUN_DIR, 'graph_contract_result.json'), 'w') as f:
            json.dump({'problems': problems, 'rates_hz': rates, 'rtf': rtf}, f, indent=1)
        with open(os.path.join(RUN_DIR, 'graph_sim.json'), 'w') as f:
            json.dump(sim_g, f, indent=1, sort_keys=True)
        with open(os.path.join(RUN_DIR, 'tf_sim.json'), 'w') as f:
            json.dump(sim_tf, f, indent=1, sort_keys=True)
        self.assertEqual(problems, [], '\n'.join(problems))
        for topic, (measured, nominal) in rates.items():
            self.assertGreater(measured, 0.85 * nominal, f'{topic}: {measured:.1f} Hz')


def _rates(node, nominal: dict, window_s: float = 4.0):
    """({topic: (rate per SIM second, nominal)}, RTF of the window)."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    counts = {t: 0 for t in nominal}
    subs = []
    for t in nominal:
        typ = JointState if 'joint_states' in t else Float64MultiArray

        def cb(_m, t=t):
            counts[t] += 1
        subs.append(node.create_subscription(typ, t, cb, qos_profile_sensor_data))
    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9
    wall_end = time.time() + 30.0
    while sim_now() <= 0.0 and time.time() < wall_end:        # /clock received
        rclpy.spin_once(node, timeout_sec=0.01)
    for t in counts:
        counts[t] = 0
    t0, w0 = sim_now(), time.time()
    while sim_now() - t0 < window_s and time.time() < wall_end + 60.0:
        rclpy.spin_once(node, timeout_sec=0.01)
    dt, dw = sim_now() - t0, time.time() - w0
    return {t: (counts[t] / dt, nominal[t]) for t in nominal}, dt / dw


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):

    def test_no_gazebo_survives(self):
        time.sleep(3.0)
        left = procs.leftovers(PARTITION)
        for pid in left:
            try:
                os.kill(pid, 9)
            except ProcessLookupError:
                pass
        self.assertEqual(left, [], f'processes left running: {left}')
