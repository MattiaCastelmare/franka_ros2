"""mpc_commander's control_loop around the QP: where the plan starts and from which q̈.

- The plan starts at t0 = now + input_delay_s (when u_0 reaches the controller), from the
  measured state predicted to t0 with the q̈ in force: q0 = q + τq̇ + ½τ²u, q̇0 = q̇ + τu.
- The q̈ in force is the shield's last output while it is fresh (a CBF downstream), else
  the MPC's own last u; it is also the jerk term's u_prev.
- The task preview is sampled from t0, and input_delay_s must stay below dt.

The real node on its own ROS context (domain 89), publishers replaced by sinks, its clock
driven from here: one control_loop per test, nothing reaches another graph.
"""

import numpy as np
import pytest
import rclpy
from rclpy.clock import ClockType
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.nodes.mpc_commander import MPCCommander
from franka_experiments.utils.constants import FR3_JOINT_NAMES
from franka_experiments.utils.human_utils import fill_ee_trajectory_msg, make_ee_trajectory_msg

Q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.578, 0.785])
QD = np.array([0.2, -0.1, 0.05, 0.3, -0.2, 0.1, 0.0])
U_OWN = np.array([1.0, -0.5, 0.2, 0.0, 0.3, -0.4, 0.1])
U_SAFE = np.array([-2.0, 0.5, 0.0, 1.0, -0.3, 0.2, 0.0])
NOW = 100.0


class Clock:
    def __init__(self, t):
        self.t = t

    def now(self):
        return Time(nanoseconds=int(round(self.t * 1e9)), clock_type=ClockType.ROS_TIME)


class Sink:
    def publish(self, msg):
        pass


@pytest.fixture(scope='module')
def context():
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=89)
    yield ctx
    rclpy.shutdown(context=ctx)


def make_node(context, **params):
    node = MPCCommander(context=context,
                        parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    node.destroy_timer(node.timer)
    node.pub = node.sp_pub = node.plan_pub = node.diag_pub = Sink()
    clock = Clock(NOW)
    node.get_clock = lambda: clock
    return node, clock


def one_cycle(node, clock, safe_age=None):
    """A fresh joint state, a fixed task preview at the current TCP and, if safe_age is not
    None, a shield output received safe_age seconds ago; then one control_loop."""
    js = JointState(name=list(FR3_JOINT_NAMES), position=Q.tolist(), velocity=QD.tolist())
    node.latest_joint_state = (js, clock.now())
    preview = make_ee_trajectory_msg(node.ee_frame, 40, 0.02)
    fill_ee_trajectory_msg(preview, clock.now().to_msg(), np.tile(node.tcp_position(Q), (40, 1)))
    node.latest_task_trajectory = (preview, clock.now())
    if safe_age is not None:
        received = Clock(clock.t - safe_age).now()
        node.latest_qddot_safe = (Float64MultiArray(data=U_SAFE.tolist()), received)
    node.u_prev = U_OWN.copy()
    node.control_loop()


@pytest.mark.parametrize('delay', [0.0, 0.018])
def test_the_plan_starts_from_the_state_predicted_to_t0(context, delay):
    node, clock = make_node(context, input_delay_s=delay)
    one_cycle(node, clock)
    p = node.problem
    assert np.allclose(p.q0, Q + delay * QD + 0.5 * delay ** 2 * U_OWN)
    assert np.allclose(p.qd0, QD + delay * U_OWN)
    assert np.allclose(p.u_prev, U_OWN) and p.diag.solved
    node.destroy_node()


@pytest.mark.parametrize('safe_age, in_force', [(0.002, U_SAFE), (0.2, U_OWN), (None, U_OWN)])
def test_the_q_ddot_in_force_is_the_shields_while_fresh(context, safe_age, in_force):
    node, clock = make_node(context, input_delay_s=0.018)
    one_cycle(node, clock, safe_age)
    p = node.problem
    assert np.allclose(p.u_prev, in_force)
    assert np.allclose(p.qd0, QD + 0.018 * in_force)
    node.destroy_node()


def test_without_a_shield_topic_the_q_ddot_is_always_ours(context):
    node, clock = make_node(context, qddot_safe_topic='')
    assert not hasattr(node, 'qddot_safe_sub')
    one_cycle(node, clock)
    assert np.allclose(node.problem.u_prev, U_OWN)
    node.destroy_node()


def test_the_preview_is_sampled_from_t0(context):
    node, clock = make_node(context, input_delay_s=0.018)
    asked = []
    sample = node.sample_task_reference
    node.sample_task_reference = lambda msg, t: asked.append(t) or sample(msg, t)
    one_cycle(node, clock)
    assert np.allclose(asked[0], NOW + 0.018 + node.dt * np.arange(1, node.problem.n_steps + 1))
    node.destroy_node()


def test_an_input_delay_of_a_whole_step_is_rejected(context):
    with pytest.raises(ValueError, match='input_delay_s'):
        MPCCommander(context=context, parameter_overrides=[Parameter('input_delay_s', value=0.05)])