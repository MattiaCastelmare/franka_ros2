"""The node that runs a student controller against the Gazebo FR3.

At every ``/joint_states`` message (1 kHz, simulation time) it:

1. SETTLE  for ``settle_time`` s brings the arm to ``initial_q`` of the task YAML
           (it sags a little while the controllers start) and holds it there;
2. RUN     calls ``Controller.update(t, q, dq)`` at ``control_rate`` Hz, clips the
           torque to the FR3 limits, adds the virtual wall's ``J_c^T F`` and
           publishes on ``/fr3_effort_controller/commands``;
3. DONE    after ``duration`` s, when the controller sets ``done``, or on a
           safety stop (joint limit, speed limit, exception): saves the log,
           holds the robot with joint damping, and exits if ``exit_on_done``.

Gazebo adds g(q) to whatever is published (as the real robot does).
Optional realism: Gaussian noise on the q, dq the controller sees (``noise_q``,
``noise_dq``); the log and the wall always use the true state. The runner also
warns when the hand body (not only the fingertip) goes into the wall.
The log is one ``.npz`` per run in ``log_dir``; see :mod:`fr3_student_sim.plot_run`.
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys
import time
import traceback
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from fr3_student_sim.body_check import body_depth
from fr3_student_sim.controller_base import StudentController
from fr3_student_sim.fr3_model import DQ_MAX, Q_MAX, Q_MIN, TAU_MAX, FR3Model
from fr3_student_sim.robot_description import ARM_JOINTS
from fr3_student_sim.wall import Wall

HOLD_KP = np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0])
HOLD_KD = np.array([50.0, 50.0, 50.0, 30.0, 25.0, 25.0, 15.0])
STOP_KD = np.array([30.0, 30.0, 30.0, 20.0, 10.0, 10.0, 5.0])
Q_MARGIN = 0.05        # [rad] safety stop this close to a joint limit
#: A torque published after joint-state sample k acts on the plant during
#: (k+1, k+2], one sample later than one would assume: measured with the
#: momentum observer (residual 0.11 instead of 0.49 N m without contact).
LATENCY_SAMPLES = 1


def load_controller_class(path: str):
    # so that the controller can import helper files placed next to it
    folder = os.path.dirname(os.path.abspath(path))
    if folder not in sys.path:
        sys.path.insert(0, folder)
    spec = importlib.util.spec_from_file_location('student_controller', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    cls = getattr(module, 'Controller', None)
    if cls is None or not issubclass(cls, StudentController):
        raise TypeError(f'{path} must define class Controller(StudentController)')
    return cls


def _load_yaml(path: str) -> dict:
    if not path:
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


class Runner(Node):

    def __init__(self):
        super().__init__('student_runner')
        P = {
            'urdf_path': '', 'controller': '', 'task_config': '', 'wall_config': '',
            'duration': 20.0, 'control_rate': 1000.0, 'settle_time': 2.0,
            'log_dir': 'runs', 'run_name': '', 'exit_on_done': True, 'wall_enabled': True,
            'stop_at_detachment': False, 'noise_q': 0.0, 'noise_dq': 0.0, 'seed': 0,
            'mass_error': 0.0, 'joint_friction': False,
        }
        for k, v in P.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        with open(g('urdf_path')) as f:
            self.robot = FR3Model(f.read())
        self.params = _load_yaml(g('task_config'))
        self.wall = Wall.from_dict(_load_yaml(g('wall_config')))
        self.wall.enabled = bool(g('wall_enabled'))
        self.duration = float(g('duration'))
        self.dt_ctrl = 1.0 / float(g('control_rate'))
        self.settle_time = float(g('settle_time'))
        self.exit_on_done = bool(g('exit_on_done'))
        self.stop_at_detachment = bool(g('stop_at_detachment'))
        self.had_contact = False
        self.noise_q, self.noise_dq = float(g('noise_q')), float(g('noise_dq'))
        self.rng = np.random.default_rng(int(g('seed')))
        self.realism = {k: g(k) for k in ('noise_q', 'noise_dq', 'seed', 'mass_error',
                                          'joint_friction')}
        self.hand_depth, self.hand_warned = 0.0, False
        self.controller_path = g('controller')
        name = g('run_name') or os.path.splitext(os.path.basename(self.controller_path))[0]
        os.makedirs(g('log_dir'), exist_ok=True)
        self.log_path = os.path.join(
            g('log_dir'), f'{name}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.npz')

        cls = load_controller_class(self.controller_path)
        self.ctrl = cls(self.robot, self.params, self.dt_ctrl)

        self.pub = self.create_publisher(Float64MultiArray, '/fr3_effort_controller/commands', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self._on_js,
                                            QoSProfile(depth=1))
        self.phase = 'WAIT'
        self.t0_sim = None
        self.t_last_ctrl = -np.inf
        self.tau_ctrl = np.zeros(7)
        self.pub_hist = deque([np.zeros(7)] * 64, maxlen=64)   # student torque per sample
        self.n_since_ctrl = 0
        self.tau_raw = np.zeros(7)
        self.ctrl_ms = 0.0
        self.rows = []
        self.user_rows = []
        self.stop_reason = ''
        self.wall_t0 = None
        self.get_logger().info(
            f'controller={self.controller_path}  duration={self.duration}s  '
            f'rate={1 / self.dt_ctrl:.0f}Hz  wall={"on" if self.wall.enabled else "off"}  '
            f'realism={self.realism}  log={self.log_path}')

    # ── main loop ─────────────────────────────────────────────────────────────
    def _on_js(self, msg: JointState):
        try:
            idx = [msg.name.index(j) for j in ARM_JOINTS]
        except ValueError:
            return
        q = np.array([msg.position[i] for i in idx])
        dq = np.array([msg.velocity[i] for i in idx])
        t_sim = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

        if self.phase == 'WAIT':
            self.phase, self.t_settle0, self.q_found = 'SETTLE', t_sim, q.copy()
            self.q_goal = np.asarray(self.params.get('initial_q', q), float)
            self.get_logger().info('robot found, moving to initial_q')
        if self.phase == 'SETTLE':
            # smooth move to q_goal in the first 3/4 of the settle time, then hold
            u = min((t_sim - self.t_settle0) / (0.75 * self.settle_time), 1.0)
            q_ref = self.q_found + (3 * u ** 2 - 2 * u ** 3) * (self.q_goal - self.q_found)
            tau_hold = HOLD_KP * (q_ref - q) - HOLD_KD * dq
            self._publish(tau_hold)
            self.pub_hist.append(tau_hold)
            if t_sim - self.t_settle0 >= self.settle_time:
                self.phase, self.t0_sim, self.wall_t0 = 'RUN', t_sim, time.monotonic()
                self.get_logger().info(
                    f'RUN started, |q - initial_q| = {np.max(np.abs(q - self.q_goal)):.4f} rad')
            return
        if self.phase == 'DONE':
            self._publish(-STOP_KD * dq)
            return

        t = t_sim - self.t0_sim
        reason = self._safety(q, dq)
        self.n_since_ctrl += 1
        if not reason and t >= self.t_last_ctrl + self.dt_ctrl - 1e-6:
            self.t_last_ctrl = t
            # what acted on the robot over the samples since the last update
            n = min(self.n_since_ctrl, len(self.pub_hist) - 1 - LATENCY_SAMPLES)
            hist = list(self.pub_hist)
            self.ctrl.tau_applied = np.mean(
                hist[len(hist) - LATENCY_SAMPLES - n:len(hist) - LATENCY_SAMPLES], axis=0)
            self.n_since_ctrl = 0
            try:
                c0 = time.perf_counter()
                q_m = q + self.noise_q * self.rng.standard_normal(7)       # what a sensor gives
                dq_m = dq + self.noise_dq * self.rng.standard_normal(7)
                tau = np.asarray(self.ctrl.update(t, q_m, dq_m), float).reshape(7)
                self.ctrl_ms = 1e3 * (time.perf_counter() - c0)
                if not np.all(np.isfinite(tau)):
                    raise ValueError(f'non-finite torque {tau}')
            except Exception:  # the student's code: report and stop safely
                self.get_logger().error(traceback.format_exc())
                tau, reason = np.zeros(7), 'exception in Controller.update'
            self.tau_raw = tau
            self.tau_ctrl = np.clip(tau, -TAU_MAX, TAU_MAX)
            self.user_rows.append((len(self.rows), dict(self.ctrl.log)))
            if self.ctrl.done and not reason:
                reason = 'controller set done'

        p, _ = self.robot.pose(q)
        Jc = self.robot.jacobian_pos(q)
        v = Jc @ dq
        F, f_n, delta, v_n = self.wall.contact(p, v)
        tau_wall = Jc.T @ F
        self._publish(self.tau_ctrl + tau_wall)
        self.pub_hist.append(self.tau_ctrl.copy())
        if self.wall.enabled and len(self.rows) % 5 == 0:
            self.hand_depth = body_depth(self.robot, q, self.wall)
            if (not self.hand_warned and self.hand_depth > 0.005
                    and self.hand_depth > max(delta, 0.0) + 0.005):
                self.hand_warned = True
                self.get_logger().warn(
                    f't = {t:.2f} s: the HAND BODY is {1e3 * self.hand_depth:.0f} mm inside the '
                    f'wall, but the wall only pushes the fingertip. Keep the tool pointing at the '
                    f'wall (e.g. control the orientation too). Logged as hand_depth.')
        self.rows.append(np.concatenate([
            [t], q, dq, self.tau_raw, self.tau_ctrl, tau_wall, p, v, F,
            [f_n, delta, v_n, float(f_n > 0), self.hand_depth, self.ctrl_ms,
             time.monotonic() - self.wall_t0]]))

        self.had_contact |= f_n > 0
        if not reason and self.stop_at_detachment and self.had_contact and delta < 0:
            reason = 'first detachment'
        if not reason and t >= self.duration:
            reason = 'duration reached'
        if reason:
            self._finish(reason)

    def _safety(self, q, dq) -> str:
        bad_q = np.where((q < Q_MIN + Q_MARGIN) | (q > Q_MAX - Q_MARGIN))[0]
        if bad_q.size:
            return f'SAFETY STOP: joint {bad_q[0] + 1} at {q[bad_q[0]]:.3f} rad, near its limit'
        bad_v = np.where(np.abs(dq) > DQ_MAX)[0]
        if bad_v.size:
            return f'SAFETY STOP: joint {bad_v[0] + 1} speed {dq[bad_v[0]]:.2f} rad/s over limit'
        return ''

    def _publish(self, tau):
        self.pub.publish(Float64MultiArray(data=[float(x) for x in tau]))

    # ── end of run ────────────────────────────────────────────────────────────
    def _finish(self, reason: str):
        self.phase, self.stop_reason = 'DONE', reason
        (self.get_logger().warn if reason.startswith(('SAFETY', 'exception'))
         else self.get_logger().info)(f'run ended: {reason}')
        self._save()
        if self.exit_on_done:
            raise SystemExit(0)

    def _save(self):
        R = np.array(self.rows)
        if R.size == 0:
            self.get_logger().warn('nothing to save')
            return
        cols, k, out = [('t', 1), ('q', 7), ('dq', 7), ('tau_cmd', 7), ('tau', 7),
                        ('tau_wall', 7), ('p', 3), ('v', 3), ('F', 3), ('F_n', 1),
                        ('delta', 1), ('v_n', 1), ('contact', 1), ('hand_depth', 1), ('ctrl_ms', 1),
                        ('wall_clock', 1)], 0, {}
        for name, n in cols:
            out[name] = R[:, k] if n == 1 else R[:, k:k + n]
            k += n
        # student log: forward-filled onto every plant sample
        keys = sorted({key for _, d in self.user_rows for key in d})
        for key in keys:
            first = next(np.asarray(d[key], float) for _, d in self.user_rows if key in d)
            arr = np.full((len(R),) + first.shape, np.nan)
            for (i0, d), nxt in zip(self.user_rows, self.user_rows[1:] + [(len(R), {})]):
                if key in d:
                    arr[i0:nxt[0]] = np.asarray(d[key], float)
            out['user_' + key] = arr
        meta = {'controller': self.controller_path, 'params': self.params,
                'wall': {'point': self.wall.point.tolist(), 'normal': self.wall.normal.tolist(),
                         'stiffness': self.wall.stiffness, 'damping': self.wall.damping,
                         'enabled': self.wall.enabled},
                'control_rate': 1.0 / self.dt_ctrl, 'duration': self.duration,
                'realism': self.realism,
                'stop_reason': self.stop_reason}
        np.savez(self.log_path, meta=json.dumps(meta), **out)
        t, wc = out['t'], out['wall_clock']
        rtf = (t[-1] - t[0]) / max(wc[-1] - wc[0], 1e-9)
        dropped = 1.0 - len(t) / max(round((t[-1] - t[0]) / 1e-3) + 1, 1)
        if dropped > 0.02:
            self.get_logger().warn(
                f'{100 * dropped:.0f}% of the 1 kHz joint states were missed: the controller is '
                f'too slow for real time. Use real_time_factor:=0.5 (or lower) or control_rate:=500.')
        self.get_logger().info(
            f'saved {self.log_path}  ({len(R)} samples, {t[-1]:.2f} s sim, real-time factor '
            f'{rtf:.2f}, controller {np.mean(out["ctrl_ms"]):.2f} ms mean / '
            f'{np.max(out["ctrl_ms"]):.2f} ms max, peak wall force {np.max(out["F_n"]):.2f} N)')


def main():
    rclpy.init()
    node = Runner()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        if node.phase not in ('DONE', 'WAIT') and node.rows:
            node.stop_reason = node.stop_reason or 'interrupted'
            node._save()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
