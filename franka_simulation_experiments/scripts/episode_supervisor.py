#!/usr/bin/env python3
"""episode_supervisor: start the episode when the stack is up, end the run, label it SIM.

SIM only. One per run:

1. waits until the closed loop is actually running: q̈_nom (commander or scenario
   commander), q̈_safe (filter) and tau_cmd (qddot_to_torque) all flowing, the
   RT controller ACTIVE in the controller manager, plus ``warmup_s`` of sim
   time. With perception on it also waits for /cbf/per_link_distances. Until
   then /sim/episode reads WAITING. (The controller check is not implied by
   tau_cmd: qddot_to_torque publishes before the stack's spawner has fired,
   and a scenario episode used to start, and end, with the arm floating on the
   plugin's gravity term.);
2. publishes /sim/episode = [state, t_start, t_now] (state 0 waiting, 1 running,
   2 done) at 50 Hz. The scenario commander and the obstacle system start their
   laws on the WAITING -> RUNNING edge, so the episode clock is one clock;
3. after ``duration_s`` of sim time (0 = until stopped) it finalises and exits.
   The launch ends on that exit.

Finalising (ground rule 5):
* run_manifest_sim.json gets the episode times, the reflex verdict and RTF;
* experiment_logger's run_manifest.json gets ``"plant": "gazebo"``, ``"SIM": true``,
  the RTF and the git hash (an adapter on the sim side, the logger is unchanged);
* franka_experiments/franka_logs is checked: anything that appeared there during
  the run is a SIM output in a hardware directory. It is moved into
  ``<run_dir>/stray_logs`` and the run is marked as such.
"""
import glob
import json
import os
import shutil
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from simexp.sim_config import git_provenance  # noqa: E402

WAITING, RUNNING, DONE = 0.0, 1.0, 2.0


def hardware_log_dir():
    for cand in ('/ros2_ws/src/franka_experiments/franka_logs',
                 os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..',
                              'franka_experiments', 'franka_logs')):
        if os.path.isdir(cand):
            return os.path.normpath(cand)
    return None


class EpisodeSupervisor(Node):
    def __init__(self):
        super().__init__('episode_supervisor')
        dp = self.declare_parameter
        self.run_dir = dp('run_dir', '').value
        self.duration_s = float(dp('duration_s', 0.0).value)
        self.warmup_s = float(dp('warmup_s', 2.0).value)
        self.need_perception = bool(dp('perception', True).value)
        # The start is announced AHEAD (start_lead_s) so the gz obstacle system,
        # the scenario commander and this node all switch at the same sim
        # instant. start_offset_s is the seeded phase within a camera frame.
        self.start_lead_s = float(dp('start_lead_s', 0.25).value)
        self.start_offset_s = float(dp('start_offset_s', 0.0).value)
        self.controller = dp('controller_name', 'rt_torque_controller').value
        self.controller_active = not self.controller
        from controller_manager_msgs.srv import ListControllers
        self._list_req = ListControllers.Request()
        self._list_cli = self.create_client(
            ListControllers, dp('controller_manager', '/NS_1/controller_manager').value
            + '/list_controllers')
        self._list_fut = None
        self.state = WAITING
        self.t_start = None
        self.seen = {'qddot_nom': None, 'qddot_safe': None, 'torque_cmd': None,
                     'per_link_distances': None}
        self.ready_since = None
        self.hw_dir = hardware_log_dir()
        self.hw_before = set(os.listdir(self.hw_dir)) if self.hw_dir else set()
        self.t_wall0 = time.time()
        self.pub = self.create_publisher(Float64MultiArray, '/sim/episode',
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        from std_msgs.msg import Float64
        self._Float64 = Float64
        self.pub_start = self.create_publisher(Float64, '/sim/episode_start', 10)
        for key, topic in (('qddot_nom', '/NS_1/qddot_nom'), ('qddot_safe', '/NS_1/qddot_safe'),
                           ('torque_cmd', '/NS_1/torque_cmd')):
            self.create_subscription(Float64MultiArray, topic,
                                     lambda m, k=key: self._seen(k), 10)
        if self.need_perception:
            from franka_msgs.msg import MultiLinkDistance
            self.create_subscription(
                MultiLinkDistance, '/cbf/per_link_distances',
                lambda m: self._seen('per_link_distances'),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.02, self._tick)
        self.finished = False
        self._n_start = 0

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _seen(self, key):
        self.seen[key] = self._now()

    def _poll_controller(self):
        if self.controller_active:
            return
        if self._list_fut is not None:
            if not self._list_fut.done():
                return
            res = self._list_fut.result()
            self._list_fut = None
            if res is not None and any(c.name == self.controller and c.state == 'active'
                                       for c in res.controller):
                self.controller_active = True
                self.get_logger().info(f'{self.controller} is active')
                return
        if self._list_cli.service_is_ready():
            self._list_fut = self._list_cli.call_async(self._list_req)

    def _ready(self, now):
        keys = ['qddot_nom', 'qddot_safe', 'torque_cmd'] + (
            ['per_link_distances'] if self.need_perception else [])
        self._poll_controller()
        return self.controller_active and all(
            self.seen[k] is not None and now - self.seen[k] < 0.5 for k in keys)

    def _tick(self):
        now = self._now()
        if self.state == WAITING:
            if self._ready(now):
                self.ready_since = self.ready_since if self.ready_since is not None else now
                if now - self.ready_since >= self.warmup_s:
                    self.state = RUNNING
                    self.t_start = now + self.start_lead_s + self.start_offset_s
                    self.get_logger().info(f'SIM episode START at t={self.t_start:.4f} s (sim), '
                                           f'announced at {now:.3f}')
            else:
                self.ready_since = None
        if self.state == RUNNING and self._n_start < 50:
            self.pub_start.publish(self._Float64(data=float(self.t_start)))
            self._n_start += 1
        if self.state == RUNNING and self.duration_s > 0 and now - self.t_start >= self.duration_s:
            self.state = DONE
            self.get_logger().info(f'SIM episode DONE at t={now:.3f} s (sim)')
        self.pub.publish(Float64MultiArray(data=[self.state, self.t_start or -1.0, now]))
        if self.state == DONE and not self.finished:
            self.finalise()
            self.finished = True
            raise SystemExit(0)

    def finalise(self):
        rd = self.run_dir
        reflex = {}
        try:
            with open(os.path.join(rd, 'reflex_summary.json')) as f:
                reflex = json.load(f)
        except (OSError, ValueError):
            pass
        stray = []
        if self.hw_dir:
            new = sorted(set(os.listdir(self.hw_dir)) - self.hw_before)
            for name in new:
                src = os.path.join(self.hw_dir, name)
                if os.path.getmtime(src) >= self.t_wall0 - 1.0:
                    dst_dir = os.path.join(rd, 'stray_logs')
                    os.makedirs(dst_dir, exist_ok=True)
                    shutil.move(src, os.path.join(dst_dir, name))
                    stray.append(name)
            if stray:
                self.get_logger().error(f'SIM output appeared in {self.hw_dir}: {stray} '
                                        f'-> moved to {rd}/stray_logs')
        episode = {'t_start_sim': self.t_start, 't_end_sim': self._now(),
                   'controller_active_at_start': bool(self.controller_active),
                   'duration_s': self.duration_s, 'warmup_s': self.warmup_s,
                   'wall_s': time.time() - self.t_wall0}
        sim_label = {'plant': 'gazebo', 'SIM': True, 'git_sim': git_provenance(),
                     'rtf': reflex.get('rtf'), 'reflex_verdict': reflex.get('verdict'),
                     'reflex_counts': reflex.get('counts'), 'episode': episode,
                     'stray_hardware_logs': stray}
        p = os.path.join(rd, 'run_manifest_sim.json')
        try:
            with open(p) as f:
                man = json.load(f)
        except (OSError, ValueError):
            man = {}
        man.update(sim_label)
        with open(p, 'w') as f:
            json.dump(man, f, indent=2, sort_keys=True, default=str)
        for lm in glob.glob(os.path.join(rd, 'logger', '*', 'run_manifest.json')):
            try:
                with open(lm) as f:
                    m = json.load(f)
                m.update({'plant': 'gazebo', 'SIM': True, 'rtf': reflex.get('rtf'),
                          'git_sim': sim_label['git_sim'],
                          'sim_run_manifest': p})
                with open(lm, 'w') as f:
                    json.dump(m, f, indent=2, sort_keys=True, default=str)
            except (OSError, ValueError) as exc:
                self.get_logger().warn(f'could not label {lm}: {exc}')


def main():
    rclpy.init()
    node = EpisodeSupervisor()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        if not node.finished:
            node.finalise()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
