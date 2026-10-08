#!/usr/bin/env python3
"""Grasp cycle on the object in the human hand: take it, release it, back home, repeat.

HOME -> PREGRASP -> APPROACH -> CLOSE -> HOLD -> OPEN -> RETREAT -> RETURN -> HOME

  HOME      hold the start pose; arm once the hand has been empty for rearm_s, then
            start when an object is confirmed (/handover/hand_object) and a fresh
            grasp pose (/handover/grasp_pose) is inside the workspace;
  PREGRASP  pregrasp_m behind the grasp along its approach, gripper open, wrist
            turning towards the grasp orientation; follows the grasp while the hand moves;
  APPROACH  slow straight motion to the grasp (grasp pose frozen);
  CLOSE     gripper close (set_gripper true), then HOLD for hold_s;
  OPEN      gripper open, RETREAT pregrasp_m back along the approach (away from the
            hand), then RETURN to the start pose and orientation.

Same controller, reference smoothing and limits as handover_qddot_commander; only
the target and the held orientation change. Any lost hand / stale grasp before the
gripper closes sends the arm back home.
"""

import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation, Slerp
from std_srvs.srv import SetBool

from franka_msgs.msg import HandObjectState

from handover_qddot_commander import HandoverQddotCommander
from franka_experiments.utils.node_runtime import run_node_main


class GraspExecutor(HandoverQddotCommander):

    def __init__(self):
        super().__init__()
        par = lambda n, v: self.declare_parameter(n, v).value
        self.pregrasp_m = float(par('pregrasp_m', 0.10))
        self.approach_step_m = float(par('approach_step_m', 0.02))   # slow: ~0.1 m/s
        self.reach_tol_m = float(par('reach_tol_m', 0.015))
        self.rot_tol = np.radians(float(par('rot_tol_deg', 10.0)))
        self.rot_rate = np.radians(float(par('rot_rate_deg_s', 45.0)))
        self.hold_s = float(par('hold_s', 2.0))
        self.gripper_s = float(par('gripper_s', 1.5))   # time given to each gripper motion
        self.rearm_s = float(par('rearm_s', 1.0))       # empty hand before the next cycle
        self.grasp_timeout_s = float(par('grasp_timeout_s', 0.3))  # fresh grasp to start
        self.lost_s = float(par('lost_s', 1.0))              # gap tolerated before PREGRASP aborts
        self.approach_lost_s = float(par('approach_lost_s', 0.4))  # same, slow final approach
        self.retry_s = float(par('retry_s', 1.0))            # wait at home after an abort
        self.max_reach_m = float(par('max_reach_m', 0.80))
        self.min_z_m = float(par('min_z_m', 0.10))
        self.phase_timeout_s = float(par('phase_timeout_s', 8.0))

        self._grasp = None          # (position, R_tcp, stamp_ns)
        self._frozen = None         # grasp used by APPROACH / CLOSE / HOLD
        self._object = False
        self._armed, self._empty_since = True, None   # a new cycle needs an empty hand first
        self._last_t = None
        self._good_t = None         # last time grasp and hand were both fine
        self._retry_at = 0.0
        self._phase, self._phase_t = 'HOME', time.monotonic()
        self._R_home = None
        self._grip = self.create_client(SetBool, par('gripper_service', 'gripper_controller/set_gripper'))
        self.create_subscription(PoseStamped, '/handover/grasp_pose', self._grasp_cb, 10)
        self.create_subscription(HandObjectState, '/handover/hand_object', self._object_cb, 10)
        self.get_logger().info('Grasp executor ready: HOME')

    # ------------------------------------------------------------ inputs
    def _grasp_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        self._grasp = (np.array([p.x, p.y, p.z]), R,
                       msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)

    def _object_cb(self, msg):
        self._object = bool(msg.valid and msg.object_present)
        if self._object or not msg.valid:
            self._empty_since = None
        elif self._empty_since is None:
            self._empty_since = time.monotonic()
        elif time.monotonic() - self._empty_since >= self.rearm_s:
            self._armed = True

    def _fresh_grasp(self, max_age=None):
        if self._grasp is None:
            return None
        age = (self.get_clock().now().nanoseconds - self._grasp[2]) * 1e-9
        p, R, _ = self._grasp
        reachable = np.linalg.norm(p[:2]) <= self.max_reach_m and p[2] >= self.min_z_m
        limit = self.grasp_timeout_s if max_age is None else max_age
        return self._grasp if 0.0 <= age <= limit and reachable else None

    def _gripper(self, close):
        if not self._grip.service_is_ready():
            self.get_logger().warn(f'{self._grip.srv_name} not available')
            return
        self._grip.call_async(SetBool.Request(data=bool(close)))
        self.get_logger().info(f'Gripper {"CLOSE" if close else "OPEN"}')

    def _set_phase(self, phase):
        self.get_logger().info(f'{self._phase} -> {phase}')
        self._phase, self._phase_t = phase, time.monotonic()

    # ------------------------------------------------------------ orientation
    def _turn_towards(self, R_goal, dt):
        """Rotate the held orientation towards R_goal at rot_rate (base controller hold)."""
        R_now = Rotation.from_matrix(self._R_des)
        angle = (R_now.inv() * Rotation.from_matrix(R_goal)).magnitude()
        if angle > 1e-4:
            f = min(1.0, self.rot_rate * dt / angle)
            R_new = Slerp([0.0, 1.0], Rotation.concatenate([R_now, Rotation.from_matrix(R_goal)]))(f)
            self._R_des[:] = R_new.as_matrix()
            self._R_des_nom[:] = self._R_des
        return angle

    def _closest_flip(self, R):
        """A parallel gripper grasp is the same turned by 180 deg about z: take the nearer one."""
        flipped = R @ np.diag([-1.0, -1.0, 1.0])
        now = Rotation.from_matrix(self._R_des)
        d = lambda A: (now.inv() * Rotation.from_matrix(A)).magnitude()
        return R if d(R) <= d(flipped) else flipped

    # ------------------------------------------------------------ target
    def desired_position(self):
        if self._hold_position is None:
            return self._p_ee.copy()
        if self._R_home is None and self._orient_ok:
            self._R_home = self._R_des.copy()
        now = time.monotonic()
        dt = min(0.05, now - self._last_t) if self._last_t is not None else 0.0
        self._last_t = now
        elapsed = now - self._phase_t
        home = self._hold_position
        grasp = self._fresh_grasp()
        if grasp is not None and self.hand_trusted():
            self._good_t = now

        if self._phase == 'HOME':
            if self._R_home is not None:
                self._turn_towards(self._R_home, dt)
            ready = {'armed (empty hand first)': self._armed, 'object in hand': self._object,
                     'fresh reachable grasp': grasp is not None, 'hand trusted': self.hand_trusted()}
            if not all(ready.values()):
                missing = ', '.join(k for k, ok in ready.items() if not ok)
                self.get_logger().info(f'HOME, waiting for: {missing}', throttle_duration_sec=2.0)
            if all(ready.values()) and now >= self._retry_at:
                self._armed, self._good_t = False, now
                self._gripper(False)
                self._set_phase('PREGRASP')
            return self._step_to(home, self.max_step())

        if self._phase in ('PREGRASP', 'APPROACH'):
            # short gaps (grasp at ~10 Hz, tracking dropouts) ride on the last grasp
            tolerance = self.lost_s if self._phase == 'PREGRASP' else self.approach_lost_s
            if self._good_t is None or now - self._good_t > tolerance:
                self.get_logger().warn('hand / grasp lost: back home, gripper stays open, retry')
                self._armed, self._retry_at = True, now + self.retry_s  # nothing was grasped
                self._set_phase('RETURN')
                return self._p_ee.copy()
            grasp = grasp or self._fresh_grasp(max_age=tolerance)
            if grasp is None and self._phase == 'PREGRASP':
                return self._p_ee.copy()  # hold while waiting for the next grasp

        if self._phase == 'PREGRASP':
            p, R, _ = grasp
            R = self._closest_flip(R)
            angle = self._turn_towards(R, dt)
            target = p - self.pregrasp_m * R[:, 2]
            if (np.linalg.norm(target - self._p_ee) < self.reach_tol_m * 2 and angle < self.rot_tol
                    and now - self._good_t <= self.grasp_timeout_s):  # final approach only on a fresh grasp
                self._frozen = (p.copy(), R.copy())
                self._set_phase('APPROACH')
            elif elapsed > self.phase_timeout_s:
                self._set_phase('RETURN')
            return self._step_to(target, self.max_step())

        if self._phase == 'APPROACH':
            p, R = self._frozen
            self._turn_towards(R, dt)
            if np.linalg.norm(p - self._p_ee) < self.reach_tol_m:
                self._gripper(True)
                self._set_phase('CLOSE')
            elif elapsed > self.phase_timeout_s:
                self._set_phase('RETURN')
            return self._step_to(p, self.approach_step_m)

        if self._phase in ('CLOSE', 'HOLD', 'OPEN'):
            if self._phase == 'CLOSE' and elapsed >= self.gripper_s:
                self._set_phase('HOLD')
            elif self._phase == 'HOLD' and elapsed >= self.hold_s:
                self._gripper(False)
                self._set_phase('OPEN')
            elif self._phase == 'OPEN' and elapsed >= self.gripper_s:
                self._set_phase('RETREAT')
            return self._step_to(self._frozen[0], self.approach_step_m)

        if self._phase == 'RETREAT':  # straight back along the approach, away from the hand
            p, R = self._frozen
            back = p - self.pregrasp_m * R[:, 2]
            if np.linalg.norm(back - self._p_ee) < self.reach_tol_m * 2 or elapsed > self.phase_timeout_s:
                self._set_phase('RETURN')
            return self._step_to(back, self.approach_step_m)

        # RETURN: home position and orientation
        if self._R_home is not None:
            angle = self._turn_towards(self._R_home, dt)
        else:
            angle = 0.0
        if np.linalg.norm(home - self._p_ee) < self.reach_tol_m * 2 and angle < self.rot_tol:
            self._frozen = None
            self._set_phase('HOME')
        return self._step_to(home, self.max_step())

    def max_step(self):
        return float(self.get_parameter('max_target_step_m').value)

    def _step_to(self, target, max_step):
        """Virtual target never farther than max_step from the TCP (speed bound)."""
        move = target - self._p_ee
        n = float(np.linalg.norm(move))
        return self._p_ee + move * (max_step / n) if n > max_step else target.copy()


def main(args=None):
    run_node_main(GraspExecutor, args=args)


if __name__ == '__main__':
    main()
