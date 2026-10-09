#!/usr/bin/env python3
"""Handover distance: hand state + robot joints -> EndEffectorState, HandoverDistance, HandoverObserver.

  gripper    every 10 ms: newest joint state (fast stream, the standard one only while the fast one is
             silent for 0.2 s) -> Pinocchio: tip = FK + ee_tip_offset along ee_tip_axis, v = J(q) qdot
  distance   palm (base frame) - gripper tip (the state nearest in time, <= max_ee_state_age_s),
             sigma along the palm-gripper direction from the palm covariance
  rate       relative: (W75 hand velocity - gripper velocity) . direction  (UPDATED / HOLD as the hand
             velocity); fallback: linear regression of the distance over the last 5 frames (DISTANCE_C5)
  TTC        distance / closing speed, when closing faster than ttc_min_closing_speed
  observer   APPROACHING / HOLD / RETREATING from the closing speed with hysteresis (observer_enter /
             observer_exit thresholds) confirmed over observer_confirm_frames fresh-rate frames;
             LOST without a distance, WARMUP without a rate
"""

import os
import time
from collections import deque

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from franka_msgs.msg import EndEffectorState, HandoverDistance, HandoverObserver, HandState
from geometry_msgs.msg import Point, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from franka_experiments.utils.cbf_utils import load_robot_config as load_control_config
from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro, load_pinocchio_model, resolve_arm_joint_ids)

D, O = HandoverDistance, HandoverObserver


def xyz(v):
    return np.array([v.x, v.y, v.z], dtype=np.float64)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def parse_joints(msg):
    """(q, qdot, stamp) of the 7 FR3 joints, None if any is missing."""
    index = {name: i for i, name in enumerate(msg.name)}
    q, qdot = np.zeros(NUM_JOINTS), np.zeros(NUM_JOINTS)

    for k, name in enumerate(FR3_JOINT_NAMES):
        i = index.get(name)
        if i is None or i >= len(msg.position) or i >= len(msg.velocity):
            return None
        q[k], qdot[k] = msg.position[i], msg.velocity[i]

    return q, qdot, msg.header.stamp


def linear_rate(history):
    """(valid, slope) of a least-squares line through (t, value), >= 3 samples."""
    if len(history) < 3:
        return False, None

    times = np.asarray([item[0] for item in history], dtype=np.float64)
    values = np.asarray([item[1] for item in history], dtype=np.float64)
    tc = times - np.mean(times)
    denom = float(np.dot(tc, tc))
    if denom <= 1e-12:
        return False, None

    rate = float(np.dot(tc, values - np.mean(values)) / denom)
    return (True, rate) if np.isfinite(rate) else (False, None)


class HandoverDistanceEstimator(Node):

    def __init__(self):
        super().__init__('distance_handover_estimator')
        param = lambda name: self.declare_parameter(  # config/hand_tracking.yaml, passed by the launch
            name, descriptor=ParameterDescriptor(dynamic_typing=True)).value
        self.declare_parameter('robot_config_path', os.path.join(
            get_package_share_directory('franka_experiments'), 'config', 'fr3_complete.yaml'))
        self.declare_parameter('max_ee_state_age_s', 0.10)

        config = load_robot_config(self.get_parameter('robot_config_path').value)
        self.base_frame = config['robot']['base_frame']
        self.max_ee_state_age_s = float(self.get_parameter('max_ee_state_age_s').value)
        self.ttc_min_closing_speed = float(param('ttc_min_closing_speed'))
        self.enter_threshold = float(param('observer_enter_threshold'))
        self.exit_threshold = float(param('observer_exit_threshold'))
        self.confirm_frames = int(param('observer_confirm_frames'))
        if (self.enter_threshold <= 0.0
                or not 0.0 <= self.exit_threshold < self.enter_threshold
                or self.confirm_frames < 1):
            raise ValueError('observer: need enter > 0, 0 <= exit < enter, confirm_frames >= 1')

        self.tf_buffer = Buffer()  # hand frame -> base (the gripper comes already in base)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ee_states = deque(maxlen=100)       # (t, EndEffectorState), for time alignment only
        self.distance_history = deque(maxlen=5)  # (t, distance) of one physical hand, for the fallback rate
        self.physical_hand = HandState.HAND_UNKNOWN
        self.state, self.pending_state, self.pending_count = O.WARMUP, None, 0
        self.setup_gripper(config)

        self.publisher = self.create_publisher(HandoverDistance, '/handover/distance', 10)
        self.observer_publisher = self.create_publisher(HandoverObserver, '/handover/observer', 10)
        self.create_subscription(HandState, '/handover/hand_state', self.callback, 10)
        self.get_logger().info(
            f'Handover distance: frame={self.base_frame}, '
            f'TTC min closing={self.ttc_min_closing_speed:.3f} m/s, '
            f'observer enter={self.enter_threshold:.3f} exit={self.exit_threshold:.3f} m/s '
            f'confirm={self.confirm_frames}')

    # ------------------------------------------------------------ gripper tip (FK + J(q) qdot)
    def setup_gripper(self, config):
        self.ee_link = config['robot'].get('ee_link', 'fr3_link8')
        self.tip_local = np.zeros(3)
        self.tip_local[int(config['distance']['ee_tip_axis'])] = float(config['distance']['ee_tip_offset'])

        # same model as qddot_to_torque
        self.model, self.data = load_pinocchio_model(generate_urdf_from_xacro())
        joints = resolve_arm_joint_ids(self.model)
        self.arm_q_ids = [self.model.joints[j].idx_q for j in joints]
        self.arm_v_ids = [self.model.joints[j].idx_v for j in joints]
        self.ee_frame = self.model.getFrameId(self.ee_link)
        self.q_neutral, self.q_full = pin.neutral(self.model), pin.neutral(self.model)
        self.qdot_full = np.zeros(self.model.nv)
        self.joints = None            # newest valid (q, qdot, stamp)
        self.primary_alive_at = None  # wall time of the fast stream

        topics = load_control_config('control')['topics']
        primary = topics.get('joint_states_fast', topics['joint_states_topic'])
        fallback = topics['joint_states_topic']

        # the fast stream runs at ~1 kHz: no callback per message (30-40 % of a core), the subscriptions
        # live on a node that is never spun and the newest message is taken from the DDS queue (depth 1)
        self.reader = rclpy.create_node('end_effector_state_joint_reader')
        self.joint_subs = {
            'primary': self.reader.create_subscription(JointState, primary, lambda msg: None, 1)}
        if fallback != primary:
            self.joint_subs['fallback'] = self.reader.create_subscription(
                JointState, fallback, lambda msg: None, 1)

        self.ee_publisher = self.create_publisher(EndEffectorState, '/handover/end_effector_state', 10)
        self.create_timer(0.01, self.update_gripper)

    def _take(self, key):
        """Newest message waiting in a subscription queue, or None."""
        sub, msg = self.joint_subs.get(key), None
        if sub is None:
            return None

        with sub.handle:
            while True:
                taken = sub.handle.take_message(sub.msg_type, sub.raw)
                if taken is None:
                    return msg
                msg = taken[0]

    def _latest_joints(self):
        now = time.monotonic()
        msg = self._take('primary')
        if msg is not None and (state := parse_joints(msg)) is not None:
            self.primary_alive_at, self.joints = now, state
            return state

        primary_alive = self.primary_alive_at is not None and now - self.primary_alive_at <= 0.20
        msg = self._take('fallback')
        if msg is not None and not primary_alive and (state := parse_joints(msg)) is not None:
            self.joints = state
        return self.joints

    def update_gripper(self):
        state = self._latest_joints()
        if state is None:
            return

        q, qdot, stamp = state
        np.copyto(self.q_full, self.q_neutral)
        self.qdot_full[:] = 0.0
        for k, (iq, iv) in enumerate(zip(self.arm_q_ids, self.arm_v_ids)):
            self.q_full[iq], self.qdot_full[iv] = q[k], qdot[k]

        pin.forwardKinematics(self.model, self.data, self.q_full, self.qdot_full)
        pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self.ee_frame]
        r_world = placement.rotation @ self.tip_local
        position = np.asarray(placement.translation) + r_world
        J = pin.computeFrameJacobian(self.model, self.data, self.q_full, self.ee_frame,
                                     pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        velocity = (J[:3, :] - pin.skew(r_world) @ J[3:, :]) @ self.qdot_full  # v_tip = v_frame + omega x r

        out = EndEffectorState()
        out.header.stamp, out.header.frame_id = stamp, self.base_frame
        out.valid = bool(np.all(np.isfinite(position)) and np.all(np.isfinite(velocity)))
        out.q, out.qdot = q.tolist(), qdot.tolist()
        out.position.x = float(position[0])
        out.position.y = float(position[1])
        out.position.z = float(position[2])
        out.velocity.x = float(velocity[0])
        out.velocity.y = float(velocity[1])
        out.velocity.z = float(velocity[2])
        self.ee_publisher.publish(out)
        self.on_ee_state(out)

    # ------------------------------------------------------------ inputs
    def on_ee_state(self, msg):
        if not msg.valid:
            return
        if msg.header.frame_id != self.base_frame:
            self.get_logger().warn(
                f'Ignoring EndEffectorState frame={msg.header.frame_id}; expected {self.base_frame}',
                throttle_duration_sec=2.0)
            return

        t = stamp_s(msg.header.stamp)
        if self.ee_states:
            if t < self.ee_states[-1][0]:
                self.ee_states.clear()
            elif t == self.ee_states[-1][0]:
                self.ee_states[-1] = (t, msg)
                return
        self.ee_states.append((t, msg))

    def ee_state_at(self, stamp):
        if not self.ee_states:
            return None, float('nan')

        t_hand = stamp_s(stamp)
        t_ee, state = min(self.ee_states, key=lambda item: abs(item[0] - t_hand))
        age = abs(t_hand - t_ee)
        return (None, age) if age > self.max_ee_state_age_s else (state, age)

    def hand_in_base(self, msg):
        """Palm, its covariance and the rotation hand frame -> base."""
        p = xyz(msg.palm_position)
        v = msg.palm_position_variance
        covariance = np.diag([max(0.0, v.x), max(0.0, v.y), max(0.0, v.z)])
        if msg.header.frame_id == self.base_frame:
            return p, covariance, np.eye(3)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception as exc:
            self.get_logger().warn(
                f'Cannot transform hand {msg.header.frame_id} -> {self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return None, None, None

        t, R = tf.transform.translation, quaternion_matrix(tf.transform.rotation)
        return R @ p + np.array([t.x, t.y, t.z], dtype=np.float64), R @ covariance @ R.T, R

    # ------------------------------------------------------------ distance
    def publish(self, out):
        self.publisher.publish(out)
        self.observe(out)

    def publish_invalid(self, out):
        self.distance_history.clear()
        self.publish(out)

    def callback(self, msg):
        hand = int(msg.physical_hand)
        if hand in (HandState.HAND_LEFT, HandState.HAND_RIGHT):
            if hand != self.physical_hand:
                self.distance_history.clear()
            self.physical_hand = hand

        out = HandoverDistance()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.base_frame
        out.rate_source = D.RATE_SOURCE_NONE
        out.rate_age_s = out.hand_velocity_age_s = out.ee_velocity_age_s = out.rate_consistency_error = float('nan')

        # palm and gripper in the base frame
        if not msg.position_valid:
            return self.publish_invalid(out)
        p_hand, covariance, hand_rotation = self.hand_in_base(msg)
        if p_hand is None:
            return self.publish_invalid(out)
        ee_state, ee_age = self.ee_state_at(msg.header.stamp)  # gripper: p = FK(q), v = J(q) qdot
        if ee_state is None:
            return self.publish_invalid(out)
        p_ee, ee_velocity = xyz(ee_state.position), xyz(ee_state.velocity)
        if not np.all(np.isfinite(p_ee)):
            return self.publish_invalid(out)
        ee_velocity_valid = bool(ee_state.valid
                                 and ee_age <= self.max_ee_state_age_s
                                 and np.all(np.isfinite(ee_velocity)))

        delta = p_hand - p_ee
        distance = float(np.linalg.norm(delta))
        if not np.isfinite(distance) or distance <= 1e-9:
            return self.publish_invalid(out)
        direction = delta / distance
        distance_variance = float(direction.T @ covariance @ direction)
        predicted = int(msg.position_source) == int(HandState.POSITION_SOURCE_PREDICTED_W75)

        # fallback rate: regression of the measured distance (not on predicted palms)
        if predicted:
            self.distance_history.clear()
            legacy_valid, legacy_rate = False, 0.0
        else:
            t = stamp_s(msg.header.stamp)
            if self.distance_history and t <= self.distance_history[-1][0]:
                self.distance_history.clear()
            self.distance_history.append((t, float(distance)))
            legacy_valid, legacy_rate = linear_rate(self.distance_history)
            legacy_rate = float(legacy_rate) if legacy_valid else 0.0

        # relative rate from the W75 hand velocity and the gripper velocity
        hand_velocity_valid = bool(msg.velocity_valid)
        hand_velocity = np.zeros(3, dtype=np.float64)
        if hand_velocity_valid:
            hand_velocity = hand_rotation @ xyz(msg.palm_velocity)
            if not np.all(np.isfinite(hand_velocity)):
                hand_velocity_valid = False
                hand_velocity[:] = 0.0

        relative_velocity = np.zeros(3, dtype=np.float64)
        relative_valid, relative_rate = False, 0.0
        if not predicted and hand_velocity_valid and ee_velocity_valid:
            relative_velocity = hand_velocity - ee_velocity
            relative_rate = float(np.dot(direction, relative_velocity))
            relative_valid = bool(np.isfinite(relative_rate))

        source = D.RATE_SOURCE_NONE
        if relative_valid:
            if int(msg.velocity_source) == HandState.VELOCITY_SOURCE_UPDATED:
                source = D.RATE_SOURCE_RELATIVE_UPDATED
            elif int(msg.velocity_source) == HandState.VELOCITY_SOURCE_HOLD:
                source = D.RATE_SOURCE_RELATIVE_HOLD
            else:
                relative_valid = False

        # chosen rate: relative, else the distance regression, else none
        if relative_valid:
            rate_valid, rate = True, relative_rate
            degraded = source == D.RATE_SOURCE_RELATIVE_HOLD
            hand_age = float(msg.velocity_age_s)
            rate_age = max(hand_age, 0.0) if np.isfinite(hand_age) else 0.0
        elif legacy_valid:
            rate_valid, rate = True, float(legacy_rate)
            source, degraded, rate_age = D.RATE_SOURCE_DISTANCE_C5, True, 0.0
        else:
            rate_valid, rate = False, 0.0
            source, degraded = D.RATE_SOURCE_NONE, bool(predicted)
            rate_age = (float(msg.position_age_s)
                        if (predicted and np.isfinite(float(msg.position_age_s)))
                        else float('nan'))

        closing = -rate if rate_valid else 0.0
        ttc_valid = bool(rate_valid and closing > self.ttc_min_closing_speed)
        legacy_closing = -legacy_rate if legacy_valid else 0.0
        legacy_ttc_valid = bool(legacy_valid and legacy_closing > 1e-6)

        out.valid = True
        out.palm_position = Point(x=float(p_hand[0]), y=float(p_hand[1]), z=float(p_hand[2]))
        out.ee_control_point = Point(x=float(p_ee[0]), y=float(p_ee[1]), z=float(p_ee[2]))
        out.ee_to_palm = Vector3(x=float(delta[0]), y=float(delta[1]), z=float(delta[2]))
        out.distance = float(distance)
        out.distance_sigma = float(np.sqrt(max(distance_variance, 0.0)))
        out.tracking_confidence = float(msg.tracking_confidence)
        out.motion_stability = float(msg.motion_stability)

        out.hand_velocity_valid = bool(hand_velocity_valid)
        out.hand_velocity_source = int(msg.velocity_source)
        out.hand_velocity_estimator = int(msg.velocity_estimator)
        out.hand_velocity_age_s = float(msg.velocity_age_s) if hand_velocity_valid else float('nan')
        out.ee_velocity_valid = bool(ee_velocity_valid)
        out.ee_velocity_age_s = float(ee_age) if ee_velocity_valid else float('nan')
        for field, value in ((out.hand_velocity, hand_velocity), (out.ee_velocity, ee_velocity),
                             (out.relative_velocity, relative_velocity)):
            field.x, field.y, field.z = float(value[0]), float(value[1]), float(value[2])

        out.legacy_rate_valid, out.legacy_distance_rate = bool(legacy_valid), float(legacy_rate)
        out.rate_consistency_error = (abs(float(relative_rate) - float(legacy_rate))
                                      if relative_valid and legacy_valid else float('nan'))
        out.rate_valid, out.rate_source, out.rate_degraded = bool(rate_valid), int(source), bool(degraded)
        out.rate_age_s, out.distance_rate, out.closing_velocity = float(rate_age), float(rate), float(closing)
        out.ttc_valid, out.ttc = ttc_valid, float(distance / closing if ttc_valid else 0.0)
        out.legacy_ttc_valid = legacy_ttc_valid
        out.legacy_ttc = float(distance / legacy_closing if legacy_ttc_valid else 0.0)
        self.publish(out)

    # ------------------------------------------------------------ observer
    def _target_state(self, closing):
        """Hysteresis: leaving APPROACHING / RETREATING needs |closing| below exit, entering needs above enter."""
        enter, exit_ = self.enter_threshold, self.exit_threshold
        if self.state == O.APPROACHING:
            return O.RETREATING if closing < -enter else O.HOLD if closing < exit_ else O.APPROACHING
        if self.state == O.RETREATING:
            return O.APPROACHING if closing > enter else O.HOLD if closing > -exit_ else O.RETREATING
        return O.APPROACHING if closing > enter else O.RETREATING if closing < -enter else O.HOLD

    def _confirm(self, target):
        if target == self.state:
            self.pending_state, self.pending_count = None, 0
            return

        self.pending_count = self.pending_count + 1 if self.pending_state == target else 1
        self.pending_state = target
        if self.pending_count >= self.confirm_frames:
            self.state, self.pending_state, self.pending_count = target, None, 0

    def observe(self, msg):
        previous = self.state
        if not msg.valid:
            self.state, self.pending_state, self.pending_count = O.LOST, None, 0
        elif not msg.rate_valid:
            # a short prediction bridge keeps an established state, but creates no new transition
            if not (msg.rate_degraded and self.state in (O.HOLD, O.APPROACHING, O.RETREATING)):
                self.state = O.WARMUP
            self.pending_state, self.pending_count = None, 0
        elif int(msg.rate_source) != D.RATE_SOURCE_RELATIVE_HOLD:  # a held rate is not new evidence
            if self.state in (O.LOST, O.WARMUP):
                self.state = O.HOLD
            self._confirm(self._target_state(float(msg.closing_velocity)))

        out = HandoverObserver()
        out.header = msg.header
        out.state, out.state_changed = int(self.state), bool(self.state != previous)
        out.candidate_state = int(self.pending_state if self.pending_state is not None else self.state)
        out.candidate_count = int(self.pending_count)
        out.distance, out.closing_velocity = float(msg.distance), float(msg.closing_velocity)
        out.rate_valid, out.rate_source = bool(msg.rate_valid), int(msg.rate_source)
        out.rate_degraded, out.rate_age_s = bool(msg.rate_degraded), float(msg.rate_age_s)
        out.tracking_confidence = float(msg.tracking_confidence)
        out.distance_sigma = float(msg.distance_sigma)
        out.ttc_valid = bool(self.state == O.APPROACHING and msg.ttc_valid)
        out.ttc = float(msg.ttc) if out.ttc_valid else 0.0
        self.observer_publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HandoverDistanceEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C / launch shutdown
        pass
    except Exception:
        if rclpy.ok():  # otherwise a shutdown race (context already gone)
            raise
    finally:
        node.reader.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
