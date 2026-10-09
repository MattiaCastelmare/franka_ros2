#!/usr/bin/env python3
"""ACTIVE / STANDBY hand selector (V3.4.1).

LEFT / RIGHT is the anatomical identity from the front-end pose; ACTIVE is the
interaction role: the hand that feeds Kalman -> W75 -> HandState.

Per hand, every frame:
  robot metric  = distance(palm, gripper tip) - horizon * closing speed
  body cues     = arm extension rate (shoulder->wrist / shoulder width) and
                  reach (wrist->torso / body scale)
A hand becomes ENGAGED when it moves towards the robot (onset). The other hand
takes over only with its own onset plus a static advantage (65 % proximity,
35 % reach), or a strong static advantage, confirmed for N frames. A missing
ENGAGED hand keeps its role for interaction_engaged_memory_s.
"""

from collections import deque

import numpy as np
from franka_msgs.msg import HandTrackingRaw
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from franka_experiments.utils.distance_utils import define_control_points, load_robot_config
from franka_experiments.utils.tf_manager import TFManager

LEFT, RIGHT, UNKNOWN = HandTrackingRaw.HAND_LEFT, HandTrackingRaw.HAND_RIGHT, HandTrackingRaw.HAND_UNKNOWN
OTHER = {LEFT: RIGHT, RIGHT: LEFT}
NAME = {LEFT: 'LEFT', RIGHT: 'RIGHT'}
POSE = {LEFT: (11, 15), RIGHT: (12, 16)}  # shoulder, wrist (COCO / MediaPipe pose ids)

PARAMS = ('interaction_switch_confirm_frames', 'interaction_switch_margin_m',
          'interaction_selector_horizon_s', 'interaction_min_closing_m_s',
          'interaction_min_arm_extension_rate_s', 'interaction_strong_closing_m_s',
          'interaction_closing_advantage_m_s', 'interaction_engaged_memory_s',
          'interaction_engaged_onset_advantage', 'interaction_engaged_recovery_advantage',
          'interaction_engaged_retract_m_s')


def linear_rate(history):
    """Least-squares slope of (t, value) samples; None with < 3 samples."""
    if len(history) < 3:
        return None

    t, v = np.array(history, dtype=float).T
    tc = t - t.mean()
    denom = float(tc @ tc)
    if denom <= 1e-12:
        return None

    rate = float(tc @ (v - v.mean()) / denom)
    return rate if np.isfinite(rate) else None


def push(history, now, value):
    """Append to a short history, restarting it after a gap > 0.35 s."""
    if history and not 0.0 < now - history[-1][0] <= 0.35:
        history.clear()
    history.append((now, value))
    return linear_rate(history)


def static_advantage(active_metric, other_metric, active_reach, other_reach):
    """Score > 0 favours the other hand: 65 % robot proximity, 35 % body reach (no velocity).
    Missing cues are dropped and the weights renormalised."""
    weighted = total = 0.0
    cues = 0

    if active_metric is not None and other_metric is not None:
        da, dc = active_metric.get('distance'), other_metric.get('distance')
        if da is not None and dc is not None and np.isfinite(da) and np.isfinite(dc):
            weighted += 0.65 * float(np.clip((da - dc) / 0.30, -1.0, 1.0))
            total += 0.65
            cues += 1

    if (active_reach is not None and other_reach is not None
            and np.isfinite(active_reach) and np.isfinite(other_reach)):
        weighted += 0.35 * float(np.clip((other_reach - active_reach) / 0.75, -1.0, 1.0))
        total += 0.35
        cues += 1

    return (None, 0) if total <= 1e-12 else (weighted / total, cues)


class HandSelector:

    def __init__(self, node, robot_config_path, target_frame, params):
        self.node, self.target_frame = node, target_frame
        for name in PARAMS:
            cast = int if name.endswith('_frames') else float
            setattr(self, name.replace('interaction_', ''), cast(params(name)))
        if self.switch_confirm_frames < 1:
            raise ValueError('interaction_switch_confirm_frames must be >= 1')

        # gripper tip from the robot TF, same control points as distance_handover_estimator
        config = load_robot_config(robot_config_path)
        self.robot_cfg, self.distance_cfg = config['robot'], config['distance']
        self.base_frame = self.robot_cfg['base_frame']
        self.ee_link = self.robot_cfg.get('ee_link', 'fr3_link8')
        segments = [s for s in self.robot_cfg['segments'] if s['end_link'] == self.ee_link]
        if not segments:
            raise RuntimeError('Hand selector: no EE segment found')
        self.ee_segment_links = [segments[-1]['start_link'], segments[-1]['end_link']]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.tf = TFManager(tf_buffer=self.tf_buffer, base_frame=self.base_frame, critical_links=[self.ee_link],
                            cache_max_age_s=float(self.distance_cfg.get('tf_cache_max_age_s', 0.5)),
                            logger=node.get_logger())

        # role state
        self.active = UNKNOWN
        self.pending, self.pending_count = UNKNOWN, 0
        self.engaged, self.engaged_seen_s = UNKNOWN, None
        self.engaged_metric, self.engaged_reach = None, None
        self.distance_history = {LEFT: deque(maxlen=4), RIGHT: deque(maxlen=4)}
        self.arm_history = {LEFT: deque(maxlen=5), RIGHT: deque(maxlen=5)}

    # ------------------------------------------------------------ robot metric
    def _to_base(self, point, stamp):
        point = np.asarray(point, dtype=float)
        if not np.isfinite(point).all():
            return None
        if self.target_frame == self.base_frame:
            return point

        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.target_frame, Time.from_msg(stamp))
        except Exception:
            return None

        t, q = tf.transform.translation, tf.transform.rotation
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix() @ point + np.array([t.x, t.y, t.z])

    def _ee_point(self, stamp):
        transforms = self.tf.lookup_all(self.ee_segment_links, stamp)
        if not transforms:
            return None

        points = [cp for cp in define_control_points(transforms, self.robot_cfg, self.distance_cfg)
                  if cp['end_link'] == self.ee_link]
        if not points:
            return None
        point = np.asarray(max(points, key=lambda cp: cp['cp_idx'])['point'], dtype=float)
        return point if np.isfinite(point).all() else None

    def _metric(self, side, palm, ee, stamp, now):
        if palm is None or ee is None:
            return None
        palm = self._to_base(palm, stamp)
        if palm is None:
            return None

        distance = float(np.linalg.norm(palm - ee))
        if not np.isfinite(distance) or distance <= 0.0:
            return None

        rate = push(self.distance_history[side], now, distance)
        closing = -rate if rate is not None else 0.0
        return {'distance': distance, 'closing': closing, 'rate_valid': rate is not None,
                'metric': distance - self.selector_horizon_s * float(np.clip(closing, -1.0, 1.0))}

    # ------------------------------------------------------------ body cues (normalised image coordinates)
    @staticmethod
    def _pose_xy(pose, index, min_visibility):
        q = pose.landmark[index]
        return np.array([float(q.x), float(q.y)]) if float(q.visibility) >= min_visibility else None

    def _extension_rate(self, pose, side, now):
        """Arm extension |shoulder - wrist| / shoulder width and its rate."""
        if pose is None or side not in POSE:
            return None
        ls, rs = self._pose_xy(pose, 11, 0.5), self._pose_xy(pose, 12, 0.5)
        shoulder, wrist = (self._pose_xy(pose, i, 0.5) for i in POSE[side])
        if ls is None or rs is None or shoulder is None or wrist is None:
            return None

        width = float(np.hypot(*(ls - rs)))
        if not np.isfinite(width) or width <= 1e-4:
            return None
        extension = float(np.hypot(*(wrist - shoulder))) / width
        if not np.isfinite(extension):
            return None
        rate = push(self.arm_history[side], now, extension)
        return {'extension_rate': rate if rate is not None else 0.0, 'rate_valid': rate is not None}

    def _reach(self, pose, side):
        """Wrist distance from the torso centre / body scale (max of shoulder width and torso height)."""
        try:
            return self._reach_unsafe(pose, side)
        except Exception:  # incomplete pose: no reach cue
            return None

    def _reach_unsafe(self, pose, side):
        if pose is None or side not in POSE:
            return None
        ls, rs = self._pose_xy(pose, 11, 0.5), self._pose_xy(pose, 12, 0.5)
        wrist = self._pose_xy(pose, POSE[side][1], 0.5)
        if ls is None or rs is None or wrist is None:
            return None

        width = float(np.hypot(*(ls - rs)))
        if not np.isfinite(width) or width <= 1e-4:
            return None

        # torso centre between shoulders and hips, scale = max(shoulder width, torso height)
        shoulders, scale = 0.5 * (ls + rs), width
        lh, rh = self._pose_xy(pose, 23, 0.4), self._pose_xy(pose, 24, 0.4)
        centre = shoulders
        if lh is not None and rh is not None:
            hips = 0.5 * (lh + rh)
            centre = 0.5 * (shoulders + hips)
            height = float(np.linalg.norm(shoulders - hips))
            if np.isfinite(height) and height > 1e-4:
                scale = max(scale, height)

        reach = float(np.linalg.norm(wrist - centre)) / scale
        return reach if np.isfinite(reach) else None

    # ------------------------------------------------------------ role switching
    def _set_active(self, side, reason):
        if self.active == side:
            return
        self.node.get_logger().info(
            f'Active hand: {NAME.get(self.active, "UNKNOWN")} -> {NAME[side]} [{reason}]')
        self.active, self.pending, self.pending_count = side, UNKNOWN, 0

    def _switch_towards(self, side, reason, now, metric, reach):
        """One more frame of evidence for `side`; after N in a row it becomes ACTIVE and ENGAGED."""
        self.pending_count = self.pending_count + 1 if self.pending == side else 1
        self.pending = side
        if self.pending_count >= self.switch_confirm_frames:
            self._set_active(side, reason)
            self.engaged, self.engaged_seen_s = side, now
            self.engaged_metric = dict(metric) if metric is not None else None
            self.engaged_reach = reach

    def _no_switch(self):
        self.pending, self.pending_count = UNKNOWN, 0

    def _transfer(self, score, cues, onset, onset_threshold):
        """(own onset + static advantage, strong static advantage)."""
        by_onset = onset and score is not None and score >= onset_threshold
        by_recovery = score is not None and cues >= 2 and score >= self.engaged_recovery_advantage
        return by_onset, by_recovery

    # ------------------------------------------------------------ main
    def select(self, result, palm_of, stamp):
        """result: front-end output; palm_of(hand landmarks) -> palm (MCP 5/9/17 mean) or None.
        Returns (active side, its landmarks, standby side, its landmarks)."""
        hands = {LEFT: getattr(result, 'left_hand_landmarks', None),
                 RIGHT: getattr(result, 'right_hand_landmarks', None)}
        pose = getattr(result, 'pose_landmarks', None)
        present = {side for side, lms in hands.items() if lms is not None}
        now = float(stamp.sec) + 1e-9 * float(stamp.nanosec)

        # per-hand cues: robot metric and arm extension
        ee = self._ee_point(stamp)
        metrics = {}
        for side in (LEFT, RIGHT):
            if hands[side] is not None:
                metric = self._metric(side, palm_of(hands[side]), ee, stamp, now)
                if metric is not None:
                    metrics[side] = metric
        body = {side: m for side in (LEFT, RIGHT) if (m := self._extension_rate(pose, side, now)) is not None}

        # bootstrap: the hand nearest the robot (or the only visible one)
        if self.active == UNKNOWN:
            if len(metrics) == 1:
                self._set_active(next(iter(metrics)), 'single robot-valid hand')
            elif len(metrics) == 2:
                self._set_active(min(metrics, key=lambda s: metrics[s]['metric']), 'robot-centric bootstrap')
            elif len(present) == 1:
                self._set_active(next(iter(present)), 'single visible hand bootstrap')

        active, other = self.active, OTHER.get(self.active, UNKNOWN)
        am, om = metrics.get(active), metrics.get(other)
        ab, ob = body.get(active), body.get(other)
        a_reach, o_reach = self._reach(pose, active), self._reach(pose, other)

        valid = lambda m: m is not None and m['rate_valid']
        closing_at_least = lambda m, v: valid(m) and m['closing'] >= v
        extending = lambda b: (b is not None and b['rate_valid']
                               and b['extension_rate'] >= self.min_arm_extension_rate_s)

        # onset = approaching the robot AND (arm extending OR strong approach)
        active_onset = closing_at_least(am, self.min_closing_m_s) and (
            extending(ab) or closing_at_least(am, self.strong_closing_m_s))
        closing_advantage = (om['closing'] - (am['closing'] if valid(am) else 0.0)) if om is not None else 0.0
        o_strong = closing_at_least(om, self.strong_closing_m_s) and (
            not valid(am) or closing_advantage >= self.closing_advantage_m_s)
        o_extending = extending(ob)
        o_onset = closing_at_least(om, self.min_closing_m_s) and (o_extending or o_strong)

        # only an onset creates engagement; while visible the ENGAGED hand refreshes its memory
        if self.engaged not in (LEFT, RIGHT) and active in present and active_onset:
            self.engaged, self.engaged_seen_s = active, now
            self.engaged_metric, self.engaged_reach = (dict(am) if am is not None else None), a_reach

        engaged = self.engaged == active
        if engaged and active in present:
            self.engaged_seen_s = now
            if am is not None:
                self.engaged_metric = dict(am)
            if a_reach is not None:
                self.engaged_reach = a_reach

        if active in present:
            if engaged:
                score, cues = static_advantage(am, om, a_reach, o_reach)
                retracting = valid(am) and am['closing'] <= -self.engaged_retract_m_s
                by_onset, by_recovery = self._transfer(
                    score, cues, o_onset, 0.10 if retracting else self.engaged_onset_advantage)
                far_ahead = (am is not None and om is not None
                             and np.isfinite(am['distance']) and np.isfinite(om['distance'])
                             and am['distance'] - om['distance'] >= 2.0 * self.switch_margin_m)
                by_recovery = by_recovery and (o_onset or far_ahead)
                if by_onset or by_recovery:
                    self._switch_towards(
                        other,
                        f'ENGAGED transfer onset={int(by_onset)} recovery={int(by_recovery)} static={score:.3f}',
                        now, om, o_reach)
                else:
                    self._no_switch()
            elif (om is not None and o_onset
                  and (am is None or om['metric'] + self.switch_margin_m < am['metric'])):
                arm_rate = ob['extension_rate'] if ob is not None else 0.0
                self._switch_towards(
                    other,
                    f'consensus onset d={om["distance"]:.3f}m closing={om["closing"]:.3f}m/s '
                    f'arm_rate={arm_rate:.3f}/s',
                    now, om, o_reach)
            else:
                self._no_switch()
        else:
            # ACTIVE missing: a short dropout of the ENGAGED hand does not end its role
            age = now - self.engaged_seen_s if (engaged and self.engaged_seen_s is not None) else float('inf')
            if age <= self.engaged_memory_s:
                score, cues = static_advantage(self.engaged_metric, om, self.engaged_reach, o_reach)
                by_onset, by_recovery = self._transfer(score, cues, o_onset, self.engaged_onset_advantage)
                if other in present and (by_onset or by_recovery):
                    self._switch_towards(other, f'ENGAGED dropout transfer age={age:.3f}s static={score:.3f}',
                                         now, om, o_reach)
                else:
                    self._no_switch()
            else:
                score, cues = (static_advantage(self.engaged_metric, om, self.engaged_reach, o_reach)
                               if (other in present and om is not None) else (None, 0))
                by_onset, by_recovery = self._transfer(score, cues, o_onset, self.engaged_onset_advantage)
                strong = o_strong and o_extending
                if other in present and (by_onset or by_recovery or strong):
                    self._switch_towards(
                        other,
                        f'ENGAGED long-dropout transfer onset={int(by_onset)} '
                        f'recovery={int(by_recovery)} strong={int(strong)}',
                        now, om, o_reach)
                else:
                    self._no_switch()

        active, other = self.active, OTHER.get(self.active, UNKNOWN)
        return active, hands.get(active), other, hands.get(other)
