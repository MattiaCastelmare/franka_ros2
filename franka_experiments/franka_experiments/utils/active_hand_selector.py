#!/usr/bin/env python3
"""
ACTIVE/STANDBY interaction-hand selector.

Contains only the validated selector logic used by
HumanHandTracker. Perception, RGB-D reconstruction and ROS
publishing remain in the tracker node.
"""

import numpy as np
from franka_msgs.msg import HandTrackingRaw

from scipy.spatial.transform import Rotation
from rclpy.time import Time
from franka_experiments.utils.distance_utils import define_control_points

class ActiveHandSelectorMixin:
    """Stateful ACTIVE/STANDBY selector used by HumanHandTracker."""


    def _selector_body_extension(
        self,
        result,
        side,
    ):
        """
        Scale-free arm extension from Holistic pose:

            shoulder -> wrist distance
            --------------------------
                 shoulder width

        Whole-body translation moves shoulder and wrist
        together, so it contributes much less than a true
        arm extension.
        """
        pose = getattr(
            result,
            'pose_landmarks',
            None,
        )
        if pose is None:
            return None
        lm = pose.landmark
        # MediaPipe Pose:
        # 11 LEFT shoulder
        # 12 RIGHT shoulder
        # 15 LEFT wrist
        # 16 RIGHT wrist
        if side == HandTrackingRaw.HAND_LEFT:
            shoulder_id = 11
            wrist_id = 15
        elif side == HandTrackingRaw.HAND_RIGHT:
            shoulder_id = 12
            wrist_id = 16
        else:
            return None
        left_shoulder = lm[11]
        right_shoulder = lm[12]
        shoulder = lm[shoulder_id]
        wrist = lm[wrist_id]
        min_visibility = 0.50
        if (
            float(left_shoulder.visibility)
            < min_visibility
            or
            float(right_shoulder.visibility)
            < min_visibility
            or
            float(shoulder.visibility)
            < min_visibility
            or
            float(wrist.visibility)
            < min_visibility
        ):
            return None
        shoulder_width = float(
            np.hypot(
                float(left_shoulder.x)
                -
                float(right_shoulder.x),
                float(left_shoulder.y)
                -
                float(right_shoulder.y),
            )
        )
        if (
            not np.isfinite(shoulder_width)
            or
            shoulder_width <= 1e-4
        ):
            return None
        arm_length = float(
            np.hypot(
                float(wrist.x)
                -
                float(shoulder.x),
                float(wrist.y)
                -
                float(shoulder.y),
            )
        )
        extension = (
            arm_length
            /
            shoulder_width
        )
        if not np.isfinite(extension):
            return None
        return extension

    def _selector_body_motion(
        self,
        result,
        side,
        stamp,
    ):
        extension = (
            self._selector_body_extension(
                result,
                side,
            )
        )
        if extension is None:
            return None
        now = self._selector_stamp_s(
            stamp
        )
        history = (
            self._interaction_arm_history[
                side
            ]
        )
        if history:
            dt = (
                now
                -
                history[-1][0]
            )
            if (
                dt <= 0.0
                or
                dt > 0.35
            ):
                history.clear()
        history.append(
            (
                now,
                extension,
            )
        )
        extension_rate = (
            self._selector_linear_rate(
                history
            )
        )
        return {
            'extension':
                float(extension),
            'extension_rate':
                (
                    float(extension_rate)
                    if extension_rate is not None
                    else 0.0
                ),
            'rate_valid':
                bool(
                    extension_rate
                    is not None
                ),
        }

    def _selector_engaged_body_reach(
        self,
        result,
        side,
    ):
        """
        Body-relative wrist reach.

        This intentionally measures configuration rather than
        global wrist speed:

            distance(wrist, torso centre)
            -----------------------------
                  shoulder width

        Whole-body translation therefore affects it much less
        than absolute wrist motion.

        This is an interaction-context cue, not a tracker or
        hand-pose estimator.
        """
        pose = getattr(
            result,
            'pose_landmarks',
            None,
        )
        if pose is None:
            return None
        try:
            lm = pose.landmark
            if side == HandTrackingRaw.HAND_LEFT:
                wrist_id = 15
            elif side == HandTrackingRaw.HAND_RIGHT:
                wrist_id = 16
            else:
                return None
            ls = lm[11]
            rs = lm[12]
            wrist = lm[wrist_id]
            if (
                float(ls.visibility) < 0.50
                or
                float(rs.visibility) < 0.50
                or
                float(wrist.visibility) < 0.50
            ):
                return None
            shoulder_width = float(
                np.hypot(
                    float(ls.x) - float(rs.x),
                    float(ls.y) - float(rs.y),
                )
            )
            if (
                not np.isfinite(shoulder_width)
                or
                shoulder_width <= 1e-4
            ):
                return None
            shoulder_mid = np.array(
                [
                    0.5 * (
                        float(ls.x)
                        +
                        float(rs.x)
                    ),
                    0.5 * (
                        float(ls.y)
                        +
                        float(rs.y)
                    ),
                ],
                dtype=float,
            )
            lh = lm[23]
            rh = lm[24]
            if (
                float(lh.visibility) >= 0.40
                and
                float(rh.visibility) >= 0.40
            ):
                hip_mid = np.array(
                    [
                        0.5 * (
                            float(lh.x)
                            +
                            float(rh.x)
                        ),
                        0.5 * (
                            float(lh.y)
                            +
                            float(rh.y)
                        ),
                    ],
                    dtype=float,
                )
                torso_center = (
                    0.5
                    *
                    (
                        shoulder_mid
                        +
                        hip_mid
                    )
                )
            else:
                torso_center = shoulder_mid
            wrist_xy = np.array(
                [
                    float(wrist.x),
                    float(wrist.y),
                ],
                dtype=float,
            )
            # V341_ROBUST_BODY_SCALE
            #
            # Shoulder span alone is perspective-sensitive
            # when the torso is viewed in profile.
            body_scale = shoulder_width
            try:
                lh_scale = lm[23]
                rh_scale = lm[24]
                if (
                    float(lh_scale.visibility) >= 0.40
                    and
                    float(rh_scale.visibility) >= 0.40
                ):
                    hip_mid_scale = np.array(
                        [
                            0.5
                            *
                            (
                                float(lh_scale.x)
                                +
                                float(rh_scale.x)
                            ),
                            0.5
                            *
                            (
                                float(lh_scale.y)
                                +
                                float(rh_scale.y)
                            ),
                        ],
                        dtype=float,
                    )
                    torso_height = float(
                        np.linalg.norm(
                            shoulder_mid
                            -
                            hip_mid_scale
                        )
                    )
                    if (
                        np.isfinite(torso_height)
                        and
                        torso_height > 1e-4
                    ):
                        body_scale = max(
                            body_scale,
                            torso_height,
                        )
            except Exception:
                pass
            reach = float(
                np.linalg.norm(
                    wrist_xy
                    -
                    torso_center
                )
                /
                body_scale
            )
            if not np.isfinite(reach):
                return None
            return reach
        except Exception:
            return None

    @staticmethod
    def _selector_engaged_static_advantage(
        active_metric,
        challenger_metric,
        active_reach,
        challenger_reach,
    ):
        """
        Positive score favours challenger.

        Deliberately excludes velocity.

        STATIC INTERACTION CONTEXT:
          65% robot proximity
          35% body-relative reach

        Missing cues are ignored and remaining weights are
        renormalized.

        No absolute robot distance is used.
        """
        weighted = 0.0
        total_weight = 0.0
        cue_count = 0
        # ROBOT PROXIMITY
        #
        # challenger closer -> positive
        if (
            active_metric is not None
            and
            challenger_metric is not None
        ):
            da = active_metric.get(
                'distance'
            )
            dc = challenger_metric.get(
                'distance'
            )
            if (
                da is not None
                and
                dc is not None
                and
                np.isfinite(da)
                and
                np.isfinite(dc)
            ):
                cue = float(
                    np.clip(
                        (
                            float(da)
                            -
                            float(dc)
                        )
                        /
                        0.30,
                        -1.0,
                        1.0,
                    )
                )
                weight = 0.65
                weighted += (
                    weight
                    *
                    cue
                )
                total_weight += weight
                cue_count += 1
        # BODY-RELATIVE REACH
        #
        # challenger farther from torso -> positive
        if (
            active_reach is not None
            and
            challenger_reach is not None
            and
            np.isfinite(active_reach)
            and
            np.isfinite(challenger_reach)
        ):
            cue = float(
                np.clip(
                    (
                        float(challenger_reach)
                        -
                        float(active_reach)
                    )
                    /
                    0.75,
                    -1.0,
                    1.0,
                )
            )
            weight = 0.35
            weighted += (
                weight
                *
                cue
            )
            total_weight += weight
            cue_count += 1
        if total_weight <= 1e-12:
            return None
        return {
            'score':
                float(
                    weighted
                    /
                    total_weight
                ),
            'cue_count':
                int(cue_count),
        }

    def _selector_candidate_palm(
        self,
        hand_landmarks,
        image_shape,
        depth_image,
        depth_encoding,
    ):
        """
        Lightweight standby/active candidate.

        Only MCP 5/9/17 are reconstructed.

        No Kalman.
        No W75.
        No geometry.
        """
        if hand_landmarks is None:
            return None
        hand = hand_landmarks.landmark
        points_camera = {}
        for landmark_id in (
            5,
            9,
            17,
        ):
            points_camera[
                landmark_id
            ] = self.landmark_to_3d(
                hand[
                    landmark_id
                ],
                image_shape,
                depth_image,
                depth_encoding,
            )
        valid_depths = [
            point[2]
            for point
            in points_camera.values()
            if point is not None
        ]
        if len(valid_depths) < 2:
            return None
        reference_depth = float(
            np.median(
                valid_depths
            )
        )
        consistent = [
            depth
            for depth in valid_depths
            if abs(
                depth
                -
                reference_depth
            )
            <=
            0.15
        ]
        if len(consistent) >= 2:
            reference_depth = float(
                np.median(
                    consistent
                )
            )
            for landmark_id in (
                5,
                9,
                17,
            ):
                if (
                    points_camera[
                        landmark_id
                    ]
                    is None
                ):
                    points_camera[
                        landmark_id
                    ] = self.landmark_to_3d(
                        hand[
                            landmark_id
                        ],
                        image_shape,
                        depth_image,
                        depth_encoding,
                        fallback_depth=(
                            reference_depth
                        ),
                    )
        if any(
            points_camera[
                landmark_id
            ]
            is None
            for landmark_id
            in (
                5,
                9,
                17,
            )
        ):
            return None
        points_target = np.asarray(
            [
                self.apply_transform(
                    points_camera[
                        landmark_id
                    ]
                )
                for landmark_id
                in (
                    5,
                    9,
                    17,
                )
            ],
            dtype=float,
        )
        if not np.isfinite(
            points_target
        ).all():
            return None
        return np.mean(
            points_target,
            axis=0,
        )

    def _selector_metric(
        self,
        side,
        palm_target,
        ee_point,
        stamp,
    ):
        if (
            palm_target is None
            or ee_point is None
        ):
            return None
        palm = self._selector_to_robot_base(
            palm_target,
            stamp,
        )
        if palm is None:
            return None
        distance = float(
            np.linalg.norm(
                palm
                -
                ee_point
            )
        )
        if (
            not np.isfinite(distance)
            or
            distance <= 0.0
        ):
            return None
        now = self._selector_stamp_s(
            stamp
        )
        history = (
            self._interaction_distance_history[
                side
            ]
        )
        if history:
            dt = (
                now
                -
                history[-1][0]
            )
            if (
                dt <= 0.0
                or
                dt > 0.35
            ):
                history.clear()
        history.append(
            (
                now,
                distance,
            )
        )
        distance_rate = (
            self._selector_linear_rate(
                history
            )
        )
        rate_valid = bool(
            distance_rate is not None
        )
        closing = (
            -float(
                distance_rate
            )
            if rate_valid
            else
            0.0
        )
        # Prevent one noisy derivative from dominating role
        # selection.
        closing_for_metric = float(
            np.clip(
                closing,
                -1.0,
                1.0,
            )
        )
        metric = (
            distance
            -
            self.interaction_selector_horizon_s
            *
            closing_for_metric
        )
        return {
            'distance':
                distance,
            'closing':
                closing,
            'rate_valid':
                rate_valid,
            'metric':
                metric,
        }

    def _set_active_side(
        self,
        side,
        reason,
    ):
        old = int(
            self.active_hand_side
        )
        side = int(
            side
        )
        if old == side:
            return
        self.active_hand_side = side
        self._pending_active_side = (
            HandTrackingRaw.HAND_UNKNOWN
        )
        self._pending_active_count = 0
        self.get_logger().info(
            'INTERACTION HAND V2: '
            f'{self._physical_side_name(old)} -> '
            f'{self._physical_side_name(side)} '
            f'[{reason}]'
        )

    def _accumulate_switch(
        self,
        side,
        reason,
    ):
        if (
            self._pending_active_side
            !=
            side
        ):
            self._pending_active_side = (
                side
            )
            self._pending_active_count = 1
        else:
            self._pending_active_count += 1
        if (
            self._pending_active_count
            >=
            self.interaction_switch_confirm_frames
        ):
            self._set_active_side(
                side,
                reason,
            )
            return True
        return False

    def _clear_switch_candidate(
        self,
    ):
        self._pending_active_side = (
            HandTrackingRaw.HAND_UNKNOWN
        )
        self._pending_active_count = 0

    def _update_engaged_memory_after_switch(
        self,
        selector_now_s,
        other_metric,
        other_reach,
    ):
        self._interaction_engaged_side = (
            int(
                self.active_hand_side
            )
        )
        self._interaction_engaged_last_seen_s = (
            selector_now_s
        )
        self._interaction_engaged_last_metric = (
            dict(other_metric)
            if other_metric is not None
            else None
        )
        self._interaction_engaged_last_reach = (
            other_reach
        )

    def _accumulate_engaged_switch(
        self,
        side,
        reason,
        old_side,
        selector_now_s,
        other_metric,
        other_reach,
    ):
        self._accumulate_switch(
            side,
            reason,
        )
        if (
            int(
                self.active_hand_side
            )
            !=
            old_side
        ):
            self._update_engaged_memory_after_switch(
                selector_now_s,
                other_metric,
                other_reach,
            )


    @staticmethod
    def _static_score_cues(cached_static):
        if cached_static is None:
            return None, 0
        return (
            cached_static["score"],
            cached_static["cue_count"],
        )

    def _select_active_holistic_hand(
        self,
        result,
        image_shape,
        depth_image,
        depth_encoding,
        stamp,
    ):
        """
        Interaction Selector V2.

        Identity:
            Holistic anatomical LEFT / RIGHT.

        Role:
            robot-centric lightweight 3-D metric.

        metric =
            current EE distance
            - short_horizon * closing_velocity

        Lower metric = stronger interaction evidence.

        ACTIVE remains sticky unless:
          - the other hand is clearly better for consecutive
            frames, or
          - ACTIVE disappears and the remaining hand provides
            persistent interaction evidence.

        The standby hand NEVER enters Kalman/W75 here.
        """
        hands = {
            HandTrackingRaw.HAND_LEFT:
                getattr(
                    result,
                    'left_hand_landmarks',
                    None,
                ),
            HandTrackingRaw.HAND_RIGHT:
                getattr(
                    result,
                    'right_hand_landmarks',
                    None,
                ),
        }
        present = {
            side
            for side, landmarks
            in hands.items()
            if landmarks is not None
        }
        previous_present = set(
            self._visible_hand_sides
        )
        ee_point = self._selector_ee_point(
            stamp
        )
        metrics = {}
        for side in (
            HandTrackingRaw.HAND_LEFT,
            HandTrackingRaw.HAND_RIGHT,
        ):
            landmarks = hands[
                side
            ]
            if landmarks is None:
                continue
            palm_target = (
                self._selector_candidate_palm(
                    landmarks,
                    image_shape,
                    depth_image,
                    depth_encoding,
                )
            )
            metric = (
                self._selector_metric(
                    side,
                    palm_target,
                    ee_point,
                    stamp,
                )
            )
            if metric is not None:
                metrics[
                    side
                ] = metric
        # V3 BODY-RELATIVE INTERACTION MOTION
        body_motion = {}
        for side in (
            HandTrackingRaw.HAND_LEFT,
            HandTrackingRaw.HAND_RIGHT,
        ):
            motion = (
                self._selector_body_motion(
                    result,
                    side,
                    stamp,
                )
            )
            if motion is not None:
                body_motion[
                    side
                ] = motion
        # BOOTSTRAP
        if (
            self.active_hand_side
            ==
            HandTrackingRaw.HAND_UNKNOWN
        ):
            metric_sides = list(
                metrics.keys()
            )
            if len(metric_sides) == 1:
                self._set_active_side(
                    metric_sides[0],
                    'single robot-valid hand',
                )
            elif len(metric_sides) == 2:
                best = min(
                    metric_sides,
                    key=lambda side:
                        metrics[
                            side
                        ][
                            'metric'
                        ],
                )
                self._set_active_side(
                    best,
                    'robot-centric bootstrap',
                )
            elif len(present) == 1:
                # TF/depth not ready yet:
                # allow only an unambiguous single hand.
                self._set_active_side(
                    next(
                        iter(present)
                    ),
                    'single visible hand bootstrap',
                )
        # LOCKED ACTIVE ROLE
        active = int(
            self.active_hand_side
        )
        other = self._other_physical_side(
            active
        )
        active_present = (
            active in present
        )
        other_present = (
            other in present
        )
        active_metric = metrics.get(
            active
        )
        other_metric = metrics.get(
            other
        )
        # V3.4 ENGAGED LOCK
        #
        # Velocity:
        #   mainly ENTER / TRANSFER cue
        #
        # Proximity + body reach + memory:
        #   STAY ENGAGED cues
        selector_now_s = (
            self._selector_stamp_s(
                stamp
            )
        )
        # Lazy state initialization.
        #
        # This avoids adding fragile startup dependencies to
        # __init__; perception can start exactly as before.
        if not hasattr(
            self,
            '_interaction_engaged_side',
        ):
            self._interaction_engaged_side = (
                HandTrackingRaw.HAND_UNKNOWN
            )
            self._interaction_engaged_last_seen_s = None
            self._interaction_engaged_last_metric = None
            self._interaction_engaged_last_reach = None
        active_reach = (
            self._selector_engaged_body_reach(
                result,
                active,
            )
        )
        other_reach = (
            self._selector_engaged_body_reach(
                result,
                other,
            )
        )
        active_body = (
            body_motion.get(
                active
            )
        )
        other_body = (
            body_motion.get(
                other
            )
        )
        # CURRENT ACTIVE ONSET
        active_approaching = bool(
            active_metric is not None
            and
            active_metric[
                'rate_valid'
            ]
            and
            active_metric[
                'closing'
            ]
            >=
            self.interaction_min_closing_m_s
        )
        active_extending = bool(
            active_body is not None
            and
            active_body[
                'rate_valid'
            ]
            and
            active_body[
                'extension_rate'
            ]
            >=
            self.interaction_min_arm_extension_rate_s
        )
        active_strong_approach = bool(
            active_metric is not None
            and
            active_metric[
                'rate_valid'
            ]
            and
            active_metric[
                'closing'
            ]
            >=
            self.interaction_strong_closing_m_s
        )
        active_onset = bool(
            active_approaching
            and
            (
                active_extending
                or
                active_strong_approach
            )
        )
        # CHALLENGER ONSET
        challenger_approaching = bool(
            other_metric is not None
            and
            other_metric[
                'rate_valid'
            ]
            and
            other_metric[
                'closing'
            ]
            >=
            self.interaction_min_closing_m_s
        )
        challenger_extending = bool(
            other_body is not None
            and
            other_body[
                'rate_valid'
            ]
            and
            other_body[
                'extension_rate'
            ]
            >=
            self.interaction_min_arm_extension_rate_s
        )
        active_closing = 0.0
        active_rate_valid = False
        if (
            active_metric is not None
            and
            active_metric[
                'rate_valid'
            ]
        ):
            active_closing = float(
                active_metric[
                    'closing'
                ]
            )
            active_rate_valid = True
        closing_advantage = (
            float(
                other_metric[
                    'closing'
                ]
            )
            -
            active_closing
            if other_metric is not None
            else 0.0
        )
        challenger_strong_approach = bool(
            other_metric is not None
            and
            other_metric[
                'rate_valid'
            ]
            and
            other_metric[
                'closing'
            ]
            >=
            self.interaction_strong_closing_m_s
            and
            (
                not active_rate_valid
                or
                closing_advantage
                >=
                self.interaction_closing_advantage_m_s
            )
        )
        challenger_onset = bool(
            challenger_approaching
            and
            (
                challenger_extending
                or
                challenger_strong_approach
            )
        )
        # STATIC CONTEXT
        static_comparison = (
            self._selector_engaged_static_advantage(
                active_metric,
                other_metric,
                active_reach,
                other_reach,
            )
        )
        # ENGAGEMENT ACQUISITION
        #
        # Only movement-related onset can CREATE engagement.
        # A stationary hand is not arbitrarily promoted.
        if (
            self._interaction_engaged_side
            not in (
                HandTrackingRaw.HAND_LEFT,
                HandTrackingRaw.HAND_RIGHT,
            )
            and
            active_present
            and
            active_onset
        ):
            self._interaction_engaged_side = (
                active
            )
            self._interaction_engaged_last_seen_s = (
                selector_now_s
            )
            self._interaction_engaged_last_metric = (
                dict(active_metric)
                if active_metric is not None
                else None
            )
            self._interaction_engaged_last_reach = (
                active_reach
            )
        # KEEP CURRENT ENGAGED PHYSICAL HAND ALIVE
        active_engaged = bool(
            self._interaction_engaged_side
            ==
            active
        )
        if (
            active_engaged
            and
            active_present
        ):
            self._interaction_engaged_last_seen_s = (
                selector_now_s
            )
            if active_metric is not None:
                self._interaction_engaged_last_metric = (
                    dict(active_metric)
                )
            if active_reach is not None:
                self._interaction_engaged_last_reach = (
                    active_reach
                )
        # ACTIVE PRESENT
        if active_present:
            active_retracting = bool(
                active_metric is not None
                and
                active_metric[
                    'rate_valid'
                ]
                and
                active_metric[
                    'closing'
                ]
                <=
                -self.interaction_engaged_retract_m_s
            )
            # CASE A: ACTIVE IS ENGAGED
            if active_engaged:
                static_score = (
                    static_comparison[
                        'score'
                    ]
                    if static_comparison is not None
                    else None
                )
                static_cues = (
                    static_comparison[
                        'cue_count'
                    ]
                    if static_comparison is not None
                    else 0
                )
                # Normal interaction transfer.
                #
                # A new hand must show its OWN onset and also
                # look statically more interaction-like.
                onset_threshold = (
                    0.10
                    if active_retracting
                    else
                    self.interaction_engaged_onset_advantage
                )
                transfer_by_onset = bool(
                    challenger_onset
                    and
                    static_score is not None
                    and
                    static_score
                    >=
                    onset_threshold
                )
                # Static recovery.
                #
                # Allows recovery if we are already locked to
                # the wrong hand and the true interaction hand
                # has become stationary.
                #
                # Require BOTH static cues and strong
                # persistent advantage.
                distance_recovery_strong = False
                if (
                    active_metric is not None
                    and
                    other_metric is not None
                ):
                    active_distance = (
                        active_metric.get(
                            'distance'
                        )
                    )
                    challenger_distance = (
                        other_metric.get(
                            'distance'
                        )
                    )
                    if (
                        active_distance is not None
                        and
                        challenger_distance is not None
                        and
                        np.isfinite(
                            active_distance
                        )
                        and
                        np.isfinite(
                            challenger_distance
                        )
                    ):
                        distance_recovery_strong = bool(
                            (
                                float(
                                    active_distance
                                )
                                -
                                float(
                                    challenger_distance
                                )
                            )
                            >=
                            (
                                2.0
                                *
                                self.interaction_switch_margin_m
                            )
                        )
                transfer_by_recovery = bool(
                    static_score is not None
                    and
                    static_cues >= 2
                    and
                    static_score
                    >=
                    self.interaction_engaged_recovery_advantage
                    and
                    (
                        challenger_onset
                        or
                        distance_recovery_strong
                    )
                )
                challenger_better = bool(
                    transfer_by_onset
                    or
                    transfer_by_recovery
                )
                if challenger_better:
                    reason = (
                        'ENGAGED transfer '
                        f'onset={int(transfer_by_onset)} '
                        f'recovery={int(transfer_by_recovery)} '
                        f'static='
                        f'{static_score:.3f}'
                    )
                    old_side = int(
                        self.active_hand_side
                    )
                    self._accumulate_engaged_switch(
                        other,
                        reason,
                        old_side,
                        selector_now_s,
                        other_metric,
                        other_reach,
                    )
                else:
                    self._clear_switch_candidate()
            # CASE B: NOT ENGAGED
            #
            # Keep original V3.1 policy unchanged.
            else:
                challenger_better = bool(
                    other_metric is not None
                    and
                    challenger_onset
                    and
                    (
                        active_metric is None
                        or
                        other_metric[
                            'metric'
                        ]
                        +
                        self.interaction_switch_margin_m
                        <
                        active_metric[
                            'metric'
                        ]
                    )
                )
                if challenger_better:
                    arm_rate = (
                        other_body[
                            'extension_rate'
                        ]
                        if other_body is not None
                        else 0.0
                    )
                    reason = (
                        'consensus onset '
                        f'd={other_metric["distance"]:.3f}m '
                        f'closing='
                        f'{other_metric["closing"]:.3f}m/s '
                        f'arm_rate={arm_rate:.3f}/s'
                    )
                    old_side = int(
                        self.active_hand_side
                    )
                    self._accumulate_engaged_switch(
                        other,
                        reason,
                        old_side,
                        selector_now_s,
                        other_metric,
                        other_reach,
                    )
                else:
                    self._clear_switch_candidate()
        # ACTIVE MISSING
        else:
            # ENGAGED dropout memory
            if (
                self._interaction_engaged_side
                ==
                active
                and
                self._interaction_engaged_last_seen_s
                is not None
            ):
                missing_age = (
                    selector_now_s
                    -
                    self._interaction_engaged_last_seen_s
                )
                engaged_memory_valid = bool(
                    missing_age
                    <=
                    self.interaction_engaged_memory_s
                )
            else:
                missing_age = float(
                    'inf'
                )
                engaged_memory_valid = False
            # ACTIVE was ENGAGED and dropout is still short.
            if engaged_memory_valid:
                cached_static = (
                    self._selector_engaged_static_advantage(
                        self._interaction_engaged_last_metric,
                        other_metric,
                        self._interaction_engaged_last_reach,
                        other_reach,
                    )
                )
                static_score, static_cues = self._static_score_cues(cached_static)
                switch_during_dropout = bool(
                    other_present
                    and
                    (
                        (
                            challenger_onset
                            and
                            static_score is not None
                            and
                            static_score
                            >=
                            self.interaction_engaged_onset_advantage
                        )
                        or
                        (
                            static_score is not None
                            and
                            static_cues >= 2
                            and
                            static_score
                            >=
                            self.interaction_engaged_recovery_advantage
                        )
                    )
                )
                if switch_during_dropout:
                    old_side = int(
                        self.active_hand_side
                    )
                    self._accumulate_engaged_switch(
                        other,
                        f'ENGAGED dropout transfer age={missing_age:.3f}s static={static_score:.3f}',
                        old_side, selector_now_s, other_metric, other_reach,)
                else:
                    # Key behaviour:
                    #
                    # one or a few missing frames DO NOT
                    # terminate physical interaction role.
                    self._clear_switch_candidate()
            # V3.4.1:
            # uncertainty after a longer ENGAGED dropout.
            #
            # Hand disappearance is NOT evidence that the
            # standby has become the interaction hand.
            else:
                cached_static = (
                    self._selector_engaged_static_advantage(self._interaction_engaged_last_metric,
                        other_metric, self._interaction_engaged_last_reach,
                        other_reach,) if (other_present and other_metric is not None) else None)
                static_score, static_cues = self._static_score_cues(cached_static)
                # A. Normal new interaction onset.
                supported_onset = bool(
                    other_present and challenger_onset and static_score is not None
                    and static_score >= self.interaction_engaged_onset_advantage)
                # B. Strong static recovery.
                #
                # Useful if the true interaction hand is
                # already stationary.
                supported_recovery = bool(
                    other_present and static_score is not None and static_cues >= 2
                    and static_score >= self.interaction_engaged_recovery_advantage)
                # C. Very clear new handover movement after
                #    long dropout.
                strong_new_takeover = bool(
                    other_present and challenger_strong_approach and challenger_extending)
                transfer_after_dropout = bool(
                    supported_onset or supported_recovery or strong_new_takeover)
                if transfer_after_dropout:
                    old_side = int(self.active_hand_side)
                    reason = ('ENGAGED long-dropout transfer ' f'onset={int(supported_onset)} '
                        f'recovery={int(supported_recovery)} ' f'strong={int(strong_new_takeover)}')
                    self._accumulate_engaged_switch(
                        other, reason, old_side, selector_now_s, other_metric, other_reach,)
                else:
                    # Preserve ROLE identity only.
                    #
                    # No stale hand position is published:
                    # downstream perception still sees the
                    # normal missing measurement.
                    self._clear_switch_candidate()
        # RETURN CURRENT ROLE
        active = int(self.active_hand_side)
        other = self._other_physical_side(active)
        active_landmarks = hands.get(active)
        standby_landmarks = hands.get(other)
        self._visible_hand_sides = (present)
        return (active, active_landmarks, other, standby_landmarks,)

    @staticmethod
    def _selector_stamp_s(stamp,):
        return (float(stamp.sec) + 1e-9 * float(stamp.nanosec))

    @staticmethod
    def _selector_linear_rate(history,):
        if len(history) < 3:
            return None
        times = np.asarray([item[0] for item in history], dtype=float,)
        values = np.asarray([item[1] for item in history], dtype=float,)
        tc = (times - np.mean(times))
        denom = float(np.dot(tc, tc,))
        if denom <= 1e-12:
            return None
        rate = float(np.dot(tc, values - np.mean(values),) / denom)
        if not np.isfinite(rate):
            return None
        return rate

    def _selector_to_robot_base(self, point, stamp,):
        point = np.asarray(point, dtype=float,)
        if not np.isfinite(point).all():
            return None
        if (self.target_frame == self.selector_base_frame):
            return point
        try:
            tf = (self.selector_tf_buffer.lookup_transform(
                    self.selector_base_frame, self.target_frame, Time.from_msg(stamp),))
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w,]).as_matrix()
        translation = np.asarray([t.x, t.y, t.z,], dtype=float,)
        return (rotation
            @ point + translation)

    def _selector_ee_point(self, stamp,):
        transforms = (self.selector_tf_manager.lookup_all(self.selector_ee_segment_links, stamp,))
        if not transforms:
            return None
        control_points = (
            define_control_points(transforms, self.selector_robot_cfg, self.selector_distance_cfg,))
        ee_points = [cp for cp in control_points if cp['end_link'] == self.selector_ee_link]
        if not ee_points:
            return None
        ee_cp = max(ee_points, key=lambda cp: cp['cp_idx'],)
        point = np.asarray(ee_cp['point'], dtype=float,)
        if not np.isfinite(point).all():
            return None
        return point

    @staticmethod
    def _physical_side_name(side):
        if side == HandTrackingRaw.HAND_LEFT:
            return 'LEFT'
        if side == HandTrackingRaw.HAND_RIGHT:
            return 'RIGHT'
        return 'UNKNOWN'

    @staticmethod
    def _other_physical_side(side):
        if side == HandTrackingRaw.HAND_LEFT:
            return HandTrackingRaw.HAND_RIGHT
        if side == HandTrackingRaw.HAND_RIGHT:
            return HandTrackingRaw.HAND_LEFT
        return HandTrackingRaw.HAND_UNKNOWN
