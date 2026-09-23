#!/usr/bin/env python3

import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node

from franka_msgs.msg import HandState, HandTrackingFiltered

from franka_experiments.utils.palm import (
    SOURCE_NONE,
    SOURCE_UPDATED,
    SOURCE_HOLD,
    W75VelocityEstimator,
    PREDICTION_CHI3_Q975_RADIUS,
    prediction_q975_error_m,
)

from franka_experiments.utils.params import (
    load_hand_tracking_defaults,
    parameter_value,
)

_HAND_STATE_DEFAULTS = load_hand_tracking_defaults("hand_state_estimator")


class HandStateEstimator(Node):

    PALM_INDICES = (1, 2, 3)

    # Palm orientation: body-aware physical side + RGBD5 PCA sign,
    # GUARD60 confirmation and One-Euro temporal filtering.

    NORMAL_INNOVATION_GATE_DEG = 60.0
    NORMAL_CONFIRM_ANGLE_DEG = 35.0
    NORMAL_RESET_GAP_S = 0.30

    NORMAL_MIN_CUTOFF = 1.0
    NORMAL_BETA = 1.0
    NORMAL_D_CUTOFF = 1.0


    def __init__(self):
        super().__init__('hand_state_estimator')
        # Absolute limit of the experimentally validated region.
        #
        # This is NOT an error threshold:
        # uncertainty grows continuously inside this interval.
        self.reacquire_frames = int(
            parameter_value(self, _HAND_STATE_DEFAULTS, "reacquire_frames")
        )
        self.confidence_sigma = float(
            parameter_value(self, _HAND_STATE_DEFAULTS, "confidence_sigma")
        )
        self.stability_speed = float(
            parameter_value(self, _HAND_STATE_DEFAULTS, "stability_speed")
        )
        self.lost_timeout = float(
            parameter_value(self, _HAND_STATE_DEFAULTS, "lost_timeout")
        )
        self.max_position_age_s = float(
            parameter_value(self, _HAND_STATE_DEFAULTS, "max_position_age_s")
        )
        self.enable_prediction_bridge = bool(
            parameter_value(self, _HAND_STATE_DEFAULTS, "enable_prediction_bridge")
        )
        self.max_prediction_age_s = float(
            parameter_value(self, _HAND_STATE_DEFAULTS, "max_prediction_age_s")
        )
        if self.max_prediction_age_s <= 0.0:
            raise ValueError(
                'max_prediction_age_s must be > 0'
            )
        self.velocity_mode = str(
            parameter_value(self, _HAND_STATE_DEFAULTS, "velocity_mode")
        ).strip().lower()
        if self.velocity_mode not in (
            'c5',
            'w75',
        ):
            raise ValueError(
                "velocity_mode must be "
                "'c5' or 'w75', "
                f"got {self.velocity_mode!r}"
            )
        # Geometry / orientation state.
        self.good_frames = 0
        self.ready = False
        self.prev_normal = None
        self.prev_longitudinal = None
        # Final anatomical-normal state.
        # Consecutive strong handedness evidence.
        #
        # The same deque is intentionally retained so this
        # patch stays local to the existing architecture.
        self.stable_handedness = (
            HandTrackingFiltered.HAND_UNKNOWN
        )
        self.semantic_normal_state = None
        self.semantic_normal_pending = None
        # Independent One-Euro state for:
        #   - palm normal
        #   - palm longitudinal axis
        #
        # Only newly available geometry updates these filters.
        # Held/predicted orientation never feeds back into them.
        # Original C5 mathematical estimator remains available.
        self.c5_history = deque(
            maxlen=5
        )
        # Frozen W75.
        self.w75 = W75VelocityEstimator()
        self.last_timestamp_s = None
        # Last trustworthy measured palm + W75 velocity.
        #
        # Predicted samples are NEVER fed back into W75.
        self.prediction_anchor_time = None
        self.prediction_anchor_position = None
        self.prediction_anchor_velocity = None
        self.publisher = self.create_publisher(
            HandState,
            '/handover/hand_state',
            10,
        )
        self.subscription = self.create_subscription(
            HandTrackingFiltered,
            '/handover/hand_tracking_filtered',
            self.callback,
            10,
        )
        self.get_logger().info(
            'Hand state estimator started: '
            f'velocity_mode={self.velocity_mode}, '
            f'max_position_age_s='
            f'{self.max_position_age_s:.3f}, '
            f'prediction_bridge='
            f'{self.enable_prediction_bridge}, '
            f'max_prediction_age_s='
            f'{self.max_prediction_age_s:.3f}'
        )

    @staticmethod
    def point_array(point):
        return np.array(
            [
                point.x,
                point.y,
                point.z,
            ],
            dtype=float,
        )

    @staticmethod
    def stamp_to_seconds(stamp):
        return (
            float(stamp.sec)
            +
            float(stamp.nanosec)
            * 1e-9
        )

    def reset_temporal_state(
        self,
        reset_orientation=True,
    ):
        self.c5_history.clear()
        self.w75.reset()
        self.good_frames = 0
        self.ready = False
        self.prediction_anchor_time = None
        self.prediction_anchor_position = None
        self.prediction_anchor_velocity = None
        if reset_orientation:
            self.prev_normal = None
            self.prev_longitudinal = None
            self.stable_handedness = (
                HandTrackingFiltered.HAND_UNKNOWN
            )
            self.semantic_normal_state = None
            self.semantic_normal_pending = None

    def estimate_c5(
        self,
        now,
        position,
        position_valid,
    ):
        """
        Same C5 regression formula used before W75.

        The exact old runtime file remains stored
        in backup/pre_w75_20260915/.
        """
        if not position_valid:
            self.c5_history.clear()
            return {
                'available':
                    False,
                'velocity':
                    np.zeros(
                        3,
                        dtype=float,
                    ),
                'source':
                    SOURCE_NONE,
                'age_s':
                    np.nan,
            }
        if (
            self.c5_history
            and
            now
            <= self.c5_history[-1][0]
        ):
            self.c5_history.clear()
        self.c5_history.append(
            (
                now,
                position.copy(),
            )
        )
        if len(
            self.c5_history
        ) < 3:
            return {
                'available':
                    False,
                'velocity':
                    np.zeros(
                        3,
                        dtype=float,
                    ),
                'source':
                    SOURCE_NONE,
                'age_s':
                    np.nan,
            }
        times = np.asarray(
            [
                item[0]
                for item
                in self.c5_history
            ],
            dtype=float,
        )
        positions = np.asarray(
            [
                item[1]
                for item
                in self.c5_history
            ],
            dtype=float,
        )
        tc = (
            times
            - np.mean(times)
        )
        denom = float(
            np.dot(
                tc,
                tc,
            )
        )
        if denom <= 0.0:
            return {
                'available':
                    False,
                'velocity':
                    np.zeros(
                        3,
                        dtype=float,
                    ),
                'source':
                    SOURCE_NONE,
                'age_s':
                    np.nan,
            }
        velocity = (
            np.sum(
                tc[:, None]
                *
                (
                    positions
                    -
                    np.mean(
                        positions,
                        axis=0,
                    )
                ),
                axis=0,
            )
            /
            denom
        )
        return {
            'available':
                True,
            'velocity':
                np.asarray(
                    velocity,
                    dtype=float,
                ),
            'source':
                SOURCE_UPDATED,
            'age_s':
                0.0,
        }



    def kalman_palm_variance(
        self,
        msg,
    ):
        variances = np.asarray(
            [
                [
                    msg.position_variance[i].x,
                    msg.position_variance[i].y,
                    msg.position_variance[i].z,
                ]
                for i in self.PALM_INDICES
            ],
            dtype=float,
        )
        if not np.isfinite(
            variances
        ).all():
            return np.zeros(
                3,
                dtype=float,
            )
        # Variance of:
        #
        # palm = mean(MCP5, MCP9, MCP17)
        #
        # assuming independent landmark estimates.
        return (
            np.sum(
                np.maximum(
                    variances,
                    0.0,
                ),
                axis=0,
            )
            / 9.0
        )

    def update_prediction_anchor(
        self,
        now,
        position,
        velocity,
    ):
        """
        Update prediction anchor only from a fresh measured palm.

        A predicted sample can never become the new anchor.
        """
        if (
            self.velocity_mode != 'w75'
            or not self.enable_prediction_bridge
            or not position['fresh']
            or not velocity['available']
        ):
            return
        p = np.asarray(
            position['position'],
            dtype=float,
        )
        v = np.asarray(
            velocity['velocity'],
            dtype=float,
        )
        if not (
            np.isfinite(p).all()
            and
            np.isfinite(v).all()
        ):
            return
        self.prediction_anchor_time = float(
            now
        )
        self.prediction_anchor_position = (
            p.copy()
        )
        self.prediction_anchor_velocity = (
            v.copy()
        )

    def prediction_bridge_state(
        self,
        msg,
        now,
    ):
        """
        Build a W75 constant-velocity palm prediction.

        Requirements:
        - W75 runtime;
        - valid fresh anchor exists;
        - all MCP 5/9/17 are still PREDICT_ONLY in Kalman;
        - prediction age is inside the experimentally validated
          interval <= 300 ms.
        """
        if (
            self.velocity_mode != 'w75'
            or not self.enable_prediction_bridge
            or self.prediction_anchor_time is None
            or self.prediction_anchor_position is None
            or self.prediction_anchor_velocity is None
        ):
            return None
        states = [
            int(
                msg.landmark_state[i]
            )
            for i in self.PALM_INDICES
        ]
        # Only bridge a true visual dropout of the palm MCPs.
        #
        # Partial measured cases keep using the original
        # position_state() policy.
        if not all(
            state
            ==
            HandTrackingFiltered.PREDICT_ONLY
            for state in states
        ):
            return None
        ages = [
            float(
                msg.age_s[i]
            )
            for i in self.PALM_INDICES
        ]
        if not all(
            np.isfinite(age)
            for age in ages
        ):
            return None
        age_s = float(
            now
            -
            self.prediction_anchor_time
        )
        if (
            age_s < -1e-9
            or
            age_s
            >
            self.max_prediction_age_s
            + 1e-9
            or
            max(ages)
            >
            self.max_prediction_age_s
            + 1e-9
        ):
            return None
        speed = float(
            np.linalg.norm(
                self.prediction_anchor_velocity
            )
        )
        empirical_q975 = (
            prediction_q975_error_m(
                age_s,
                speed,
            )
        )
        if not np.isfinite(
            empirical_q975
        ):
            return None
        predicted_position = (
            self.prediction_anchor_position
            +
            self.prediction_anchor_velocity
            *
            age_s
        )
        if not np.isfinite(
            predicted_position
        ).all():
            return None
        # Kalman covariance already grows during PREDICT_ONLY.
        #
        # Add the independently calibrated W75 extrapolation error
        # so downstream distance_sigma reflects both effects.
        kalman_variance = (
            self.kalman_palm_variance(
                msg
            )
        )
        empirical_sigma = (
            empirical_q975
            /
            PREDICTION_CHI3_Q975_RADIUS
        )
        prediction_variance = (
            kalman_variance
            +
            empirical_sigma ** 2
        )
        return {
            'valid':
                True,
            'fresh':
                False,
            'predicted':
                True,
            'age_s':
                age_s,
            'position':
                predicted_position,
            'variance':
                prediction_variance,
            'velocity':
                self.prediction_anchor_velocity.copy(),
        }

    @staticmethod
    def _semantic_normalise(
        vector,
    ):
        vector = np.asarray(
            vector,
            dtype=float,
        )
        if not np.isfinite(
            vector
        ).all():
            return None
        norm = float(
            np.linalg.norm(
                vector
            )
        )
        if norm <= 1e-12:
            return None
        return vector / norm

    @staticmethod
    def _semantic_angle_deg(
        a,
        b,
    ):
        a = (
            HandStateEstimator
            ._semantic_normalise(a)
        )
        b = (
            HandStateEstimator
            ._semantic_normalise(b)
        )
        if (
            a is None
            or b is None
        ):
            return np.nan
        dot = float(
            np.clip(
                np.dot(a, b),
                -1.0,
                1.0,
            )
        )
        return float(
            np.degrees(
                np.arccos(dot)
            )
        )

    @staticmethod
    def _normal_filter_alpha(
        cutoff,
        dt,
    ):
        cutoff = max(
            float(cutoff),
            1e-6,
        )
        tau = (
            1.0
            /
            (
                2.0
                * np.pi
                * cutoff
            )
        )
        return (
            1.0
            /
            (
                1.0
                + tau / dt
            )
        )

    def _reset_semantic_filter(
        self,
        now,
        normal,
        handedness,
    ):
        normal = (
            self._semantic_normalise(
                normal
            )
        )
        if normal is None:
            return None
        self.semantic_normal_state = {
            'timestamp':
                float(now),
            'raw':
                normal.copy(),
            'filtered':
                normal.copy(),
            'derivative':
                np.zeros(
                    3,
                    dtype=float,
                ),
            'handedness':
                int(handedness),
        }
        self.semantic_normal_pending = None
        return normal.copy()

    def _semantic_oneeuro_update(
        self,
        now,
        normal,
    ):
        """
        One-Euro update with semantic sign preserved.

        There is intentionally NO:
            if dot(new, old) < 0:
                new = -new
        """
        normal = (
            self._semantic_normalise(
                normal
            )
        )
        if normal is None:
            return None
        state = (
            self.semantic_normal_state
        )
        if state is None:
            return normal.copy()
        dt = (
            float(now)
            -
            float(
                state[
                    'timestamp'
                ]
            )
        )
        if (
            not np.isfinite(dt)
            or
            dt <= 0.0
            or
            dt > self.NORMAL_RESET_GAP_S
        ):
            return normal.copy()
        derivative = (
            normal
            -
            state[
                'raw'
            ]
        ) / dt
        alpha_d = (
            self._normal_filter_alpha(
                self.NORMAL_D_CUTOFF,
                dt,
            )
        )
        derivative_filtered = (
            alpha_d
            * derivative
            +
            (
                1.0
                - alpha_d
            )
            * state[
                'derivative'
            ]
        )
        output = np.zeros(
            3,
            dtype=float,
        )
        for axis in range(3):
            cutoff = (
                self.NORMAL_MIN_CUTOFF
                +
                self.NORMAL_BETA
                * abs(
                    derivative_filtered[
                        axis
                    ]
                )
            )
            alpha_x = (
                self._normal_filter_alpha(
                    cutoff,
                    dt,
                )
            )
            output[
                axis
            ] = (
                alpha_x
                * normal[
                    axis
                ]
                +
                (
                    1.0
                    - alpha_x
                )
                * state[
                    'filtered'
                ][
                    axis
                ]
            )
        output = (
            self._semantic_normalise(
                output
            )
        )
        if output is None:
            output = normal.copy()
        self.semantic_normal_state = {
            'timestamp':
                float(now),
            'raw':
                normal.copy(),
            'filtered':
                output.copy(),
            'derivative':
                derivative_filtered.copy(),
            'handedness':
                int(
                    state[
                        'handedness'
                    ]
                ),
        }
        return output.copy()

    def _update_stable_handedness(
        self,
        msg,
    ):
        """
        PHYSICAL_SIDE_DIRECT_V1

        msg.handedness is now the BODY-AWARE anatomical
        physical side selected upstream by Holistic.

        Therefore this function no longer tries to repair a
        noisy MediaPipe Hands handedness classifier.

        LEFT/RIGHT identity:
            Holistic/body.

        Interaction role switch:
            confirmed upstream.

        Palm-normal sign:
            uses that confirmed physical side directly.

        A real physical-side change resets W75, prediction
        history and orientation state so samples from the two
        hands can never be mixed.
        """
        if (
            int(msg.filter_state)
            ==
            int(
                HandTrackingFiltered.LOST
            )
        ):
            self.stable_handedness = (
                HandTrackingFiltered.HAND_UNKNOWN
            )
            self.semantic_normal_state = None
            self.semantic_normal_pending = None
            return self.stable_handedness
        raw_hand = int(
            msg.handedness
        )
        valid_hands = (
            HandTrackingFiltered.HAND_LEFT,
            HandTrackingFiltered.HAND_RIGHT,
        )
        if raw_hand not in valid_hands:
            return self.stable_handedness
        score = float(
            msg.handedness_score
        )
        position_fresh = bool(
            self.position_state(
                msg
            )[
                'fresh'
            ]
        )
        # Tracker publishes score=1 for an anatomically
        # resolved Holistic active hand.
        physical_side_current = (
            int(msg.filter_state)
            ==
            int(
                HandTrackingFiltered.TRACKING
            )
            and
            position_fresh
            and
            np.isfinite(score)
            and
            score >= 0.99
        )
        if not physical_side_current:
            return self.stable_handedness
        if (
            self.stable_handedness
            ==
            raw_hand
        ):
            return self.stable_handedness
        previous = int(
            self.stable_handedness
        )
        if previous in valid_hands:
            # A different physical hand is a new temporal
            # signal. Never carry W75 / prediction /
            # orientation state across the switch.
            self.reset_temporal_state(
                reset_orientation=True
            )
        self.stable_handedness = (
            raw_hand
        )
        self.semantic_normal_state = None
        self.semantic_normal_pending = None
        return self.stable_handedness

    def _anatomical_raw_normal(
        self,
        msg,
        stable_handedness,
        now,
    ):
        """
        HOLISTIC_TEMPORAL_SIGN_V1

        RGBD5 PCA provides an unsigned palm-plane normal.

        Holistic/body provides reliable physical LEFT/RIGHT,
        but the RGB-D anatomical anchor can occasionally flip
        because of per-landmark depth/geometry variation.

        Policy:
          - acquisition / real reset:
                anatomical anchor selects the initial branch;
          - same physical-hand episode:
                keep the PCA +/- branch temporally continuous;
          - GUARD60 and One-Euro remain downstream unchanged.

        This does NOT prevent a real UP -> SIDE -> DOWN
        rotation: a physical palm rotation evolves
        continuously over consecutive frames.
        """
        if not bool(
            msg.palm_plane_valid
        ):
            return None
        if stable_handedness not in (
            HandTrackingFiltered.HAND_LEFT,
            HandTrackingFiltered.HAND_RIGHT,
        ):
            return None
        plane = (
            self._semantic_normalise(
                [
                    msg.palm_plane_normal.x,
                    msg.palm_plane_normal.y,
                    msg.palm_plane_normal.z,
                ]
            )
        )
        anchor = (
            self._semantic_normalise(
                [
                    msg.palm_anchor_cross.x,
                    msg.palm_anchor_cross.y,
                    msg.palm_anchor_cross.z,
                ]
            )
        )
        if (
            plane is None
            or anchor is None
        ):
            return None
        reference = None
        state = (
            self.semantic_normal_state
        )
        # Same physical hand + recent geometry:
        # resolve only the arbitrary PCA +/- ambiguity using
        # temporal continuity.
        #
        # Use previous RAW semantic normal rather than the
        # filtered one, so One-Euro lag cannot determine sign.
        if state is not None:
            dt = (
                float(now)
                -
                float(
                    state[
                        'timestamp'
                    ]
                )
            )
            same_hand = (
                int(
                    state[
                        'handedness'
                    ]
                )
                ==
                int(
                    stable_handedness
                )
            )
            recent = (
                np.isfinite(dt)
                and
                dt > 0.0
                and
                dt
                <=
                self.NORMAL_RESET_GAP_S
            )
            if (
                same_hand
                and recent
            ):
                reference = (
                    self._semantic_normalise(
                        state[
                            'raw'
                        ]
                    )
                )
        # New semantic episode.
        #
        # Anchor is used ONLY to seed the branch here.
        #
        # Restore the original anatomical convention:
        #   RIGHT -> +anchor
        #   LEFT  -> -anchor
        if reference is None:
            if (
                stable_handedness
                ==
                HandTrackingFiltered.HAND_RIGHT
            ):
                reference = (
                    anchor
                )
            else:
                reference = (
                    -anchor
                )
        if (
            np.dot(
                plane,
                reference,
            )
            < 0.0
        ):
            plane = -plane
        return plane

    def _longitudinal_from_filtered(
        self,
        msg,
        normal,
    ):
        """
        Longitudinal axis remains based on the existing
        Kalman-smoothed landmarks.

        palm centre is STILL MCP 5/9/17.
        Landmark 13 is NOT introduced into position/W75.
        """
        try:
            wrist = self.point_array(
                msg.positions[0]
            )
            palm = np.mean(
                np.asarray(
                    [
                        self.point_array(
                            msg.positions[index]
                        )
                        for index
                        in self.PALM_INDICES
                    ],
                    dtype=float,
                ),
                axis=0,
            )
        except Exception:
            return None
        if not (
            np.isfinite(wrist).all()
            and
            np.isfinite(palm).all()
        ):
            return None
        longitudinal = (palm - wrist)
        # Project into the palm plane.
        longitudinal = (longitudinal - normal * np.dot(longitudinal, normal,))
        return (self._semantic_normalise(longitudinal))

    def update_geometry_anatomical(self, msg, position_valid, now,):
        """
        Final candidate geometry pipeline:

          RGBD5 PCA
              -> stable handedness
              -> anatomical sign
              -> GUARD60
              -> One Euro beta=1

        Large real rotations ARE allowed.

        GUARD60 only asks for one additional coherent sample
        when an innovation is >60 degrees, because causally a
        one-frame spike cannot be distinguished from a true
        sudden rotation at its first observation.
        """
        stable_hand = (self._update_stable_handedness(msg))
        raw_normal = (self._anatomical_raw_normal(msg, stable_hand, now,))
        action = 'NONE'
        filtered = None
        if (position_valid and raw_normal is not None):
            state = (self.semantic_normal_state)
            # First valid semantic orientation or reset.
            reset = (state is None)
            if not reset:
                dt = (float(now) - float(state['timestamp']))
                reset = (not np.isfinite(dt) or dt <= 0.0
                    or dt > self.NORMAL_RESET_GAP_S or int(state['handedness']) != int(stable_hand))
            if reset:
                filtered = (self._reset_semantic_filter(now, raw_normal, stable_hand,))
                action = 'RESET'
            else:
                innovation = (self._semantic_angle_deg(raw_normal, state['filtered'],))
                if (np.isfinite(innovation) and innovation > self.NORMAL_INNOVATION_GATE_DEG):
                    confirmed = False
                    pending = (self.semantic_normal_pending)
                    if pending is not None:
                        same_hand = (int(pending['handedness']) == int(stable_hand))
                        recent = (
                            float(now) - float(pending['timestamp']) <= self.NORMAL_RESET_GAP_S)
                        candidate_angle = (self._semantic_angle_deg(raw_normal, pending['normal'],))
                        confirmed = (same_hand and recent and np.isfinite(candidate_angle
                            ) and candidate_angle <= self.NORMAL_CONFIRM_ANGLE_DEG)
                    if confirmed:
                        # A genuine large rotation is now
                        # accepted. We DO NOT rate-limit it
                        # and we DO NOT invert its sign.
                        filtered = (self._semantic_oneeuro_update(now, raw_normal,))
                        self.semantic_normal_pending = None
                        action = 'CONFIRM'
                    else:
                        # First >60° observation:
                        # retain the previous filtered normal
                        # for one frame while requesting causal
                        # confirmation.
                        self.semantic_normal_pending = {'timestamp':
                                float(now), 'normal':
                                raw_normal.copy(), 'handedness':
                                int(stable_hand),}
                        filtered = (state['filtered'].copy())
                        action = 'HOLD'
                else:
                    self.semantic_normal_pending = None
                    filtered = (self._semantic_oneeuro_update(now, raw_normal,))
                    action = 'UPDATE'
        # No new valid geometry:
        # retain previous orientation for visual continuity.
        # It is NOT geometry_ok.
        if filtered is None:
            if (self.semantic_normal_state is not None):
                filtered = (self.semantic_normal_state['filtered'].copy())
            elif self.prev_normal is not None:
                filtered = (self.prev_normal.copy())
            else:
                filtered = np.zeros(3, dtype=float,)
        longitudinal = (self._longitudinal_from_filtered(
                msg, filtered,) if np.linalg.norm(filtered) > 1e-12 else None)
        if longitudinal is None:
            longitudinal = (self.prev_longitudinal.copy()
                if self.prev_longitudinal is not None else np.zeros(3, dtype=float,))
        # Geometry validity.
        #
        # HOLD is intentionally degraded.
        #
        # CONFIRM IS VALID:
        # a large jump is not considered an error once the
        # second coherent observation confirms it.
        geometry_ok = (action in ('RESET', 'UPDATE', 'CONFIRM',
            ) and np.linalg.norm(filtered) > 1e-12 and np.linalg.norm(longitudinal) > 1e-12)
        if geometry_ok:
            self.prev_normal = (filtered.copy())
            self.prev_longitudinal = (longitudinal.copy())
            if not self.ready:
                self.good_frames += 1
                self.ready = (self.good_frames >= self.reacquire_frames)
        else:
            if not self.ready:
                self.good_frames = 0
            if (int(msg.filter_state) == int(HandTrackingFiltered.LOST)):
                self.good_frames = 0
                self.ready = False
        return (bool(geometry_ok), longitudinal, filtered,)

    def tracking_confidence(self, msg,):
        scores = []
        for i, landmark_state in enumerate(msg.landmark_state):
            if (landmark_state == HandTrackingFiltered.TRACKING):
                score = 1.0
            elif (landmark_state == HandTrackingFiltered.PREDICT_ONLY):
                age_factor = max(0.0, 1.0 - float(msg.age_s[i]) / self.lost_timeout,)
                score = (0.6 * age_factor)
            else:
                score = 0.0
            if (msg.measurement_type[i] == HandTrackingFiltered.ESTIMATED):
                score *= 0.7
            scores.append(score)
        variances = []
        for variance in msg.position_variance:
            variances.extend([variance.x, variance.y, variance.z,])
        mean_sigma = np.sqrt(max(0.0, float(np.mean(variances)),))
        uncertainty = np.exp(-mean_sigma / self.confidence_sigma)
        return float(np.clip(np.mean(scores) * uncertainty, 0.0, 1.0,))

    def position_state(self, msg,):
        """
        Frozen position policy:

        - MCP 5/9/17 must be TRACKING or PREDICT_ONLY
        - at least 2/3 must be TRACKING
        - max age <= 0.10 s
        - coordinates finite

        A sample enters the W75 histories only when
        all three MCP landmarks are TRACKING.
        """
        indices = self.PALM_INDICES
        states = [int(msg.landmark_state[i]) for i in indices]
        ages = [float(msg.age_s[i]) for i in indices]
        state_usable = all(state in (HandTrackingFiltered.TRACKING,
                HandTrackingFiltered.PREDICT_ONLY,) for state in states)
        tracking_count = sum(state == HandTrackingFiltered.TRACKING for state in states)
        ages_finite = all(np.isfinite(age) for age in ages)
        position_age = (max(ages) if ages_finite else np.nan)
        valid = (state_usable
            and tracking_count >= 1 and ages_finite and position_age <= self.max_position_age_s)
        fresh = (valid and tracking_count == 3)
        palm = np.zeros(3, dtype=float,)
        variance = np.zeros(3, dtype=float,)
        if valid:
            palm_points = np.asarray(
                [self.point_array(msg.positions[i]) for i in indices], dtype=float,)
            if not np.isfinite(palm_points).all():
                valid = False
                fresh = False
            else:
                palm = np.mean(palm_points, axis=0,)
                position_variances = (
                    np.asarray([[msg.position_variance[i].x, msg.position_variance[i].y,
                                msg.position_variance[i].z,] for i in indices], dtype=float,))
                variance = (np.sum(position_variances, axis=0,) / 9.0)
        return {'valid':
                bool(valid), 'fresh':
                bool(fresh), 'age_s':
                float(position_age), 'position':
                palm, 'variance':
                variance,}

    def callback(self, msg,):
        start = time.perf_counter()
        out = HandState()
        out.header = msg.header
        out.filter_state = int(msg.filter_state)
        out.velocity_estimator = (HandState.VELOCITY_ESTIMATOR_W75
            if self.velocity_mode == 'w75' else HandState.VELOCITY_ESTIMATOR_C5)
        out.velocity_source = (HandState.VELOCITY_SOURCE_NONE)
        out.position_age_s = (float('nan'))
        out.velocity_age_s = (float('nan'))
        if (len(msg.positions) < 4 or len(msg.landmark_state) < 4 or
            len(msg.age_s) < 4 or len(msg.position_variance) < 4 or len(msg.measurement_type) < 4):
            self.reset_temporal_state(reset_orientation=True)
            out.processing_latency_ms = (time.perf_counter() - start) * 1000.0
            self.publisher.publish(out)
            return
        now = self.stamp_to_seconds(msg.header.stamp)
        if (self.last_timestamp_s is not None and now <= self.last_timestamp_s):
            self.reset_temporal_state(reset_orientation=True)
        self.last_timestamp_s = now
        # MEASURED POSITION + VELOCITY
        # IMPORTANT:
        #
        # W75 only sees the original measured/filtered position.
        # Predicted palm samples never enter its histories.
        measured_position = (self.position_state(msg))
        # PHYSICAL HAND SWITCH — RESET BEFORE W75
        #
        # LEFT/RIGHT now comes from the body-aware Holistic
        # frontend.  If the selected physical hand changes,
        # temporal state from the previous hand must be
        # cleared BEFORE the first sample of the new hand
        # enters W75 / C5 / prediction.
        raw_side = int(msg.handedness)
        valid_sides = (HandTrackingFiltered.HAND_LEFT, HandTrackingFiltered.HAND_RIGHT,)
        physical_switch = (raw_side in valid_sides and self.stable_handedness in valid_sides
            and raw_side != self.stable_handedness and int(msg.filter_state)
                == int(HandTrackingFiltered.TRACKING) and bool(measured_position['fresh'])
            and np.isfinite(float(msg.handedness_score)) and float(msg.handedness_score) >= 0.99)
        if physical_switch:
            previous_side = int(self.stable_handedness)
            self.reset_temporal_state(reset_orientation=True)
            # reset_temporal_state() intentionally sets
            # handedness UNKNOWN.  Restore the newly confirmed
            # physical side immediately so geometry does not
            # perform a second reset later in this frame.
            self.stable_handedness = (raw_side)
            self.get_logger().info(
                'HandState physical-hand reset: ' f'{previous_side} -> {raw_side}')
        if self.velocity_mode == 'w75':
            measured_velocity = (self.w75.update(now, measured_position[
                        'position'], measured_position['fresh'], measured_position['valid'],))
        else:
            measured_velocity = (
                self.estimate_c5(now, measured_position['position'], measured_position['valid'],))
        self.update_prediction_anchor(now, measured_position, measured_velocity,)
        position = dict(measured_position)
        position['predicted'] = False
        velocity = dict(measured_velocity)
        # W75 PREDICTION BRIDGE
        if not measured_position['valid']:
            predicted = (self.prediction_bridge_state(msg, now,))
            if predicted is not None:
                position = predicted
                # This is the frozen W75 velocity which generated
                # the palm prediction.
                #
                # It remains available as an aged HOLD estimate,
                # but the distance node will NOT promote it to
                # rate_valid while the palm itself is predicted.
                velocity = {'available':
                        True, 'velocity':
                        predicted['velocity'].copy(), 'source':
                        SOURCE_HOLD, 'age_s':
                        float(predicted['age_s']),}
        # POSITION OUTPUT
        out.position_valid = bool(position['valid'])
        out.position_fresh = bool(position['fresh'])
        out.position_age_s = float(position['age_s'])
        if position['valid']:
            p = position['position']
            var = position['variance']
            out.palm_position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]),)
            out.palm_position_variance = Vector3(x=float(var[0]), y=float(var[1]), z=float(var[2]),)
        # VELOCITY OUTPUT
        out.velocity_valid = bool(velocity['available'])
        out.velocity_source = int(velocity['source'])
        out.velocity_age_s = float(velocity['age_s'])
        if velocity['available']:
            v = np.asarray(velocity['velocity'], dtype=float,)
            speed = float(np.linalg.norm(v))
            out.palm_velocity = Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]),)
            out.palm_speed = speed
            out.motion_stability = float(np.clip(1.0 - speed / self.stability_speed, 0.0, 1.0,))
        # GEOMETRY / ORIENTATION
        if position.get('predicted', False,):
            # Orientation/normal prediction was NOT validated.
            #
            # Keep the previous axes only for continuity /
            # visualization, but declare geometry invalid.
            #
            # Do not reset readiness for a short visual blink.
            geometry_ok = False
            longitudinal = (self.prev_longitudinal.copy()
                if self.prev_longitudinal is not None else np.zeros(3, dtype=float,))
            normal = (self.prev_normal.copy()
                if self.prev_normal is not None else np.zeros(3, dtype=float,))
        else:
            (geometry_ok,
            longitudinal, normal,) = self.update_geometry_anatomical(msg, position['valid'], now,)
        out.geometry_ok = bool(geometry_ok)
        out.palm_longitudinal = Vector3(
            x=float(longitudinal[0]), y=float(longitudinal[1]), z=float(longitudinal[2]),)
        out.palm_normal = Vector3(x=float(normal[0]), y=float(normal[1]), z=float(normal[2]),)
        # QUALITY
        out.tracking_confidence = (self.tracking_confidence(msg) if position['valid'] else 0.0)
        # Legacy complete HandState.
        #
        # Consumers needing only position must use
        # position_valid.
        #
        # Consumers needing velocity must use
        # velocity_valid/source/age.
        out.valid = bool(
            out.position_valid and out.velocity_valid and out.geometry_ok and self.ready)
        if not out.velocity_valid:
            out.palm_speed = 0.0
            out.motion_stability = 0.0
        out.processing_latency_ms = (time.perf_counter() - start) * 1000.0
        self.publisher.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = HandStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
