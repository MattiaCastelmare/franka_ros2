#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from franka_msgs.msg import (
    HandoverDistance,
    HandoverObserver,
)

from franka_experiments.utils.params import (
    load_hand_tracking_defaults,
    parameter_value,
)

_OBSERVER_DEFAULTS = load_hand_tracking_defaults("handover_observer")

class HandoverObserverNode(Node):

    def __init__(self):
        super().__init__('handover_observer')
        self.enter_threshold = float(
            parameter_value(self, _OBSERVER_DEFAULTS, "enter_threshold")
        )
        self.exit_threshold = float(
            parameter_value(self, _OBSERVER_DEFAULTS, "exit_threshold")
        )
        self.confirm_frames = int(
            parameter_value(self, _OBSERVER_DEFAULTS, "confirm_frames")
        )
        if self.enter_threshold <= 0.0:
            raise ValueError(
                'enter_threshold must be > 0'
            )
        if not (
            0.0
            <=
            self.exit_threshold
            <
            self.enter_threshold
        ):
            raise ValueError(
                'exit_threshold must satisfy '
                '0 <= exit < enter'
            )
        if self.confirm_frames < 1:
            raise ValueError(
                'confirm_frames must be >= 1'
            )
        self.current_state = (
            HandoverObserver.WARMUP
        )
        self.pending_state = None
        self.pending_count = 0
        self.publisher = self.create_publisher(
            HandoverObserver,
            '/handover/observer',
            10,
        )
        self.subscription = (
            self.create_subscription(
                HandoverDistance,
                '/handover/distance',
                self.callback,
                10,
            )
        )
        self.get_logger().info(
            'Handover observer started '
            f'(enter='
            f'{self.enter_threshold:.3f} m/s, '
            f'exit='
            f'{self.exit_threshold:.3f} m/s, '
            f'confirm='
            f'{self.confirm_frames})'
        )

    def reset_pending(self):
        self.pending_state = None
        self.pending_count = 0

    def target_state(
        self,
        closing,
    ):
        if (
            self.current_state
            ==
            HandoverObserver.APPROACHING
        ):
            if (
                closing
                <
                -self.enter_threshold
            ):
                return (
                    HandoverObserver.
                    RETREATING
                )
            if (
                closing
                <
                self.exit_threshold
            ):
                return (
                    HandoverObserver.HOLD
                )
            return (
                HandoverObserver.
                APPROACHING
            )
        if (
            self.current_state
            ==
            HandoverObserver.RETREATING
        ):
            if (
                closing
                >
                self.enter_threshold
            ):
                return (
                    HandoverObserver.
                    APPROACHING
                )
            if (
                closing
                >
                -self.exit_threshold
            ):
                return (
                    HandoverObserver.HOLD
                )
            return (
                HandoverObserver.
                RETREATING
            )
        if (
            closing > self.enter_threshold):
            return (HandoverObserver.APPROACHING)
        if (closing < -self.enter_threshold):
            return (HandoverObserver.RETREATING)
        return HandoverObserver.HOLD

    def update_confirmed_state(self, target,):
        if target == self.current_state:
            self.reset_pending()
            return
        if self.pending_state == target:
            self.pending_count += 1
        else:
            self.pending_state = target
            self.pending_count = 1
        if (self.pending_count >= self.confirm_frames):
            self.current_state = target
            self.reset_pending()

    def callback(self, msg: HandoverDistance,):
        out = HandoverObserver()
        out.header = msg.header
        previous_state = (self.current_state)
        if not msg.valid:
            self.current_state = (HandoverObserver.LOST)
            self.reset_pending()
        elif not msg.rate_valid:
            degraded_prediction = bool(msg.valid and msg.rate_degraded)
            if (degraded_prediction and self.current_state in (HandoverObserver.HOLD,
                    HandoverObserver.APPROACHING, HandoverObserver.RETREATING,)):
                # A short prediction bridge preserves an already
                # established observer state.
                #
                # But no new APPROACHING / RETREATING transition
                # may be created without fresh rate evidence.
                self.reset_pending()
            else:
                self.current_state = (HandoverObserver.WARMUP)
                self.reset_pending()
        else:
            held_rate = bool(int(msg.rate_source) == HandoverDistance.RATE_SOURCE_RELATIVE_HOLD)
            if (held_rate and self.current_state in (HandoverObserver.HOLD,
                    HandoverObserver.APPROACHING, HandoverObserver.RETREATING,)):
                # Preserve the current state.
                # Pending fresh evidence remains pending.
                pass
            else:
                if (self.current_state in (HandoverObserver.LOST, HandoverObserver.WARMUP,)):
                    self.current_state = (HandoverObserver.HOLD)
                target = self.target_state(float(msg.closing_velocity))
                self.update_confirmed_state(target)
        out.state = int(self.current_state)
        out.state_changed = bool(self.current_state != previous_state)
        out.candidate_state = int(
            self.pending_state if self.pending_state is not None else self.current_state)
        out.candidate_count = int(self.pending_count)
        out.distance = float(msg.distance)
        out.closing_velocity = float(msg.closing_velocity)
        out.rate_valid = bool(msg.rate_valid)
        out.rate_source = int(msg.rate_source)
        out.rate_degraded = bool(msg.rate_degraded)
        out.rate_age_s = float(msg.rate_age_s)
        out.tracking_confidence = float(msg.tracking_confidence)
        out.distance_sigma = float(msg.distance_sigma)
        out.ttc_valid = bool(self.current_state == HandoverObserver.APPROACHING and msg.ttc_valid)
        out.ttc = (float(msg.ttc) if out.ttc_valid else 0.0)
        self.publisher.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = HandoverObserverNode()
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
