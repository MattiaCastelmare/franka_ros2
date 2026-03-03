#!/usr/bin/env python3
"""Velocity blender / mux node for Franka FR3 joint-velocity control.

Subscribes to one or more ``Float64MultiArray`` velocity-contribution topics,
blends (sums) them, clamps per-joint, and publishes the result to the
``fr3_forward_velocity_controller/commands`` topic.

Currently **active** input channels:

* ``/<ns>/tracking_qdot``  – task-space tracking joint velocities.

**Predisposed** (interface only – subscription created, but callback just
stores the message; contribution weight is zero until you enable it):

* ``/<ns>/avoidance_qdot`` – obstacle / self-collision avoidance overlay.

Safety features
---------------
* Per-joint velocity clamp (``qdot_max``, default 0.2 rad/s).
* Per-channel watchdog: if no message arrives within ``watchdog_s`` (default
  0.2 s), that channel's contribution is zeroed.
* On SIGINT / SIGTERM / Ctrl-C the node publishes zero velocities for ~0.5 s
  and then exits cleanly.
* Cosine ramp-up over ``ramp_s`` (default 1.0 s) from first non-zero publish.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.constants import NUM_JOINTS, CONTROLLER_NAME
from franka_experiments.utils.ros import (
    build_namespaced_topic,
    run_node_main,
)
from franka_experiments.utils.math_utils import cosine_ramp, clamp_joints


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║  VelocityChannel – bookkeeping for one input contribution             ║
# ╚═════════════════════════════════════════════════════════════════════════╝

class _VelocityChannel:
    """Stores the latest qdot contribution and watchdog state for one input."""

    __slots__ = ("name", "qdot", "stamp", "enabled")

    def __init__(self, name: str, *, enabled: bool = True) -> None:
        self.name = name
        self.qdot: np.ndarray = np.zeros(NUM_JOINTS)
        self.stamp: float = 0.0
        self.enabled: bool = enabled

    def update(self, data: list[float]) -> None:
        """Store incoming data and refresh timestamp."""
        if len(data) != NUM_JOINTS:
            return
        self.qdot[:] = data
        self.stamp = time.monotonic()

    def timed_out(self, timeout: float) -> bool:
        """Return True if no message for *timeout* seconds."""
        if self.stamp == 0.0:
            return True
        return (time.monotonic() - self.stamp) > timeout

    def contribution(self, timeout: float) -> np.ndarray:
        """Return qdot if enabled and alive, else zeros."""
        if not self.enabled or self.timed_out(timeout):
            return np.zeros(NUM_JOINTS)
        return self.qdot


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║  VelocityBlenderNode                                                   ║
# ╚═════════════════════════════════════════════════════════════════════════╝

class VelocityBlenderNode(Node):
    """Subscribes to velocity channels, blends, clamps, and publishes."""

    def __init__(self) -> None:
        super().__init__("velocity_blender")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter(
            "command_topic",
            build_namespaced_topic(f"{CONTROLLER_NAME}/commands"),
        )
        self.declare_parameter(
            "tracking_topic", build_namespaced_topic("tracking_qdot"))
        self.declare_parameter(
            "avoidance_topic", build_namespaced_topic("avoidance_qdot"))
        self.declare_parameter("rate_hz", 200.0)
        self.declare_parameter("qdot_max", 0.2)
        self.declare_parameter("watchdog_s", 0.2)
        self.declare_parameter("ramp_s", 1.0)

        cmd_topic: str = self.get_parameter("command_topic").value
        tracking_topic: str = self.get_parameter("tracking_topic").value
        avoidance_topic: str = self.get_parameter("avoidance_topic").value
        self._rate_hz: float = self.get_parameter("rate_hz").value
        self._qdot_max: float = self.get_parameter("qdot_max").value
        self._watchdog_s: float = self.get_parameter("watchdog_s").value
        self._ramp_s: float = self.get_parameter("ramp_s").value

        # ── Channels ─────────────────────────────────────────────────
        self._ch_tracking = _VelocityChannel("tracking", enabled=True)
        self._ch_avoidance = _VelocityChannel("avoidance", enabled=False)

        # ── Pub / Sub ────────────────────────────────────────────────
        self._pub = self.create_publisher(Float64MultiArray, cmd_topic, 1)

        self.create_subscription(
            Float64MultiArray, tracking_topic, self._cb_tracking, 1)
        self.create_subscription(
            Float64MultiArray, avoidance_topic, self._cb_avoidance, 1)

        # ── Timer (main loop) ────────────────────────────────────────
        self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)

        # ── Internal state ───────────────────────────────────────────
        self._t0: Optional[float] = None
        self._shutting_down: bool = False
        self._shutdown_start: float = 0.0
        self.done: bool = False

        # ── Throttled logging counter ────────────────────────────────
        self._tick_count: int = 0

        # ── Startup log ──────────────────────────────────────────────
        self.get_logger().info(
            f"VelocityBlender started\n"
            f"  publish → {cmd_topic}\n"
            f"  tracking  ← {tracking_topic}  (enabled)\n"
            f"  avoidance ← {avoidance_topic} (predisposed, disabled)\n"
            f"  rate={self._rate_hz} Hz  qdot_max={self._qdot_max}  "
            f"watchdog={self._watchdog_s} s  ramp={self._ramp_s} s"
        )

    # ── Subscription callbacks ────────────────────────────────────────

    def _cb_tracking(self, msg: Float64MultiArray) -> None:
        self._ch_tracking.update(list(msg.data))

    def _cb_avoidance(self, msg: Float64MultiArray) -> None:
        self._ch_avoidance.update(list(msg.data))

    # ── Main tick ─────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self.done:
            return

        # Shutdown phase: publish zeros for ~0.5 s then signal done.
        if self._shutting_down:
            self._publish_zeros()
            if time.monotonic() - self._shutdown_start > 0.5:
                self._timer.cancel()
                self.done = True
            return

        # Blend contributions
        qdot = (
            self._ch_tracking.contribution(self._watchdog_s)
            + self._ch_avoidance.contribution(self._watchdog_s)
        )

        # Cosine ramp-up
        ramp = 1.0
        if np.any(qdot != 0.0):
            if self._t0 is None:
                self._t0 = time.monotonic()
            ramp = cosine_ramp(time.monotonic() - self._t0, self._ramp_s)
        qdot *= ramp

        # Per-joint clamp
        qdot = clamp_joints(qdot, self._qdot_max)

        # Publish
        msg = Float64MultiArray()
        msg.data = qdot.tolist()
        self._pub.publish(msg)

        # Throttled log (1 Hz)
        self._tick_count += 1
        if self._tick_count % int(self._rate_hz) == 0:
            trk = ("OK" if not self._ch_tracking.timed_out(self._watchdog_s)
                   else "TIMEOUT")
            avd = ("OK" if not self._ch_avoidance.timed_out(self._watchdog_s)
                   else "off")
            peak = float(np.max(np.abs(qdot)))
            self.get_logger().info(
                f"trk={trk}  avd={avd}  |qdot|_max={peak:.4f}  ramp={ramp:.2f}"
            )

    # ── Helpers ───────────────────────────────────────────────────────

    def _publish_zeros(self) -> None:
        msg = Float64MultiArray()
        msg.data = [0.0] * NUM_JOINTS
        self._pub.publish(msg)

    def request_stop(self) -> None:
        """Begin graceful shutdown (called from main or signal handler)."""
        if not self._shutting_down:
            self._shutting_down = True
            self._shutdown_start = time.monotonic()
            if rclpy.ok():
                self.get_logger().info("Shutdown requested — sending zeros …")


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║  main()                                                                ║
# ╚═════════════════════════════════════════════════════════════════════════╝

import rclpy  # noqa: E402  (already imported via utils, but explicit for main)


def main(args=None) -> None:
    run_node_main(VelocityBlenderNode, args=args)


if __name__ == "__main__":
    main()
