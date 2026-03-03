"""Joint-state auto-detection and Pinocchio mapping helper."""

from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState

from .constants import FR3_JOINT_NAMES, AUTO_SENTINEL
from .ros import get_namespace_from_config

try:
    import pinocchio as pin
except ImportError as exc:
    raise ImportError(
        'pinocchio is required but not installed. '
        'Install with: pip install pin'
    ) from exc


class JointStateManager:
    """Manages joint-state subscription with optional auto-detection.

    Composes into any ``Node`` — does **not** inherit from ``Node``.

    Parameters
    ----------
    node : Node
        Parent ROS 2 node (used for subscriptions, timers, logging).
    pin_model, pin_joint_ids
        Pinocchio model and list of FR3 arm joint IDs.
    topic_param : str
        If ``AUTO_SENTINEL``, auto-detection is used; otherwise subscribe
        directly to the given topic.
    """

    def __init__(
        self,
        node: Node,
        pin_model,
        pin_joint_ids: List[int],
        *,
        topic_param: str = AUTO_SENTINEL,
    ) -> None:
        self._node = node
        self._pin_model = pin_model
        self._pin_joint_ids = pin_joint_ids

        # Public read-only state
        self._q: Optional[np.ndarray] = None
        self._q_full: Optional[np.ndarray] = None
        self._stamp = node.get_clock().now()

        # Internal
        self._js_index_map: Optional[List[int]] = None
        self._sub: Optional[object] = None
        self._topic_resolved: Optional[str] = None
        self._auto_detect = (topic_param == AUTO_SENTINEL)
        self._candidates: List[str] = []
        self._on_fallback = False

        if self._auto_detect:
            ns = get_namespace_from_config()
            if ns:
                self._candidates.append(f'/{ns}/franka/joint_states')
                self._candidates.append(f'/{ns}/joint_states')
            self._candidates.append('/franka/joint_states')
            self._candidates.append('/joint_states')

            self._discovery_start = time.monotonic()
            self._discovery_timer = node.create_timer(
                1.0, self._discover_topic)
            node.get_logger().info(
                f'Joint-state auto-detect enabled  '
                f'candidates={self._candidates}')
        else:
            self._topic_resolved = topic_param
            self._sub = node.create_subscription(
                JointState, topic_param, self._joint_state_cb, 10)
            node.get_logger().info(
                f'Joint-state topic (explicit): {topic_param}')

    # -- Properties --------------------------------------------------------

    @property
    def q(self) -> Optional[np.ndarray]:
        """Latest 7 arm joint positions, or ``None``."""
        return self._q

    @property
    def q_full(self) -> Optional[np.ndarray]:
        """Latest full Pinocchio q vector, or ``None``."""
        return self._q_full

    @property
    def stamp(self):
        """ROS Time of last joint-state update."""
        return self._stamp

    @property
    def topic_resolved(self) -> Optional[str]:
        """Currently subscribed topic (or ``None`` if not yet resolved)."""
        return self._topic_resolved

    # -- Discovery ---------------------------------------------------------

    def _discover_topic(self) -> None:
        """Try to find a JointState topic on the ROS graph (called at 1 Hz)."""
        if self._q is not None:
            self._node.get_logger().info(
                f'Receiving joint states on '
                f'{self._topic_resolved} — discovery complete')
            self._discovery_timer.cancel()
            return

        available = self._node.get_topic_names_and_types()
        js_topics = {
            name
            for name, types in available
            if 'sensor_msgs/msg/JointState' in types
        }

        chosen: Optional[str] = None
        for candidate in self._candidates:
            if candidate in js_topics:
                chosen = candidate
                break

        if chosen is None:
            extras = sorted(js_topics - set(self._candidates))
            if extras:
                chosen = extras[0]

        if chosen is not None and chosen != self._topic_resolved:
            self._subscribe(chosen, source='auto-detected')
            self._on_fallback = False
            return

        elapsed = time.monotonic() - self._discovery_start
        if elapsed > 2.0 and self._sub is None:
            fallback = '/joint_states'
            self._subscribe(fallback, source='fallback (2 s timeout)')
            self._on_fallback = True
            self._node.get_logger().warn(
                f'No JointState topic discovered after {elapsed:.1f} s\n'
                f'  candidates : {self._candidates}\n'
                f'  graph      : {sorted(js_topics) or "(none)"}\n'
                f'  → falling back to {fallback}, will keep retrying …')

    def _subscribe(self, topic: str, *, source: str = '') -> None:
        """(Re-)create the JointState subscription on *topic*."""
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
        self._sub = self._node.create_subscription(
            JointState, topic, self._joint_state_cb, 10)
        self._topic_resolved = topic
        self._node.get_logger().info(
            f'Joint-state topic: {topic}  ({source})\n'
            f'  candidates tried: {self._candidates}')

    # -- Callback ----------------------------------------------------------

    def _joint_state_cb(self, msg: JointState) -> None:
        """Store latest joint positions for the 7 arm joints."""
        if self._js_index_map is None:
            try:
                self._js_index_map = [
                    msg.name.index(jn) for jn in FR3_JOINT_NAMES
                ]
            except ValueError:
                return

        if len(msg.position) < max(self._js_index_map) + 1:
            return

        q7 = np.array([msg.position[i] for i in self._js_index_map])
        self._q = q7

        q_full = pin.neutral(self._pin_model)
        for k, pid in enumerate(self._pin_joint_ids):
            idx_q = self._pin_model.joints[pid].idx_q
            q_full[idx_q] = q7[k]
        self._q_full = q_full
        self._stamp = self._node.get_clock().now()

    # -- Cleanup -----------------------------------------------------------

    def cancel_discovery(self) -> None:
        """Cancel the discovery timer (call during shutdown)."""
        if self._auto_detect and hasattr(self, '_discovery_timer'):
            try:
                self._discovery_timer.cancel()
            except Exception:
                pass
