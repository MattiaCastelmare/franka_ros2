"""Node lifecycle and topic-name resolution at runtime.

OWNS
----
What a *running* node needs from the ROS graph, independent of any domain:

* :func:`run_node_main` / :func:`teardown` — the spin/shutdown boilerplate every
  ``main()`` in this package uses
* :func:`get_namespace_from_config` and the ``resolve_*_topic`` family —
  turning the bringup namespace into concrete topic names

DOES NOT OWN
------------
* Launch-time composition (argument declaration, controller YAML generation) —
  that is ``utils.launch_support``.
* Reading YAML off disk — that is ``utils.config``.
* Parameter declaration/validation — that is ``utils.params``.

Hot-path note: nothing here runs inside a timer or subscription callback.
``run_node_main`` is called once per process; the resolvers run in ``__init__``.

Moved out of ``utils.ros`` in Phase 2; the bodies are byte-identical relocations
and every symbol is still importable from ``utils.ros``.
"""

from __future__ import annotations

import os
from typing import Any, Callable, Optional, Sequence

import yaml

import rclpy
import rclpy.executors
from rclpy.node import Node

from .constants import AVOIDANCE_TOPIC_SUFFIX, TRACKING_TOPIC_SUFFIX
from .config import load_franka_config_defaults  # noqa: F401


def get_namespace_from_config(robot_key: str = 'ROBOT1') -> str:
    """Read namespace from ``franka_bringup/config/franka.config.yaml``.

    Returns the namespace string, or ``''`` if not found / any error.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        bringup_share = get_package_share_directory('franka_bringup')
        config_path = os.path.join(
            bringup_share, 'config', 'franka.config.yaml')
        with open(config_path, 'r') as fh:
            config = yaml.safe_load(fh)
        if config and robot_key in config:
            return str(config[robot_key].get('namespace', '')).strip()
    except Exception:  # noqa: BLE001
        pass
    return ''

def build_namespaced_topic(suffix: str, robot_key: str = 'ROBOT1') -> str:
    """Prefix *suffix* with ``/<ns>/`` if a namespace is configured."""
    ns = get_namespace_from_config(robot_key)
    if ns:
        return f'/{ns}/{suffix}'
    return f'/{suffix}'


_RT_CONTROLLER_NAME: str = 'rt_velocity_executor_controller'

def resolve_tracking_topic(robot_key: str = 'ROBOT1') -> str:
    """Auto-detect namespace → build ``tracking_qdot`` topic."""
    return build_namespaced_topic(TRACKING_TOPIC_SUFFIX, robot_key)

def resolve_avoidance_topic(robot_key: str = 'ROBOT1') -> str:
    """Auto-detect namespace → build ``avoidance_qdot`` topic."""
    return build_namespaced_topic(AVOIDANCE_TOPIC_SUFFIX, robot_key)

def resolve_topic_with_deprecated_alias(
    node: Node,
    new_param: str,
    deprecated_param: str,
    default_value: str,
) -> str:
    """Resolve a topic parameter honouring a deprecated alias.

    If *deprecated_param* is set and *new_param* still equals *default_value*,
    the deprecated value wins (with a deprecation warning).
    """
    new_val: str = node.get_parameter(new_param).value
    old_val: str = node.get_parameter(deprecated_param).value
    if old_val:
        node.get_logger().warn(
            f"Parameter '{deprecated_param}' is DEPRECATED — "
            f"use '{new_param}' instead.")
        if new_val == default_value:
            return old_val
    return new_val

def run_node_main(
    node_factory: Callable[[], Node],
    args: Optional[Sequence[str]] = None,
) -> None:
    """Executor-based main loop for nodes with ``done`` flag + ``request_stop``.

    The node returned by *node_factory* must expose:

    * ``done: bool`` — set to ``True`` when the node is finished.
    * ``request_stop()`` — called on first ``KeyboardInterrupt``.
    """
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    node = node_factory()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.request_stop()
        # Keep spinning so the timer can publish zeros and set node.done
        try:
            while rclpy.ok() and not node.done:
                executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            pass  # 2nd Ctrl-C → exit immediately
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        teardown(node)

def teardown(node: Node) -> None:
    """Destroy *node* and shutdown rclpy (best-effort, no exceptions)."""
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


# MOVED here from nodes/experiment_logger.py (Phase 3): ROS time and
# message marshalling belong with the other runtime helpers. Bodies
# unchanged; names kept (leading underscore included) so no call site moves.
import math  # noqa: E402
import numpy as np  # noqa: E402
from typing import Sequence  # noqa: E402

from .constants import NUM_JOINTS  # noqa: E402


def _now_sec(node: Node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


def _stamp_to_sec(msg_stamp, fallback: float) -> float:
    sec = float(msg_stamp.sec) + float(msg_stamp.nanosec) * 1e-9
    return sec if sec > 0.0 else fallback


def _as_list7(data: Sequence[float]) -> np.ndarray:
    arr = np.full(NUM_JOINTS, np.nan, dtype=float)
    n = min(NUM_JOINTS, len(data))
    if n > 0:
        arr[:n] = np.asarray(data[:n], dtype=float)
    return arr
