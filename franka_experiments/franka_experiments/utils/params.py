"""Declaration and validation of ROS 2 node parameters.

OWNS
----
The single, uniform way a ``franka_experiments`` node turns a configuration
value into a live, validated, self-documenting ROS parameter:

* :func:`declare_float`  — real-valued knob with optional range check
* :func:`declare_int`    — integer knob with optional range check
* :func:`declare_bool`   — boolean flag
* :func:`declare_str`    — string knob (topic name, frame name, enum choice)
* :func:`declare_vec`    — fixed-length list of floats

Every helper declares the parameter on the node (so it appears in
``ros2 param list`` and can be overridden from a launch file), reads it back,
validates it, and returns the typed value.  A validation failure logs at ERROR
naming the parameter, the received value and the expected range, then raises
:class:`ValueError` — the node dies at construction rather than running with a
bad gain.

DOES NOT OWN
------------
* Reading YAML off disk — that is ``utils.cbf_utils.load_robot_config`` and
  ``utils.ros.load_launch_defaults``.  These helpers take an already-resolved
  default and never touch the filesystem.
* Launch-argument declaration — that is ``utils.ros.declare_*``.
* Any domain knowledge about what a sensible gain is; callers pass the bounds.

Hot-path note: every helper here runs exactly once, during ``__init__``.
Nothing in this module is called from a timer or a subscription callback.
"""

from __future__ import annotations

from typing import Any, Optional, Sequence

from rclpy.node import Node


def _fail(node: Node, name: str, value: Any, expected: str) -> None:
    """Log at ERROR and raise :class:`ValueError` for a bad parameter value.

    Args:
        node: Node whose logger reports the failure.
        name: Parameter name, as declared.
        value: The value actually received.
        expected: Human-readable description of what was expected.

    Raises:
        ValueError: Always.
    """
    msg = (f'parameter "{name}": invalid value {value!r} — expected {expected}')
    node.get_logger().error(msg)
    raise ValueError(msg)


def declare_float(
    node: Node,
    name: str,
    default: float,
    *,
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
    positive: bool = False,
    description: str = '',
) -> float:
    """Declare a float parameter, validate it, and return it.

    Args:
        node: Node to declare the parameter on.
        name: Parameter name.
        default: Value used when nothing overrides it.
        minimum: Inclusive lower bound, or ``None`` for unbounded.
        maximum: Inclusive upper bound, or ``None`` for unbounded.
        positive: When True, require ``value > 0`` (stricter than ``minimum=0``).
        description: Free-text description shown by ``ros2 param describe``.

    Returns:
        The validated parameter value.

    Raises:
        ValueError: If the value is not finite, not positive when required, or
            outside ``[minimum, maximum]``.
    """
    node.declare_parameter(name, float(default))
    value = float(node.get_parameter(name).value)
    if value != value or value in (float('inf'), float('-inf')):
        _fail(node, name, value, 'a finite number')
    if positive and value <= 0.0:
        _fail(node, name, value, 'a strictly positive number')
    if minimum is not None and value < minimum:
        _fail(node, name, value, f'>= {minimum}')
    if maximum is not None and value > maximum:
        _fail(node, name, value, f'<= {maximum}')
    return value


def declare_int(
    node: Node,
    name: str,
    default: int,
    *,
    minimum: Optional[int] = None,
    maximum: Optional[int] = None,
    positive: bool = False,
    description: str = '',
) -> int:
    """Declare an integer parameter, validate it, and return it.

    Args:
        node: Node to declare the parameter on.
        name: Parameter name.
        default: Value used when nothing overrides it.
        minimum: Inclusive lower bound, or ``None`` for unbounded.
        maximum: Inclusive upper bound, or ``None`` for unbounded.
        positive: When True, require ``value > 0``.
        description: Free-text description shown by ``ros2 param describe``.

    Returns:
        The validated parameter value.

    Raises:
        ValueError: If the value is out of range.
    """
    node.declare_parameter(name, int(default))
    value = int(node.get_parameter(name).value)
    if positive and value <= 0:
        _fail(node, name, value, 'a strictly positive integer')
    if minimum is not None and value < minimum:
        _fail(node, name, value, f'>= {minimum}')
    if maximum is not None and value > maximum:
        _fail(node, name, value, f'<= {maximum}')
    return value


def declare_bool(node: Node, name: str, default: bool,
                 *, description: str = '') -> bool:
    """Declare a boolean parameter and return it.

    Args:
        node: Node to declare the parameter on.
        name: Parameter name.
        default: Value used when nothing overrides it.
        description: Free-text description shown by ``ros2 param describe``.

    Returns:
        The parameter value.
    """
    node.declare_parameter(name, bool(default))
    return bool(node.get_parameter(name).value)


def declare_str(
    node: Node,
    name: str,
    default: str,
    *,
    allow_empty: bool = False,
    choices: Optional[Sequence[str]] = None,
    description: str = '',
) -> str:
    """Declare a string parameter, validate it, and return it.

    Args:
        node: Node to declare the parameter on.
        name: Parameter name.
        default: Value used when nothing overrides it.
        allow_empty: When False (the default), an empty string is rejected —
            use this for frame names and topic names.
        choices: When given, the value must be one of these.
        description: Free-text description shown by ``ros2 param describe``.

    Returns:
        The validated parameter value.

    Raises:
        ValueError: If the value is empty when not allowed, or not in
            ``choices``.
    """
    node.declare_parameter(name, str(default))
    value = str(node.get_parameter(name).value)
    if not allow_empty and not value.strip():
        _fail(node, name, value, 'a non-empty string')
    if choices is not None and value not in choices:
        _fail(node, name, value, f'one of {list(choices)}')
    return value


def declare_vec(
    node: Node,
    name: str,
    default: Sequence[float],
    *,
    length: Optional[int] = None,
    description: str = '',
) -> list[float]:
    """Declare a float-list parameter, validate its length, and return it.

    Args:
        node: Node to declare the parameter on.
        name: Parameter name.
        default: Value used when nothing overrides it.
        length: Required number of elements, or ``None`` to accept any length.
        description: Free-text description shown by ``ros2 param describe``.

    Returns:
        The validated parameter value as a list of floats.

    Raises:
        ValueError: If the length does not match ``length``, or an element is
            not finite.
    """
    node.declare_parameter(name, [float(v) for v in default])
    value = [float(v) for v in node.get_parameter(name).value]
    if length is not None and len(value) != length:
        _fail(node, name, value, f'exactly {length} elements')
    for v in value:
        if v != v or v in (float('inf'), float('-inf')):
            _fail(node, name, value, 'all elements finite')
    return value
