"""The LinkDistance track fields: defaults, round-trip, and safety of the zeros.

These four fields are the wire between the tracker node and the safety filter.
The requirement that dominates them is not accuracy, it is that their DEFAULTS
mean "no track" AND are safe — because every publisher that predates the
tracker (real_time_distance itself, every recorded bag, every replay harness)
emits exactly those defaults, and the filter has to keep working when it sees
them.

Needs the built franka_msgs, so it is skipped where the message package is not
on the path.
"""

import numpy as np
import pytest

franka_msgs = pytest.importorskip('franka_msgs.msg')

LinkDistance = franka_msgs.LinkDistance
MultiLinkDistance = franka_msgs.MultiLinkDistance


# ── Defaults ────────────────────────────────────────────────────────────────

def test_a_default_message_says_no_track():
    """0 is the reserved "no track" id — TrackManager hands out 1 and up."""
    ld = LinkDistance()
    assert ld.track_id == 0
    assert ld.frames_seen == 0


def test_a_default_velocity_is_the_zero_vector():
    ld = LinkDistance()
    assert (ld.obstacle_velocity.x, ld.obstacle_velocity.y,
            ld.obstacle_velocity.z) == (0.0, 0.0, 0.0)


def test_a_default_covariance_is_nine_zeros():
    ld = LinkDistance()
    assert len(ld.velocity_covariance) == 9
    assert np.array_equal(np.asarray(ld.velocity_covariance), np.zeros(9))


def test_the_defaults_are_safe_for_the_consumer():
    """The whole backward-compatibility argument in one assertion. With the
    defaults, the projected obstacle speed the barrier would use is exactly 0.0
    and the uncertainty margin is exactly 0.0 — neither can loosen a row, and
    both reduce to not evaluating the term at all."""
    ld = LinkDistance()
    n_hat = np.array([0.3, -0.6, 0.74])
    v = np.array([ld.obstacle_velocity.x, ld.obstacle_velocity.y,
                  ld.obstacle_velocity.z])
    P = np.asarray(ld.velocity_covariance).reshape(3, 3)
    assert max(float(n_hat @ v), 0.0) == 0.0
    assert float(np.sqrt(max(n_hat @ P @ n_hat, 0.0))) == 0.0
    assert ld.frames_seen < 3, 'the defaults must fail the frames_seen gate'


# ── Round-trip ──────────────────────────────────────────────────────────────

def test_a_populated_message_round_trips_through_serialisation():
    from rclpy.serialization import deserialize_message, serialize_message

    ld = LinkDistance()
    ld.robot_link_name = 'fr3_link5'
    ld.distance = 0.123
    ld.valid = True
    ld.track_id = 7
    ld.frames_seen = 42
    ld.obstacle_velocity.x = -0.25
    ld.obstacle_velocity.y = 0.5
    ld.obstacle_velocity.z = 1.5
    P = np.arange(9, dtype=np.float64) * 0.01
    ld.velocity_covariance = P

    back = deserialize_message(serialize_message(ld), LinkDistance)
    assert back.track_id == 7 and back.frames_seen == 42
    assert back.obstacle_velocity.z == 1.5
    assert np.array_equal(np.asarray(back.velocity_covariance), P)
    assert back.robot_link_name == 'fr3_link5' and back.distance == 0.123


def test_a_default_message_round_trips_unchanged():
    """A bag recorded before the tracker existed must deserialise to the safe
    defaults, not to garbage."""
    from rclpy.serialization import deserialize_message, serialize_message

    back = deserialize_message(serialize_message(LinkDistance()), LinkDistance)
    assert back.track_id == 0 and back.frames_seen == 0
    assert np.array_equal(np.asarray(back.velocity_covariance), np.zeros(9))


def test_multi_link_distance_still_carries_the_extended_entries():
    msg = MultiLinkDistance()
    a, b = LinkDistance(), LinkDistance()
    a.track_id = 3
    msg.links = [a, b]
    assert [x.track_id for x in msg.links] == [3, 0]


# ── Field contracts ─────────────────────────────────────────────────────────

def test_covariance_is_fixed_length_nine():
    """float64[9], not a bounded sequence: a 3x3 row-major matrix has exactly
    nine entries and a consumer reshapes it without checking."""
    ld = LinkDistance()
    with pytest.raises((AssertionError, ValueError)):
        ld.velocity_covariance = np.zeros(6)


def test_track_id_is_unsigned():
    ld = LinkDistance()
    with pytest.raises((AssertionError, ValueError, OverflowError)):
        ld.track_id = -1


def test_the_covariance_reshapes_row_major_into_a_symmetric_matrix():
    """The publisher writes P.ravel() and the consumer reads reshape(3, 3); if
    the two ever disagreed on the order, an asymmetric quadratic form would go
    unnoticed because n^T P n uses the symmetric part anyway — so pin it here."""
    ld = LinkDistance()
    P = np.array([[4.0, 1.0, 0.5], [1.0, 9.0, -0.25], [0.5, -0.25, 1.0]])
    ld.velocity_covariance = P.ravel()
    back = np.asarray(ld.velocity_covariance).reshape(3, 3)
    assert np.array_equal(back, P)
