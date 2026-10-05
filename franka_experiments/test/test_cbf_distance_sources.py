"""cbf_safety_filter merges several distance sources into one ObstacleSnap."""

import types

from builtin_interfaces.msg import Time
from franka_msgs.msg import LinkDistance, MultiLinkDistance

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter


class _FilterStub:
    _latency_fields = staticmethod(CBFSafetyFilter._latency_fields)
    _range_field = staticmethod(CBFSafetyFilter._range_field)

    def __init__(self):
        self.P = types.SimpleNamespace(distance_capture_skew_tol=0.05,
                                       distance_capture_age_max=1.0,
                                       zone_r_active=1.0, d_safe=0.10)
        self.now = 100.0
        self._obs_src = {}
        self._obs = None
        self._cap_warned = False
        self._diag_cap_age = 0.0
        self._empty_since = None
        self._empty_close = False
        self._empty_faulted = False
        self.rate_calls = []

    def _now(self):
        return self.now

    def _track_input_rate(self, t_cap):
        self.rate_calls.append(t_cap)

    def get_logger(self):
        return types.SimpleNamespace(warn=lambda *a, **k: None)


def _msg(t_cap, links=('fr3_link5', 'fr3_link5')):
    msg = MultiLinkDistance()
    msg.header.stamp = Time(sec=int(t_cap), nanosec=int(round((t_cap % 1) * 1e9)))
    for link in links:
        ld = LinkDistance(robot_link_name=link, distance=0.3, valid=True)
        msg.links.append(ld)
    return msg


def _feed(f, msg, src, now):
    f.now = now
    CBFSafetyFilter._on_distances(f, msg, src)


def test_single_source_is_unchanged():
    f = _FilterStub()
    _feed(f, _msg(99.98), 0, 100.0)
    assert f._obs is f._obs_src[0]
    assert [ob.cp_label for ob in f._obs.items] == ['fr3_link5#0', 'fr3_link5#1']
    assert all(ob.t_cap is None for ob in f._obs.items)
    assert f.rate_calls == [f._obs.t_cap]


def test_two_sources_keep_labels_capture_times_and_oldest_stamp():
    f = _FilterStub()
    _feed(f, _msg(99.90), 0, 99.95)
    _feed(f, _msg(99.97, links=('fr3_link5',)), 1, 100.0)
    labels = [ob.cp_label for ob in f._obs.items]
    assert labels == ['fr3_link5#0', 'fr3_link5#1', '1:fr3_link5#0']
    t_caps = [round(ob.t_cap, 3) for ob in f._obs.items]
    assert t_caps == [99.90, 99.90, 99.97]
    # staleness follows the source that published longest ago
    assert f._obs.stamp == 99.95
    # the input rate is measured on source 0 only
    assert len(f.rate_calls) == 1


def test_rate_follows_the_only_publishing_source():
    f = _FilterStub()
    _feed(f, _msg(99.97), 1, 100.0)
    assert f._obs is f._obs_src[1]
    assert f.rate_calls and f._obs.items[0].cp_label == '1:fr3_link5#0'
