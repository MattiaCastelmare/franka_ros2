"""One definition of the control-point label, and both sides of the wire on it.

``link#k`` is a contract between three places: ``perception_msgs`` writes the
track fields of the entry a ``skip_keys`` label names, ``cbf_safety_filter``
keys every per-control-point filter on the same label, and CBFDIAG prints it.

It used to be open-coded on each side and the two disagreed. The publisher
counted every entry; the consumer counted only the entries that had survived
its own filters (valid, inside the obstacle horizon, enough Jacobian leverage,
finite), so dropping one control point shifted every later one on that link
down by one. A self-detection skip then suppressed the wrong control point, and
the builder's per-label filters — the barrier's recovery EMA, the residual
closing-speed estimator, its rotation guard, the v_obs median, the uncertainty
EMA — were handed another control point's state mid-run.

Needs the built franka_msgs, so it is skipped where the message package is not
on the path.
"""

import pytest

franka_msgs = pytest.importorskip('franka_msgs.msg')

from franka_experiments.utils.perception_msgs import labelled_links  # noqa: E402

LinkDistance = franka_msgs.LinkDistance
MultiLinkDistance = franka_msgs.MultiLinkDistance


def _msg(spec):
    """spec: list of (link, valid). Order is the wire order."""
    m = MultiLinkDistance()
    out = []
    for link, valid in spec:
        ld = LinkDistance()
        ld.robot_link_name = link
        ld.valid = bool(valid)
        ld.distance = 0.25
        out.append(ld)
    m.links = out
    return m


def test_the_counter_is_per_link_and_positional():
    m = _msg([('fr3_link5', True), ('fr3_link5', True),
              ('fr3_link6', True), ('fr3_link5', True)])
    assert [lbl for lbl, _ in labelled_links(m)] == [
        'fr3_link5#0', 'fr3_link5#1', 'fr3_link6#0', 'fr3_link5#2']


def test_an_invalid_entry_still_advances_the_counter():
    """THE regression. The middle control point has no usable measurement this
    frame; the third must keep being #2, not become #1."""
    m = _msg([('fr3_link5', True), ('fr3_link5', False), ('fr3_link5', True)])
    assert [lbl for lbl, _ in labelled_links(m)] == [
        'fr3_link5#0', 'fr3_link5#1', 'fr3_link5#2']


def test_the_label_of_an_entry_does_not_depend_on_the_others_validity():
    """Same wire order, different validity pattern frame to frame: a given
    POSITION keeps its label, which is what makes it an identity."""
    spec = [('fr3_link5', True), ('fr3_link5', True), ('fr3_link5', True)]
    base = [lbl for lbl, _ in labelled_links(_msg(spec))]
    for k in range(3):
        flipped = [(l, i != k) for i, (l, _) in enumerate(spec)]
        assert [lbl for lbl, _ in labelled_links(_msg(flipped))] == base


def test_the_consumer_labels_agree_with_the_publisher_labels():
    """The publisher annotates through labelled_links; the consumer parses
    through it. Both must land on the same string for the same entry, and the
    consumer keeps only the valid ones — WITHOUT renumbering them."""
    m = _msg([('fr3_link5', True), ('fr3_link5', False),
              ('fr3_link6', True), ('fr3_link6', True)])
    publisher = {lbl for lbl, _ in labelled_links(m)}
    consumer = [lbl for lbl, ld in labelled_links(m) if ld.valid]
    assert consumer == ['fr3_link5#0', 'fr3_link6#0', 'fr3_link6#1']
    assert set(consumer) <= publisher


def test_a_skip_key_lands_on_the_control_point_it_names():
    """SelfDetectionMonitor suppresses a velocity by label. Exercise the real
    annotate_track_fields against a pipeline double and check the skipped entry
    is the one whose label was passed — behind an invalid entry, which is the
    case that used to be off by one."""
    from franka_experiments.utils.perception_msgs import annotate_track_fields

    class _Pipe:
        def velocity_for_point(self, p):
            return 5, 20, (0.0, 0.0, 1.0), tuple([0.0] * 9)

    m = _msg([('fr3_link5', True), ('fr3_link5', False),
              ('fr3_link5', True), ('fr3_link5', True)])
    annotate_track_fields(m, _Pipe(), skip_keys=('fr3_link5#2',))
    got = {lbl: ld.track_id for lbl, ld in labelled_links(m)}
    assert got['fr3_link5#0'] == 5      # annotated
    assert got['fr3_link5#1'] == 0      # invalid, never annotated
    assert got['fr3_link5#2'] == 0      # SKIPPED — the one named
    assert got['fr3_link5#3'] == 5      # annotated
