"""The CBFDIAG line: the three velocities that say who is winning.

The line already carried the robot's own radial speed (as ``hdot``) and the
fastest obstacle over ALL rows (as ``vobs``), which are for DIFFERENT rows and
so cannot be read against each other. ``vel[obs/rob/rel]`` reports all three
for the ONE row the rest of the line describes.

``vobs`` stays — the fastest approach anywhere is worth a field — but it is now
labelled with the control point that reported it. A hardware line reading
``vobs=+1.850 vel[obs/rob/rel]=+0.067/...`` is not a contradiction and not a
bug in either number: it is two obstacles, and until the label was there it was
unreadable.
"""

import numpy as np

from _cbf_builder_harness import NV, make_builder, make_obstacle, run
from franka_experiments.utils.cbf_state_rows import N_SLACK
from franka_experiments.utils.logging_utils import format_cbf_diag

PR = (0.5, 0.0, 0.5)
PH = (0.5, -0.25, 0.5)


class _Rows:
    diag_h_hold = diag_v_obs = diag_vapp = diag_hbrake = 0.0
    diag_hunc = diag_hlat = diag_esc_w = diag_outrun_r = diag_outrun_w = 0.0
    diag_sigma = float('nan')
    diag_w = diag_wq = None


def _line(con, qdot, rows=None):
    n = con.A.shape[0]
    return format_cbf_diag(
        now=1.0, con=con, rows=rows if rows is not None else _Rows(),
        caps=(0.0, 0.0, 0.0, 0.0),
        h_qp=np.zeros(n), qdot=qdot, qdot_cbf=qdot,
        qddot_safe=np.zeros(NV), qddot_nom=np.zeros(NV), qddot_real=np.zeros(NV),
        slack=np.zeros(N_SLACK), n_active_cps=0, vel_ratio=np.zeros(NV),
        vel_bite=np.zeros(NV, bool), slew_bite=np.zeros(NV, bool), cap_age=0.0)


def _vel(line):
    tok = [t for t in line.split() if t.startswith('vel[obs/rob/rel]=')][0]
    return [float(x) for x in tok.split('=')[1].split('/')]


def test_the_three_velocities_are_reported_and_consistent():
    qdot = np.full(NV, 0.05)
    con = run(make_builder(obstacle_velocity_source='tracker'),
              [make_obstacle(d=0.25, pr=PR, ph=PH, v=(0.0, 0.4, 0.0), frames_seen=20)],
              n_frames=5, qdot=0.05)
    obs, rob, rel = _vel(_line(con, qdot))
    assert np.isclose(obs, con.v_obs[0], atol=1e-3)
    assert np.isclose(rob, float(con.A[0] @ qdot), atol=1e-3)
    assert np.isclose(rel, rob - obs, atol=1e-3), (obs, rob, rel)


def test_the_obstacle_speed_is_this_row_s_not_the_global_maximum():
    """Two obstacles, the FAR one moving fast: the line is labelled by the
    close one, and its velocity must be the close one's. Reading the global
    maximum next to this row's hdot is what made the two incomparable."""
    b = make_builder(obstacle_velocity_source='tracker')
    con = run(b, [make_obstacle(d=0.20, pr=PR, ph=PH, link='fr3_link5',
                                v=(0.0, 0.05, 0.0), frames_seen=20),
                  make_obstacle(d=0.60, pr=(0.5, 0.0, 0.9), ph=(0.5, -0.6, 0.9),
                                link='fr3_link6', v=(0.0, 1.2, 0.0), frames_seen=20)],
              n_frames=5, qdot=0.0)
    i = int(np.argmin(con.h_bar))
    assert con.links[i].startswith('fr3_link5'), con.links
    obs, _, _ = _vel(_line(con, np.zeros(NV)))
    assert np.isclose(obs, con.v_obs[i], atol=1e-3)
    assert obs < 0.5 * con.v_obs.max(), (obs, con.v_obs)


def test_a_static_obstacle_reports_the_robot_s_own_motion_as_the_relative_rate():
    """Nothing moving out there: rel is exactly rob, so the line says plainly
    that every metre per second of closure is the robot's own doing."""
    con = run(make_builder(obstacle_velocity_source='tracker'),
              [make_obstacle(d=0.25, pr=PR, ph=PH)], n_frames=5, qdot=0.05)
    obs, rob, rel = _vel(_line(con, np.full(NV, 0.05)))
    assert obs == 0.0 and np.isclose(rel, rob, atol=1e-9)


# ── vobs, and which control point it belongs to ─────────────────────────────

def _field(line, key):
    return [t for t in line.split() if t.startswith(key + '=')][0].split('=', 1)[1]


def test_the_global_maximum_is_labelled_with_the_row_that_reported_it():
    """Two obstacles, the FAR one moving fast. The line is labelled by the
    close one, so vobs must say out loud that it is talking about the other."""
    b = make_builder(obstacle_velocity_source='tracker')
    con = run(b, [make_obstacle(d=0.20, pr=PR, ph=PH, link='fr3_link5',
                                v=(0.0, 0.05, 0.0), frames_seen=20),
                  make_obstacle(d=0.60, pr=(0.5, 0.0, 0.9), ph=(0.5, -0.6, 0.9),
                                link='fr3_link6', v=(0.0, 1.2, 0.0),
                                frames_seen=20)],
             n_frames=5, qdot=0.0)
    line = _line(con, np.zeros(NV), rows=b)
    i = int(np.argmin(con.h_bar))
    assert con.links[i].startswith('fr3_link5')
    # vobs is the fast FAR obstacle, and it names link6 — not the row the rest
    # of the line is about.
    assert float(_field(line, 'vobs')) > 0.5
    assert _field(line, 'vobs_cp').startswith('fr3_link6')


def test_vobs_has_a_placeholder_when_nothing_is_approaching():
    """Value parsers split on `key=`; the label must never be empty."""
    b = make_builder(obstacle_velocity_source='tracker')
    con = run(b, [make_obstacle(d=0.25, pr=PR, ph=PH)], n_frames=5, qdot=0.0)
    assert _field(_line(con, np.zeros(NV), rows=b), 'vobs_cp') == '-'


def test_vobs_stays_a_bare_float():
    """The label is its own key rather than a suffix on vobs=, so anything
    doing float() on that value keeps working."""
    b = make_builder(obstacle_velocity_source='tracker')
    con = run(b, [make_obstacle(d=0.25, pr=PR, ph=PH, v=(0.0, 0.4, 0.0),
                                frames_seen=20)], n_frames=5, qdot=0.0)
    assert float(_field(_line(con, np.zeros(NV), rows=b), 'vobs')) > 0.0


def test_the_identity_reset_counter_is_on_the_line_whatever_the_flags():
    """nid= reports discarded closing-speed state. It is outside _zone_field
    on purpose: unlike nrot= it has nothing to do with the zone ladder, so it
    must be visible with the ladder off."""
    b = make_builder(obstacle_velocity_source='residual',
                     obstacle_velocity_identity_jump=0.10,
                     enable_zone_ladder=False)
    con = run(b, [make_obstacle(d=0.25, pr=PR, ph=PH)], n_frames=3, qdot=0.0)
    line = _line(con, np.zeros(NV), rows=b)
    assert 'zone=' not in line
    assert int(_field(line, 'nid')) == 0
