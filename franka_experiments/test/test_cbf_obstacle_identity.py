"""Two obstacles, and the state that must not be shared between them.

Every temporal filter on the closing-speed path is keyed on the CONTROL POINT
and assumes the control point keeps looking at the same obstacle. With one
static obstacle and one moving one — the scene this repository actually runs —
that assumption breaks the instant the moving obstacle becomes the nearest,
because ``closest_point_human`` is the argmin over all obstacle pixels.

Three separate defects met there on hardware. Each has its own test below:

* the residual estimator differenced the distance ACROSS the switch and
  reported up to the 2.0 m/s clamp of fabricated approach speed;
* ``rows_vobs`` was appended outside the finiteness guard, so one dropped
  obstacle shifted every later row's ``v_obs`` by one — invisible with a single
  obstacle row, which is why it survived;
* the control-point label was rebuilt from the rows the builder had KEPT, so
  dropping one control point handed its filter state to another.
"""

import numpy as np

from _cbf_builder_harness import make_builder, make_js, make_obs, make_obstacle

PR = (0.5, 0.0, 0.5)


def _ph(y):
    return (0.5, y, 0.5)


def _drive(builder, frames):
    """Feed (obstacles, t) pairs and return the last ConstraintSnap."""
    con = None
    for obstacles, t in frames:
        con = builder.build(make_js(q=0.1, qdot=0.0, stamp=t),
                            make_obs(obstacles, stamp=t), t)
    return con


# ── The residual must not difference across an obstacle switch ──────────────

def _switch_frames(label='fr3_link5#0'):
    """Frame 0-2: a static obstacle at 0.40 m. Frame 3: a DIFFERENT obstacle,
    0.16 m nearer and 0.24 m away in space — a switch, not a motion.

    0.24 m over 33 ms is 7.2 m/s, far past obstacle_velocity_max, so no
    obstacle this filter admits could have travelled it.
    """
    dt = 1.0 / 30.0
    far = make_obstacle(d=0.40, pr=PR, ph=_ph(-0.40), track_id=7,
                        cp_label=label)
    near = make_obstacle(d=0.24, pr=PR, ph=_ph(-0.16), track_id=9,
                         cp_label=label)
    return [([far], 0.0), ([far], dt), ([far], 2 * dt), ([near], 3 * dt)]


def test_without_the_guard_the_switch_fabricates_approach_speed():
    """The bug, pinned so the fix has something to be a fix OF.

    0.16 m of step over one 33 ms frame is 4.8 m/s of implied approach, clipped
    to obstacle_velocity_max = 2.0 and then EMA'd at alpha = 0.7, so the first
    frame reports 0.3 * 2.0 = 0.6 m/s for two obstacles that are both standing
    still. Transient — it decays once the estimator has re-anchored — but
    through k1 = 10.5 that single frame is 6.3 rad/s^2 of demanded retreat on a
    row that binds every tick, and it also scales the retreat cap and feeds the
    outrun/escalation test. On hardware the switching persisted and it sat at
    the 2.0 m/s clamp.
    """
    con = _drive(make_builder(obstacle_velocity_source='residual',
                              obstacle_velocity_identity_jump=0.0),
                 _switch_frames())
    assert con.v_obs[0] > 0.5, con.v_obs


def test_with_the_guard_a_switch_reports_no_obstacle_velocity():
    b = make_builder(obstacle_velocity_source='residual',
                     obstacle_velocity_identity_jump=0.10)
    con = _drive(b, _switch_frames())
    assert con.v_obs[0] == 0.0, con.v_obs
    assert b.diag_ident_reset == 1, b.diag_ident_reset


def test_the_barrier_still_takes_the_closer_distance_immediately():
    """The reset must not cost reactivity: h̄ follows the NEW obstacle on the
    same frame. Only the velocity estimate is discarded, never the geometry."""
    b = make_builder(obstacle_velocity_source='residual',
                     obstacle_velocity_identity_jump=0.10)
    con = _drive(b, _switch_frames())
    assert np.isclose(con.h_bar[0], 0.24 - b._P.d_safe, atol=1e-9), con.h_bar


def test_the_tracked_velocity_survives_the_reset():
    """The tracker is memoryless — it projects the NEW track's own Kalman
    velocity — so a genuinely approaching obstacle is still reported on the
    frame it takes over. This is what makes returning 0.0 for the residual a
    safe answer rather than a blind one."""
    dt = 1.0 / 30.0
    lbl = 'fr3_link5#0'
    far = make_obstacle(d=0.40, pr=PR, ph=_ph(-0.40), track_id=7, cp_label=lbl,
                        v=(0.0, 0.0, 0.0), frames_seen=20)
    # New obstacle, closing at 0.5 m/s along +y, i.e. toward the control point.
    near = make_obstacle(d=0.24, pr=PR, ph=_ph(-0.16), track_id=9, cp_label=lbl,
                         v=(0.0, 0.5, 0.0), frames_seen=20)
    frames = [([far], 0.0), ([far], dt), ([far], 2 * dt), ([near], 3 * dt)]
    b = make_builder(obstacle_velocity_source='tracker',
                     obstacle_velocity_identity_jump=0.10)
    con = _drive(b, frames)
    assert b.diag_ident_reset == 1
    assert np.isclose(con.v_obs[0], 0.5, atol=1e-6), con.v_obs


def test_a_track_id_change_alone_is_enough():
    """Same point in space, new identity: the tracker says these are different
    bodies and that is authoritative, ids being never reused."""
    dt = 1.0 / 30.0
    lbl = 'fr3_link5#0'
    a = make_obstacle(d=0.40, pr=PR, ph=_ph(-0.40), track_id=7, cp_label=lbl)
    bb = make_obstacle(d=0.38, pr=PR, ph=_ph(-0.38), track_id=8, cp_label=lbl)
    b = make_builder(obstacle_velocity_source='residual',
                     obstacle_velocity_identity_jump=0.10)
    _drive(b, [([a], 0.0), ([a], dt), ([bb], 2 * dt)])
    assert b.diag_ident_reset == 1, b.diag_ident_reset


def test_losing_a_track_is_not_an_identity_change():
    """track_id 0 means "no confirmed track behind this point", which one
    obstacle produces routinely through an occlusion or the scene guard.
    Resetting there would silence the residual in the case it exists for."""
    dt = 1.0 / 30.0
    lbl = 'fr3_link5#0'
    tracked = make_obstacle(d=0.40, pr=PR, ph=_ph(-0.40), track_id=7,
                            cp_label=lbl)
    lost = make_obstacle(d=0.39, pr=PR, ph=_ph(-0.39), track_id=0,
                         cp_label=lbl)
    b = make_builder(obstacle_velocity_source='residual',
                     obstacle_velocity_identity_jump=0.10)
    _drive(b, [([tracked], 0.0), ([tracked], dt), ([lost], 2 * dt)])
    assert b.diag_ident_reset == 0, b.diag_ident_reset


def test_the_argmin_s_own_patch_hopping_does_not_trigger_it():
    """The nearest point hops between neighbouring surface patches of ONE body
    every frame — a few centimetres. Triggering on that would discard the
    residual permanently and take the close-in limb detection with it."""
    dt = 1.0 / 30.0
    lbl = 'fr3_link5#0'
    b = make_builder(obstacle_velocity_source='residual',
                     obstacle_velocity_identity_jump=0.10)
    frames = []
    for k in range(8):
        hop = 0.03 * (k % 2)          # 3 cm of patch hop, alternating
        frames.append(([make_obstacle(d=0.40 - hop, pr=PR,
                                      ph=_ph(-0.40 + hop), track_id=7,
                                      cp_label=lbl)], k * dt))
    _drive(b, frames)
    assert b.diag_ident_reset == 0, b.diag_ident_reset


def test_a_dropped_depth_frame_relaxes_the_bound_instead_of_tripping_it():
    """The threshold is max(floor, obstacle_velocity_max * dt): three frame
    periods of genuine 1.5 m/s motion is 0.15 m and must NOT read as a switch,
    or every dropped frame during a real approach would discard the estimate."""
    lbl = 'fr3_link5#0'
    b = make_builder(obstacle_velocity_source='residual',
                     obstacle_velocity_identity_jump=0.10)
    dt = 3.0 / 30.0                   # two frames dropped
    o0 = make_obstacle(d=0.50, pr=PR, ph=_ph(-0.50), track_id=7, cp_label=lbl)
    o1 = make_obstacle(d=0.35, pr=PR, ph=_ph(-0.35), track_id=7, cp_label=lbl)
    _drive(b, [([o0], 0.0), ([o0], dt), ([o1], 2 * dt)])
    assert b.diag_ident_reset == 0, b.diag_ident_reset


# ── v_obs must be indexed BY ROW ────────────────────────────────────────────

def test_a_dropped_obstacle_does_not_shift_the_other_s_closing_speed():
    """A non-finite distance is dropped by the builder's finiteness guard. Its
    v_obs used to be appended anyway, so the row that DID survive read the
    dropped obstacle's zero instead of its own closing speed — the moving
    obstacle's velocity landing on the static obstacle's row, and vice versa.
    """
    # First entry: NaN distance. It clears the horizon test (NaN > horizon is
    # False) and is dropped later, by the finiteness guard — which is exactly
    # the path the misalignment lived on.
    dropped = make_obstacle(d=float('nan'), pr=PR, ph=_ph(-0.30),
                            link='fr3_link4', cp_label='fr3_link4#0')
    kept = make_obstacle(d=0.25, pr=PR, ph=_ph(-0.25), link='fr3_link5',
                         cp_label='fr3_link5#0', v=(0.0, 0.4, 0.0),
                         frames_seen=20)
    b = make_builder(obstacle_velocity_source='tracker')
    con = _drive(b, [([dropped, kept], k / 30.0) for k in range(5)])

    assert con.A.shape[0] == con.v_obs.size, (con.A.shape, con.v_obs.size)
    i = list(con.links).index('fr3_link5#0')
    assert np.isclose(con.v_obs[i], 0.4, atol=1e-6), (con.links, con.v_obs)


def test_the_rows_behind_a_drop_keep_their_own_closing_speed():
    """The shift did not stop at the obstacle family. Joint-limit rows and
    retreat-cap rows append their own v_obs of 0.0 — both sides of a joint
    limit are the robot's own and are already in q̇, and a cap row's speed is
    folded into cap_v instead — so an extra entry ahead of them handed each one
    the NEXT row's number. Turn those families on so there is something behind
    the obstacle row to be shifted."""
    from franka_experiments.utils.cbf_state_rows import G_OBS
    dropped = make_obstacle(d=float('nan'), pr=PR, ph=_ph(-0.30),
                            link='fr3_link4', cp_label='fr3_link4#0')
    kept = make_obstacle(d=0.25, pr=PR, ph=_ph(-0.25), link='fr3_link5',
                         cp_label='fr3_link5#0', v=(0.0, 0.9, 0.0),
                         frames_seen=20)
    b = make_builder(obstacle_velocity_source='tracker',
                     joint_limit_rows_enabled=True, retreat_cap_enabled=True)
    con = _drive(b, [([dropped, kept], k / 30.0) for k in range(5)])

    assert con.A.shape[0] == con.v_obs.size, (con.A.shape, con.v_obs.size)
    non_obs = [j for j, g in enumerate(con.group) if g != G_OBS]
    assert non_obs, con.group          # the test would be vacuous otherwise
    for j in non_obs:
        assert con.v_obs[j] == 0.0, (j, con.links[j], con.v_obs[j])
    i = list(con.links).index('fr3_link5#0')
    assert np.isclose(con.v_obs[i], 0.9, atol=1e-6), (con.links, con.v_obs)


# ── The label must not depend on which rows survived ────────────────────────

def test_the_label_follows_the_wire_not_the_surviving_rows():
    """Drop the FIRST control point of a link and the second must keep its own
    label. It used to inherit the first one's — and with it the first one's
    barrier EMA, residual anchor, rotation-guard normal and median window."""
    far = make_obstacle(d=99.0, pr=PR, ph=(50.0, 0.0, 0.5), link='fr3_link5',
                        cp_label='fr3_link5#0')          # beyond the horizon
    near = make_obstacle(d=0.25, pr=PR, ph=_ph(-0.25), link='fr3_link5',
                         cp_label='fr3_link5#1')
    con = _drive(make_builder(), [([far, near], 0.0)])
    assert list(con.links)[0] == 'fr3_link5#1', con.links


def test_a_long_gap_does_not_relax_the_bound_into_uselessness():
    """The physical term is Δt-capped. Without the cap a control point that
    went out of the obstacle horizon for seconds came back with a metres-wide
    threshold that nothing could trip — and the residual would then difference
    a distance measured seconds ago against this one."""
    lbl = 'fr3_link5#0'
    b = make_builder(obstacle_velocity_source='residual',
                     obstacle_velocity_identity_jump=0.10)
    o0 = make_obstacle(d=0.50, pr=PR, ph=_ph(-0.50), track_id=7, cp_label=lbl)
    o1 = make_obstacle(d=0.50, pr=PR, ph=_ph(-0.50), track_id=7, cp_label=lbl)
    # 5 s of silence, then the nearest point is 0.45 m away from where it was.
    later = make_obstacle(d=0.30, pr=PR, ph=(0.5, -0.05, 0.5), track_id=7,
                          cp_label=lbl)
    _drive(b, [([o0], 0.0), ([o1], 1 / 30.0), ([later], 5.0)])
    assert b.diag_ident_reset == 1, b.diag_ident_reset
