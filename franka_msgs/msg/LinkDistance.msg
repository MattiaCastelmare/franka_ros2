# Distance from ONE robot control point to the nearest observed obstacle point.
#
# Produced by franka_experiments/utils/perception_msgs.build_cp_messages from a
# DistanceEngine control-point result; consumed by cbf_safety_filter (one HOCBF
# row per entry), rl_policy_commander, and the distance visualiser.

# Link the control point is attached to. NOT unique within a MultiLinkDistance:
# a link carries several control points and each gets its own entry. Consumers
# use it only to resolve a kinematic frame; the point itself is
# closest_point_robot.
string robot_link_name

# The control point, on the segment AXIS (base frame).
geometry_msgs/Point closest_point_robot

# Nearest observed obstacle point (base frame). A depth camera returns only the
# front surface, so this is always a surface point.
geometry_msgs/Point closest_point_human

# Unit normal, obstacle -> control point. EMA-smoothed across frames.
geometry_msgs/Vector3 direction

# SURFACE gap in metres: ||closest_point_robot - closest_point_human|| minus the
# capsule radius minus the mask-dilation margin, clamped at 0.
#
# Use THIS for a barrier, never the norm of the two points above — that norm is
# the segment-AXIS distance and overstates the clearance by radius + margin
# (0.014 m at Z=0.5 m, ~0.11 m at Z=2 m).
#
# 0.0 means "at the capsule surface" and is a valid, maximally-urgent
# measurement, not a dropout.
float64 distance

# False only when the measurement is absent (non-finite distance, or no usable
# direction). A distance of 0.0 is valid.
bool valid

# Currently hardcoded to 1.0 on this message. Only the legacy MultiDistance
# topic carries a real find_pt_confidence() score.
float32 confidence

# Safety-zone label from the `zones` section of the robot config.
string zone

# ── Obstacle track (obstacle_tracker_node) ───────────────────────────────────
# Filled by the cluster/track/Kalman pipeline when this control point's
# closest_point_human falls inside a CONFIRMED track's cluster. All four fields
# are defaults (0 / zero vector) when there is no track, which is the state a
# publisher that knows nothing about tracking produces — so an unmodified
# real_time_distance and an unmodified consumer keep working unchanged.
#
# DEFAULTS MUST BE SAFE, and they are: the consumer gates on
# frames_seen >= 3 and then uses max(dot(direction, obstacle_velocity), 0.0),
# so a zero velocity contributes exactly nothing and the barrier falls back to
# the residual estimator's behaviour. There is no value of these fields that
# can LOOSEN a constraint.

# Stable identity of the obstacle track this control point was matched to.
# 0 means NO TRACK — either nothing was tracked, or this control point's nearest
# obstacle point did not fall inside any confirmed track's cluster. Ids are
# never reused after a track dies.
uint32 track_id

# Perception frames the track has been UPDATED on (not predicted). Counts
# measurements only, so a track that has been coasting through an occlusion does
# not accumulate evidence it does not have. 0 when track_id is 0.
uint32 frames_seen

# Obstacle velocity in the BASE frame [m/s], from the track's Kalman state.
# A full 3D vector, unlike the scalar residual it replaces: the component the
# barrier needs is dot(direction, obstacle_velocity), and `direction` turns
# continuously as the arm moves, so a scalar would have to be re-estimated from
# scratch every time it did.
geometry_msgs/Vector3 obstacle_velocity

# Row-major 3x3 covariance of obstacle_velocity [m^2/s^2], in the same BASE
# frame — the velocity block of the track's state covariance, rotated. All
# zeros means "no estimate", not "perfectly certain": consumers must gate on
# frames_seen, never on this being non-zero.
#
# Consumed by the uncertainty margin (enable_uncertainty_margin): the variance
# of the speed the barrier actually sees is dot(n, P_vv * n), and its square
# root times a latency is a distance the barrier can be tightened by. An EMA'd
# scalar cannot report this at all, which is why it is on the wire.
float64[9] velocity_covariance
