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
