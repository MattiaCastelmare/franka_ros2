# Human-robot minimum distance measurement.
#
# Published by the human-robot distance estimation node.
# Consumed by the human avoidance controller in franka_experiments.

std_msgs/Header header

# Name of the robot link closest to the human (e.g. "fr3_link5").
# Must match a valid Pinocchio frame name in the robot model.
string robot_link_name

# Minimum distance between human and robot [m].
float32 distance

# Unit direction vector pointing FROM the human TOWARD the robot
# (i.e. the "escape" direction the robot should follow to increase distance).
# Expressed in world frame.
geometry_msgs/Vector3 direction

# Whether this measurement is valid and should be used for avoidance.
bool valid

# Closest point on the robot surface, in world frame [m].
geometry_msgs/Point closest_point_robot

# Safety zone classification (e.g. "safe", "warning", "danger", "critical").
string zone

# Confidence of the distance estimate [0.0, 1.0].
float32 confidence
