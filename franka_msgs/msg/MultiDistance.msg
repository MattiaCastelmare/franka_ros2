std_msgs/Header header

# One entry per active robot segment (a segment with control_points > 0 in the
# robot config), ordered by seg_idx. Each entry holds the closest control point
# found on that segment.
#
# VARIABLE LENGTH, and it must stay that way. This was HumanRobotDistance[5],
# which hard-codes into the interface a property of one configuration file:
# fr3_complete.yaml happens to give five segments control points (seg_idx 3-7).
# Give a sixth segment a control point, or take one away, and
# perception_msgs.build_cp_messages raises inside real_time_distance's compute
# loop on EVERY frame -- the loop catches it, so the node stays up while
# publishing nothing, and the safety filter sees its distances go stale and
# brakes. A fixed size also cannot be padded honestly: the padding would be
# fabricated distances on a topic the experiment logger records.
#
# Nothing indexes this array: the only subscriber (experiment_logger) iterates
# it, and no bag in this repository carries the message.
HumanRobotDistance[] distances
