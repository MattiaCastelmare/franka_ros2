std_msgs/Header header

# One entry per active robot segment (seg_idx 3-7), ordered by seg_idx.
# Each entry holds the closest control point found on that segment.
HumanRobotDistance[5] distances
