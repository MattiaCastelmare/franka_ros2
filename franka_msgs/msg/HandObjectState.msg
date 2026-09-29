# Experimental hand-associated object evidence, NOT grasp or handover intent.
# Header uses the observation time and the same base frame as HandState.
std_msgs/Header header
uint8 physical_hand
# False means unobservable/stale/ambiguous, not an empty hand.
bool valid
bool object_present
# Heuristic evidence score [0,1], not a calibrated probability.
float32 object_confidence
# Visible cluster centroid in metres; NaN unless valid and present.
geometry_msgs/Point object_centroid_3d
# Seconds since the last accepted cluster observation; infinity if none.
float32 object_age
