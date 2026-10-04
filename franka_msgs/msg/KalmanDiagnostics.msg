std_msgs/Header header
geometry_msgs/Vector3[] innovations
float64[] p_traces
# Normalized innovation squared per keypoint (chi^2 with 3 dof for a consistent
# filter), NaN when the keypoint had no usable measurement this frame.
float64[] nis
# True when the measurement was rejected by the Mahalanobis gate.
bool[] gated
