# Robot-obstacle distances for one camera frame.
#
# One LinkDistance per CONTROL POINT — NOT per link. With the segment layout in
# fr3_complete.yaml that is up to 11 entries over 5 distinct link names, and
# cbf_safety_filter builds one HOCBF row for each, all active simultaneously.
#
# An EMPTY links array is a liveness heartbeat meaning "no control point in the
# publish band", which lets consumers tell that apart from a dead perception
# pipeline. Treat a missing message, not an empty one, as the fault.
std_msgs/Header header
LinkDistance[] links
