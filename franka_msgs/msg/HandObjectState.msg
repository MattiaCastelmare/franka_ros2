# Object held by the ACTIVE hand (Hands23 on a palm crop), NOT grasp or handover intent.
# Header uses the HandState time and the same base frame as HandState.
std_msgs/Header header
uint8 physical_hand
# False means unobservable/stale, not an empty hand.
bool valid
bool object_present
# Hands23 object score of the last accepted detection, not a calibrated probability.
float32 object_confidence
# Centroid of the visible object surface in metres; NaN unless present.
geometry_msgs/Point object_centroid_3d
# Seconds since the last detection that saw the object; infinity if none.
float32 object_age
# Object box and outline in the colour image [px], moved with the palm between
# detections. bbox = u_min, v_min, u_max, v_max; contour = u0, v0, u1, v1, ...
float32[4] bbox_px
int32[] contour_px
# Extents of the visible surface along its principal axes [m], descending.
# The axis along the camera ray is underestimated (only one side is seen).
geometry_msgs/Vector3 dimensions
