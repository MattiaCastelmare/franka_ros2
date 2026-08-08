# Misure RGB-D raw dei landmark MediaPipe.
# Ordine fisso: [0, 5, 9, 17].

std_msgs/Header header

uint8 NO_HAND=0
uint8 TRACKING_PARTIAL=1
uint8 TRACKING_FULL=2
uint8 TRACKING_ESTIMATED=3
uint8 INVALID_DEPTH=4
uint8 tracking_state

uint8 DIRECT=0
uint8 ESTIMATED=1
uint8 INVALID=2

uint8[4] landmark_ids
geometry_msgs/Point[4] positions
bool[4] valid
uint8[4] measurement_type

float32 processing_latency_ms
