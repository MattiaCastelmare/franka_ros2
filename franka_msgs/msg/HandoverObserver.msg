std_msgs/Header header

uint8 LOST=0
uint8 WARMUP=1
uint8 HOLD=2
uint8 APPROACHING=3
uint8 RETREATING=4

uint8 state
bool state_changed

uint8 candidate_state
uint16 candidate_count

float32 distance
float32 closing_velocity

bool rate_valid
uint8 rate_source
bool rate_degraded
float32 rate_age_s

float32 tracking_confidence
float32 distance_sigma

bool ttc_valid
float32 ttc
