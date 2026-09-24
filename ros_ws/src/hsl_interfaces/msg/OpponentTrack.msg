hsl_interfaces/ContractHeader meta
string track_id
uint8 state
float64 x
float64 y
float64 vx
float64 vy
float64[16] covariance
bool position_valid
float64 yaw
float64 yaw_variance
bool yaw_valid
builtin_interfaces/Time last_measurement_stamp
uint8 sensor_profile
string model_id
string association_status

uint8 SEARCHING = 0
uint8 TRACKED = 1
uint8 COASTING = 2
uint8 OCCLUDED_BELIEF = 3
uint8 LOST = 4
uint8 REAL_CLOUD = 0
uint8 SIM_CLOUD = 1
uint8 SYNTHETIC_DETECTION = 2
uint8 REPLAY_CLOUD = 3
