hsl_interfaces/ContractHeader meta
string option_instance_id
uint64 lease_generation
string path_id
uint64 ego_seq
uint64 obstacle_seq
float64 v
float64 omega
builtin_interfaces/Duration horizon
uint8 control_mode
string limits_id

uint8 HOLD = 0
uint8 TRACK_PATH = 1
uint8 ALIGN = 2
uint8 APPROACH_CAPTURE = 3
