hsl_interfaces/ContractHeader meta
uint8 mode
uint8 decision
string primary_reason
string[] reasons
uint64 candidate_seq
float64 proposed_v
float64 proposed_omega
float64 applied_v
float64 applied_omega
float64 checked_clearance_m
float64 required_stop_distance_m
float64 response_bound_s
bool clearance_valid
string limits_id
builtin_interfaces/Duration evaluation_time

uint8 DISARMED = 0
uint8 READY = 1
uint8 ACTIVE = 2
uint8 BRAKING = 3
uint8 FAULT_LATCHED = 4
uint8 STOP = 0
uint8 ADMIT = 1
uint8 LIMIT = 2
