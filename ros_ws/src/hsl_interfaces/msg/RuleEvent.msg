hsl_interfaces/ContractHeader meta
string event_id
uint8 kind
uint8 evidence
uint8 predicate
builtin_interfaces/Time event_time_lower
builtin_interfaces/Time event_time_upper
string[] input_ids
float64 distance_m
float64 bearing_rad
uint8 los
string reason

uint8 NONE = 0
uint8 CAPTURE = 1
uint8 ARRIVAL = 2
uint8 TIMEOUT = 3
uint8 OFFICIAL_ABORT = 4
uint8 AMBIGUOUS = 5
uint8 ESTIMATE = 0
uint8 SIM_TRUTH = 1
uint8 OFFICIAL = 2
uint8 UNKNOWN = 0
uint8 FALSE = 1
uint8 TRUE = 2
