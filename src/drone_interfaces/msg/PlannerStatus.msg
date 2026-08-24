uint8 WAITING_FOR_MAP=0
uint8 STALE_MAP=1
uint8 WAITING_FOR_TF=2
uint8 WAITING_FOR_AIRBORNE=3
uint8 NO_PATH=4
uint8 READY=5
uint8 INVALID_MAP=6
uint8 GOAL_REACHED=7

uint8 state
uint64 trajectory_id
builtin_interfaces/Time stamp
bool map_fresh
bool trajectory_valid
string reason
