uint8 STANDBY=0
uint8 TAKEOFF=1
uint8 INSPECTING=2
uint8 HANDLING_EXCEPTION=3
uint8 RETURNING_HOME=4
uint8 LANDING=5
uint8 COMPLETE=6

builtin_interfaces/Time stamp
uint64 sequence
uint8 from_state
uint8 to_state
int32 waypoint_index
string event
string reason
