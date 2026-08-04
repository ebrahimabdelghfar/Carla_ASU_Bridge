# Urban IDD §10.6 — scenario metadata (T08: /mission/scenario_context)
string active_scenario          # e.g. "follow_lane", "lane_change", "intersection"
string[] available_lanes        # lane IDs
float64 speed_limit_mps
bool traffic_light_detected
uint8 traffic_light_state       # 0=unknown, 1=red, 2=yellow, 3=green
bool crosswalk_ahead
float64 crosswalk_distance_m
bool intersection_ahead
float64 intersection_distance_m
builtin_interfaces/Time stamp

uint8 TRAFFIC_LIGHT_UNKNOWN=0
uint8 TRAFFIC_LIGHT_RED=1
uint8 TRAFFIC_LIGHT_YELLOW=2
uint8 TRAFFIC_LIGHT_GREEN=3
