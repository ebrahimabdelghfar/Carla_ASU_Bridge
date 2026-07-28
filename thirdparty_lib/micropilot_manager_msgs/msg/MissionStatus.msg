# Real-time mission telemetry published periodically
# Maps to micropilot::system_manager::mission::DetailedMissionContext

builtin_interfaces/Time stamp

string  mission_id
uint8   status
string  status_name

# Mission status constants
uint8 IDLE=0
uint8 IN_PROGRESS=1
uint8 PAUSED=2
uint8 COMPLETED=3
uint8 ABORTED=4
uint8 FAILED=5

# Completion
float64 completion_percentage    # 0–100

# Goals / Laps
uint32  current_goal_index
uint32  current_lap
uint32  total_laps
uint32  goals_visited
uint32  goals_remaining
uint32  total_goals_per_lap
uint32  total_goals_all_laps

# Distance (metres)
float64 distance_traveled
float64 remaining_distance
float64 total_distance

# Speed (m/s)
float64 current_speed
float64 average_moving_speed
bool    is_robot_stopped

# Time (seconds)
float64 elapsed_time
float64 estimated_total_time
float64 remaining_time
float64 total_stopped_time

# Battery
float64 battery_soc              # state-of-charge %
float64 battery_consumption_rate # %/s

# Flags
bool is_rtl_phase
bool rtl_enabled

# Current target goal (local frame, metres)
float64 target_goal_x
float64 target_goal_y
float64 target_goal_z
uint32  current_segment
