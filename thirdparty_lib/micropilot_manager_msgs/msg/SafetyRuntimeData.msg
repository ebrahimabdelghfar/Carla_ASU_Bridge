# IDD §10.6 (offroad) + §10.7 (urban) — SLM / safety aggregator runtime snapshot (T31: /safety/runtime_data)
# Union of offroad and urban field sets; unused fields default to zero/false.

# Localization
float64 localization_covariance_norm
bool gnss_available
float64 localization_age_sec
bool vrtk_available
float64 hdmap_match_confidence

# Perception
uint32 tracked_object_count
float64 perception_latency_ms
bool perception_pipeline_healthy
float64 closest_obstacle_distance_m
float64 closest_obstacle_ttc_sec
bool road_model_valid
string active_scenario_type

# Cost mapping
bool costmap_valid
float64 costmap_age_sec

# Planning (offroad: trajectory_valid; urban: path_plan_valid + velocity_plan_valid)
bool trajectory_valid
bool path_plan_valid
bool velocity_plan_valid
float64 trajectory_ttc_sec
bool collision_imminent

# Control
float64 lateral_error_m
float64 longitudinal_error_mps
bool control_saturated
float64 current_speed_mps

# Vehicle (offroad includes battery_soc; urban adds wheel odometry speed)
float64 battery_soc
bool emergency_stop_circuit_healthy
bool actuators_responsive
bool collision_monitor_hw_clear
float64 wheel_odometry_speed_mps

# HMI / network
bool teleop_link_active
float64 teleop_latency_ms

builtin_interfaces/Time stamp
