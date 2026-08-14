# Per-wheel tire state and forces, taken straight from CARLA's own
# WheelTelemetryData (Vehicle::GetTelemetryData). Every field below is the
# simulator's own value — no analytic tire model sits in between, so these
# forces cannot disagree with the physics the vehicle is actually driven by.
#
# Wheel order for every array below: FL, FR, RL, RR.

builtin_interfaces/Time stamp

string[4]  wheel_names          # ["FL", "FR", "RL", "RR"]
float64[4] slip_angle           # rad, CARLA telemetry lat_slip per wheel (CARLA reports degrees; converted here)
float64[4] slip_ratio           # CARLA telemetry long_slip per wheel
float64[4] normal_load          # CARLA telemetry tire_load per wheel, N
# Validated against rigid-body dynamics on this vehicle (2026-08-14): in steady
# cornering sum(lateral_force) tracks mass * a_y with corr +0.95 and slope
# 0.95-1.08, so this is a real contact-patch force in the ROS body frame.
float64[4] lateral_force        # N, CARLA telemetry lat_force per wheel, sign-flipped into the ROS body frame (+y = left)

# NOT validated, and NOT a contact-patch force. CARLA reports it as exactly
# -torque / wheel_radius, i.e. the drivetrain torque at the axle expressed as a
# force. It equals the force the chassis feels only while the tire is rolling
# without slipping. On this vehicle the wheels spin (wheel surface speed >20%
# above ground speed) in ~43% of cruise samples, and during those windows
# sum(longitudinal_force) reaches +/-6600 N while mass * a_x stays under 500 N.
# Treat it as drivetrain effort, not tire force.
float64[4] longitudinal_force   # N, CARLA telemetry long_force per wheel (+x = forward). See caveat above
float64[4] wheel_torque         # N·m, CARLA telemetry torque per wheel (CARLA's own sign: torque = -long_force * wheel radius)
