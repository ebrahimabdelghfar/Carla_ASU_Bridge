# Per-wheel tire state and forces, taken from CARLA's own WheelTelemetryData
# (Vehicle::GetTelemetryData). Every field below is either the simulator's own
# value or a kinematic identity computed from it — no analytic tire model sits
# in between. Unit and sign conversions are noted per field.
#
# Wheel order for every array below: FL, FR, RL, RR.
#
# Below 0.5 m/s of chassis speed, slip_angle, slip_ratio, lateral_force,
# longitudinal_force and wheel_torque are all published as 0. PhysX sleeps a
# parked vehicle, and the wheel telemetry then repeats byte for byte on every
# frame afterwards (-4.4 / -6.6 / -5.1 / -4.8 N of lateral force held unchanged
# over 4 s of standstill), so without the gate the topic looks live while
# nothing behind it updates; and the frame where motion stops carries a spike,
# because lat_slip is an atan2 of two vanishing velocities — at 0.01 m/s it
# read -2.0 to +2.6 deg with +/-334 N of lateral force on wheels already at
# omega = 0. normal_load, tire_friction and wheel_speed stay live at rest.

# Sim-clock stamp of the world frame the telemetry RPC was served on, so it is
# byte-identical to the /odom, /sim/feedback/imu and LiDAR stamps of that same
# frame. One message per frame at most.
builtin_interfaces/Time stamp

string[4]  wheel_names          # ["FL", "FR", "RL", "RR"]

# Validated live in a steady 4 m/s / 0.20 rad turn against the slip angle
# rebuilt from the chassis state (body velocity and yaw rate at each hub minus
# the real wheel steer angle): equal magnitude to within 2% front / 15% rear.
float64[4] slip_angle           # rad, ROS body frame (+ = force acts to the left). CARLA reports degrees in the left-handed frame; both conventions invert, so the raw value converts directly

# Effective friction coefficient the physics step is using for this tire —
# CARLA's WheelTelemetryData.tire_friction. This is NOT the value configured in
# vehicle.physics.wheels[].tire_friction: the server multiplies it by the road
# surface's own friction, measured as a constant 0.70 factor (1.05 published
# against 1.5 configured) on this map. Friction-ellipse checks (|F| <= mu * Fz)
# must use this value, not the configured one.
float64[4] tire_friction

# Wheel angular velocity, CARLA's WheelTelemetryData.omega passed through.
# Positive drives the vehicle forward. Published so consumers can form their
# own slip ratio, or check the one below, without a second data source.
float64[4] wheel_speed          # rad/s

# SAE slip ratio kappa = (omega * R - v) / v: positive when the wheel overspins
# the ground (driving), negative under braking. R is the configured wheel
# radius and v the chassis speed (CARLA's telem.speed), so in cornering each
# wheel carries the error of using chassis speed instead of hub speed, up to
# (half_track * yaw_rate) / v. Zero below 0.5 m/s, where the ratio is
# numerically meaningless.
#
# NOT a passthrough of CARLA's WheelTelemetryData.long_slip: that field is
# inconsistent with the omega and radius the same message reports. At a steady
# 2.33 m/s cruise it read +0.177 (i.e. hard braking, once negated into the SAE
# convention) on a wheel turning 8.99 rad/s, whose kinematic ratio is -0.034,
# and its per-wheel ordering does not follow omega either.
float64[4] slip_ratio

# Passthrough of CARLA's tire_load. sum() runs 13-15% above mass * g. Two
# causes, measured by sweeping vehicle.physics.mass over 240 / 400 / 600 kg:
# the wheels' own rest load sums to (mass + 24.3 kg) * g, a constant +238 N of
# unsprung weight PhysX carries outside vehicle.physics.mass; the rest is
# standing suspension compression (tire_load sits 3-8% above rest load with the
# car parked). Load transfer is only ~15% of the rigid-body expectation
# (18 N per m/s^2 of a_y against 119 N for a 0.3 m CG) whenever
# vehicle.physics.center_of_mass is left at ground level.
float64[4] normal_load          # N

# Validated against rigid-body dynamics on this vehicle: in steady cornering
# sum(lateral_force) tracks mass * a_y with corr +0.95 to +0.97 and slope
# 0.92-1.08, so this is a real contact-patch force. -lateral_force plotted
# against alpha_rad * normal_load gives 11.3 /rad (corr +0.93), i.e. peak grip
# near 5.3 deg of slip — the expected range for a racing tire.
float64[4] lateral_force        # N, CARLA telemetry lat_force per wheel, sign-flipped into the ROS body frame (+y = left)

# NOT the force the chassis receives, and no configuration change makes it one:
# it behaves as a friction-capacity report. Over 8268 wheel samples from 17
# captures, |longitudinal_force| sits above 0.80 * tire_friction * normal_load
# half the time and exactly at that limit 15% of the time, while
# corr(sum, mass * a_x) is +0.04 over 750 samples. Cutting the engine's peak
# torque from 900 to 100 N*m — enough that the car could no longer exceed
# 0.87 m/s — left it pinned at 100% of the limit, and at a steady cruise it
# reports a saturated 703.4 N (= 1.05 * 669.9, the friction limit to four
# decimals) on a wheel whose lateral force is 3.7 N, so it is not even coupled
# to the friction ellipse it appears to saturate. Use tire_friction *
# normal_load for the capacity and the chassis IMU for the delivered force.
float64[4] longitudinal_force   # N, CARLA telemetry long_force per wheel (+x = forward). See caveat above
float64[4] wheel_torque         # N·m, CARLA telemetry torque per wheel. Exactly -longitudinal_force * wheel radius (identity to 1e-4 N), so it carries no independent information
