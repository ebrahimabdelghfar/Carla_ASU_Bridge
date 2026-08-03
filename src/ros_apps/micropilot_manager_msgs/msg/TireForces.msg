# Ground-truth per-wheel tire forces, computed independently of CARLA's
# internal tire model: lateral force via a configurable Magic Formula
# (Pacejka) driven by CARLA's own slip telemetry, longitudinal force via a
# configurable motor-torque model. See config/tire_model_config.yaml.
#
# Wheel order for every array below: FL, FR, RL, RR.

builtin_interfaces/Time stamp

string[4]  wheel_names          # ["FL", "FR", "RL", "RR"]
float64[4] slip_angle           # CARLA telemetry lat_slip per wheel (Magic Formula input)
float64[4] slip_ratio           # CARLA telemetry long_slip per wheel (reference/logging)
float64[4] normal_load          # CARLA telemetry tire_load per wheel, N
float64[4] lateral_force        # N, Magic Formula output
float64[4] longitudinal_force   # N, motor-coefficient model output
