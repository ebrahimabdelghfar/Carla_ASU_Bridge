#pragma once

#include <optional>
#include <string>

#include "carla_telemetry/types.hpp"

namespace carla_telemetry {

// Software battery simulation — no CARLA sensor required.
// Direct port of CarlaBattery from battery.py.
class CarlaBattery {
 public:
  explicit CarlaBattery(const std::string& consumption_mode = "velocity_based",
                        double voltage = 12.6, double e0 = 12.6,
                        double e1 = -2.8, double capacity = 10.0,
                        double initial_charge = 10.0, double resistance = 0.061,
                        double tau = 2.0, double base_power_load = 50.0,
                        double power_per_speed = 15.0,
                        bool start_draining = true,
                        bool enable_recharge = false,
                        double charging_time = 1.0, double update_rate = 10.0,
                        double ambient_temperature = 25.0);

  // Advance the battery model by dt seconds at the given speed.
  BatteryState update(double speed_ms, double dt);

  const BatteryState& state() const { return state_; }
  bool is_depleted() const { return is_depleted_; }
  bool is_draining() const { return is_draining_; }

  void start_draining();
  void stop_draining();
  void start_charging();
  void stop_charging();

 private:
  BatteryState make_state(double total_power = 0.0, double v_oc = -1.0) const;

  double v_nom_, e0_, e1_;
  double capacity_, charge_, resistance_, tau_;
  double base_power_load_, power_per_speed_;
  double charging_time_;
  std::string consumption_mode_;

  double smoothed_current_ = 0.0;
  bool is_depleted_ = false;
  bool is_draining_ = true;
  bool is_charging_ = false;

  // Thermal model
  double ambient_temperature_ = 25.0;
  double current_temperature_ = 25.0;
  double thermal_mass_ = 10000.0;        // Joules/Kelvin (approx 10kg battery)
  double heat_dissipation_rate_ = 10.0;  // Watts/Kelvin

  BatteryState state_;
};

}  // namespace carla_telemetry
