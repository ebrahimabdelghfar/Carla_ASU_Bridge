#pragma once

#include <optional>
#include <string>

#include "carla_telemetry/types.hpp"

namespace carla_telemetry {

/**
 * @brief Software battery simulation — no CARLA sensor required.
 * Direct port of CarlaBattery from battery.py.
 */
class CarlaBattery {
 public:
  /**
   * @brief Construct a new CarlaBattery object.
   * @param consumption_mode Consumption mode.
   * @param voltage Voltage.
   * @param e0 Open-circuit voltage.
   * @param e1 Open-circuit voltage.
   * @param capacity Capacity.
   * @param initial_charge Initial charge.
   * @param resistance Resistance.
   * @param tau Tau.
   * @param base_power_load Base power load.
   * @param power_per_speed Power per speed.
   * @param start_draining Start draining.
   * @param enable_recharge Enable recharge.
   * @param charging_time Charging time.
   * @param update_rate Update rate.
   * @param ambient_temperature Ambient temperature.
   */
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

  /**
   * @brief Advance battery model. Returns updated state.
   * @param speed_ms Speed in meters per second.
   * @param dt Time step in seconds.
   * @return BatteryState.
   */
  BatteryState update(double speed_ms, double dt);

  /**
   * @brief Get the latest battery state.
   * @return const BatteryState& The latest battery state.
   */
  const BatteryState& state() const { return state_; }
  /**
   * @brief Check if the battery is depleted.
   * @return true if the battery is depleted.
   * @return false otherwise.
   */
  bool is_depleted() const { return is_depleted_; }
  /**
   * @brief Check if the battery is draining.
   * @return true if the battery is draining.
   * @return false otherwise.
   */
  bool is_draining() const { return is_draining_; }

  /**
   * @brief Start draining.
   */
  void start_draining();
  /**
   * @brief Stop draining.
   */
  void stop_draining();
  /**
   * @brief Start charging.
   */
  void start_charging();
  /**
   * @brief Stop charging.
   */
  void stop_charging();

 private:
  /**
   * @brief Make a battery state.
   * @param total_power Total power consumption.
   * @param v_oc Open-circuit voltage.
   * @return BatteryState.
   */
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
