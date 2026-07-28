#include "carla_telemetry/sensors/battery.hpp"

#include <algorithm>
#include <cmath>

namespace carla_telemetry {

CarlaBattery::CarlaBattery(const std::string& consumption_mode, double voltage,
                           double e0, double e1, double capacity,
                           double initial_charge, double resistance, double tau,
                           double base_power_load, double power_per_speed,
                           bool start_draining, bool enable_recharge,
                           double charging_time, double update_rate,
                           double ambient_temperature)
    : v_nom_(voltage),
      e0_(e0),
      e1_(e1),
      capacity_(capacity),
      charge_(initial_charge),
      resistance_(resistance),
      tau_(tau),
      base_power_load_(base_power_load),
      power_per_speed_(power_per_speed),
      charging_time_(charging_time),
      consumption_mode_(consumption_mode),
      is_draining_(start_draining),
      ambient_temperature_(ambient_temperature),
      current_temperature_(ambient_temperature) {
  (void)enable_recharge;
  (void)update_rate;

  if (v_nom_ > 0.0 && e0_ > 0.0 && std::abs(v_nom_ - e0_) > 0.1) {
    double scale = v_nom_ / e0_;
    e0_ *= scale;
    e1_ *= scale;
  }

  state_ = make_state();
}

BatteryState CarlaBattery::update(double speed_ms, double dt) {
  if (is_depleted_ && !is_charging_) return state_;

  // Charging path
  if (is_charging_) {
    double charge_rate = capacity_ / std::max(charging_time_, 0.001);  // Ah/h

    // Update temperature (charge_rate Ah/h is numerically equal to current in
    // Amps)
    double heat_generated = charge_rate * charge_rate * resistance_;
    double heat_dissipated =
        heat_dissipation_rate_ * (current_temperature_ - ambient_temperature_);
    current_temperature_ +=
        ((heat_generated - heat_dissipated) / thermal_mass_) * dt;

    charge_ = std::min(capacity_, charge_ + charge_rate * (dt / 3600.0));
    if (charge_ >= capacity_) {
      is_charging_ = false;
      is_depleted_ = false;
    }
    state_ = make_state();
    return state_;
  }

  // Discharging path
  if (!is_draining_) return state_;

  double total_power;
  if (consumption_mode_ == "velocity_based") {
    total_power = base_power_load_ + power_per_speed_ * speed_ms;
  } else {
    total_power = base_power_load_;
  }

  double v_oc = e0_ + e1_ * (1.0 - charge_ / capacity_);
  v_oc = std::max(v_oc, 0.01);

  double i_instant = total_power / v_oc;

  if (tau_ > 0.0) {
    smoothed_current_ += (dt / tau_) * (i_instant - smoothed_current_);
  } else {
    smoothed_current_ = i_instant;
  }

  // Update temperature
  double heat_generated = smoothed_current_ * smoothed_current_ * resistance_;
  double heat_dissipated =
      heat_dissipation_rate_ * (current_temperature_ - ambient_temperature_);
  current_temperature_ +=
      ((heat_generated - heat_dissipated) / thermal_mass_) * dt;

  charge_ -= smoothed_current_ * (dt / 3600.0);
  charge_ = std::max(charge_, 0.0);

  if (charge_ <= 0.0) {
    is_depleted_ = true;
    is_draining_ = false;
  }

  state_ = make_state(total_power, v_oc);
  return state_;
}

void CarlaBattery::start_draining() {
  is_depleted_ = false;
  is_draining_ = true;
  is_charging_ = false;
}

void CarlaBattery::stop_draining() { is_draining_ = false; }

void CarlaBattery::start_charging() {
  is_charging_ = true;
  is_draining_ = false;
}

void CarlaBattery::stop_charging() { is_charging_ = false; }

BatteryState CarlaBattery::make_state(double total_power, double v_oc) const {
  double oc = v_oc;
  if (oc < 0.0) {
    oc = e0_ + e1_ * (1.0 - charge_ / capacity_);
  }
  double v_terminal = oc - smoothed_current_ * resistance_;
  double frac = std::clamp(charge_ / capacity_, 0.0, 1.0);

  BatteryState s;
  s.voltage = v_terminal;
  s.open_circuit_v = oc;
  s.current = smoothed_current_;
  s.charge = charge_;
  s.capacity = capacity_;
  s.charge_fraction = frac;
  s.percentage = frac * 100.0;
  s.power_load = total_power;
  s.temperature = current_temperature_;
  s.is_depleted = is_depleted_;
  return s;
}

}  // namespace carla_telemetry
