#pragma once

#include <carla/client/Actor.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/sensor/data/GnssMeasurement.h>

#include <cstdint>
#include <mutex>
#include <optional>
#include <random>
#include <string>

#include "carla_telemetry/types.hpp"

namespace carla_telemetry {

/// CARLA GNSS sensor wrapper with Gauss-Markov + white-noise model.
/// Direct port of CarlaGPS from gps.py.
class CarlaGPS {
 public:
  struct Config {
    double update_rate = 10.0;
    double gps_xy_random_walk = 2.0;
    double gps_z_random_walk = 4.0;
    double gps_correlation_time = 60.0;
    double gps_xy_noise_density = 2e-4;
    double gps_z_noise_density = 4e-4;
    double gps_vxy_noise_density = 0.2;
    double gps_vz_noise_density = 0.4;
    double eph = 1.0;
    double epv = 1.0;
    int satellites_visible = 10;
    float sp_x = 1.0f, sp_y = 0.0f,
          sp_z = 2.8f;  // Spawn point of GPS on the vehicle
    float sp_roll = 0.0f, sp_pitch = 0.0f, sp_yaw = 0.0f;
  };

  CarlaGPS(carla::client::World& world,
           carla::SharedPtr<carla::client::Actor> vehicle, const Config& cfg,
           double origin_lat, double origin_lon, double origin_alt);

  /**
   * @brief Compute noisy GPS reading. Returns state or nullopt if no GNSS data
   * yet.
   *
   * @param vx Velocity in x direction
   * @param vy Velocity in y direction
   * @param vz Velocity in z direction
   * @param dt Time step
   * @return std::optional<GpsState> The latest GPS state
   */
  std::optional<GpsState> update(double vx, double vy, double vz, double dt);

  /**
   * @brief Get the latest GPS state.
   *
   * @return const GpsState& The latest GPS state
   */
  const GpsState& state() const { return state_; }

  /**
   * @brief Get the latest ground-truth GPS reading.
   *
   * @param lat Latitude
   * @param lon Longitude
   * @param alt Altitude
   * @return true if successful
   * @return false if failed
   */
  bool get_latest_gt(double& lat, double& lon, double& alt);

  /**
   * @brief Get the latest East-North-Up (ENU) coordinates.
   *
   * @param east East coordinate
   * @param north North coordinate
   * @param up Up coordinate
   * @param use_noise Whether to use noise
   * @return true if successful
   * @return false if failed
   */
  bool get_latest_enu(double& east, double& north, double& up,
                      bool use_noise = true) const;

  /**
   * @brief Identity of the GNSS measurement the cached ENU vectors were built
   * from: the world frame it was produced in and its sim timestamp.
   *
   * Consumers use this to tell a fresh fix from a re-read of the same one —
   * the GNSS stream runs at its own rate, so polling faster than that serves
   * the same bytes repeatedly. Freshness is keyed on the measurement's own
   * identity, never on comparing the values.
   *
   * @param frame The world frame of the measurement behind the ENU cache
   * @param sim_time The sim timestamp (seconds) of that measurement
   * @return true if the cache holds a measurement
   * @return false if no GNSS fix has been processed yet
   */
  bool enu_source(uint64_t& frame, double& sim_time) const;

  /**
   * @brief Destroy the GPS sensor.
   */
  void destroy();

 private:
  void on_gnss(carla::SharedPtr<carla::sensor::SensorData> data);

  double origin_lat_, origin_lon_, origin_alt_;
  double map_ref_lat_, map_ref_lon_, map_ref_alt_;  // CARLA map geo-reference
  Config cfg_;

  // Noise model state
  double gps_bias_[3] = {0.0, 0.0, 0.0};
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};

  // Cached ENU vectors (metres from user origin), written by update(), read by
  // get_latest_enu()
  double enu_noisy_[3] = {0.0, 0.0,
                          0.0};  // noisy ENU (Gauss-Markov + white noise)
  double enu_gt_[3] = {0.0, 0.0,
                       0.0};  // ground-truth ENU (perfect GNSS reprojected)
  bool enu_valid_ = false;    // true once update() has completed at least once
  // Identity of the GNSS measurement the cached ENU vectors came from, written
  // together with them under lock_ so a reader never pairs one fix's frame
  // number with another fix's position.
  uint64_t enu_frame_ = 0;
  double enu_sim_time_ = 0.0;

  mutable std::mutex lock_;
  carla::SharedPtr<carla::sensor::data::GnssMeasurement> last_gnss_;
  carla::SharedPtr<carla::client::Sensor> sensor_;

  GpsState state_;
};

}  // namespace carla_telemetry
