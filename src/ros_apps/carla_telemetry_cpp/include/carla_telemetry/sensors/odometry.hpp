#pragma once

#include <carla/client/Actor.h>

#include <optional>
#include <random>
#include <string>

#include "carla_telemetry/types.hpp"

namespace carla_telemetry {

class CarlaGPS;

/**
 * @brief Configuration for odometry noise.
 */
struct OdometryNoiseConfig {
  bool enabled = false;
  double pos_stddev_x = 0.0;
  double pos_stddev_y = 0.0;
  double pos_stddev_z = 0.0;
  double ori_stddev_roll = 0.0;   // deg
  double ori_stddev_pitch = 0.0;  // deg
  double ori_stddev_yaw = 0.0;    // deg
  double vel_stddev_x = 0.0;
  double vel_stddev_y = 0.0;
  double vel_stddev_z = 0.0;
  double ang_vel_stddev_x = 0.0;  // deg/s
  double ang_vel_stddev_y = 0.0;  // deg/s
  double ang_vel_stddev_z = 0.0;  // deg/s
};

/// Ground-truth odometry from CARLA vehicle state.
class CarlaOdometry {
 public:
  explicit CarlaOdometry(
      const std::string& frame_id = "odom",
      const std::string& child_frame_id = "base_link",
      double update_rate = 20.0, bool broadcast_tf = false,
      const std::string& mode = "standard", double origin_lat = 0.0,
      double origin_lon = 0.0, double origin_alt = 0.0,
      bool gnss_use_noise = true,
      const OdometryNoiseConfig& noise_cfg = OdometryNoiseConfig());

  /// Compute odometry from a CARLA vehicle actor.
  OdometryState get_state(carla::SharedPtr<carla::client::Actor> vehicle,
                          CarlaGPS* gps = nullptr);

  /**
   * @brief Get whether the odometry is broadcasting TF.
   *
   * @return true if the odometry is broadcasting TF
   * @return false if the odometry is not broadcasting TF
   */
  bool broadcast_tf() const { return broadcast_tf_; }

 private:
  std::string frame_id_;
  std::string child_frame_id_;
  double update_rate_;
  bool broadcast_tf_;
  std::string mode_;
  double origin_lat_;
  double origin_lon_;
  double origin_alt_;
  bool gnss_use_noise_;
  OdometryNoiseConfig noise_cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
};

/**
 * @brief Convert RPY (radians) to quaternion (x, y, z, w).
 * @param roll Roll angle in radians
 * @param pitch Pitch angle in radians
 * @param yaw Yaw angle in radians
 * @param qx Quaternion x component
 * @param qy Quaternion y component
 * @param qz Quaternion z component
 * @param qw Quaternion w component
 */
void euler_to_quaternion(double roll, double pitch, double yaw, double& qx,
                         double& qy, double& qz, double& qw);

}  // namespace carla_telemetry
