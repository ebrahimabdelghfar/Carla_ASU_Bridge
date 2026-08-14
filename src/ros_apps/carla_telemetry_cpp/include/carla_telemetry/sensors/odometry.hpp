#pragma once

#include <carla/client/Actor.h>

#include <cstdint>
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
   * @brief Bind the GPS sensor whose stream feeds position in "gnss" mode.
   *
   * Only used by position_source(); get_state() still takes the pointer as an
   * argument.
   *
   * @param gps The GPS sensor (may be nullptr)
   */
  void bind_gps(CarlaGPS* gps) { gps_ = gps; }

  /**
   * @brief Identity of the measurement the position currently comes from.
   *
   * In "standard" mode position is read from the vehicle transform, which
   * advances with the world frame — the caller's own frame number already
   * identifies it, so this returns false and the caller keeps its default.
   * In "gnss" mode position comes from the GNSS stream, which advances on its
   * own schedule, so the world frame is the wrong identity: this reports the
   * frame and sim timestamp of the fix behind the ENU cache instead.
   *
   * @param frame Set to the source measurement's world frame (gnss mode only)
   * @param sim_time Set to the source measurement's sim timestamp (seconds)
   * @return true if the position source is tracked separately (gnss mode)
   * @return false if the caller's world frame is the correct identity
   */
  bool position_source(uint64_t& frame, double& sim_time) const;

  /**
   * @brief Dead-reckon the last state built by get_state() forward by dt.
   *
   * Constant-velocity translation plus an axis-angle rotation increment, both
   * from the world-frame velocities that state was built with. No fresh noise
   * is drawn — the prediction inherits whatever noise the source state carries,
   * so a noise-enabled config does not random-walk between real samples.
   *
   * @param dt Seconds to integrate forward (sim seconds)
   * @return std::optional<OdometryState> The predicted state, or nullopt if no
   *         state has been produced yet
   */
  std::optional<OdometryState> predict(double dt) const;

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
  CarlaGPS* gps_ = nullptr;  // position source in "gnss" mode; not owned

  // Last state produced by get_state(), plus the world-frame velocities it was
  // built from (noise included, so predict() reproduces the same twist rather
  // than a fresh draw). predict() integrates these; both are written and read
  // from the odometry loop thread only.
  OdometryState last_state_;
  bool have_last_ = false;
  double vx_world_ = 0.0, vy_world_ = 0.0, vz_world_ = 0.0;  // m/s
  double wx_world_ = 0.0, wy_world_ = 0.0, wz_world_ = 0.0;  // rad/s
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
