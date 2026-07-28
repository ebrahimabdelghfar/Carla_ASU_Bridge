#include "carla_telemetry/sensors/odometry.hpp"

#include <cmath>

#include "carla_telemetry/sensors/gps.hpp"

namespace carla_telemetry {

CarlaOdometry::CarlaOdometry(const std::string& frame_id,
                             const std::string& child_frame_id,
                             double update_rate, bool broadcast_tf,
                             const std::string& mode, double origin_lat,
                             double origin_lon, double origin_alt,
                             bool gnss_use_noise,
                             const OdometryNoiseConfig& noise_cfg)
    : frame_id_(frame_id),
      child_frame_id_(child_frame_id),
      update_rate_(update_rate),
      broadcast_tf_(broadcast_tf),
      mode_(mode),
      origin_lat_(origin_lat),
      origin_lon_(origin_lon),
      origin_alt_(origin_alt),
      gnss_use_noise_(gnss_use_noise),
      noise_cfg_(noise_cfg),
      rng_(std::random_device{}()) {}

OdometryState CarlaOdometry::get_state(
    carla::SharedPtr<carla::client::Actor> vehicle, CarlaGPS* gps) {
  auto transform = vehicle->GetTransform();
  auto velocity = vehicle->GetVelocity();
  auto ang_vel_raw = vehicle->GetAngularVelocity();

  double pos_x = 0.0;
  double pos_y = 0.0;
  double pos_z = 0.0;

  bool has_gnss = false;
  if (mode_ == "gnss" && gps) {
    // Read the ENU offset (metres from the user's global origin) directly from
    // the GPS sensor cache — no lat/lon → UTM round-trip needed.
    // gnss_use_noise_ selects whether to use the noisy (realistic) or GT
    // (debug) signal.
    double east = 0.0, north = 0.0, up = 0.0;
    if (gps->get_latest_enu(east, north, up, gnss_use_noise_)) {
      // ENU: East → ROS X, North → ROS Y, Up → ROS Z
      // This matches the ROS ENU convention directly; no axis flip required.
      pos_x = east;
      pos_y = north;
      pos_z = up;
      has_gnss = true;
    }
  }

  if (!has_gnss) {
    // Position (CARLA→ROS)
    // CARLA uses Unreal Engine's Left-Handed system: X=East, Y=South, Z=Up
    // ROS usually requires a Right-Handed system like FLU or ENU.
    // We flip Y (right→left) to make X=East, Y=North, Z=Up (ENU alignment)
    auto& loc = transform.location;
    pos_x = static_cast<double>(loc.x);
    pos_y = -static_cast<double>(loc.y);
    pos_z = static_cast<double>(loc.z);
  }

  if (noise_cfg_.enabled) {
    pos_x += noise_cfg_.pos_stddev_x * normal_(rng_);
    pos_y += noise_cfg_.pos_stddev_y * normal_(rng_);
    pos_z += noise_cfg_.pos_stddev_z * normal_(rng_);
  }

  // Orientation (Euler deg → quaternion, ROS convention)
  auto& rot = transform.rotation;
  double roll = static_cast<double>(rot.roll) * M_PI / 180.0;
  double pitch = static_cast<double>(-rot.pitch) * M_PI / 180.0;  // negate
  double yaw = static_cast<double>(-rot.yaw) * M_PI / 180.0;  // negate CW→CCW

  if (noise_cfg_.enabled) {
    roll += (noise_cfg_.ori_stddev_roll * M_PI / 180.0) * normal_(rng_);
    pitch += (noise_cfg_.ori_stddev_pitch * M_PI / 180.0) * normal_(rng_);
    yaw += (noise_cfg_.ori_stddev_yaw * M_PI / 180.0) * normal_(rng_);
  }

  double qx, qy, qz, qw;
  euler_to_quaternion(roll, pitch, yaw, qx, qy, qz, qw);

  // Linear velocity: CARLA world→ROS world, then rotate into body frame
  double vx_world = static_cast<double>(velocity.x);
  double vy_world = -static_cast<double>(velocity.y);
  double vz_world = static_cast<double>(velocity.z);

  // Angular velocity: deg/s → rad/s, CARLA→ROS world, then rotate into body
  // frame In CARLA (LHS, Y=South), a positive pitch is nose down, and a
  // positive yaw is clockwise. In ROS (RHS, Y=North), the y and z rotation
  // direction conventions perfectly negate the axis flip, so we only need to
  // negate the angular velocity for the flipped Y and Z axes to match ROS RHS.
  double wx_world = static_cast<double>(ang_vel_raw.x) * M_PI / 180.0;
  double wy_world = -static_cast<double>(ang_vel_raw.y) * M_PI / 180.0;
  double wz_world = -static_cast<double>(ang_vel_raw.z) * M_PI / 180.0;

  // Rotate world-frame vectors into body frame using inverse quaternion
  // q_inv for unit quaternion = conjugate = (-qx, -qy, -qz, qw)
  auto rotate_to_body = [&](double wx_in, double wy_in, double wz_in,
                            double& out_x, double& out_y, double& out_z) {
    // v' = q_inv * v * q  (quaternion-vector rotation)
    // Expanded using q_inv = (-qx, -qy, -qz, qw):
    double t0 = qw * wx_in - qy * wz_in + qz * wy_in;
    double t1 = qw * wy_in - qz * wx_in + qx * wz_in;
    double t2 = qw * wz_in - qx * wy_in + qy * wx_in;
    double t3 = qx * wx_in + qy * wy_in + qz * wz_in;

    out_x = t0 * qw + t3 * qx + t1 * qz - t2 * qy;
    out_y = t1 * qw + t3 * qy + t2 * qx - t0 * qz;
    out_z = t2 * qw + t3 * qz + t0 * qy - t1 * qx;
  };

  double vx, vy, vz;
  rotate_to_body(vx_world, vy_world, vz_world, vx, vy, vz);

  double wx, wy, wz;
  rotate_to_body(wx_world, wy_world, wz_world, wx, wy, wz);

  if (noise_cfg_.enabled) {
    vx += noise_cfg_.vel_stddev_x * normal_(rng_);
    vy += noise_cfg_.vel_stddev_y * normal_(rng_);
    vz += noise_cfg_.vel_stddev_z * normal_(rng_);

    wx += (noise_cfg_.ang_vel_stddev_x * M_PI / 180.0) * normal_(rng_);
    wy += (noise_cfg_.ang_vel_stddev_y * M_PI / 180.0) * normal_(rng_);
    wz += (noise_cfg_.ang_vel_stddev_z * M_PI / 180.0) * normal_(rng_);
  }

  OdometryState s;
  s.frame_id = frame_id_;
  s.child_frame_id = child_frame_id_;
  s.pos_x = pos_x;
  s.pos_y = pos_y;
  s.pos_z = pos_z;
  s.qx = qx;
  s.qy = qy;
  s.qz = qz;
  s.qw = qw;
  s.vx = vx;
  s.vy = vy;
  s.vz = vz;
  s.wx = wx;
  s.wy = wy;
  s.wz = wz;
  s.broadcast_tf = broadcast_tf_;
  return s;
}

void euler_to_quaternion(double roll, double pitch, double yaw, double& qx,
                         double& qy, double& qz, double& qw) {
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

}  // namespace carla_telemetry
