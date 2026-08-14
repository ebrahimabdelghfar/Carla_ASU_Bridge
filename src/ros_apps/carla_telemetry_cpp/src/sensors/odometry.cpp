#include "carla_telemetry/sensors/odometry.hpp"

#include <cmath>

#include "carla_telemetry/sensors/gps.hpp"

namespace carla_telemetry {

namespace {

// Rotate a world-frame vector into the body frame described by the unit
// quaternion (qx, qy, qz, qw): v' = q_inv * v * q, with q_inv = conjugate.
void rotate_to_body(double qx, double qy, double qz, double qw, double in_x,
                    double in_y, double in_z, double& out_x, double& out_y,
                    double& out_z) {
  double t0 = qw * in_x - qy * in_z + qz * in_y;
  double t1 = qw * in_y - qz * in_x + qx * in_z;
  double t2 = qw * in_z - qx * in_y + qy * in_x;
  double t3 = qx * in_x + qy * in_y + qz * in_z;

  out_x = t0 * qw + t3 * qx + t1 * qz - t2 * qy;
  out_y = t1 * qw + t3 * qy + t2 * qx - t0 * qz;
  out_z = t2 * qw + t3 * qz + t0 * qy - t1 * qx;
}

// Inverse of rotate_to_body: body frame → world frame (v' = q * v * q_inv).
void rotate_to_world(double qx, double qy, double qz, double qw, double in_x,
                     double in_y, double in_z, double& out_x, double& out_y,
                     double& out_z) {
  rotate_to_body(-qx, -qy, -qz, qw, in_x, in_y, in_z, out_x, out_y, out_z);
}

}  // namespace

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
  double vx, vy, vz;
  rotate_to_body(qx, qy, qz, qw, vx_world, vy_world, vz_world, vx, vy, vz);

  double wx, wy, wz;
  rotate_to_body(qx, qy, qz, qw, wx_world, wy_world, wz_world, wx, wy, wz);

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

  // Stash what predict() needs to carry this state forward: the state itself
  // and its velocities back in the world frame. They are re-derived from the
  // final body-frame values rather than reused from vx_world/wx_world above so
  // that any noise applied to the twist is carried too — the prediction then
  // reproduces the same twist instead of drawing a new sample.
  rotate_to_world(qx, qy, qz, qw, vx, vy, vz, vx_world_, vy_world_, vz_world_);
  rotate_to_world(qx, qy, qz, qw, wx, wy, wz, wx_world_, wy_world_, wz_world_);
  last_state_ = s;
  have_last_ = true;
  return s;
}

bool CarlaOdometry::position_source(uint64_t& frame, double& sim_time) const {
  if (mode_ != "gnss" || !gps_) return false;
  return gps_->enu_source(frame, sim_time);
}

std::optional<OdometryState> CarlaOdometry::predict(double dt) const {
  if (!have_last_) return std::nullopt;
  if (!(dt > 0.0)) return last_state_;

  OdometryState s = last_state_;

  // Position: constant velocity on the full 3D world vector. Steering-mode
  // agnostic — no bicycle/Ackermann assumption, so crab and parallel steering
  // integrate as correctly as normal steering.
  s.pos_x += vx_world_ * dt;
  s.pos_y += vy_world_ * dt;
  s.pos_z += vz_world_ * dt;

  // Orientation: dq is the axis-angle exponential of omega*dt. omega is
  // expressed in the world frame, so dq is applied on the LEFT
  // (q_new = dq (x) q_last), then renormalized against drift.
  double w_norm = std::sqrt(wx_world_ * wx_world_ + wy_world_ * wy_world_ +
                            wz_world_ * wz_world_);
  if (w_norm > 1e-12) {
    double half = 0.5 * w_norm * dt;
    double scale = std::sin(half) / w_norm;  // sin(theta/2) * (axis / |omega|)
    double dqx = wx_world_ * scale;
    double dqy = wy_world_ * scale;
    double dqz = wz_world_ * scale;
    double dqw = std::cos(half);

    double lx = last_state_.qx, ly = last_state_.qy, lz = last_state_.qz,
           lw = last_state_.qw;
    s.qw = dqw * lw - dqx * lx - dqy * ly - dqz * lz;
    s.qx = dqw * lx + dqx * lw + dqy * lz - dqz * ly;
    s.qy = dqw * ly - dqx * lz + dqy * lw + dqz * lx;
    s.qz = dqw * lz + dqx * ly - dqy * lx + dqz * lw;

    double n = std::sqrt(s.qx * s.qx + s.qy * s.qy + s.qz * s.qz + s.qw * s.qw);
    if (n > 1e-12) {
      s.qx /= n;
      s.qy /= n;
      s.qz /= n;
      s.qw /= n;
    }
  }

  // Body twist: re-derive by rotating the world vectors through the PREDICTED
  // orientation, not the old one, so twist.linear stays consistent with the
  // pose it ships with.
  rotate_to_body(s.qx, s.qy, s.qz, s.qw, vx_world_, vy_world_, vz_world_, s.vx,
                 s.vy, s.vz);
  rotate_to_body(s.qx, s.qy, s.qz, s.qw, wx_world_, wy_world_, wz_world_, s.wx,
                 s.wy, s.wz);
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
