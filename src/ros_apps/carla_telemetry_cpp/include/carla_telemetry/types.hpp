#pragma once

/**
 * @file types.hpp
 * @brief Common data structures for CARLA telemetry sensor states and ground
 * truth messages.
 */

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace carla_telemetry {

/**
 * @brief Represents the state of a GPS sensor and corresponding ground-truth
 * coordinates.
 */
struct GpsState {
  double latitude = 0.0;      ///< Measured latitude in degrees.
  double longitude = 0.0;     ///< Measured longitude in degrees.
  double altitude = 0.0;      ///< Measured altitude in meters.
  double latitude_gt = 0.0;   ///< Ground-truth latitude in degrees.
  double longitude_gt = 0.0;  ///< Ground-truth longitude in degrees.
  double altitude_gt = 0.0;   ///< Ground-truth altitude in meters.
  double speed = 0.0;         ///< Ground speed in meters per second.
  double velocity_north =
      0.0;  ///< Velocity component toward North in meters per second.
  double velocity_east =
      0.0;  ///< Velocity component toward East in meters per second.
  double velocity_down =
      0.0;           ///< Velocity component toward Down in meters per second.
  int fix_type = 3;  ///< GPS fix type (e.g., 3 for 3D fix).
  double eph =
      1.0;  ///< Standard deviation of horizontal position error in meters.
  double epv =
      1.0;  ///< Standard deviation of vertical position error in meters.
  int satellites = 10;  ///< Number of visible satellites.
  double capture_time =
      0.0;  ///< Wall-clock timestamp of the capture in epoch seconds.
};

/**
 * @brief Represents the state and metrics of the vehicle's battery.
 */
struct BatteryState {
  double voltage = 0.0;         ///< Battery voltage in Volts.
  double open_circuit_v = 0.0;  ///< Open-circuit voltage in Volts.
  double current =
      0.0;  ///< Battery current in Amperes (negative when discharging).
  double charge = 0.0;    ///< Current charge in Ampere-hours.
  double capacity = 0.0;  ///< Total battery capacity in Ampere-hours.
  double charge_fraction =
      0.0;  ///< Fraction of remaining charge (range [0.0, 1.0]).
  double percentage =
      0.0;  ///< Percentage of remaining charge (range [0.0, 100.0]).
  double power_load = 0.0;    ///< Active power consumption load in Watts.
  double temperature = 25.0;  ///< Battery temperature in degrees Celsius.
  bool is_depleted =
      false;  ///< Flag indicating whether the battery is depleted.
  double capture_time =
      0.0;  ///< Wall-clock timestamp of the capture in epoch seconds.
};

/**
 * @brief Represents the measurements from an Inertial Measurement Unit (IMU).
 */
struct ImuState {
  double accel_x = 0.0;  ///< Linear acceleration along X-axis in m/s^2.
  double accel_y = 0.0;  ///< Linear acceleration along Y-axis in m/s^2.
  double accel_z = 0.0;  ///< Linear acceleration along Z-axis in m/s^2.
  double gyro_x = 0.0;   ///< Angular velocity around X-axis in rad/s.
  double gyro_y = 0.0;   ///< Angular velocity around Y-axis in rad/s.
  double gyro_z = 0.0;   ///< Angular velocity around Z-axis in rad/s.
  double qx = 0.0;       ///< X component of orientation quaternion.
  double qy = 0.0;       ///< Y component of orientation quaternion.
  double qz = 0.0;       ///< Z component of orientation quaternion.
  double qw = 1.0;       ///< W (scalar) component of orientation quaternion.
  std::string frame_id =
      "imu_link";  ///< Coordinate frame ID for the IMU sensor.
  double capture_time =
      0.0;  ///< Wall-clock timestamp of the capture in epoch seconds.
};

/**
 * @brief Represents 3D odometry state including position, orientation, linear
 * and angular velocities.
 */
struct OdometryState {
  std::string frame_id =
      "odom";  ///< Reference coordinate frame ID (typically "odom").
  std::string child_frame_id =
      "base_link";     ///< Child coordinate frame ID (typically vehicle's
                       ///< "base_link").
  double pos_x = 0.0;  ///< Position X coordinate in meters.
  double pos_y = 0.0;  ///< Position Y coordinate in meters.
  double pos_z = 0.0;  ///< Position Z coordinate in meters.
  double qx = 0.0;     ///< X component of orientation quaternion.
  double qy = 0.0;     ///< Y component of orientation quaternion.
  double qz = 0.0;     ///< Z component of orientation quaternion.
  double qw = 1.0;     ///< W (scalar) component of orientation quaternion.
  double vx = 0.0;     ///< Linear velocity along X-axis in m/s.
  double vy = 0.0;     ///< Linear velocity along Y-axis in m/s.
  double vz = 0.0;     ///< Linear velocity along Z-axis in m/s.
  double wx = 0.0;     ///< Angular velocity around X-axis in rad/s.
  double wy = 0.0;     ///< Angular velocity around Y-axis in rad/s.
  double wz = 0.0;     ///< Angular velocity around Z-axis in rad/s.
  bool broadcast_tf =
      false;  ///< Whether to broadcast this odometry as a ROS TF transform.
  double capture_time =
      0.0;  ///< Wall-clock timestamp of the capture in epoch seconds.
};

/**
 * @brief One ground-truth 3D bounding box, in the ego base_link frame.
 */
struct GroundTruthBox {
  std::string
      label;  ///< Waymo classification class: vehicle|pedestrian|sign|cyclist.
  uint32_t track_id =
      0;  ///< CARLA actor ID tracking this object across frames.
  float score =
      1.0f;  ///< Detection confidence score (ground truth → always 1.0).
  double px = 0.0;  ///< Center position X coordinate in meters.
  double py = 0.0;  ///< Center position Y coordinate in meters.
  double pz = 0.0;  ///< Center position Z coordinate in meters.
  double qx = 0.0;  ///< X component of orientation quaternion.
  double qy = 0.0;  ///< Y component of orientation quaternion.
  double qz = 0.0;  ///< Z component of orientation quaternion.
  double qw = 1.0;  ///< W (scalar) component of orientation quaternion.
  double sx =
      0.0;  ///< Full bounding box dimension along X-axis (length) in meters.
  double sy =
      0.0;  ///< Full bounding box dimension along Y-axis (width) in meters.
  double sz =
      0.0;  ///< Full bounding box dimension along Z-axis (height) in meters.
  double vx = 0.0;  ///< Linear velocity component along X-axis in base_link
                    ///< frame in m/s.
  double vy = 0.0;  ///< Linear velocity component along Y-axis in base_link
                    ///< frame in m/s.
  std::string attribute;  ///< Motion state attribute: "moving" | "stopped".
  bool is_ego = false;  ///< True if this box is the ego vehicle (ego → green
                        ///< marker, others → red).
};

/**
 * @brief A container for a collection of ground-truth 3D bounding boxes in a
 * specific frame.
 */
struct GroundTruthBoxes {
  std::vector<GroundTruthBox>
      boxes;  ///< List of ground-truth 3D bounding boxes.
  std::string frame_id =
      "base_link";  ///< Coordinate frame ID for the bounding boxes.
  double capture_time =
      0.0;  ///< Wall-clock timestamp of the capture in epoch seconds.
};

/**
 * @brief Holds raw image data and camera calibration parameters.
 */
struct CameraData {
  std::vector<uint8_t>
      rgb;  ///< Flat BGRA image buffer of size H*W*4 (native CARLA format).
  std::array<double, 9>
      intrinsics{};  ///< 3x3 row-major camera intrinsics (K matrix).
  int width = 0;     ///< Image width in pixels.
  int height = 0;    ///< Image height in pixels.
  std::string frame_id =
      "camera";  ///< Coordinate frame ID for the camera sensor.
  std::string encoding =
      "rgb8";  ///< Image encoding format (e.g., "rgb8" or "bgra8").
  double capture_time =
      0.0;  ///< Wall-clock timestamp of the capture in epoch seconds.
};

/**
 * @brief Holds point cloud data and metadata for a LiDAR sensor.
 */
struct LidarData {
  std::vector<uint8_t> data;  ///< Interleaved XYZI float32 point cloud data (16
                              ///< bytes per point).
  size_t num_points = 0;  ///< Number of 3D points in the point cloud buffer.
  std::string frame_id =
      "lidar";  ///< Coordinate frame ID for the LiDAR sensor.
  std::string
      lidar_name;  ///< Descriptive name or identifier of the LiDAR sensor.
  double capture_time =
      0.0;  ///< Wall-clock timestamp of the capture in epoch seconds.
};

}  // namespace carla_telemetry
