#pragma once

// Common data structures for CARLA telemetry sensor states and ground truth
// messages.

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace carla_telemetry {

struct GpsState {
  double latitude = 0.0;        // deg, measured
  double longitude = 0.0;       // deg, measured
  double altitude = 0.0;        // m, measured
  double latitude_gt = 0.0;     // deg, ground truth
  double longitude_gt = 0.0;    // deg, ground truth
  double altitude_gt = 0.0;     // m, ground truth
  double speed = 0.0;           // m/s, ground speed
  double velocity_north = 0.0;  // m/s
  double velocity_east = 0.0;   // m/s
  double velocity_down = 0.0;   // m/s
  int fix_type = 3;             // e.g. 3 = 3D fix
  double eph = 1.0;             // m, horizontal position error stddev
  double epv = 1.0;             // m, vertical position error stddev
  int satellites = 10;
  double capture_time = 0.0;  // epoch seconds
};

struct BatteryState {
  double voltage = 0.0;
  double open_circuit_v = 0.0;
  double current = 0.0;          // A, negative when discharging
  double charge = 0.0;           // Ah
  double capacity = 0.0;         // Ah
  double charge_fraction = 0.0;  // [0.0, 1.0]
  double percentage = 0.0;       // [0.0, 100.0]
  double power_load = 0.0;       // W
  double temperature = 25.0;     // deg C
  bool is_depleted = false;
  double capture_time = 0.0;  // epoch seconds
};

struct ImuState {
  double accel_x = 0.0;  // m/s^2
  double accel_y = 0.0;  // m/s^2
  double accel_z = 0.0;  // m/s^2
  double gyro_x = 0.0;   // rad/s
  double gyro_y = 0.0;   // rad/s
  double gyro_z = 0.0;   // rad/s
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  std::string frame_id = "imu_link";
  double capture_time = 0.0;  // epoch seconds
};

struct OdometryState {
  std::string frame_id = "odom";
  std::string child_frame_id = "base_link";
  double pos_x = 0.0;  // m
  double pos_y = 0.0;  // m
  double pos_z = 0.0;  // m
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  double vx = 0.0;  // m/s
  double vy = 0.0;  // m/s
  double vz = 0.0;  // m/s
  double wx = 0.0;  // rad/s
  double wy = 0.0;  // rad/s
  double wz = 0.0;  // rad/s
  bool broadcast_tf = false;
  double capture_time = 0.0;  // epoch seconds
};

// One ground-truth 3D bounding box, in the ego base_link frame.
struct GroundTruthBox {
  std::string label;      // Waymo class: vehicle|pedestrian|sign|cyclist
  uint32_t track_id = 0;  // CARLA actor ID, tracked across frames
  float score = 1.0f;     // ground truth is always confident
  double px = 0.0;        // m
  double py = 0.0;        // m
  double pz = 0.0;        // m
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  double sx = 0.0;        // m, full box length (X)
  double sy = 0.0;        // m, full box width (Y)
  double sz = 0.0;        // m, full box height (Z)
  double vx = 0.0;        // m/s, base_link frame
  double vy = 0.0;        // m/s, base_link frame
  std::string attribute;  // "moving" | "stopped"
  bool is_ego = false;    // ego renders green, others red
};

struct GroundTruthBoxes {
  std::vector<GroundTruthBox> boxes;
  std::string frame_id = "base_link";
  double capture_time = 0.0;  // epoch seconds
};

struct CameraData {
  std::vector<uint8_t> rgb;  // BGRA buffer, H*W*4 (native CARLA format)
  std::array<double, 9> intrinsics{};  // 3x3 row-major K matrix
  int width = 0;
  int height = 0;
  std::string frame_id = "camera";
  std::string encoding = "rgb8";
  double capture_time = 0.0;  // epoch seconds
};

struct LidarData {
  std::vector<uint8_t> data;  // interleaved XYZI float32, 16 bytes/point
  size_t num_points = 0;
  std::string frame_id = "lidar";
  std::string lidar_name;
  double capture_time = 0.0;  // epoch seconds
};

}  // namespace carla_telemetry
