#pragma once

#include <carla/client/Actor.h>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/sensor/data/IMUMeasurement.h>

#include <mutex>
#include <optional>
#include <string>

#include "carla_telemetry/types.hpp"

namespace carla_telemetry {

// CARLA IMU sensor wrapper. Direct port of CarlaIMU from imu.py.
class CarlaIMU {
 public:
  CarlaIMU(carla::client::World& world,
           carla::SharedPtr<carla::client::Actor> vehicle,
           const std::string& frame_id = "imu_link", float x = 0.0f,
           float y = 0.0f, float z = 0.0f, float roll = 0.0f,
           float pitch = 0.0f, float yaw = 0.0f);

  std::optional<ImuState> get_state();

  void destroy();

 private:
  void on_imu(carla::SharedPtr<carla::sensor::SensorData> data);

  std::string frame_id_;
  carla::SharedPtr<carla::client::Sensor> sensor_;
  std::mutex lock_;
  carla::SharedPtr<carla::sensor::data::IMUMeasurement> last_data_;
};

}  // namespace carla_telemetry
