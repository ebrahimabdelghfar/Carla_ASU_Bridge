#include "carla_telemetry/sensors/imu.hpp"

#include <carla/actors/BlueprintLibrary.h>
#include <carla/geom/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>

#include "carla_telemetry/perf_monitor.hpp"

namespace carla_telemetry {

CarlaIMU::CarlaIMU(carla::client::World& world,
                   carla::SharedPtr<carla::client::Actor> vehicle,
                   const std::string& frame_id, float x, float y, float z,
                   float roll, float pitch, float yaw)
    : frame_id_(frame_id) {
  auto bp_lib = world.GetBlueprintLibrary();
  auto bp = *bp_lib->Find("sensor.other.imu");

  carla::geom::Transform transform(carla::geom::Location(x, y, z),
                                   carla::geom::Rotation(pitch, yaw, roll));

  auto actor = world.SpawnActor(bp, transform, vehicle.get());
  sensor_ = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);

  sensor_->Listen([this](auto data) { this->on_imu(data); });
}

void CarlaIMU::on_imu(carla::SharedPtr<carla::sensor::SensorData> data) {
  auto imu =
      boost::dynamic_pointer_cast<carla::sensor::data::IMUMeasurement>(data);
  if (!imu) return;
  // Rate the server delivers IMU measurements to this client (inter-arrival).
  perf.record_interval(frame_id_ + ".server_dt");
  std::lock_guard<std::mutex> lk(lock_);
  last_data_ = imu;
}

std::optional<ImuState> CarlaIMU::get_state() {
  carla::SharedPtr<carla::sensor::data::IMUMeasurement> data;
  {
    std::lock_guard<std::mutex> lk(lock_);
    data = last_data_;
  }
  if (!data) return std::nullopt;

  auto clamp = [](double v) -> double { return std::clamp(v, -99.9, 99.9); };

  auto accel = data->GetAccelerometer();
  auto gyro = data->GetGyroscope();
  auto transform = data->GetSensorTransform();
  auto& rot = transform.rotation;

  double roll = static_cast<double>(rot.roll) * M_PI / 180.0;
  double pitch = static_cast<double>(-rot.pitch) * M_PI / 180.0;
  double yaw = M_PI / 2.0 - static_cast<double>(data->GetCompass());

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);  //+ 0.2098132

  ImuState s;
  s.accel_x = clamp(static_cast<double>(accel.x));
  s.accel_y = clamp(static_cast<double>(accel.y));
  s.accel_z = clamp(static_cast<double>(accel.z));
  s.gyro_x = clamp(static_cast<double>(gyro.x));
  s.gyro_y = clamp(static_cast<double>(gyro.y));
  s.gyro_z = clamp(static_cast<double>(gyro.z));
  s.qx = q.x();
  s.qy = q.y();
  s.qz = q.z();
  s.qw = q.w();
  s.frame_id = frame_id_;
  return s;
}

void CarlaIMU::destroy() {
  if (sensor_) {
    try {
      sensor_->Stop();
      sensor_->Destroy();
    } catch (...) {
    }
    sensor_ = nullptr;
  }
}

}  // namespace carla_telemetry
