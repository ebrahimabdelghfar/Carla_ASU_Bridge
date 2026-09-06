#include "carla_telemetry/sensors/collision.hpp"

#include <carla/actors/BlueprintLibrary.h>
#include <carla/geom/Transform.h>

#include "carla_telemetry/ros2_backend.hpp"
#include "carla_telemetry/sensors/sensor_clock.hpp"

namespace carla_telemetry {

CarlaCollision::CarlaCollision(carla::client::World& world,
                               carla::SharedPtr<carla::client::Actor> vehicle,
                               CarlaROS2Backend* backend,
                               const std::string& frame_id)
    : frame_id_(frame_id), backend_(backend) {
  auto bp_lib = world.GetBlueprintLibrary();
  auto bp = *bp_lib->Find("sensor.other.collision");

  // The blueprint has no extent or offset: it reports the vehicle's own
  // collisions wherever they happen, so it spawns at the actor origin.
  auto actor = world.SpawnActor(bp, carla::geom::Transform(), vehicle.get());
  sensor_ = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);

  sensor_->Listen([this](auto data) { this->on_collision(data); });
}

void CarlaCollision::on_collision(
    carla::SharedPtr<carla::sensor::SensorData> data) {
  auto event =
      boost::dynamic_pointer_cast<carla::sensor::data::CollisionEvent>(data);
  if (!event || !backend_) return;

  CollisionState s;
  s.capture_time = sensor_sim_to_epoch(event->GetTimestamp());
  s.frame_id = frame_id_;
  auto other = event->GetOtherActor();
  s.other_actor_id = other ? other->GetId() : 0;
  auto impulse = event->GetNormalImpulse();
  s.impulse_x = impulse.x;
  s.impulse_y = -impulse.y;  // CARLA is left-handed; ROS is not
  s.impulse_z = impulse.z;

  // One event per contact frame, so a scrape arrives as a burst. Publish every
  // one: what counts as a crash is the consumer's threshold, not ours.
  backend_->publish_collision(s);
}

void CarlaCollision::destroy() {
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
