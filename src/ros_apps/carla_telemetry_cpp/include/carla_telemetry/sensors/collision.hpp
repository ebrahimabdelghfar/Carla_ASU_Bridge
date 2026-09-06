#pragma once

#include <carla/client/Actor.h>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/sensor/data/CollisionEvent.h>

#include <string>

#include "carla_telemetry/types.hpp"

namespace carla_telemetry {

class CarlaROS2Backend;

// CARLA collision sensor. Event-driven rather than polled: the server fires a
// CollisionEvent per contact frame, so there is no state to sample and the
// callback publishes straight through the backend (the camera pattern).
class CarlaCollision {
 public:
  CarlaCollision(carla::client::World& world,
                 carla::SharedPtr<carla::client::Actor> vehicle,
                 CarlaROS2Backend* backend,
                 const std::string& frame_id = "base_link");

  void destroy();

 private:
  void on_collision(carla::SharedPtr<carla::sensor::SensorData> data);

  std::string frame_id_;
  CarlaROS2Backend* backend_;
  carla::SharedPtr<carla::client::Sensor> sensor_;
};

}  // namespace carla_telemetry
