#pragma once

#include <carla/client/Actor.h>
#include <carla/client/Vehicle.h>
#include <carla/client/World.h>
#include <carla/geom/Location.h>

#include <memory>
#include <string>
#include <vector>

namespace carla {
namespace traffic_manager {
class TrafficManager;
}
}  // namespace carla

// Manages the lifecycle of NPC vehicles within the simulation.
namespace carla_telemetry {

class CarlaNpcVehicles {
 public:
  struct Config {
    bool enabled = false;
    int count = 0;
    std::vector<std::string> blueprints;
    bool autopilot = true;
    uint16_t tm_port = 8000;
    float max_speed_kmh =
        0.0f;  // km/h absolute cap (0 = disabled, use pct instead)
    float speed_difference_pct =
        0.0f;  // % slower than road speed limit (negative = faster)
    float distance_to_leading = 5.0f;  // meters
    bool auto_lane_change = false;
    bool hybrid_physics = true;           // TM hybrid physics mode
    float hybrid_physics_radius = 70.0f;  // meters from hero for full physics
    bool sync_mode = true;                // must match world sync mode
  };

  // world must outlive this object. ego_loc excludes the ego vehicle from
  // spawn point selection.
  CarlaNpcVehicles(carla::client::World& world, const Config& cfg,
                   const carla::geom::Location& ego_loc);

  ~CarlaNpcVehicles();

  void destroy();

 private:
  std::vector<carla::SharedPtr<carla::client::Actor>> vehicles_;

  // Handle to the locally-hosted CARLA Traffic Manager. A locally-hosted TM
  // stops driving NPCs immediately on handle destruction, so this is kept
  // alive for the object's full lifetime. destroy() explicitly switches it
  // from synchronous to asynchronous mode before release, so a stale TM
  // server doesn't linger on tm_port across node reconfigure cycles.
  std::unique_ptr<carla::traffic_manager::TrafficManager> tm_;
};

}  // namespace carla_telemetry
