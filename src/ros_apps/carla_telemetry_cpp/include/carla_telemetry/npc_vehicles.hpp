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

/**
 * @brief Manages the lifecycle of NPC vehicles within the simulation.
 */
namespace carla_telemetry {

/**
 * @brief Configuration structure for NPC vehicle behavior and spawning.
 */
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

  /**
   * @brief Construct a new CarlaNpcVehicles object
   *
   * @param world CARLA world reference (must outlive this object).
   * @param cfg Parsed NPC vehicle configuration.
   * @param ego_loc Location of the ego vehicle to exclude.
   */
  CarlaNpcVehicles(carla::client::World& world, const Config& cfg,
                   const carla::geom::Location& ego_loc);

  /**
   * @brief Destroy the CarlaNpcVehicles object
   */
  ~CarlaNpcVehicles();

  /**
   * @brief Destroy the CarlaNpcVehicles object
   */
  void destroy();

 private:
  std::vector<carla::SharedPtr<carla::client::Actor>> vehicles_;

  /**
   * @brief Persistent handle to the locally-hosted CARLA Traffic Manager.
   *
   * @details This handle is maintained for the entire lifetime of the object
   * to ensure continuous control of autonomous NPCs, and is explicitly torn
   * down during lifecycle transitions.
   *
   * @note Key lifecycle and teardown constraints:
   *
   * 1. @b Lifetime @b persistence: A locally-hosted Traffic Manager stops
   *    driving NPCs immediately upon handle destruction. Therefore, the handle
   *    must remain valid while the simulation is active.
   *
   * 2. @b Clean @b teardown: Within @c destroy(), the instance is explicitly
   *    transitioned from synchronous to asynchronous mode before release. This
   *    prevents stale Traffic Manager server instances from lingering on
   *    @c tm_port across node reconfigure cycles.
   */
  std::unique_ptr<carla::traffic_manager::TrafficManager> tm_;
};

}  // namespace carla_telemetry
