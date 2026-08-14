#pragma once

#include <carla/client/Actor.h>
#include <carla/client/World.h>
#include <carla/geom/BoundingBox.h>
#include <carla/rpc/EnvironmentObject.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "carla_telemetry/types.hpp"

namespace carla_telemetry {

// Ground-truth 3D bounding boxes of world actors, expressed in the ego
// base_link frame. Read directly from CARLA each tick — no sensor spawned.
class CarlaGroundTruthBoxes {
 public:
  // Detection window — mimics a LiDAR's field of view. Boxes whose
  // ego-relative center falls outside the window are dropped before
  // publishing. The ego box is always kept. Defaults publish everything
  // (unbounded 360 degrees).
  struct RangeWindow {
    double max_range = 0.0;         // m, radial XY distance; <=0 = unlimited
    double min_range = 0.0;         // m, radial XY blind zone around ego
    double horizontal_fov = 360.0;  // deg, centered on ego +x (forward)
    double z_min = 0.0;  // m, lower vertical bound rel. ego base_link
    double z_max =
        0.0;  // m, upper vertical bound; applied only if z_max > z_min
  };

  CarlaGroundTruthBoxes(const std::string& frame_id, const RangeWindow& window);

  // Scan all world actors (vehicles / cyclists / pedestrians / signs) and
  // return their boxes relative to the ego vehicle's base_link frame.
  GroundTruthBoxes get_boxes(carla::client::World& world,
                             carla::SharedPtr<carla::client::Actor> ego);

 private:
  std::string frame_id_;
  RangeWindow window_;
  // Bounding boxes are static per actor; this fork's Actor::GetBoundingBox()
  // does a blocking per-actor RPC, so cache by actor id and fetch once.
  std::unordered_map<uint32_t, carla::geom::BoundingBox> bbox_cache_;

  // Parked / map-baked vehicles are environment objects (CityObjectLabel::Car),
  // not actors, so GetActors() misses them. They never move — fetch once (one
  // RPC) and cache their world-space boxes; ego-relative pose is recomputed
  // each tick.
  bool env_vehicles_loaded_ = false;
  std::vector<carla::rpc::EnvironmentObject> env_vehicles_;
};

}  // namespace carla_telemetry
