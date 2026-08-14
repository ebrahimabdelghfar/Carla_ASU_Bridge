#include "carla_telemetry/dynamic_props.hpp"

#include <carla/actors/BlueprintLibrary.h>
#include <carla/client/Actor.h>
#include <carla/client/ActorList.h>
#include <carla/client/Map.h>

#include <cmath>
#include <iostream>
#include <random>

namespace carla_telemetry {

CarlaDynamicProps::CarlaDynamicProps(carla::client::World& world,
                                     const Config& cfg,
                                     const carla::geom::Location& ego_loc) {
  if (!cfg.enabled || cfg.count <= 0) return;

  // Resolve blueprints
  auto bp_lib = world.GetBlueprintLibrary();
  std::vector<carla::actors::ActorBlueprint> valid_bps;

  for (const auto& bp_str : cfg.blueprints) {
    auto bps = bp_lib->Filter(bp_str);
    if (bps) {
      for (size_t i = 0; i < bps->size(); ++i) {
        valid_bps.push_back((*bps)[i]);
      }
    }
  }

  if (valid_bps.empty()) {
    std::cerr
        << "[CarlaDynamicProps] No prop blueprints matched the requested types."
        << std::endl;
    return;
  }

  // Cleanup previous props
  auto all_actors = world.GetActors();
  int cleaned = 0;
  for (auto a : *all_actors) {
    if (a->GetTypeId().find("prop") != std::string::npos) {
      for (const auto& attr : a->GetAttributes()) {
        if (attr.GetId() == "role_name" && attr.GetValue() == "dynamic_prop") {
          if (a->IsAlive()) {
            try {
              a->Destroy();
              cleaned++;
            } catch (...) {
            }
          }
          break;
        }
      }
    }
  }
  if (cleaned > 0) {
    std::cerr << "[CarlaDynamicProps] Cleaned up " << cleaned
              << " leftover props from previous run." << std::endl;
  }

  // Check if the map has roads (if GetTopology is empty, the map might be a raw
  // mesh without roads)
  auto map = world.GetMap();
  bool map_has_roads = map && !map->GetTopology().empty();

  std::mt19937 rng(std::random_device{}());
  // Use a minimum distance to avoid spawning ON the robot
  float min_distance =
      cfg.min_distance_from_ego;  // Keepaway radius to avoid ego collision
  float max_dist = std::max(min_distance + 2.0f, cfg.max_distance);
  std::uniform_real_distribution<float> dist_r(min_distance, max_dist);
  std::uniform_real_distribution<float> dist_theta(0.0f, 2.0f * M_PI);
  std::uniform_real_distribution<float> dist_impulse_xy(-15.0f, 15.0f);
  std::uniform_real_distribution<float> dist_impulse_z(0.0f, 5.0f);

  int spawned = 0;
  int max_attempts = cfg.count * 10;
  int attempts = 0;

  std::cerr << "[CarlaDynamicProps] Spawning up to " << cfg.count
            << " dynamic props within " << max_dist << "m of the ego vehicle..."
            << std::endl;

  for (int i = 0; spawned < cfg.count && attempts < max_attempts; ++i) {
    attempts++;
    // Generate random coordinates using polar math relative to ego_loc
    float r = dist_r(rng);
    float theta = dist_theta(rng);

    float x = ego_loc.x + r * std::cos(theta);
    float y = ego_loc.y + r * std::sin(theta);

    carla::geom::Location spawn_loc(x, y, ego_loc.z);

    // Project to road if configured and possible
    if (cfg.spawn_on_roads && map_has_roads) {
      auto waypoint = map->GetWaypoint(
          spawn_loc, true,
          static_cast<int32_t>(carla::road::Lane::LaneType::Any));
      if (waypoint) {
        auto wp_transform = waypoint->GetTransform();
        float lane_width = waypoint->GetLaneWidth();

        // Add a random lateral offset so props aren't perfectly aligned in the
        // center of the lane
        std::uniform_real_distribution<float> dist_offset(
            -lane_width / 2.0f + 0.5f, lane_width / 2.0f - 0.5f);
        float offset = dist_offset(rng);
        auto right_vec = wp_transform.GetRightVector();

        spawn_loc.x = wp_transform.location.x + right_vec.x * offset;
        spawn_loc.y = wp_transform.location.y + right_vec.y * offset;
        spawn_loc.z = wp_transform.location.z;
      } else {
        continue;  // Skip this point if we can't find a road
      }
    }

    spawn_loc.z += cfg.spawn_height;

    // Ensure this spawn point is far enough from previously spawned props to
    // avoid overlap
    bool too_close = false;
    for (const auto& existing_prop : props_) {
      if (existing_prop->GetLocation().Distance(spawn_loc) <
          cfg.prop_to_prop_distance) {
        too_close = true;
        break;
      }
    }
    if (too_close) {
      continue;  // Skip this spawn point and try again
    }

    carla::geom::Transform spawn_point(spawn_loc,
                                       carla::geom::Rotation(0.0f, 0.0f, 0.0f));

    auto bp = valid_bps[rng() % valid_bps.size()];

    // Add a specific role name to identify these props for future cleanup
    if (bp.ContainsAttribute("role_name")) {
      bp.SetAttribute("role_name", "dynamic_prop");
    }

    auto actor = world.TrySpawnActor(bp, spawn_point);
    if (actor) {
      props_.push_back(actor);
      spawned++;

      // Enable physics
      actor->SetSimulatePhysics(true);

      // Add a random directional force (impulse)
      carla::geom::Vector3D impulse(dist_impulse_xy(rng), dist_impulse_xy(rng),
                                    dist_impulse_z(rng));
      actor->AddImpulse(impulse);
    }
  }

  std::cerr << "[CarlaDynamicProps] Successfully spawned " << spawned
            << " props." << std::endl;
}

CarlaDynamicProps::~CarlaDynamicProps() { destroy(); }

void CarlaDynamicProps::destroy() {
  for (auto& prop : props_) {
    if (prop && prop->IsAlive()) {
      try {
        prop->Destroy();
      } catch (...) {
      }
    }
  }
  props_.clear();
}

}  // namespace carla_telemetry
