#include "carla_telemetry/npc_vehicles.hpp"

#include <carla/actors/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/trafficmanager/TrafficManager.h>

#include <algorithm>
#include <iostream>
#include <random>

namespace carla_telemetry {

CarlaNpcVehicles::CarlaNpcVehicles(carla::client::World& world,
                                   const Config& cfg,
                                   const carla::geom::Location& ego_loc) {
  if (!cfg.enabled || cfg.count <= 0) return;

  // ── Resolve blueprints ──────────────────────────────────────────────
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
    std::cerr << "[CarlaNpcVehicles] No vehicle blueprints matched the "
                 "requested types."
              << std::endl;
    return;
  }

  // ── Collect road-aligned spawn points via xodr waypoints ─────────────
  // Use GenerateWaypoints to get positions on actual road network.
  // This avoids spawning at off-road recommended points on custom maps
  // which cause TM to circle/jerk trying to find the nearest road.
  auto map = world.GetMap();
  auto waypoints = map->GenerateWaypoints(20.0);  // every 20m along roads

  if (waypoints.empty()) {
    std::cerr << "[CarlaNpcVehicles] No road waypoints found on this map."
              << std::endl;
    return;
  }

  // Build candidate transforms from waypoints, excluding near ego
  constexpr float kEgoExclusionRadius = 30.0f;  // meters
  constexpr float kMinSpacing = 15.0f;  // min distance between NPC spawns
  std::vector<carla::geom::Transform> candidates;

  for (const auto& wp : waypoints) {
    auto t = wp->GetTransform();

    // Skip if too close to ego spawn
    float dx = t.location.x - ego_loc.x;
    float dy = t.location.y - ego_loc.y;
    if (dx * dx + dy * dy < kEgoExclusionRadius * kEgoExclusionRadius) continue;

    // Raise spawn point slightly to avoid ground collision on spawn
    t.location.z += 0.5f;
    candidates.push_back(t);
  }

  if (candidates.empty()) {
    std::cerr
        << "[CarlaNpcVehicles] No road waypoints remaining after ego exclusion."
        << std::endl;
    return;
  }

  // Shuffle to avoid deterministic clustering
  std::mt19937 rng(std::random_device{}());
  std::shuffle(candidates.begin(), candidates.end(), rng);

  // ── Spawn vehicles with minimum spacing ─────────────────────────────
  int spawned = 0;
  std::vector<carla::geom::Location> used_locations;

  for (size_t i = 0; i < candidates.size() && spawned < cfg.count; ++i) {
    // Enforce minimum spacing between spawned vehicles
    bool too_close = false;
    for (const auto& loc : used_locations) {
      float dx = candidates[i].location.x - loc.x;
      float dy = candidates[i].location.y - loc.y;
      if (dx * dx + dy * dy < kMinSpacing * kMinSpacing) {
        too_close = true;
        break;
      }
    }
    if (too_close) continue;

    auto bp = valid_bps[rng() % valid_bps.size()];

    // Set role_name to avoid conflict with ego "hero"
    if (bp.ContainsAttribute("role_name")) {
      bp.SetAttribute("role_name", "autopilot");
    }

    auto actor = world.TrySpawnActor(bp, candidates[i]);
    if (!actor) continue;

    vehicles_.push_back(actor);
    used_locations.push_back(candidates[i].location);
    spawned++;
  }

  std::cerr << "[CarlaNpcVehicles] Spawned " << spawned
            << " NPC vehicles out of " << cfg.count << " requested."
            << std::endl;

  // Wait for a frame so actors register in the simulation. In sync mode the
  // client drives the frame; in async we wait for the server-driven tick.
  if (cfg.sync_mode) {
    world.Tick(std::chrono::seconds(10));
  } else {
    world.WaitForTick(std::chrono::seconds(10));
  }

  // ── Enable autopilot via Traffic Manager ────────────────────────────
  // TM registration is an RPC to the TM server on tm_port with a hardcoded
  // 2000ms timeout (TM_TIMEOUT). A stale TM server from a prior unclean run,
  // or an overloaded CARLA server, makes register_vehicle time out. Catch it
  // here and degrade gracefully (static NPCs) instead of letting the throw
  // abort the whole lifecycle on_configure.
  if (cfg.autopilot) {
    try {
      // Held as a member: a locally-hosted TM stops driving NPCs the
      // moment its handle dies, so it must outlive this constructor.
      tm_ = std::make_unique<carla::traffic_manager::TrafficManager>(
          world.GetEpisode(), cfg.tm_port);

      // Set TM sync to match world sync mode. In async, TM sync must be false
      // so the Traffic Manager steps vehicles on its own; otherwise it waits
      // for a client tick that never comes and NPCs stay frozen.
      tm_->SetSynchronousMode(cfg.sync_mode);

      // Hybrid physics: NPCs beyond the radius from the hero stop running full
      // PhysX and are dead-reckoned instead. Isolates TM's physics cost from
      // the ego/sensor budget without changing nearby NPC behavior.
      if (cfg.hybrid_physics) {
        tm_->SetHybridPhysicsMode(true);
        tm_->SetHybridPhysicsRadius(cfg.hybrid_physics_radius);
      }

      // Apply global settings
      tm_->SetGlobalDistanceToLeadingVehicle(cfg.distance_to_leading);

      for (auto& actor : vehicles_) {
        auto vehicle =
            boost::dynamic_pointer_cast<carla::client::Vehicle>(actor);
        if (vehicle) {
          vehicle->SetAutopilot(true, cfg.tm_port);

          // Speed control: prefer percentage-based (smooth, TM-native)
          // over absolute cap (fights TM PID → jerky)
          if (cfg.max_speed_kmh > 0.0f) {
            tm_->SetDesiredSpeed(actor, cfg.max_speed_kmh / 3.6f);
          } else {
            tm_->SetPercentageSpeedDifference(actor, cfg.speed_difference_pct);
          }

          // Safe following distance
          tm_->SetDistanceToLeadingVehicle(actor, cfg.distance_to_leading);

          // Lane change behavior
          tm_->SetAutoLaneChange(actor, cfg.auto_lane_change);

          // Respect other vehicles (0% chance of ignoring)
          tm_->SetPercentageIgnoreVehicles(actor, 0.0f);
        }
      }

      std::cerr << "[CarlaNpcVehicles] Traffic Manager configured: port="
                << cfg.tm_port << " speed_diff=" << cfg.speed_difference_pct
                << "%"
                << " dist=" << cfg.distance_to_leading << "m" << std::endl;
    } catch (const std::exception& e) {
      // Drop the handle so destroy() doesn't touch a half-built TM.
      tm_.reset();
      std::cerr << "[CarlaNpcVehicles] Traffic Manager unavailable on port "
                << cfg.tm_port << " (" << e.what() << "). NPCs spawned "
                << "WITHOUT autopilot. Check for a stale TM server or an "
                << "overloaded CARLA server." << std::endl;
    }
  }
}

CarlaNpcVehicles::~CarlaNpcVehicles() { destroy(); }

void CarlaNpcVehicles::destroy() {
  if (tm_) {
    try {
      tm_->UnregisterVehicles(vehicles_);
    } catch (...) {
    }

    // TrafficManager is only a handle (port + lookup into the static
    // TrafficManager::_tm_map); resetting it frees nothing. The real
    // TrafficManagerLocal holds a STRONG EpisodeProxy (shared_ptr<Simulator>)
    // and its own `new ::rpc::server(tm_port)` threads, so leaking it keeps
    // the old client alive past vehicle_.reset() (its threads then run during
    // static destruction -> heap corruption at exit) and makes the next
    // on_configure reuse this stale TM, still bound to the dead episode.
    // Release() stops it, kills its RPC server and drops the episode proxy —
    // the same call CARLA makes on episode restart. Must run while our client
    // is still connected: ~TrafficManagerLocal RPCs DestroyTrafficManager.
    try {
      carla::traffic_manager::TrafficManager::Release();
    } catch (...) {
    }
    tm_.reset();
  }
  for (auto& v : vehicles_) {
    if (v) {
      try {
        v->Destroy();
      } catch (...) {
      }
    }
  }
  vehicles_.clear();
}

}  // namespace carla_telemetry
