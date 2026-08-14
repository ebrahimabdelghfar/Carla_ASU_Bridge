#include "carla_telemetry/walkers.hpp"

#include <carla/actors/BlueprintLibrary.h>
#include <carla/client/ActorList.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/rpc/Command.h>

#include <iostream>
#include <unordered_map>

namespace carla_telemetry {

CarlaWalkers::CarlaWalkers(carla::client::World& world,
                           carla::client::Client& client, const Config& cfg) {
  if (!cfg.enabled || cfg.count <= 0) return;

  world_ = &world;
  sync_mode_ = cfg.sync_mode;

  // CARLA's client-side navigation crowd (Recast/Detour) is built with a
  // fixed capacity of MAX_AGENTS = 500 (LibCarla nav/Navigation.cpp). Beyond
  // that, dtCrowd::addAgent() returns -1, so the walker never enters the crowd
  // and every SetMaxSpeed/GoToLocation on it fails ("NAV: ..." warning flood)
  // while it stands frozen. Clamp the request to the crowd capacity.
  constexpr int kMaxCrowdAgents = 500;
  int count = cfg.count;
  if (count > kMaxCrowdAgents) {
    std::cerr << "[CarlaWalkers] Requested " << count
              << " walkers but the navigation "
              << "crowd caps at " << kMaxCrowdAgents << "; clamping to "
              << kMaxCrowdAgents << "." << std::endl;
    count = kMaxCrowdAgents;
  }

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
        << "[CarlaWalkers] No walker blueprints matched the requested types."
        << std::endl;
    return;
  }

  auto controller_bps = bp_lib->Filter("controller.ai.walker");
  if (controller_bps->empty()) {
    std::cerr << "[CarlaWalkers] No controller.ai.walker blueprint found."
              << std::endl;
    return;
  }
  auto controller_bp = (*controller_bps)[0];

  // Tick/WaitForTick helper: in sync mode the client drives the frame; in
  // async we wait for the server-driven tick.
  auto advance_frame = [&]() {
    if (cfg.sync_mode) {
      world.Tick(std::chrono::seconds(10));
    } else {
      world.WaitForTick(std::chrono::seconds(10));
    }
  };

  // Each RPC round-trip is sequential, so spawning N walkers + N controllers
  // one-by-one is O(2N) blocking calls — slow at scale. Instead issue each
  // spawn phase as a single ApplyBatchSync (one round-trip for the whole
  // batch). Controllers still need a separate batch after their walkers are
  // registered (canonical generate_traffic.py pattern): a controller attaches
  // to a walker id, which only exists once the walker's spawn frame ticks.
  // The AI setup (Start/collision/speed/destination) has no batch command, so
  // it stays a loop — but those are light nav ops, not actor spawns.

  // Phase 1: batch-spawn all walkers
  std::vector<carla::rpc::Command> spawn_batch;
  spawn_batch.reserve(count);
  for (int i = 0; i < count; ++i) {
    auto walker_bp = valid_bps[rand() % valid_bps.size()];
    if (walker_bp.ContainsAttribute("is_invincible")) {
      walker_bp.SetAttribute("is_invincible", "false");
    }
    auto loc = world.GetRandomLocationFromNavigation();
    if (!loc) {
      std::cerr << "[CarlaWalkers] Warning: No navigation location found. "
                   "Skipping spawn."
                << std::endl;
      continue;
    }
    carla::geom::Transform spawn_transform(loc.get(), carla::geom::Rotation());
    spawn_batch.emplace_back(carla::rpc::Command::SpawnActor(
        walker_bp.MakeActorDescription(), spawn_transform));
  }

  std::vector<carla::ActorId> walker_ids;
  for (auto& resp : client.ApplyBatchSync(spawn_batch, false)) {
    if (!resp.HasError()) walker_ids.push_back(resp.Get());
  }

  // Let walkers register in the simulation before attaching controllers.
  advance_frame();

  // Phase 2: batch-spawn one AI controller per walker
  auto controller_desc = controller_bp.MakeActorDescription();
  std::vector<carla::rpc::Command> ctrl_batch;
  ctrl_batch.reserve(walker_ids.size());
  for (auto wid : walker_ids) {
    ctrl_batch.emplace_back(carla::rpc::Command::SpawnActor(
        controller_desc, carla::geom::Transform(), wid));
  }

  std::vector<carla::ActorId> ctrl_ids, paired_walker_ids;
  std::vector<carla::rpc::Command> orphan_cleanup;
  auto ctrl_resp = client.ApplyBatchSync(ctrl_batch, false);
  for (size_t i = 0; i < ctrl_resp.size(); ++i) {
    if (!ctrl_resp[i].HasError()) {
      paired_walker_ids.push_back(walker_ids[i]);
      ctrl_ids.push_back(ctrl_resp[i].Get());
    } else {
      orphan_cleanup.emplace_back(
          carla::rpc::Command::DestroyActor(walker_ids[i]));
    }
  }
  if (!orphan_cleanup.empty()) client.ApplyBatchSync(orphan_cleanup, false);

  // Let controllers register before Start().
  advance_frame();

  // Resolve actor handles in one round-trip each, then pair them 1:1.
  std::unordered_map<carla::ActorId, carla::SharedPtr<carla::client::Actor>>
      by_id;
  for (auto ids : {paired_walker_ids, ctrl_ids}) {
    auto actors = world.GetActors(ids);
    for (size_t i = 0; i < actors->size(); ++i) {
      auto a = actors->at(i);
      by_id[a->GetId()] = a;
    }
  }
  for (size_t i = 0; i < paired_walker_ids.size(); ++i) {
    auto walker = by_id[paired_walker_ids[i]];
    auto ai = boost::dynamic_pointer_cast<carla::client::WalkerAIController>(
        by_id[ctrl_ids[i]]);
    if (walker && ai) {
      walkers_.push_back(walker);
      controllers_.push_back(ai);
    }
  }

  std::cerr << "[CarlaWalkers] Spawned " << walkers_.size()
            << " walkers out of " << count << " requested." << std::endl;

  // Start controllers and assign destinations. controllers_ and walkers_ are
  // index-aligned (paired 1:1 in phase 2).
  for (size_t i = 0; i < controllers_.size(); ++i) {
    auto& ai = controllers_[i];
    ai->Start();
    // WalkerAIController::Start() disables the walker's collision so the
    // navigation crowd can drive it kinematically. But the ray-cast LiDAR
    // is a PhysX line trace — with collision off, rays pass straight
    // through and the pedestrian is invisible to LiDAR (cameras still see
    // it, being render-based). Re-enable collision; physics stays off so
    // navigation keeps moving the walker.
    walkers_[i]->SetCollisions(true);
    ai->SetMaxSpeed(cfg.speed);
    auto dest = world.GetRandomLocationFromNavigation();
    if (dest) {
      ai->GoToLocation(dest.get());
    } else {
      std::cerr << "[CarlaWalkers] Warning: No navigation destination found."
                << std::endl;
    }
  }
}

void CarlaWalkers::navigation_tick() {
  // Pedestrian navigation (Recast/Detour crowd) is a CLIENT-side step run only
  // inside Simulator::WaitForTick()/Tick() via NavigationTick(). In sync mode
  // the world Tick() in the node's control loop already pumps it. In async
  // nothing else does, so without this call the crowd never advances and every
  // walker stays frozen (while sensors still stream, being server-push). Pump
  // one server frame here; the node calls this every timer tick.
  if (sync_mode_ || !world_ || walkers_.empty()) return;
  try {
    world_->WaitForTick(std::chrono::seconds(1));
  } catch (...) {
  }
}

CarlaWalkers::~CarlaWalkers() { destroy(); }

void CarlaWalkers::destroy() {
  for (auto& ai : controllers_) {
    if (ai) {
      try {
        ai->Stop();
        ai->Destroy();
      } catch (...) {
      }
    }
  }
  controllers_.clear();

  for (auto& w : walkers_) {
    if (w) {
      try {
        w->Destroy();
      } catch (...) {
      }
    }
  }
  walkers_.clear();
}

}  // namespace carla_telemetry
