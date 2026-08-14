#pragma once

// Spawning and navigation management for CARLA pedestrians (walkers).

#include <carla/client/Actor.h>
#include <carla/client/Client.h>
#include <carla/client/WalkerAIController.h>
#include <carla/client/World.h>

#include <memory>
#include <string>
#include <vector>

namespace carla_telemetry {

// Spawns and manages CARLA pedestrian actors and their AI controllers.
class CarlaWalkers {
 public:
  struct Config {
    bool enabled = false;
    int count = 0;
    std::vector<std::string> blueprints;
    float speed = 1.4f;
    bool sync_mode = true;
  };

  CarlaWalkers(carla::client::World& world, carla::client::Client& client,
               const Config& cfg);

  ~CarlaWalkers();

  // Pump the client-side pedestrian navigation (Recast/Detour crowd). Must be
  // called every frame or walkers stay frozen. In sync mode the world Tick()
  // already pumps it, so this only acts in async mode, where nothing else
  // triggers NavigationTick(). No-op when no walkers are active.
  void navigation_tick();

  void destroy();

 private:
  carla::client::World* world_ = nullptr;
  bool sync_mode_ = true;
  std::vector<carla::SharedPtr<carla::client::Actor>> walkers_;
  std::vector<carla::SharedPtr<carla::client::WalkerAIController>> controllers_;
};

}  // namespace carla_telemetry
