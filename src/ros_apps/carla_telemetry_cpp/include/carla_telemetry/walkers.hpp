#pragma once

/**
 * @file walkers.hpp
 * @brief Spawning and navigation management for CARLA pedestrians (walkers).
 */

#include <carla/client/Actor.h>
#include <carla/client/Client.h>
#include <carla/client/WalkerAIController.h>
#include <carla/client/World.h>

#include <memory>
#include <string>
#include <vector>

namespace carla_telemetry {

/**
 * @brief Spawns and manages CARLA pedestrian actors and their AI controllers.
 */
class CarlaWalkers {
 public:
  /**
   * @brief Configuration parameters for pedestrian spawning and movement.
   */
  struct Config {
    bool enabled = false;  ///< Whether pedestrian spawning is enabled.
    int count =
        0;  ///< Target number of pedestrian walkers to spawn in the simulation.
    std::vector<std::string> blueprints;  ///< List of walker blueprint IDs or
                                          ///< patterns to use when spawning.
    float speed =
        1.4f;  ///< Target walking speed for spawned pedestrians in m/s.
    bool sync_mode =
        true;  ///< Whether simulation is running in synchronous mode.
  };

  /**
   * @brief Construct a new CarlaWalkers object, spawning pedestrians and
   * attaching AI controllers.
   *
   * @param world Reference to the CARLA world where pedestrians will be
   * spawned.
   * @param client Reference to the CARLA client used for batch spawning
   * commands.
   * @param cfg Pedestrian configuration parameters including count, speed, and
   * blueprints.
   */
  CarlaWalkers(carla::client::World& world, carla::client::Client& client,
               const Config& cfg);

  /**
   * @brief Destroy the CarlaWalkers object, destroying all spawned walkers and
   * AI controllers.
   */
  ~CarlaWalkers();

  /**
   * @brief Pump the client-side pedestrian navigation (Recast/Detour crowd).
   *
   * @details Must be called every frame or walkers stay frozen. In sync mode
   * the world Tick() already pumps it, so this only acts in async mode, where
   * nothing else triggers NavigationTick(). No-op when no walkers are active.
   */
  void navigation_tick();

  /**
   * @brief Stop AI controllers and destroy all spawned walker and controller
   * actors in the world.
   */
  void destroy();

 private:
  carla::client::World* world_ =
      nullptr;  ///< Pointer to the CARLA world instance.
  bool sync_mode_ =
      true;  ///< Flag indicating if synchronous simulation mode is enabled.
  std::vector<carla::SharedPtr<carla::client::Actor>>
      walkers_;  ///< List of spawned pedestrian walker actors.
  std::vector<carla::SharedPtr<carla::client::WalkerAIController>>
      controllers_;  ///< List of walker AI controller actors driving pedestrian
                     ///< navigation.
};

}  // namespace carla_telemetry
