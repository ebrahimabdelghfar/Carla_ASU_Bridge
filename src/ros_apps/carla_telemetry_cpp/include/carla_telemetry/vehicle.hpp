#pragma once

/**
 * @file vehicle.hpp
 * @brief CARLA client lifecycle management and ego vehicle spawning and
 * control.
 */

#include <carla/client/Actor.h>
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/rpc/EpisodeSettings.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace carla_telemetry {

/**
 * @brief CARLA client lifecycle and ego vehicle management.
 * Direct port of CarlaVehicle from vehicle.py.
 */
class CarlaVehicle {
 public:
  /**
   * @brief Configuration parameters for connection, world setup, and ego
   * spawning.
   */
  struct Config {
    std::string host = "localhost";  ///< CARLA server hostname or IP address.
    int port = 2000;                 ///< CARLA server TCP port.
    double timeout = 10.0;           ///< RPC connection timeout in seconds.
    bool sync_mode =
        true;  ///< Enable synchronous mode in the simulation world.
    double fixed_delta =
        0.01667;  ///< Simulation step delta time in seconds (~60 FPS).
    std::string
        town;  ///< Name of the map/town to load (if empty, uses current).
    int spawn_index = 0;  ///< Index of the predefined map spawn point to use.
    bool use_coords = false;   ///< If true, spawn using explicit coordinates
                               ///< rather than spawn_index.
    float spawn_x = 0.0f;      ///< Explicit spawn X coordinate in meters.
    float spawn_y = 0.0f;      ///< Explicit spawn Y coordinate in meters.
    float spawn_z = 0.0f;      ///< Explicit spawn Z coordinate in meters.
    float spawn_roll = 0.0f;   ///< Explicit spawn roll orientation in degrees.
    float spawn_pitch = 0.0f;  ///< Explicit spawn pitch orientation in degrees.
    float spawn_yaw = 0.0f;    ///< Explicit spawn yaw orientation in degrees.
    std::string blueprint =
        "vehicle.tesla.model3";  ///< CARLA actor blueprint ID for the ego
                                 ///< vehicle.
    std::string
        role_name;  ///< Role name attribute assigned to the ego vehicle actor.
    std::string generation =
        "2";  ///< Actor blueprint generation attribute (e.g., "1" or "2").
    std::string color;  ///< Color attribute for the vehicle RGB paint (e.g.,
                        ///< "255,0,0").
    bool open_manual_control =
        false;  ///< Whether to spawn the manual_control.py pygame window.
    std::string manual_control_res =
        "640x360";  ///< Pygame window and display camera resolution (smaller
                    ///< window reduces GPU load on world tick).
    double manual_control_render_rate =
        10.0;  ///< Manual control render rate in Hz (<= 0 means render every
               ///< tick; capping prevents starving telemetry sensors).
    bool use_current_world = false;  ///< If true, skips LoadWorld and uses the
                                     ///< currently loaded CARLA world.
  };

  /**
   * @brief Vehicle physical attributes, engine parameters, transmission, and
   * wheel properties.
   */
  struct PhysicsConfig {
    bool enabled = false;  ///< Whether custom physics configuration is enabled.
    float mass = 1500.0f;  ///< Total vehicle mass in kilograms.
    float drag = 0.3f;     ///< Aerodynamic drag coefficient.
    bool override_center_of_mass =
        false;  ///< Whether to override the default center of mass offset.
    float com_x = 0.0f;       ///< Center of mass offset along X-axis in meters.
    float com_y = 0.0f;       ///< Center of mass offset along Y-axis in meters.
    float com_z = 0.0f;       ///< Center of mass offset along Z-axis in meters.
    float max_rpm = 6000.0f;  ///< Maximum engine RPM.
    float moi = 1.0f;         ///< Engine moment of inertia (kg*m^2).
    float damping_full = 0.15f;  ///< Damping rate when throttle is full.
    float damping_zero_eng =
        2.0f;  ///< Damping rate when throttle is zero with clutch engaged.
    float damping_zero_dis =
        0.35f;  ///< Damping rate when throttle is zero with clutch disengaged.
    std::vector<std::pair<float, float>>
        torque_curve;  ///< Engine torque curve points (RPM -> torque in Nm).
    std::vector<std::pair<float, float>>
        steering_curve;  ///< Steering curve points (speed -> steering angle in
                         ///< degrees).
    bool use_sweep = false;  ///< Enable wheel collision sweep tests.

    std::string transmission_type =
        "automatic";  ///< Transmission type ("automatic" or "manual").
    float gear_switch_time =
        0.5f;  ///< Time duration required to switch gears in seconds.
    float clutch_strength = 10.0f;  ///< Clutch coupling strength.
    float final_ratio =
        4.0f;  ///< Fixed gear ratio between transmission and wheels.

    /**
     * @brief Individual gear ratio specification.
     */
    struct Gear {
      float ratio;       ///< Gear transmission ratio.
      float down_ratio;  ///< RPM ratio threshold for shifting down.
      float up_ratio;    ///< RPM ratio threshold for shifting up.
    };
    std::vector<Gear>
        forward_gears;  ///< List of forward gears and their shift parameters.

    std::string drive_mode = "AWD";  ///< Drive mode ("AWD", "FWD", or "RWD").

    /**
     * @brief Individual wheel physics and geometry specification.
     */
    struct Wheel {
      std::string position;  ///< Wheel identifier or position name (e.g., "FL",
                             ///< "FR", "RL", "RR").
      float tire_friction = 3.5f;  ///< Coefficient of friction for the tire.
      float damping_rate = 0.25f;  ///< Wheel damping rate.
      float max_steer_angle =
          70.0f;  ///< Maximum steering angle for the wheel in degrees.
      float radius = 36.0f;              ///< Wheel radius in centimeters.
      float max_brake_torque = 1500.0f;  ///< Maximum braking torque in Nm.
      float max_handbrake_torque =
          3000.0f;  ///< Maximum handbrake torque in Nm.
    };
    std::vector<Wheel>
        wheels;  ///< List of configurations for all vehicle wheels (up to 4).
  };

  /**
   * @brief Tire friction assigned to non-driven wheels to emulate FWD/RWD on a
   * CARLA vehicle that is physically AWD.
   *
   * @details Also honoured by runtime friction changes (see
   * CarlaROS2Backend::apply_tire_friction) so commanding a new ground friction
   * does not silently turn an FWD/RWD vehicle back into AWD.
   */
  static constexpr float kNonDrivenTireFriction = 0.1f;

  /**
   * @brief Construct a new CarlaVehicle object, establishing server connection
   * and spawning the ego vehicle.
   *
   * @param cfg General vehicle and server configuration.
   * @param physics Custom vehicle physics configuration.
   */
  CarlaVehicle(const Config& cfg, const PhysicsConfig& physics);

  /**
   * @brief Destroy the CarlaVehicle object, cleaning up actors and
   * subprocesses.
   */
  ~CarlaVehicle();

  /**
   * @brief Access the underlying CARLA world reference.
   *
   * @return carla::client::World& Reference to the CARLA world.
   */
  carla::client::World& world() { return *world_; }

  /**
   * @brief Access the underlying CARLA client reference.
   *
   * @return carla::client::Client& Reference to the CARLA client.
   */
  carla::client::Client& client() { return *client_; }

  /**
   * @brief Access the CARLA actor representing the ego vehicle.
   *
   * @return carla::SharedPtr<carla::client::Actor> Shared pointer to the ego
   * vehicle actor.
   */
  carla::SharedPtr<carla::client::Actor> actor() { return actor_; }

  /**
   * @brief Advance the simulation world by one tick in synchronous mode.
   */
  void tick();

  /**
   * @brief Pause simulation ticking (temporarily halts simulation time
   * progression).
   */
  void pause();

  /**
   * @brief Resume simulation ticking.
   */
  void resume();

  /**
   * @brief Check whether the simulation ticking is active and running.
   *
   * @return true If simulation ticking is active.
   * @return false If simulation ticking is paused.
   */
  bool is_running() const { return running_; }

  /**
   * @brief Optionally spawn manual_control.py as an external subprocess.
   */
  void spawn_manual_control();

  /**
   * @brief Terminate the manual_control.py subprocess (SIGTERM, escalate to
   * SIGKILL, then reap).
   *
   * @details Idempotent. Safe to call before destroy() so the pygame window
   * closes promptly even if later CARLA RPCs block.
   */
  void stop_manual_control();

  /**
   * @brief Destroy the spawned ego vehicle actor and restore original world
   * settings.
   */
  void destroy();

 private:
  /**
   * @brief Establish RPC connection to CARLA server and configure world
   * settings.
   *
   * @param cfg Configuration containing host, port, timeouts, and map
   * parameters.
   */
  void connect(const Config& cfg);

  /**
   * @brief Apply customized physics parameters to the ego vehicle actor.
   *
   * @param phys Configuration containing mass, drag, torque curves, and wheel
   * setups.
   */
  void apply_physics(const PhysicsConfig& phys);

  Config cfg_;  ///< Stored configuration used for manual_control subprocess
                ///< arguments.
  std::unique_ptr<carla::client::Client>
      client_;  ///< Unique pointer to the CARLA client instance.
  std::unique_ptr<carla::client::World>
      world_;  ///< Unique pointer to the loaded CARLA world instance.
  carla::SharedPtr<carla::client::Actor>
      actor_;  ///< Shared pointer to the spawned ego vehicle actor.
  carla::rpc::EpisodeSettings
      original_settings_;  ///< Snapshot of CARLA episode settings prior to
                           ///< modification.
  bool running_ =
      true;  ///< Flag indicating if synchronous simulation ticking is active.
  bool sync_mode_ = true;  ///< Flag indicating if synchronous mode was enabled
                           ///< by this client.

  int manual_control_pid_ =
      -1;  ///< PID of manual_control.py subprocess (−1 = not running).
};

}  // namespace carla_telemetry
