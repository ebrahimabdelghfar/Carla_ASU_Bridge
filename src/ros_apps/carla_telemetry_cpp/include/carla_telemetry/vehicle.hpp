#pragma once

// CARLA client lifecycle management and ego vehicle spawning and control.

#include <carla/client/Actor.h>
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/rpc/EpisodeSettings.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace carla_telemetry {

// CARLA client lifecycle and ego vehicle management.
// Direct port of CarlaVehicle from vehicle.py.
class CarlaVehicle {
 public:
  struct Config {
    std::string host = "localhost";
    int port = 2000;
    double timeout = 10.0;
    bool sync_mode = true;
    double fixed_delta = 0.01667;  // s, ~60 FPS
    std::string town;              // map to load; empty = keep current
    int spawn_index = 0;
    bool use_coords = false;  // spawn at spawn_x/y/z instead of spawn_index
    float spawn_x = 0.0f;
    float spawn_y = 0.0f;
    float spawn_z = 0.0f;
    float spawn_roll = 0.0f;
    float spawn_pitch = 0.0f;
    float spawn_yaw = 0.0f;
    std::string blueprint = "vehicle.tesla.model3";
    std::string role_name;
    std::string generation = "2";
    std::string color;                 // e.g. "255,0,0"
    bool open_manual_control = false;  // spawn manual_control.py pygame window
    std::string manual_control_res = "640x360";  // smaller window = less GPU
                                                 // load on world tick
    double manual_control_render_rate =
        10.0;  // Hz; <= 0 renders every tick, capping avoids starving
               // telemetry sensors
    bool use_current_world = false;  // skip LoadWorld, use the currently
                                     // loaded CARLA world
  };

  struct PhysicsConfig {
    bool enabled = false;
    float mass = 1500.0f;  // kg
    float drag = 0.3f;
    bool override_center_of_mass = false;
    float com_x = 0.0f;  // m
    float com_y = 0.0f;  // m
    float com_z = 0.0f;  // m
    float max_rpm = 6000.0f;
    float moi = 1.0f;  // kg*m^2
    float damping_full = 0.15f;
    float damping_zero_eng = 2.0f;
    float damping_zero_dis = 0.35f;
    std::vector<std::pair<float, float>> torque_curve;    // RPM -> torque (Nm)
    std::vector<std::pair<float, float>> steering_curve;  // speed -> angle
                                                          // (deg)
    bool use_sweep = false;

    std::string transmission_type = "automatic";  // "automatic" | "manual"
    float gear_switch_time = 0.5f;                // s
    float clutch_strength = 10.0f;
    float final_ratio = 4.0f;

    struct Gear {
      float ratio;
      float down_ratio;
      float up_ratio;
    };
    std::vector<Gear> forward_gears;

    std::string drive_mode = "AWD";  // "AWD" | "FWD" | "RWD"

    struct Wheel {
      std::string position;  // "FL" | "FR" | "RL" | "RR"
      float tire_friction = 3.5f;
      float damping_rate = 0.25f;
      float max_steer_angle = 70.0f;         // deg
      float radius = 36.0f;                  // cm
      float max_brake_torque = 1500.0f;      // Nm
      float max_handbrake_torque = 3000.0f;  // Nm
    };
    std::vector<Wheel> wheels;  // up to 4
  };

  // Tire friction assigned to non-driven wheels to emulate FWD/RWD on a CARLA
  // vehicle that is physically AWD. Also honoured by runtime friction changes
  // (see CarlaROS2Backend::apply_tire_friction) so commanding a new ground
  // friction does not silently turn an FWD/RWD vehicle back into AWD.
  static constexpr float kNonDrivenTireFriction = 0.1f;

  CarlaVehicle(const Config& cfg, const PhysicsConfig& physics);
  ~CarlaVehicle();

  carla::client::World& world() { return *world_; }
  carla::client::Client& client() { return *client_; }
  carla::SharedPtr<carla::client::Actor> actor() { return actor_; }

  void tick();
  void pause();
  void resume();
  bool is_running() const { return running_; }

  void spawn_manual_control();

  // Terminate the manual_control.py subprocess (SIGTERM, escalate to
  // SIGKILL, then reap). Idempotent — safe to call before destroy() so the
  // pygame window closes promptly even if later CARLA RPCs block.
  void stop_manual_control();

  void destroy();

 private:
  void connect(const Config& cfg);
  void apply_physics(const PhysicsConfig& phys);

  Config cfg_;  // kept for manual_control subprocess arguments
  std::unique_ptr<carla::client::Client> client_;
  std::unique_ptr<carla::client::World> world_;
  carla::SharedPtr<carla::client::Actor> actor_;
  carla::rpc::EpisodeSettings
      original_settings_;  // snapshot before modification
  bool running_ = true;
  bool sync_mode_ = true;  // whether this client enabled sync mode

  int manual_control_pid_ = -1;  // -1 = not running
};

}  // namespace carla_telemetry
