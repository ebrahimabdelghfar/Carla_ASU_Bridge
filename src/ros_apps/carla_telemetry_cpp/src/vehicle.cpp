#include "carla_telemetry/vehicle.hpp"

#include <carla/actors/BlueprintLibrary.h>
#include <carla/client/ActorList.h>
#include <carla/geom/Transform.h>
#include <carla/rpc/GearPhysicsControl.h>
#include <carla/rpc/VehicleControl.h>
#include <carla/rpc/VehiclePhysicsControl.h>
#include <carla/rpc/WheelPhysicsControl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "carla_telemetry/perf_monitor.hpp"

namespace carla_telemetry {

CarlaVehicle::CarlaVehicle(const Config& cfg, const PhysicsConfig& physics)
    : cfg_(cfg) {
  connect(cfg);

  // Spawn ego vehicle
  auto bp_lib = world_->GetBlueprintLibrary();
  auto filtered = bp_lib->Filter(cfg.blueprint);
  if (filtered->empty()) {
    throw std::runtime_error("[CarlaVehicle] No blueprint matches: " +
                             cfg.blueprint);
  }

  // Filter by generation (read-only attribute, cannot SetAttribute)
  auto bp = (*filtered)[0];
  if (!cfg.generation.empty() && cfg.generation != "All") {
    bool found = false;
    for (size_t i = 0; i < filtered->size(); ++i) {
      auto& candidate = (*filtered)[i];
      if (candidate.ContainsAttribute("generation") &&
          candidate.GetAttribute("generation").GetValue() == cfg.generation) {
        bp = candidate;
        found = true;
        break;
      }
    }
    if (!found) {
      std::cerr << "[CarlaVehicle] Warning: no blueprint with generation="
                << cfg.generation << ", using first match." << std::endl;
    }
  }

  // Set role_name (default to "hero" for manual_control attach)
  if (cfg.role_name.empty()) {
    cfg_.role_name = "hero";
  }
  if (bp.ContainsAttribute("role_name")) {
    bp.SetAttribute("role_name", cfg_.role_name);
  }

  // Set color
  if (bp.ContainsAttribute("color") && !cfg.color.empty()) {
    bp.SetAttribute("color", cfg.color);
  }

  // Pick spawn point
  carla::geom::Transform spawn_transform;
  if (cfg.use_coords) {
    spawn_transform.location =
        carla::geom::Location(cfg.spawn_x, cfg.spawn_y, cfg.spawn_z);
    spawn_transform.rotation =
        carla::geom::Rotation(cfg.spawn_pitch, cfg.spawn_yaw, cfg.spawn_roll);
  } else {
    auto spawn_points = world_->GetMap()->GetRecommendedSpawnPoints();
    if (spawn_points.empty()) {
      throw std::runtime_error("[CarlaVehicle] No spawn points on map.");
    }

    size_t idx = 0;
    if (cfg.spawn_index >= 0 &&
        static_cast<size_t>(cfg.spawn_index) < spawn_points.size()) {
      idx = static_cast<size_t>(cfg.spawn_index);
    }
    spawn_transform = spawn_points[idx];
  }

  actor_ = world_->SpawnActor(bp, spawn_transform);
  if (!actor_) {
    throw std::runtime_error("[CarlaVehicle] Failed to spawn vehicle.");
  }

  if (cfg.use_coords) {
    std::cerr << "[CarlaVehicle] Spawned " << cfg.blueprint
              << " at coordinates (" << cfg.spawn_x << ", " << cfg.spawn_y
              << ", " << cfg.spawn_z << ")" << std::endl;
  } else {
    std::cerr << "[CarlaVehicle] Spawned " << cfg.blueprint << " at index "
              << cfg.spawn_index << std::endl;
  }

  // Apply physics if configured
  if (physics.enabled) {
    apply_physics(physics);
  }

  // In synchronous mode, tick the world once so the newly spawned actor
  // becomes visible to other CARLA clients (e.g. manual_control.py --attach).
  if (sync_mode_ && world_) {
    world_->Tick(std::chrono::seconds(10));
  }

  // manual_control subprocess
  if (cfg.open_manual_control) {
    spawn_manual_control();
  }
}

CarlaVehicle::~CarlaVehicle() { destroy(); }

void CarlaVehicle::connect(const Config& cfg) {
  client_ = std::make_unique<carla::client::Client>(
      cfg.host, static_cast<uint16_t>(cfg.port));
  client_->SetTimeout(std::chrono::duration<double>(cfg.timeout));

  // Always use the currently loaded world.
  // When ScenarioRunner reloads the world for a scenario (e.g. Town04),
  // the bridge must connect to that world — not reload back to the
  // config's town (e.g. Aramco_Map).  The config town field is only
  // used on fresh startup when no external tool has loaded a world.

  // Retry loop: after load_world() the CARLA server may still be
  // loading the new map, causing transient TimeoutExceptions.
  constexpr int kMaxRetries = 30;
  constexpr int kRetrySleepSec = 2;
  std::string current_map;
  std::optional<carla::client::World> current_world;

  for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
    try {
      current_world.emplace(client_->GetWorld());
      current_map = current_world->GetMap()->GetName();
      break;  // success
    } catch (const std::exception& e) {
      std::cerr << "[CarlaVehicle] Waiting for CARLA server (attempt "
                << attempt << "/" << kMaxRetries << "): " << e.what()
                << std::endl;
      if (attempt == kMaxRetries) {
        throw;  // give up
      }
      std::this_thread::sleep_for(std::chrono::seconds(kRetrySleepSec));
    }
  }

  if (cfg.use_current_world) {
    // Reconfigure path: ScenarioRunner already loaded the desired world.
    // Do NOT call LoadWorld — just use whatever is currently loaded.
    std::cerr << "[CarlaVehicle] Using current world (reconfigure): "
              << current_map << std::endl;
    world_ = std::make_unique<carla::client::World>(*current_world);
  } else if (!cfg.town.empty() &&
             current_map.find(cfg.town) == std::string::npos) {
    // First configure: config says a different map — load it.
    std::cerr << "[CarlaVehicle] Loading world: " << cfg.town << std::endl;
    world_ =
        std::make_unique<carla::client::World>(client_->LoadWorld(cfg.town));
  } else {
    std::cerr << "[CarlaVehicle] Connected to world: " << current_map
              << std::endl;
    world_ = std::make_unique<carla::client::World>(*current_world);
  }

  // Configure synchronous mode
  original_settings_ = world_->GetSettings();
  sync_mode_ = cfg.sync_mode;

  // Apply fixed_delta_seconds in BOTH modes. In async this bounds each physics
  // step ("async fixed step") so Traffic Manager NPCs get a deterministic dt
  // instead of the variable server dt that makes them drive erratically.
  auto settings = world_->GetSettings();
  settings.synchronous_mode = cfg.sync_mode;
  settings.fixed_delta_seconds = cfg.fixed_delta;
  world_->ApplySettings(settings, carla::time_duration::seconds(10));
}

void CarlaVehicle::apply_physics(const PhysicsConfig& phys) {
  auto vehicle = boost::dynamic_pointer_cast<carla::client::Vehicle>(actor_);
  if (!vehicle) return;

  auto pc = vehicle->GetPhysicsControl();

  pc.mass = phys.mass;
  pc.drag_coefficient = phys.drag;

  // Center of mass
  if (phys.override_center_of_mass) {
    pc.center_of_mass =
        carla::geom::Location(phys.com_x, phys.com_y, phys.com_z);
  }

  pc.max_rpm = phys.max_rpm;
  pc.moi = phys.moi;
  pc.damping_rate_full_throttle = phys.damping_full;
  pc.damping_rate_zero_throttle_clutch_engaged = phys.damping_zero_eng;
  pc.damping_rate_zero_throttle_clutch_disengaged = phys.damping_zero_dis;

  // Torque curve
  if (!phys.torque_curve.empty()) {
    std::vector<carla::geom::Vector2D> tc;
    for (auto& [rpm, nm] : phys.torque_curve) {
      tc.emplace_back(rpm, nm);
    }
    pc.torque_curve = tc;
  }

  // Steering curve
  if (!phys.steering_curve.empty()) {
    std::vector<carla::geom::Vector2D> sc;
    for (auto& [spd, frac] : phys.steering_curve) {
      sc.emplace_back(spd, frac);
    }
    pc.steering_curve = sc;
  }

  pc.use_sweep_wheel_collision = phys.use_sweep;

  // Transmission
  pc.use_gear_autobox = (phys.transmission_type == "automatic");
  pc.gear_switch_time = phys.gear_switch_time;
  pc.clutch_strength = phys.clutch_strength;
  pc.final_ratio = phys.final_ratio;

  // Forward gears
  if (!phys.forward_gears.empty()) {
    std::vector<carla::rpc::GearPhysicsControl> gears;
    for (auto& g : phys.forward_gears) {
      carla::rpc::GearPhysicsControl gpc;
      gpc.ratio = g.ratio;
      gpc.down_ratio = g.down_ratio;
      gpc.up_ratio = g.up_ratio;
      gears.push_back(gpc);
    }
    pc.SetForwardGears(gears);
  }

  // Wheels
  auto wheels = pc.GetWheels();
  for (size_t i = 0; i < std::min(phys.wheels.size(), wheels.size()); ++i) {
    auto& pw = phys.wheels[i];
    wheels[i].tire_friction = pw.tire_friction;
    wheels[i].damping_rate = pw.damping_rate;
    wheels[i].max_steer_angle = pw.max_steer_angle;
    wheels[i].radius = pw.radius;
    wheels[i].max_brake_torque = pw.max_brake_torque;
    wheels[i].max_handbrake_torque = pw.max_handbrake_torque;
  }
  pc.SetWheels(wheels);

  // Drive mode: adjust tire friction for FWD/RWD
  if (phys.drive_mode == "FWD" && wheels.size() >= 4) {
    wheels[2].tire_friction = kNonDrivenTireFriction;
    wheels[3].tire_friction = kNonDrivenTireFriction;
    pc.SetWheels(wheels);
  } else if (phys.drive_mode == "RWD" && wheels.size() >= 4) {
    wheels[0].tire_friction = kNonDrivenTireFriction;
    wheels[1].tire_friction = kNonDrivenTireFriction;
    pc.SetWheels(wheels);
  }

  vehicle->ApplyPhysicsControl(pc);
  std::cerr << "[CarlaVehicle] Physics applied. mass=" << phys.mass
            << " max_rpm=" << phys.max_rpm << " drive=" << phys.drive_mode
            << std::endl;
}

void CarlaVehicle::tick() {
  if (sync_mode_ && running_ && world_) {
    double t_rpc = PerfMonitor::tick();
    world_->Tick(std::chrono::seconds(10));
    perf.record("rpc.world_tick", t_rpc);
  }
}

void CarlaVehicle::pause() { running_ = false; }
void CarlaVehicle::resume() { running_ = true; }

void CarlaVehicle::spawn_manual_control() {
  pid_t pid = fork();
  if (pid == 0) {
    // Child process — run as installed Python module with attach args
    std::string port_str = std::to_string(cfg_.port);
    std::string rate_str = std::to_string(cfg_.manual_control_render_rate);
    execlp("python3", "python3", "-m", "carla_telemetry.manual_control",
           "--host", cfg_.host.c_str(), "-p", port_str.c_str(), "--rolename",
           cfg_.role_name.c_str(), "--filter", cfg_.blueprint.c_str(),
           "--generation", cfg_.generation.c_str(), "--res",
           cfg_.manual_control_res.c_str(), "--render-rate", rate_str.c_str(),
           "--attach", nullptr);
    _exit(1);  // exec failed
  } else if (pid > 0) {
    manual_control_pid_ = pid;
    std::cerr << "[CarlaVehicle] manual_control.py spawned (PID=" << pid
              << ") --attach --rolename " << cfg_.role_name << std::endl;
  }
}

void CarlaVehicle::stop_manual_control() {
  if (manual_control_pid_ <= 0) return;

  pid_t pid = manual_control_pid_;
  kill(pid, SIGTERM);

  // Wait up to ~2s for a graceful exit. The Python SIGTERM handler can only
  // run once it returns from any in-flight CARLA RPC, so WNOHANG alone would
  // leave the window open (and the child a zombie). Poll, then SIGKILL.
  int status;
  for (int i = 0; i < 200; ++i) {
    if (waitpid(pid, &status, WNOHANG) == pid) {
      manual_control_pid_ = -1;
      std::cerr << "[CarlaVehicle] manual_control terminated (PID=" << pid
                << ")." << std::endl;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Still alive -> force kill and reap (blocking) so no zombie / lingering
  // window.
  kill(pid, SIGKILL);
  waitpid(pid, &status, 0);
  manual_control_pid_ = -1;
  std::cerr << "[CarlaVehicle] manual_control force-killed (PID=" << pid << ")."
            << std::endl;
}

void CarlaVehicle::destroy() {
  // Kill manual_control subprocess
  stop_manual_control();

  // Destroy CARLA actor
  if (actor_) {
    try {
      actor_->Destroy();
    } catch (...) {
    }
    actor_ = nullptr;
  }

  // Restore original settings
  if (world_) {
    try {
      world_->ApplySettings(original_settings_,
                            carla::time_duration::seconds(10));
    } catch (...) {
    }
  }

  std::cerr << "[CarlaVehicle] Destroyed." << std::endl;
}

}  // namespace carla_telemetry
