#pragma once

#include <carla/client/Actor.h>
#include <carla/client/ActorList.h>
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/rpc/ActorId.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "carla_telemetry/sensors/camera.hpp"
#include "carla_telemetry/sensors/depth_lidar.hpp"
#include "carla_telemetry/sensors/lidar.hpp"

namespace carla_telemetry {

class CarlaROS2Backend;  // forward

/// Owns a dedicated CARLA client connection that captures a single sensor.
/// Used when carla.dedicated_clients.enabled is true to parallelize sensor
/// streaming across multiple TCP connections (one client per sensor).
template <class TSensor>
class SensorClient {
 public:
  using Config = typename TSensor::Config;

  /**
   * @brief Construct a new SensorClient object
   *
   * @param host The host of the CARLA server
   * @param port The port of the CARLA server
   * @param timeout The timeout for the CARLA server
   * @param vehicle_actor_id The actor ID of the ego vehicle
   * @param role_name The role name of the ego vehicle
   * @param cfg The configuration for the sensor
   */
  SensorClient(const std::string& host, int port, double timeout,
               carla::rpc::ActorId vehicle_actor_id,
               const std::string& role_name, const Config& cfg)
      : name_(cfg.name) {
    try {
      client_ = std::make_unique<carla::client::Client>(
          host, static_cast<uint16_t>(port));
      client_->SetTimeout(std::chrono::duration<double>(timeout));

      // Connect to the SAME episode the main client already loaded.
      // Do NOT touch world settings — main client owns sync mode + tick.
      world_ = std::make_unique<carla::client::World>(client_->GetWorld());

      vehicle_ = resolve_vehicle(vehicle_actor_id, role_name);
      if (!vehicle_) {
        std::cerr << "[SensorClient] '" << name_
                  << "' could not resolve ego vehicle (id=" << vehicle_actor_id
                  << ", role=" << role_name << "); skipping this client."
                  << std::endl;
        return;
      }

      // TSensor spawns the sensor in THIS client's world, attached to
      // the resolved vehicle, and streams over THIS client's connection.
      sensor_ = std::make_unique<TSensor>(*world_, vehicle_, cfg);
      ok_ = true;
      std::cerr << "[SensorClient] '" << name_
                << "' connected on dedicated client." << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "[SensorClient] '" << name_ << "' init failed: " << e.what()
                << std::endl;
      ok_ = false;
    } catch (...) {
      std::cerr << "[SensorClient] '" << name_
                << "' init failed: unknown exception" << std::endl;
      ok_ = false;
    }
  }

  /**
   * @brief Start the sensor.
   *
   * @param backend The backend to use for publishing.
   */
  void start(CarlaROS2Backend* backend) {
    if (ok_ && sensor_) sensor_->start(backend);
  }

  /**
   * @brief Stop the sensor.
   */
  void stop() {
    if (sensor_) sensor_->stop();
  }

  /**
   * @brief Destroy the sensor.
   */
  void destroy() {
    if (sensor_) {
      sensor_->destroy();
      sensor_.reset();
    }
    vehicle_ = nullptr;
    world_.reset();
    client_.reset();
    ok_ = false;
  }

  /**
   * @brief Check if the sensor is OK.
   *
   * @return true if the sensor is OK
   * @return false if the sensor is not OK
   */
  bool ok() const { return ok_; }

  /**
   * @brief Get the name of the sensor.
   *
   * @return const std::string& The name of the sensor
   */
  const std::string& name() const { return name_; }

 private:
  /**
   * @brief Resolve the vehicle actor from the actor id and role name.
   *
   * @param id The actor id of the ego vehicle
   * @param role_name The role name of the ego vehicle
   * @return carla::SharedPtr<carla::client::Actor> The ego vehicle actor
   */
  carla::SharedPtr<carla::client::Actor> resolve_vehicle(
      carla::rpc::ActorId id, const std::string& role_name) {
    // The main client spawns the vehicle and ticks once before sensor setup,
    // so the actor is normally visible immediately. Retry briefly for safety.
    constexpr int kMaxRetries = 20;  // ~2s total
    const auto kSleep = std::chrono::milliseconds(100);

    for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
      try {
        // Preferred: unambiguous lookup by actor id.
        auto actor = world_->GetActor(id);
        if (actor) return actor;

        // Fallback: scan by role_name attribute.
        if (!role_name.empty()) {
          auto actors = world_->GetActors();
          for (auto a : *actors) {
            for (const auto& attr : a->GetAttributes()) {
              if (attr.GetId() == "role_name" && attr.GetValue() == role_name) {
                return a;
              }
            }
          }
        }
      } catch (const std::exception&) {
        // transient during world reload — keep retrying
      }
      std::this_thread::sleep_for(kSleep);
    }
    return nullptr;
  }

  std::string name_;
  bool ok_ = false;

  std::unique_ptr<carla::client::Client> client_;
  std::unique_ptr<carla::client::World> world_;
  carla::SharedPtr<carla::client::Actor> vehicle_;
  std::unique_ptr<TSensor> sensor_;
};

using CameraClient = SensorClient<CarlaCamera>;
using LidarClient = SensorClient<CarlaLidar>;
using DepthLidarClient = SensorClient<CarlaDepthLidar>;

}  // namespace carla_telemetry
