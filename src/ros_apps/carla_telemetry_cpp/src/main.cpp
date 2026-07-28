#include <chrono>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "carla_telemetry/node.hpp"

int main(int argc, char* argv[]) {
  // Default SignalHandlerOptions::All installs BOTH SIGINT and SIGTERM
  // handlers, so `kill`/`pkill` (SIGTERM) also requests a graceful shutdown.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<carla_telemetry::CarlaTelemetryNode>();
  // Multi-threaded so the control timer callback cannot block subscriptions
  // and services (and vice versa) on a single executor thread.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  // Bounded shutdown watchdog. Tearing the node down destroys the dedicated
  // CARLA sensor clients, which issue blocking RPCs (Stop/Destroy) that can
  // hang if the server is unresponsive or gone. If teardown overruns, force
  // exit so this process never lingers — a live-but-orphaned bridge keeps its
  // sensor clients reconnecting and floods the next CARLA server with
  // "Invalid session: no stream available".
  std::thread([] {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cerr << "[carla_telemetry] shutdown exceeded 5s; forcing exit to "
                 "avoid an orphaned bridge."
              << std::endl;
    std::_Exit(EXIT_SUCCESS);
  }).detach();

  node.reset();  // runs ~CarlaTelemetryNode() -> shutdown()
  rclcpp::shutdown();
  return 0;
}
