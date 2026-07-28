#include "carla_micropilot_interface/carla_micropilot_interface_node.hpp"

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarlaMicropilotInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}