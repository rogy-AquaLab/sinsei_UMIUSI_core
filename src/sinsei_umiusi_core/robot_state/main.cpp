#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_core/robot_state/core.hpp"

auto main(int argc, char ** argv) -> int
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sinsei_umiusi_core::robot_state::Core>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
