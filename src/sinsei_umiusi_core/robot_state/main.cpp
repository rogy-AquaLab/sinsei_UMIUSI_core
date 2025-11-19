#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_core/robot_state/robot_state.hpp"

auto main(int argc, char ** argv) -> int
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sinsei_umiusi_core::robot_state::RobotState>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
