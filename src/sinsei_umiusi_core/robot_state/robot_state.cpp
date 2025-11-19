#include "sinsei_umiusi_core/robot_state/robot_state.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <functional>
#include <memory>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using namespace std::chrono_literals;

sinsei_umiusi_core::robot_state::RobotState::RobotState()
: rclcpp::Node("robot_state_node"), timer(nullptr), tree(nullptr)
{
    RCLCPP_INFO(this->get_logger(), "RobotState node has been started.");

    this->declare_parameter(
      std::string(PARAM_NAME_BEHAVIOR_TREE_FILE), "unknown",
      rcl_interfaces::msg::ParameterDescriptor()
        .set__description("BehaviorTree file (*.xml)")
        .set__read_only(true)
        .set__type(rclcpp::ParameterType::PARAMETER_STRING));

    this->timer =
      this->create_wall_timer(1s, std::bind(&RobotState::timer_callback, this), nullptr, false);

    auto factory = BT::BehaviorTreeFactory();
    // factory.registerNodeType<YourCustomNode>("YourCustomNode");
    auto tree = factory.createTreeFromFile(
      this->get_parameter(std::string(PARAM_NAME_BEHAVIOR_TREE_FILE)).as_string());
    this->tree = std::make_unique<BT::Tree>(std::move(tree));

    this->timer->reset();
}

auto sinsei_umiusi_core::robot_state::RobotState::timer_callback() const -> void
{
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
    if (!tree) {
        RCLCPP_WARN(this->get_logger(), "Behavior tree is not initialized.");
        return;
    }
    BT::NodeStatus status = tree->rootNode()->executeTick();
    RCLCPP_INFO(
      this->get_logger(), "Behavior tree tick executed with status: %s", BT::toStr(status).c_str());
}
