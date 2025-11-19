#include "sinsei_umiusi_core/robot_state/power_off.hpp"

#include <rclcpp/rclcpp.hpp>

sinsei_umiusi_core::robot_state::PowerOff::PowerOff(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), ros_node(nullptr)
{
    this->ros_node = rclcpp::Node::make_unique("robot_state__poweroff");
}

auto sinsei_umiusi_core::robot_state::PowerOff::onStart() -> BT::NodeStatus
{
    RCLCPP_INFO(this->ros_node->get_logger(), "Robot is powering off");
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_state::PowerOff::onRunning() -> BT::NodeStatus
{
    RCLCPP_INFO(this->ros_node->get_logger(), "Power off sequence complete");
    return BT::NodeStatus::SUCCESS;
}

auto sinsei_umiusi_core::robot_state::PowerOff::onHalted() -> void
{
    RCLCPP_WARN(this->ros_node->get_logger(), "Power off action has been halted");
}
