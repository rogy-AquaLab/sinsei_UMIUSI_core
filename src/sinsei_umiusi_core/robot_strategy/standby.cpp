#include "sinsei_umiusi_core/robot_strategy/standby.hpp"

sinsei_umiusi_core::robot_strategy::Standby::Standby(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode{name, config},
  ros_node{nullptr},
  robot_state_standby_msg{
    sinsei_umiusi_msgs::msg::RobotState{}.set__state(sinsei_umiusi_msgs::msg::RobotState::STANDBY)}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_standby");
    this->robot_state_pub = this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::RobotState>(
      "/robot_state", rclcpp::SystemDefaultsQoS{});
}

auto sinsei_umiusi_core::robot_strategy::Standby::onStart() -> BT::NodeStatus
{
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_strategy::Standby::onRunning() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);

    this->robot_state_pub->publish(this->robot_state_standby_msg);
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_strategy::Standby::onHalted() -> void {}
