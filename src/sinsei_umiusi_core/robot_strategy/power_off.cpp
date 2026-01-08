#include "sinsei_umiusi_core/robot_strategy/power_off.hpp"

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

using namespace std::placeholders;

sinsei_umiusi_core::robot_strategy::PowerOff::PowerOff(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  ros_node{nullptr},
  main_power_output_pub{nullptr},
  main_power_disabled_msg{sinsei_umiusi_msgs::msg::MainPowerOutput{}.set__enabled(false)},
  robot_state_power_off_msg{sinsei_umiusi_msgs::msg::RobotState{}.set__state(
    sinsei_umiusi_msgs::msg::RobotState::POWER_OFF)}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_power_off");
    this->main_power_output_pub =
      this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::MainPowerOutput>(
        "/cmd/main_power_output", rclcpp::SystemDefaultsQoS{});
    this->robot_state_pub = this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::RobotState>(
      "/robot_state", rclcpp::SystemDefaultsQoS{});
}

auto sinsei_umiusi_core::robot_strategy::PowerOff::tick() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);

    this->main_power_output_pub->publish(this->main_power_disabled_msg);
    this->robot_state_pub->publish(this->robot_state_power_off_msg);
    return BT::NodeStatus::SUCCESS;
}

auto sinsei_umiusi_core::robot_strategy::PowerOff::halt() -> void {}
