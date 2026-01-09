#include "sinsei_umiusi_core/robot_strategy/power_on.hpp"

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

using namespace std::placeholders;

sinsei_umiusi_core::robot_strategy::PowerOn::PowerOn(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  ros_node{nullptr},
  main_power_output_pub{nullptr},
  main_power_enabled_msg{sinsei_umiusi_msgs::msg::MainPowerOutput{}.set__enabled(true)}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_power_on");
    this->main_power_output_pub =
      this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::MainPowerOutput>(
        "/cmd/main_power_output", rclcpp::SystemDefaultsQoS{});
}

auto sinsei_umiusi_core::robot_strategy::PowerOn::tick() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);

    this->main_power_output_pub->publish(this->main_power_enabled_msg);
    return BT::NodeStatus::SUCCESS;
}

auto sinsei_umiusi_core::robot_strategy::PowerOn::halt() -> void {}
