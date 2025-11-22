#include "sinsei_umiusi_core/robot_strategy/power_off.hpp"

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "sinsei_umiusi_msgs/msg/thruster_enabled_all.hpp"

using namespace std::placeholders;

sinsei_umiusi_core::robot_strategy::PowerOff::PowerOff(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  ros_node{nullptr},
  main_power_output_pub{nullptr},
  thruster_enabled_pub{nullptr},
  main_power_disabled_msg{sinsei_umiusi_msgs::msg::MainPowerOutput{}.set__enabled(false)},
  thrusters_disabled_msg{
    sinsei_umiusi_msgs::msg::ThrusterEnabledAll{}
      .set__lf(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false))
      .set__lb(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false))
      .set__rf(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false))
      .set__rb(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false))},
  robot_state_power_off_msg{sinsei_umiusi_msgs::msg::RobotState{}.set__state(
    sinsei_umiusi_msgs::msg::RobotState::POWER_OFF)}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_power_off");
    this->main_power_output_pub =
      this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::MainPowerOutput>(
        "/cmd/main_power_output", rclcpp::SystemDefaultsQoS{});
    this->thruster_enabled_pub =
      this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::ThrusterEnabledAll>(
        "/cmd/thruster_enabled_all", rclcpp::SystemDefaultsQoS{});
    this->robot_state_pub = this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::RobotState>(
      "/robot_state", rclcpp::SystemDefaultsQoS{});
}

auto sinsei_umiusi_core::robot_strategy::PowerOff::onStart() -> BT::NodeStatus
{
    this->main_power_output_pub->publish(this->main_power_disabled_msg);
    this->thruster_enabled_pub->publish(this->thrusters_disabled_msg);
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_strategy::PowerOff::onRunning() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);

    this->main_power_output_pub->publish(this->main_power_disabled_msg);
    this->thruster_enabled_pub->publish(this->thrusters_disabled_msg);
    this->robot_state_pub->publish(this->robot_state_power_off_msg);
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_strategy::PowerOff::onHalted() -> void
{
    this->main_power_output_pub->publish(
      sinsei_umiusi_msgs::msg::MainPowerOutput{}.set__enabled(true));
    this->thruster_enabled_pub->publish(
      sinsei_umiusi_msgs::msg::ThrusterEnabledAll{}
        .set__lf(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(true).set__servo(true))
        .set__lb(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(true).set__servo(true))
        .set__rf(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(true).set__servo(true))
        .set__rb(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(true).set__servo(true)));
}
