#include "sinsei_umiusi_core/robot_state/standby.hpp"

#include "sinsei_umiusi_msgs/msg/thruster_enabled.hpp"

sinsei_umiusi_core::robot_state::Standby::Standby(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  ros_node(nullptr),
  power_off_srv(nullptr),
  main_power_output_pub(nullptr),
  thruster_enabled_all_pub(nullptr),
  power_off_requested(false)
{
    this->ros_node = rclcpp::Node::make_shared("_bt_standby");

    this->power_off_srv = this->ros_node->create_service<sinsei_umiusi_msgs::srv::PowerOff>(
      "/user_input/power_off",
      [this](
        const sinsei_umiusi_msgs::srv::PowerOff::Request::SharedPtr /* request */,
        sinsei_umiusi_msgs::srv::PowerOff::Response::SharedPtr response) {
          response->set__success(true);
          this->power_off_requested = true;
          return;
      });

    this->main_power_output_pub =
      this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::MainPowerOutput>(
        "/cmd/main_power_output", rclcpp::SystemDefaultsQoS{});

    this->thruster_enabled_all_pub =
      this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::ThrusterEnabledAll>(
        "/cmd/thruster_enabled_all", rclcpp::SystemDefaultsQoS{});
}

auto sinsei_umiusi_core::robot_state::Standby::onStart() -> BT::NodeStatus
{
    this->power_off_requested = false;
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_state::Standby::onRunning() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);
    if (this->power_off_requested) {
        return BT::NodeStatus::FAILURE;
    }
    this->main_power_output_pub->publish(
      sinsei_umiusi_msgs::msg::MainPowerOutput().set__enabled(true));
    this->thruster_enabled_all_pub->publish(
      sinsei_umiusi_msgs::msg::ThrusterEnabledAll{}
        .set__lf(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false))
        .set__lb(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false))
        .set__rb(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false))
        .set__rf(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false)));
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_state::Standby::onHalted() -> void {}
