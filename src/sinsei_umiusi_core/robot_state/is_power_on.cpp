#include "sinsei_umiusi_core/robot_state/is_power_on.hpp"

sinsei_umiusi_core::robot_state::IsPowerOn::IsPowerOn(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config),
  ros_node(nullptr),
  power_off_srv(nullptr),
  power_on_srv(nullptr),
  is_power_on(false)
{
    this->ros_node = rclcpp::Node::make_shared("_bt_is_power_on");
    this->power_off_srv = this->ros_node->create_service<sinsei_umiusi_msgs::srv::PowerOff>(
      "/user_input/power_off",
      [this](
        const sinsei_umiusi_msgs::srv::PowerOff::Request::SharedPtr /* request */,
        sinsei_umiusi_msgs::srv::PowerOff::Response::SharedPtr response) {
          this->is_power_on = false;
          response->set__success(true);
          return;
      });
    this->power_on_srv = this->ros_node->create_service<sinsei_umiusi_msgs::srv::PowerOn>(
      "/user_input/power_on",
      [this](
        const sinsei_umiusi_msgs::srv::PowerOn::Request::SharedPtr /* request */,
        sinsei_umiusi_msgs::srv::PowerOn::Response::SharedPtr response) {
          this->is_power_on = true;
          response->set__success(true);
          return;
      });
}

auto sinsei_umiusi_core::robot_state::IsPowerOn::tick() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);
    return this->is_power_on ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
