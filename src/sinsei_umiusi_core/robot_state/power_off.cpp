#include "sinsei_umiusi_core/robot_state/power_off.hpp"

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "sinsei_umiusi_msgs/msg/health_check_result.hpp"
#include "sinsei_umiusi_msgs/srv/power_on.hpp"

using namespace std::placeholders;

sinsei_umiusi_core::robot_state::PowerOff::PowerOff(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  ros_node(nullptr),
  low_power_health_check_result_sub(nullptr),
  high_power_health_check_result_sub(nullptr),
  power_on_service(nullptr),
  low_power_circuit_is_ok(false),
  high_power_circuit_is_ok(false),
  is_ready_to_power_on(false)
{
    this->ros_node = rclcpp::Node::make_shared("_bt_power_off");
    this->low_power_health_check_result_sub =
      this->ros_node->create_subscription<sinsei_umiusi_msgs::msg::HealthCheckResult>(
        "/low_power_health_check_result", rclcpp::SystemDefaultsQoS{},
        [this](const sinsei_umiusi_msgs::msg::HealthCheckResult::SharedPtr msg) {
            this->low_power_circuit_is_ok = msg->is_ok;
        });
    this->high_power_health_check_result_sub =
      this->ros_node->create_subscription<sinsei_umiusi_msgs::msg::HealthCheckResult>(
        "/high_power_health_check_result", rclcpp::SystemDefaultsQoS{},
        [this](const sinsei_umiusi_msgs::msg::HealthCheckResult::SharedPtr msg) {
            this->high_power_circuit_is_ok = msg->is_ok;
        });
    this->power_on_service = this->ros_node->create_service<sinsei_umiusi_msgs::srv::PowerOn>(
      "/user_input/power_on",
      [this](
        const sinsei_umiusi_msgs::srv::PowerOn::Request::SharedPtr /* request */,
        sinsei_umiusi_msgs::srv::PowerOn::Response::SharedPtr response) {
          if (!this->low_power_circuit_is_ok) {
              response->set__success(false);
              response->set__error_msg("Low power circuit is not OK");
              return;
          }
          if (!this->high_power_circuit_is_ok) {
              response->set__success(false);
              response->set__error_msg("High power circuit is not OK");
              return;
          }
          this->is_ready_to_power_on = true;
          response->set__success(true);
          return;
      });
}

auto sinsei_umiusi_core::robot_state::PowerOff::onStart() -> BT::NodeStatus
{
    this->is_ready_to_power_on = false;
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_state::PowerOff::onRunning() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);
    if (this->is_ready_to_power_on) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_state::PowerOff::onHalted() -> void {}
