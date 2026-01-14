#include "sinsei_umiusi_core/robot_strategy/should_power_on.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

sinsei_umiusi_core::robot_strategy::ShouldPowerOn::ShouldPowerOn(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode{name, config},
  ros_node{nullptr},
  power_off_srv{nullptr},
  power_on_srv{nullptr},
  low_power_health_check_sub{nullptr},
  high_power_health_check_sub{nullptr},
  client_count_sub{nullptr},
  low_power_circuit_is_ok{false},
  high_power_circuit_is_ok{false},
  clients_num{0},
  should_power_on{false}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_is_power_on");
    this->low_power_health_check_sub =
      this->ros_node->create_subscription<sinsei_umiusi_msgs::msg::HealthCheckResult>(
        "/low_power_health_check_result", rclcpp::SystemDefaultsQoS{},
        [this](const sinsei_umiusi_msgs::msg::HealthCheckResult::SharedPtr msg) {
            this->low_power_circuit_is_ok = msg->is_ok;
        });
    this->high_power_health_check_sub =
      this->ros_node->create_subscription<sinsei_umiusi_msgs::msg::HealthCheckResult>(
        "/high_power_health_check_result", rclcpp::SystemDefaultsQoS{},
        [this](const sinsei_umiusi_msgs::msg::HealthCheckResult::SharedPtr msg) {
            this->high_power_circuit_is_ok = msg->is_ok;
        });
    this->client_count_sub = this->ros_node->create_subscription<std_msgs::msg::Int32>(
      "/client_count", rclcpp::SystemDefaultsQoS{},
      [this](const std_msgs::msg::Int32::SharedPtr msg) { this->clients_num = msg->data; });
    this->power_off_srv = this->ros_node->create_service<sinsei_umiusi_msgs::srv::PowerOff>(
      "/user_input/power_off",
      [this](
        const sinsei_umiusi_msgs::srv::PowerOff::Request::SharedPtr /* request */,
        sinsei_umiusi_msgs::srv::PowerOff::Response::SharedPtr response) {
          this->should_power_on = false;
          response->set__success(true);
          return;
      });
    this->power_on_srv = this->ros_node->create_service<sinsei_umiusi_msgs::srv::PowerOn>(
      "/user_input/power_on",
      [this](
        const sinsei_umiusi_msgs::srv::PowerOn::Request::SharedPtr /* request */,
        sinsei_umiusi_msgs::srv::PowerOn::Response::SharedPtr response) {
          if (!this->low_power_circuit_is_ok) {
              response->set__success(false);
              response->set__error_msg("There is something wrong in low power circuit.");
              return;
          }
          if (!this->high_power_circuit_is_ok) {
              response->set__success(false);
              response->set__error_msg("There is something wrong in high power circuit.");
              return;
          }
          if (this->clients_num <= 0) {
              response->set__success(false);
              response->set__error_msg("No clients connected to rosbridge server.");
              return;
          }
          response->set__success(true);
          this->should_power_on = true;
          return;
      });
}

auto sinsei_umiusi_core::robot_strategy::ShouldPowerOn::tick() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);

    if (this->clients_num <= 0) {
        this->should_power_on = false;
        RCLCPP_WARN_THROTTLE(
          this->ros_node->get_logger(), *this->ros_node->get_clock(), 3000,
          "No clients connected to rosbridge server.");
    }

    return this->should_power_on ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
