#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include "sinsei_umiusi_msgs/msg/health_check_result.hpp"
#include "sinsei_umiusi_msgs/msg/robot_state.hpp"
#include "sinsei_umiusi_msgs/srv/power_on.hpp"

namespace sinsei_umiusi_core::robot_state
{

class PowerOff : public BT::StatefulActionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Subscription<sinsei_umiusi_msgs::msg::HealthCheckResult>::SharedPtr
      low_power_health_check_result_sub;
    rclcpp::Subscription<sinsei_umiusi_msgs::msg::HealthCheckResult>::SharedPtr
      high_power_health_check_result_sub;
    rclcpp::Service<sinsei_umiusi_msgs::srv::PowerOn>::SharedPtr power_on_service;

    bool low_power_circuit_is_ok;
    bool high_power_circuit_is_ok;
    bool is_ready_to_power_on;

  public:
    PowerOff(const std::string & name, const BT::NodeConfiguration & config);

    static auto providedPorts() -> BT::PortsList { return {}; }

    auto onStart() -> BT::NodeStatus override;
    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_state

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP
