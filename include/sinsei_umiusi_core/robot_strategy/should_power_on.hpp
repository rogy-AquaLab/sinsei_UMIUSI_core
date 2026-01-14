#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_CONTROL_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_CONTROL_HPP

#include <behaviortree_cpp/condition_node.h>

#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_msgs/msg/health_check_result.hpp"
#include "sinsei_umiusi_msgs/srv/power_off.hpp"
#include "sinsei_umiusi_msgs/srv/power_on.hpp"
#include "std_msgs/msg/int32.hpp"

namespace sinsei_umiusi_core::robot_strategy
{

class ShouldPowerOn : public BT::ConditionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Service<sinsei_umiusi_msgs::srv::PowerOff>::SharedPtr power_off_srv;
    rclcpp::Service<sinsei_umiusi_msgs::srv::PowerOn>::SharedPtr power_on_srv;
    rclcpp::Subscription<sinsei_umiusi_msgs::msg::HealthCheckResult>::SharedPtr
      low_power_health_check_sub;
    rclcpp::Subscription<sinsei_umiusi_msgs::msg::HealthCheckResult>::SharedPtr
      high_power_health_check_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr client_count_sub;

    bool low_power_circuit_is_ok;
    bool high_power_circuit_is_ok;
    int32_t clients_num;
    bool should_power_on;

  public:
    ShouldPowerOn(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts() { return {}; }

    auto tick() -> BT::NodeStatus override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_CONTROL_HPP
