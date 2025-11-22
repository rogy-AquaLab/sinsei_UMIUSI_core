#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include "sinsei_umiusi_msgs/msg/main_power_output.hpp"
#include "sinsei_umiusi_msgs/msg/thruster_enabled_all.hpp"

namespace sinsei_umiusi_core::robot_strategy
{

class PowerOff : public BT::StatefulActionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::MainPowerOutput>::SharedPtr main_power_output_pub;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::ThrusterEnabledAll>::SharedPtr thruster_enabled_pub;

    const sinsei_umiusi_msgs::msg::MainPowerOutput main_power_disabled_msg;
    const sinsei_umiusi_msgs::msg::ThrusterEnabledAll thrusters_disabled_msg;

  public:
    PowerOff(const std::string & name, const BT::NodeConfiguration & config);

    static auto providedPorts() -> BT::PortsList { return {}; }

    auto onStart() -> BT::NodeStatus override;
    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP
