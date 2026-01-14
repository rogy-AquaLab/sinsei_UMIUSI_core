#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_ON_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_ON_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include "sinsei_umiusi_msgs/msg/main_power_output.hpp"

namespace sinsei_umiusi_core::robot_strategy
{

class PowerOn : public BT::ActionNodeBase
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::MainPowerOutput>::SharedPtr main_power_output_pub;

    const sinsei_umiusi_msgs::msg::MainPowerOutput main_power_enabled_msg;

  public:
    PowerOn(const std::string & name, const BT::NodeConfiguration & config);

    static auto providedPorts() -> BT::PortsList { return {}; }

    auto tick() -> BT::NodeStatus override;
    auto halt() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_ON_HPP
