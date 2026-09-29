#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include "sinsei_umiusi_msgs/msg/power_distribution_output.hpp"
#include "sinsei_umiusi_msgs/msg/robot_state.hpp"

namespace sinsei_umiusi_core::robot_strategy
{

class PowerOff : public BT::ActionNodeBase
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::PowerDistributionOutput>::SharedPtr
      power_distribution_output_pub;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::RobotState>::SharedPtr robot_state_pub;

    const sinsei_umiusi_msgs::msg::PowerDistributionOutput power_distribution_disabled_msg;
    const sinsei_umiusi_msgs::msg::RobotState robot_state_power_off_msg;

  public:
    PowerOff(const std::string & name, const BT::NodeConfiguration & config);

    static auto providedPorts() -> BT::PortsList { return {}; }

    auto tick() -> BT::NodeStatus override;
    auto halt() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP
