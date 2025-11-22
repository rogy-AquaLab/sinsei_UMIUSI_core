#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_CONTROL_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_CONTROL_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>

namespace sinsei_umiusi_core::robot_strategy
{

class PowerControl : public BT::StatefulActionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;

  public:
    PowerControl(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts() { return {}; }

    auto onStart() -> BT::NodeStatus override;
    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_CONTROL_HPP
