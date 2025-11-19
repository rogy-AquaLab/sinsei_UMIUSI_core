#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>

namespace sinsei_umiusi_core::robot_state
{

class PowerOff : public BT::StatefulActionNode
{
  private:
    rclcpp::Node::UniquePtr ros_node;

  public:
    PowerOff(const std::string & name, const BT::NodeConfiguration & config);

    static auto providedPorts() -> BT::PortsList { return {}; }

    auto onStart() -> BT::NodeStatus override;

    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_state

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_POWER_OFF_HPP
