#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_STANDBY_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_STANDBY_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>

namespace sinsei_umiusi_core::robot_strategy
{

class Standby : public BT::StatefulActionNode
{
  public:
    Standby(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts() { return {}; }

    auto onStart() -> BT::NodeStatus override;
    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_STANDBY_HPP
