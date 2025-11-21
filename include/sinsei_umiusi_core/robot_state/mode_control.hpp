#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_CONTROL_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_CONTROL_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_msgs/srv/set_state.hpp"

namespace sinsei_umiusi_core::robot_state
{

class ModeControl : public BT::SyncActionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Service<sinsei_umiusi_msgs::srv::SetState>::SharedPtr set_state_srv;

    sinsei_umiusi_msgs::srv::SetState::Request::_state_type current_mode;

  public:
    ModeControl(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts()
    {
        using StateType = sinsei_umiusi_msgs::srv::SetState::Request::_state_type;
        return {
          BT::OutputPort<StateType>("mode"),  // MANUAL, AUTO, DEBUG
        };
    }

    auto tick() -> BT::NodeStatus override;
};

}  // namespace sinsei_umiusi_core::robot_state

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_CONTROL_HPP
