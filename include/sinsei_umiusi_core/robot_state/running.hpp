#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_RUNNING_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_RUNNING_HPP

#include <behaviortree_cpp/action_node.h>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_msgs/srv/power_off.hpp"
#include "sinsei_umiusi_msgs/srv/set_state.hpp"

namespace sinsei_umiusi_core::robot_state
{

class Running : public BT::StatefulActionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Service<sinsei_umiusi_msgs::srv::PowerOff>::SharedPtr power_off_srv;
    rclcpp::Service<sinsei_umiusi_msgs::srv::SetState>::SharedPtr set_state_srv;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_manual_clt;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_auto_clt;
    // rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_debug_clt;

    bool is_ready_to_power_off;
    bool is_ready_to_standby;
    sinsei_umiusi_msgs::srv::SetState::Request::_state_type mode;

  public:
    Running(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts()
    {
        using StateType = sinsei_umiusi_msgs::srv::SetState::Request::_state_type;
        return {
          BT::InputPort<StateType>("mode")  // MANUAL, AUTO, DEBUG
        };
    }

    auto onStart() -> BT::NodeStatus override;
    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_state

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_RUNNING_HPP
