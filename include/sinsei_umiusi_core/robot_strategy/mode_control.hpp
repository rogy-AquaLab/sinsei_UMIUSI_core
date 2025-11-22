#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_CONTROL_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_CONTROL_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_msgs/srv/set_mode.hpp"

namespace sinsei_umiusi_core::robot_strategy
{

class ModeControl : public BT::SyncActionNode
{
  public:
    using Mode = sinsei_umiusi_msgs::srv::SetMode::Request::_mode_type;

    static constexpr Mode MODE_STANDBY = sinsei_umiusi_msgs::srv::SetMode::Request::STANDBY;
    static constexpr Mode MODE_MANUAL = sinsei_umiusi_msgs::srv::SetMode::Request::MANUAL;
    static constexpr Mode MODE_AUTO = sinsei_umiusi_msgs::srv::SetMode::Request::AUTO;
    static constexpr Mode MODE_DEBUG = sinsei_umiusi_msgs::srv::SetMode::Request::DEBUG;

  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Service<sinsei_umiusi_msgs::srv::SetMode>::SharedPtr set_state_srv;

    Mode current_mode;

  public:
    ModeControl(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts()
    {
        return {
          BT::OutputPort<Mode>("mode"),  // MANUAL, AUTO, DEBUG
        };
    }

    auto tick() -> BT::NodeStatus override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_CONTROL_HPP
