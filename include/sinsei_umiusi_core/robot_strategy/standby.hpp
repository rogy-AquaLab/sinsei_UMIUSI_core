#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_STANDBY_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_STANDBY_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_msgs/msg/robot_state.hpp"

namespace sinsei_umiusi_core::robot_strategy
{

class Standby : public BT::StatefulActionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::RobotState>::SharedPtr robot_state_pub;

    const sinsei_umiusi_msgs::msg::RobotState robot_state_standby_msg;

  public:
    Standby(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts() { return {}; }

    auto onStart() -> BT::NodeStatus override;
    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_STANDBY_HPP
