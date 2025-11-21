#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_MANUAL_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_MANUAL_HPP

#include <behaviortree_cpp/action_node.h>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sinsei_umiusi_core::robot_state
{

class Manual : public BT::StatefulActionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_target_generator_clt;

  public:
    Manual(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts() { return {}; }

    auto onStart() -> BT::NodeStatus override;
    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_state

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_MANUAL_HPP
