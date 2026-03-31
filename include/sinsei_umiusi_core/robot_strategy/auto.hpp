#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_AUTO_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_AUTO_HPP

#include <behaviortree_cpp/action_node.h>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_msgs/msg/robot_state.hpp"
#include "sinsei_umiusi_msgs/msg/thruster_runnable_all.hpp"

namespace sinsei_umiusi_core::robot_strategy
{

class Auto : public BT::StatefulActionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_target_generator_clt;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::ThrusterRunnableAll>::SharedPtr
      thruster_runnable_pub;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::RobotState>::SharedPtr robot_state_pub;

    const sinsei_umiusi_msgs::msg::RobotState robot_state_auto_msg;
    const sinsei_umiusi_msgs::msg::ThrusterRunnableAll thruster_all_runnable_msg;

  public:
    Auto(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts() { return {}; }

    auto onStart() -> BT::NodeStatus override;
    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_AUTO_HPP
