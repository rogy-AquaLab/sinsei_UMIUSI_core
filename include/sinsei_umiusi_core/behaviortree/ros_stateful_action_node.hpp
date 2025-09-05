#ifndef SINSEI_UMIUSI_CORE_BEHAVIORTREE_ROS_STATEFUL_ACTION_NODE_HPP
#define SINSEI_UMIUSI_CORE_BEHAVIORTREE_ROS_STATEFUL_ACTION_NODE_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>

namespace sinsei_umiusi_core::behavortree
{

class RosStatefulActionNode : public BT::StatefulActionNode
{
  protected:
    rclcpp::Node::SharedPtr node;

  public:
    RosStatefulActionNode(
      const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node);
    virtual ~RosStatefulActionNode() = default;

    virtual auto onStart() -> BT::NodeStatus override = 0;
    virtual auto onRunning() -> BT::NodeStatus override = 0;
    virtual void onHalted() override = 0;
};

}  // namespace sinsei_umiusi_core::behavortree

#endif  // SINSEI_UMIUSI_CORE_BEHAVIORTREE_ROS_STATEFUL_ACTION_NODE_HPP
