#ifndef SINSEI_UMIUSI_CORE_BEHAVIORTREE_ROS_SYNC_ACTION_NODE_HPP
#define SINSEI_UMIUSI_CORE_BEHAVIORTREE_ROS_SYNC_ACTION_NODE_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>

namespace sinsei_umiusi_core::behavortree
{

class RosSyncActionNode : public BT::SyncActionNode
{
  protected:
    rclcpp::Node::SharedPtr node;

  public:
    RosSyncActionNode(
      const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node);
    virtual ~RosSyncActionNode() = default;

    virtual auto tick() -> BT::NodeStatus override = 0;
};

}  // namespace sinsei_umiusi_core::behavortree

#endif  // SINSEI_UMIUSI_CORE_BEHAVIORTREE_ROS_SYNC_ACTION_NODE_HPP
