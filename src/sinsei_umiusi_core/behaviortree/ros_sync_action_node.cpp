#include "sinsei_umiusi_core/behaviortree/ros_sync_action_node.hpp"

sinsei_umiusi_core::behavortree::RosSyncActionNode::RosSyncActionNode(
  const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node(node)
{
}
