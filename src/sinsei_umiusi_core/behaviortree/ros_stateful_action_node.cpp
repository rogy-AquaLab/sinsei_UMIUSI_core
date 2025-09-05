#include "sinsei_umiusi_core/behaviortree/ros_stateful_action_node.hpp"

sinsei_umiusi_core::behavortree::RosStatefulActionNode::RosStatefulActionNode(
  const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node(node)
{
}
