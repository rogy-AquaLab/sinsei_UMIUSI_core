#include "sinsei_umiusi_core/robot_strategy/find_client.hpp"

sinsei_umiusi_core::robot_strategy::FindClient::FindClient(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode{name, config}, ros_node{nullptr}, client_count_sub{nullptr}, clients_num{0}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_find_client");
    this->client_count_sub = this->ros_node->create_subscription<std_msgs::msg::Int32>(
      "/client_count", rclcpp::SystemDefaultsQoS{},
      [this](const std_msgs::msg::Int32::SharedPtr msg) { this->clients_num = msg->data; });
}

auto sinsei_umiusi_core::robot_strategy::FindClient::tick() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);
    return this->clients_num > 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
