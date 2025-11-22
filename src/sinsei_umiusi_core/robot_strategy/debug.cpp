// existing content

#include "sinsei_umiusi_core/robot_strategy/debug.hpp"
sinsei_umiusi_core::robot_strategy::Debug::Debug(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode{name, config}, ros_node{nullptr}, change_state_thruster_output_clt{nullptr}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_output");
    this->change_state_thruster_output_clt =
      this->ros_node->create_client<lifecycle_msgs::srv::ChangeState>(
        "/debug_thruster_output/change_state");
}
auto sinsei_umiusi_core::robot_strategy::Debug::onStart() -> BT::NodeStatus
{
    // Request to activate debug target generator
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    this->change_state_thruster_output_clt->wait_for_service();
    auto future = this->change_state_thruster_output_clt->async_send_request(req);
    // Wait for the result.
    auto result = rclcpp::spin_until_future_complete(this->ros_node, future);
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->ros_node->get_logger(), "Debug target generator activated.");
            return BT::NodeStatus::RUNNING;
        } else {
            RCLCPP_ERROR(
              this->ros_node->get_logger(), "Failed to activate debug target generator.");
            return BT::NodeStatus::FAILURE;
        }
    } else {
        RCLCPP_ERROR(
          this->ros_node->get_logger(), "Service call to activate debug target generator failed.");
        return BT::NodeStatus::FAILURE;
    }
}
auto sinsei_umiusi_core::robot_strategy::Debug::onRunning() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);
    return BT::NodeStatus::RUNNING;
}
auto sinsei_umiusi_core::robot_strategy::Debug::onHalted() -> void
{
    // Request to deactivate debug target generator
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));

    this->change_state_thruster_output_clt->wait_for_service();
    auto future = this->change_state_thruster_output_clt->async_send_request(req);
    // Wait for the result.
    auto result = rclcpp::spin_until_future_complete(this->ros_node, future);
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->ros_node->get_logger(), "Debug target generator deactivated.");
        } else {
            RCLCPP_ERROR(
              this->ros_node->get_logger(), "Failed to deactivate debug target generator.");
        }
    } else {
        RCLCPP_ERROR(
          this->ros_node->get_logger(),
          "Service call to deactivate debug target generator failed.");
    }
}
