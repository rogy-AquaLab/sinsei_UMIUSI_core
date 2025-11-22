#include "sinsei_umiusi_core/robot_strategy/manual.hpp"

sinsei_umiusi_core::robot_strategy::Manual::Manual(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode{name, config},
  ros_node{nullptr},
  change_state_target_generator_clt{nullptr},
  robot_state_manual_msg{
    sinsei_umiusi_msgs::msg::RobotState{}.set__state(sinsei_umiusi_msgs::msg::RobotState::MANUAL)}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_manual");
    this->change_state_target_generator_clt =
      this->ros_node->create_client<lifecycle_msgs::srv::ChangeState>(
        "/manual_target_generator/change_state");
    this->robot_state_pub = this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::RobotState>(
      "/robot_state", rclcpp::SystemDefaultsQoS{});
}

auto sinsei_umiusi_core::robot_strategy::Manual::onStart() -> BT::NodeStatus
{
    // Request to activate manual target generator
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    this->change_state_target_generator_clt->wait_for_service();
    auto future = this->change_state_target_generator_clt->async_send_request(req);
    // Wait for the result.
    auto result = rclcpp::spin_until_future_complete(this->ros_node, future);
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->ros_node->get_logger(), "Manual target generator activated.");
            return BT::NodeStatus::RUNNING;
        } else {
            RCLCPP_ERROR(
              this->ros_node->get_logger(), "Failed to activate manual target generator.");
            return BT::NodeStatus::FAILURE;
        }
    } else {
        RCLCPP_ERROR(
          this->ros_node->get_logger(), "Service call to activate manual target generator failed.");
        return BT::NodeStatus::FAILURE;
    }
}

auto sinsei_umiusi_core::robot_strategy::Manual::onRunning() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);

    this->robot_state_pub->publish(this->robot_state_manual_msg);
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_strategy::Manual::onHalted() -> void
{
    // Request to deactivate manual target generator
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));

    this->change_state_target_generator_clt->wait_for_service();
    auto future = this->change_state_target_generator_clt->async_send_request(req);
    // Wait for the result.
    auto result = rclcpp::spin_until_future_complete(this->ros_node, future);
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->ros_node->get_logger(), "Manual target generator deactivated.");
        } else {
            RCLCPP_ERROR(
              this->ros_node->get_logger(), "Failed to deactivate manual target generator.");
        }
    } else {
        RCLCPP_ERROR(
          this->ros_node->get_logger(),
          "Service call to deactivate manual target generator failed.");
    }
}
