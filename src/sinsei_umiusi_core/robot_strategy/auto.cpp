#include "sinsei_umiusi_core/robot_strategy/auto.hpp"

sinsei_umiusi_core::robot_strategy::Auto::Auto(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode{name, config},
  ros_node{nullptr},
  change_state_target_generator_clt{nullptr},
  thruster_enabled_pub{nullptr},
  robot_state_auto_msg{
    sinsei_umiusi_msgs::msg::RobotState{}.set__state(sinsei_umiusi_msgs::msg::RobotState::AUTO)}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_auto");
    this->change_state_target_generator_clt =
      this->ros_node->create_client<lifecycle_msgs::srv::ChangeState>(
        "/auto_target_generator/change_state");
    this->thruster_enabled_pub =
      this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::ThrusterEnabledAll>(
        "/cmd/thruster_enabled_all", rclcpp::SystemDefaultsQoS{});
    this->robot_state_pub = this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::RobotState>(
      "/robot_state", rclcpp::SystemDefaultsQoS{});
}

auto sinsei_umiusi_core::robot_strategy::Auto::onStart() -> BT::NodeStatus
{
    // Request to activate auto target generator
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    this->change_state_target_generator_clt->wait_for_service();
    auto future = this->change_state_target_generator_clt->async_send_request(req);

    this->thruster_enabled_pub->publish(
      sinsei_umiusi_msgs::msg::ThrusterEnabledAll{}
        .set__lf(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(true).set__servo(true))
        .set__lb(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(true).set__servo(true))
        .set__rf(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(true).set__servo(true))
        .set__rb(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(true).set__servo(true)));

    // Wait for the result.
    auto result = rclcpp::spin_until_future_complete(this->ros_node, future);
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->ros_node->get_logger(), "Auto target generator activated.");
            return BT::NodeStatus::RUNNING;
        } else {
            RCLCPP_ERROR(this->ros_node->get_logger(), "Failed to activate auto target generator.");
            return BT::NodeStatus::FAILURE;
        }
    } else {
        RCLCPP_ERROR(
          this->ros_node->get_logger(), "Service call to activate auto target generator failed.");
        return BT::NodeStatus::FAILURE;
    }
}

auto sinsei_umiusi_core::robot_strategy::Auto::onRunning() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);

    this->robot_state_pub->publish(this->robot_state_auto_msg);
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_strategy::Auto::onHalted() -> void
{
    // Request to deactivate auto target generator
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));

    this->change_state_target_generator_clt->wait_for_service();
    auto future = this->change_state_target_generator_clt->async_send_request(req);

    this->thruster_enabled_pub->publish(
      sinsei_umiusi_msgs::msg::ThrusterEnabledAll{}
        .set__lf(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false))
        .set__lb(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false))
        .set__rf(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false))
        .set__rb(sinsei_umiusi_msgs::msg::ThrusterEnabled{}.set__esc(false).set__servo(false)));

    // Wait for the result.
    auto result = rclcpp::spin_until_future_complete(this->ros_node, future);
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->ros_node->get_logger(), "Auto target generator deactivated.");
        } else {
            RCLCPP_ERROR(
              this->ros_node->get_logger(), "Failed to deactivate auto target generator.");
        }
    } else {
        RCLCPP_ERROR(
          this->ros_node->get_logger(), "Service call to deactivate auto target generator failed.");
    }
}
