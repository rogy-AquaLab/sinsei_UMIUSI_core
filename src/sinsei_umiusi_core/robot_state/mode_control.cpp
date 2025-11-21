#include "sinsei_umiusi_core/robot_state/mode_control.hpp"

sinsei_umiusi_core::robot_state::ModeControl::ModeControl(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  ros_node(nullptr),
  set_state_srv(nullptr),
  current_mode(sinsei_umiusi_msgs::srv::SetState::Request::STANDBY)
{
    this->ros_node = rclcpp::Node::make_shared("_bt_mode_control");
    this->set_state_srv = this->ros_node->create_service<sinsei_umiusi_msgs::srv::SetState>(
      "/user_input/set_state",
      [this](
        const sinsei_umiusi_msgs::srv::SetState::Request::SharedPtr request,
        sinsei_umiusi_msgs::srv::SetState::Response::SharedPtr response) {
          if (this->current_mode == request->state) {
              response->set__success(true);
              return;
          }
          if (
            this->current_mode == sinsei_umiusi_msgs::srv::SetState::Request::STANDBY ||
            request->state == sinsei_umiusi_msgs::srv::SetState::Request::STANDBY) {
              // from STANDBY to MANUAL, AUTO, DEBUG or from MANUAL, AUTO, DEBUG to STANDBY
              response->set__success(true);
              this->current_mode = request->state;
              return;
          }
          // from MANUAL, AUTO, DEBUG to MANUAL, AUTO, DEBUG
          response->set__success(false);
          response->set__error_msg(
            "Invalid transition. Only STANDBY is available from MANUAL, AUTO, DEBUG state.");
          return;
      });
}

auto sinsei_umiusi_core::robot_state::ModeControl::tick() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);
    this->setOutput("mode", this->current_mode);
    return BT::NodeStatus::SUCCESS;
}
