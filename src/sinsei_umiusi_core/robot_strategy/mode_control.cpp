#include "sinsei_umiusi_core/robot_strategy/mode_control.hpp"

sinsei_umiusi_core::robot_strategy::ModeControl::ModeControl(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode{name, config},
  ros_node{nullptr},
  set_state_srv{nullptr},
  main_power_output_pub{nullptr},
  main_power_enabled_msg{sinsei_umiusi_msgs::msg::MainPowerOutput{}.set__enabled(true)},
  current_mode{MODE_STANDBY}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_mode_control");
    this->set_state_srv = this->ros_node->create_service<sinsei_umiusi_msgs::srv::SetMode>(
      "/user_input/set_mode", [this](
                                const sinsei_umiusi_msgs::srv::SetMode::Request::SharedPtr request,
                                sinsei_umiusi_msgs::srv::SetMode::Response::SharedPtr response) {
          if (this->current_mode == request->mode) {
              response->set__success(true);
              return;
          }
          if (this->current_mode == MODE_STANDBY || request->mode == MODE_STANDBY) {
              // from STANDBY to MANUAL, AUTO, DEBUG or from MANUAL, AUTO, DEBUG to STANDBY
              response->set__success(true);
              this->current_mode = request->mode;
              return;
          }
          // from MANUAL, AUTO, DEBUG to MANUAL, AUTO, DEBUG
          response->set__success(false);
          response->set__error_msg(
            "Invalid transition. Only STANDBY is available from MANUAL, AUTO, DEBUG state.");
          return;
      });
}

auto sinsei_umiusi_core::robot_strategy::ModeControl::tick() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);
    this->setOutput("mode", this->current_mode);
    this->main_power_output_pub->publish(this->main_power_enabled_msg);
    return BT::NodeStatus::SUCCESS;
}
