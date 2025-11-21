#include "sinsei_umiusi_core/robot_state/running.hpp"

#include <rclcpp/client.hpp>

#include "sinsei_umiusi_msgs/srv/set_state.hpp"

sinsei_umiusi_core::robot_state::Running::Running(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  ros_node(nullptr),
  power_off_srv(nullptr),
  set_state_srv(nullptr),
  change_state_manual_clt(nullptr),
  change_state_auto_clt(nullptr),
  // change_state_debug_clt(nullptr),
  is_ready_to_power_off(false),
  is_ready_to_standby(false),
  mode(sinsei_umiusi_msgs::srv::SetState::Request::STANDBY)
{
    this->ros_node = rclcpp::Node::make_shared("_bt_running");
    this->change_state_manual_clt = this->ros_node->create_client<lifecycle_msgs::srv::ChangeState>(
      "/manual_target_generator/change_state");
    this->change_state_auto_clt = this->ros_node->create_client<lifecycle_msgs::srv::ChangeState>(
      "/auto_target_generator/change_state");
    // this->change_state_debug_clt = this->ros_node->create_client<lifecycle_msgs::srv::ChangeState>(
    //   "/thruster_debug_output/change_state");
    {
        this->power_off_srv = this->ros_node->create_service<sinsei_umiusi_msgs::srv::PowerOff>(
          "/user_input/power_off",
          [this](
            const sinsei_umiusi_msgs::srv::PowerOff::Request::SharedPtr /* request */,
            sinsei_umiusi_msgs::srv::PowerOff::Response::SharedPtr response) {
              response->set__success(true);
              this->is_ready_to_power_off = true;
              return;
          });
        this->set_state_srv = this->ros_node->create_service<sinsei_umiusi_msgs::srv::SetState>(
          "/user_input/set_state",
          [this](
            const sinsei_umiusi_msgs::srv::SetState::Request::SharedPtr request,
            sinsei_umiusi_msgs::srv::SetState::Response::SharedPtr response) {
              if (request->state != sinsei_umiusi_msgs::srv::SetState::Request::STANDBY) {
                  response->set__success(false);
                  response->set__error_msg(
                    "Invalid transition. Only STANDBY is available from MANUAL, AUTO, DEBUG "
                    "state.");
                  return;
              }
              response->set__success(true);
              this->is_ready_to_standby = true;
              return;
          });
    }
}

auto sinsei_umiusi_core::robot_state::Running::onStart() -> BT::NodeStatus
{
    using StateType = sinsei_umiusi_msgs::srv::SetState::Request::_state_type;

    this->is_ready_to_power_off = false;
    this->is_ready_to_standby = false;
    auto mode_result = this->getInput<StateType>("mode");
    if (!mode_result) {
        RCLCPP_ERROR(
          this->ros_node->get_logger(), "Missing required input port [mode]: %s",
          mode_result.error().c_str());
        return BT::NodeStatus::FAILURE;
    }
    auto activate_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    activate_request->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
    switch (mode_result.value()) {
        case sinsei_umiusi_msgs::srv::SetState::Request::STANDBY:
            return BT::NodeStatus::SUCCESS;  // go to STANDBY mode
        case sinsei_umiusi_msgs::srv::SetState::Request::MANUAL: {
            this->change_state_manual_clt->wait_for_service();
            this->change_state_manual_clt->async_send_request(activate_request);
            break;
        }
        case sinsei_umiusi_msgs::srv::SetState::Request::AUTO: {
            this->change_state_auto_clt->wait_for_service();
            this->change_state_auto_clt->async_send_request(activate_request);
            break;
        }
        case sinsei_umiusi_msgs::srv::SetState::Request::DEBUG: {
            // this->change_state_debug_clt->wait_for_service();
            // this->change_state_debug_clt->async_send_request(activate_request);
            break;
        }
    }
    this->mode = mode_result.value();
    RCLCPP_INFO(
      this->ros_node->get_logger(), "Entering mode (%s)",
      this->mode == sinsei_umiusi_msgs::srv::SetState::Request::MANUAL ? "MANUAL"
      : this->mode == sinsei_umiusi_msgs::srv::SetState::Request::AUTO ? "AUTO"
                                                                       : "DEBUG");
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_state::Running::onRunning() -> BT::NodeStatus
{
    using StateType = sinsei_umiusi_msgs::srv::SetState::Request::_state_type;

    rclcpp::spin_some(this->ros_node);
    if (this->is_ready_to_power_off) {
        return BT::NodeStatus::FAILURE;
    }
    auto mode = this->getInput<StateType>("mode").value();  // already checked in onStart()
    if (mode != this->mode) {
        RCLCPP_ERROR(
          this->ros_node->get_logger(), "Unexpected mode change (%d -> %d)! Going to STANDBY mode",
          this->mode, mode);
        return BT::NodeStatus::SUCCESS;  // go to STANDBY mode
    }
    if (this->is_ready_to_standby) {
        auto deactivate_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        deactivate_request->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
        switch (this->mode) {
            case sinsei_umiusi_msgs::srv::SetState::Request::MANUAL: {
                this->change_state_manual_clt->wait_for_service();
                this->change_state_manual_clt->async_send_request(deactivate_request);
                break;
            }
            case sinsei_umiusi_msgs::srv::SetState::Request::AUTO: {
                this->change_state_auto_clt->wait_for_service();
                this->change_state_auto_clt->async_send_request(deactivate_request);
                break;
            }
            case sinsei_umiusi_msgs::srv::SetState::Request::DEBUG: {
                // this->change_state_debug_clt->wait_for_service();
                // this->change_state_debug_clt->async_send_request(deactivate_request);
                break;
            }
        }
        return BT::NodeStatus::SUCCESS;  // go to STANDBY mode
    }

    switch (this->mode) {
        case sinsei_umiusi_msgs::srv::SetState::Request::MANUAL:
        case sinsei_umiusi_msgs::srv::SetState::Request::AUTO:
        case sinsei_umiusi_msgs::srv::SetState::Request::DEBUG:
            // NOTE: If you have any specific actions to perform in these modes, add them here.
            break;
        default:
            // unreachable
            return BT::NodeStatus::SUCCESS;  // go to STANDBY mode
    }
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_state::Running::onHalted() -> void {}
