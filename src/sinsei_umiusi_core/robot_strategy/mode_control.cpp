#include "sinsei_umiusi_core/robot_strategy/mode_control.hpp"

#include <cstdio>
#include <optional>

namespace
{

constexpr auto _mode_to_child_index(sinsei_umiusi_core::robot_strategy::ModeControl::Mode mode)
  -> size_t
{
    return static_cast<size_t>(mode);
}

}  // namespace

sinsei_umiusi_core::robot_strategy::ModeControl::ModeControl(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ControlNode{name, config},
  ros_node{nullptr},
  set_state_srv{nullptr},
  current_mode{MODE_STANDBY},
  current_child_index{std::nullopt}
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
    if (this->childrenCount() != 4) {
        throw BT::LogicError("ModeControl must have exactly 4 children");
    }
    for (size_t i = 0; i < 4; ++i) {
        if (CHILD_NAMES[i] != this->children_nodes_[i]->name()) {
            char err_msg[256] = "";
            std::sprintf(
              err_msg, "The child node at index %zu must be named '%s'", i, CHILD_NAMES[i].data());
            throw BT::LogicError(err_msg);
        }
    }

    auto child_index = _mode_to_child_index(this->current_mode);
    if (this->current_child_index.has_value() && this->current_child_index.value() != child_index) {
        // Halt the currently running child if the mode will be changed
        this->haltChild(this->current_child_index.value());
    }

    auto child_status = this->children_nodes_[child_index]->executeTick();
    switch (child_status) {
        case BT::NodeStatus::RUNNING:
            rclcpp::spin_some(this->ros_node);
            this->current_child_index = child_index;
            break;
        case BT::NodeStatus::SKIPPED:
            this->current_child_index = std::nullopt;
            break;
        case BT::NodeStatus::SUCCESS:
        case BT::NodeStatus::FAILURE:
        case BT::NodeStatus::IDLE:
            this->resetChildren();
            this->current_child_index = std::nullopt;
            break;
    }
    return child_status;
}

auto sinsei_umiusi_core::robot_strategy::ModeControl::halt() -> void
{
    this->current_mode = MODE_STANDBY;
    this->current_child_index = std::nullopt;
    BT::ControlNode::halt();
}
