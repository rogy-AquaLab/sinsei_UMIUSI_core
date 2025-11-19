#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_ROBOT_STATE_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_ROBOT_STATE_HPP

#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string_view>

#include "lifecycle_msgs/srv/change_state.hpp"

namespace sinsei_umiusi_core::robot_state
{

class RobotState : public rclcpp::Node
{
  public:
    static constexpr std::string_view PARAM_NAME_BEHAVIOR_TREE_FILE = "behavior_tree_file";

  private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr
      change_state_manual_target_generator;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_auto_target_generator;
    // rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_debug_thruster_output;
    std::unique_ptr<BT::Tree> tree;

    auto timer_callback() const -> void;

  public:
    RobotState();
    ~RobotState() = default;
};

}  // namespace sinsei_umiusi_core::robot_state

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_ROBOT_STATE_HPP
