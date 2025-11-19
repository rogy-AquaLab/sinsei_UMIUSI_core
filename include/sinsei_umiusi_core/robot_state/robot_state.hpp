#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_ROBOT_STATE_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_ROBOT_STATE_HPP

#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string_view>

namespace sinsei_umiusi_core::robot_state
{

class RobotState : public rclcpp::Node
{
  public:
    static constexpr std::string_view PARAM_NAME_BEHAVIOR_TREE_FILE = "behavior_tree_file";
    RobotState();
    ~RobotState() = default;

  private:
    rclcpp::TimerBase::SharedPtr timer;
    std::unique_ptr<BT::Tree> tree;

    auto timer_callback() const -> void;
};

}  // namespace sinsei_umiusi_core::robot_state

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_ROBOT_STATE_HPP
