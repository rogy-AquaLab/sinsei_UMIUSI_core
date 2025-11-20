#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_ROBOT_STATE_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_ROBOT_STATE_HPP

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>
#include <optional>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string_view>

namespace sinsei_umiusi_core::robot_state
{

class Core : public rclcpp::Node
{
  public:
    static constexpr std::string_view PARAM_NAME_BEHAVIOR_TREE_FILE = "behavior_tree_file";

  private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_manual_clt;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_auto_clt;
    // rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_debug_clt;

    std::unique_ptr<BT::Tree> tree;
    std::optional<BT::Groot2Publisher> groot2_publisher;

    auto timer_callback() const -> void;

  public:
    Core();
    ~Core() = default;
};

}  // namespace sinsei_umiusi_core::robot_state

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_ROBOT_STATE_HPP
