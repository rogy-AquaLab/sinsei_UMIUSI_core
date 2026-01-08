#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_CONTROL_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_CONTROL_HPP

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/control_node.h>

#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_msgs/msg/main_power_output.hpp"
#include "sinsei_umiusi_msgs/srv/set_mode.hpp"

namespace sinsei_umiusi_core::robot_strategy
{

class ModeControl : public BT::ControlNode
{
  public:
    using Mode = sinsei_umiusi_msgs::srv::SetMode::Request::_mode_type;

    static constexpr std::string_view CHILD_NAMES[4] = {"Standby", "Manual", "Auto", "Debug"};
    static constexpr Mode MODE_STANDBY = sinsei_umiusi_msgs::srv::SetMode::Request::STANDBY;
    static constexpr Mode MODE_MANUAL = sinsei_umiusi_msgs::srv::SetMode::Request::MANUAL;
    static constexpr Mode MODE_AUTO = sinsei_umiusi_msgs::srv::SetMode::Request::AUTO;
    static constexpr Mode MODE_DEBUG = sinsei_umiusi_msgs::srv::SetMode::Request::DEBUG;

  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Service<sinsei_umiusi_msgs::srv::SetMode>::SharedPtr set_state_srv;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::MainPowerOutput>::SharedPtr main_power_output_pub;

    const sinsei_umiusi_msgs::msg::MainPowerOutput main_power_enabled_msg;

    Mode current_mode;
    std::optional<size_t> current_child_index;

  public:
    ModeControl(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts() { return {}; }

    auto tick() -> BT::NodeStatus override;
    auto halt() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_CONTROL_HPP
