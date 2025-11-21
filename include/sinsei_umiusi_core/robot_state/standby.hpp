#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_STANDBY_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_STANDBY_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_msgs/msg/main_power_output.hpp"
#include "sinsei_umiusi_msgs/msg/thruster_enabled_all.hpp"
#include "sinsei_umiusi_msgs/srv/power_off.hpp"
#include "sinsei_umiusi_msgs/srv/set_state.hpp"

namespace sinsei_umiusi_core::robot_state
{

class Standby : public BT::StatefulActionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Service<sinsei_umiusi_msgs::srv::PowerOff>::SharedPtr power_off_srv;
    rclcpp::Service<sinsei_umiusi_msgs::srv::SetState>::SharedPtr set_state_srv;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::MainPowerOutput>::SharedPtr main_power_output_pub;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::ThrusterEnabledAll>::SharedPtr
      thruster_enabled_all_pub;

    bool power_off_requested;
    sinsei_umiusi_msgs::srv::SetState::Request::_state_type target_state;

  public:
    Standby(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts()
    {
        using StateType = sinsei_umiusi_msgs::srv::SetState::Request::_state_type;
        return {
          BT::OutputPort<StateType>("mode")  // MANUAL, AUTO, DEBUG
        };
    }

    auto onStart() -> BT::NodeStatus override;
    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_state

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_STANDBY_HPP
