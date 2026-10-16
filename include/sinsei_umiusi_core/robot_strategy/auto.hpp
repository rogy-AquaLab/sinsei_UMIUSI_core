#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_AUTO_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_AUTO_HPP

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <functional>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "sinsei_umiusi_msgs/msg/attitude_target.hpp"
#include "sinsei_umiusi_msgs/msg/robot_state.hpp"
#include "sinsei_umiusi_msgs/msg/target.hpp"
#include "sinsei_umiusi_msgs/msg/thruster_runnable_all.hpp"

namespace sinsei_umiusi_core::robot_strategy
{

// Auto の間、auto_target_generator の ~/status が途絶えたら推力 0 を出し、
// ノードが立ち上がり直していれば（launch の respawn）configure / activate し直す。
// 持ち時間は最初に Auto に入った時刻から数え続ける（パラメータ run_started_at で渡す）
class Auto : public BT::StatefulActionNode
{
  private:
    using Clock = std::chrono::steady_clock;

    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_target_generator_clt;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_target_generator_clt;
    rclcpp::AsyncParametersClient::SharedPtr target_generator_params_clt;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_generator_status_sub;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::ThrusterRunnableAll>::SharedPtr
      thruster_runnable_pub;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::RobotState>::SharedPtr robot_state_pub;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::Target>::SharedPtr target_pub;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::AttitudeTarget>::SharedPtr attitude_target_pub;

    const sinsei_umiusi_msgs::msg::RobotState robot_state_auto_msg;
    const sinsei_umiusi_msgs::msg::ThrusterRunnableAll thruster_all_runnable_msg;

    Clock::time_point started_at;
    Clock::time_point last_status_at;
    Clock::time_point recover_requested_at;
    bool recovering;

    auto set_run_started_at(std::function<void()> then) -> void;
    auto change_state_async(uint8_t transition, std::function<void(bool)> done) -> void;
    auto recover() -> void;
    auto publish_zero_target() -> void;

  public:
    Auto(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts() { return {}; }

    auto onStart() -> BT::NodeStatus override;
    auto onRunning() -> BT::NodeStatus override;
    auto onHalted() -> void override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_AUTO_HPP
