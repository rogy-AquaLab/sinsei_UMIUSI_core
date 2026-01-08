#ifndef SINSEI_UMIUSI_CORE_ROBOT_STRATEGY_FIND_CLIENT_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STRATEGY_FIND_CLIENT_HPP

#include <behaviortree_cpp/condition_node.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace sinsei_umiusi_core::robot_strategy
{

class FindClient : public BT::ConditionNode
{
  private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr client_count_sub;

    int32_t clients_num;

  public:
    FindClient(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts() { return {}; }

    auto tick() -> BT::NodeStatus override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STRATEGY_FIND_CLIENT_HPP
