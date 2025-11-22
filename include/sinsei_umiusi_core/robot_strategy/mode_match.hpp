#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_MATCH_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_MATCH_HPP

#include <behaviortree_cpp/condition_node.h>

#include "sinsei_umiusi_msgs/srv/set_mode.hpp"

namespace sinsei_umiusi_core::robot_strategy
{
class ModeMatch : public BT::ConditionNode
{
  public:
    using Mode = sinsei_umiusi_msgs::srv::SetMode::Request::_mode_type;

    static constexpr Mode MODE_STANDBY = sinsei_umiusi_msgs::srv::SetMode::Request::STANDBY;
    static constexpr Mode MODE_MANUAL = sinsei_umiusi_msgs::srv::SetMode::Request::MANUAL;
    static constexpr Mode MODE_AUTO = sinsei_umiusi_msgs::srv::SetMode::Request::AUTO;
    static constexpr Mode MODE_DEBUG = sinsei_umiusi_msgs::srv::SetMode::Request::DEBUG;

  private:
    Mode target_mode;

  public:
    ModeMatch(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts()
    {
        return {
          BT::InputPort<Mode>("mode"),     // MANUAL, AUTO, DEBUG
          BT::InputPort<Mode>("match_to")  // MANUAL, AUTO, DEBUG
        };
    }

    auto tick() -> BT::NodeStatus override;
};

}  // namespace sinsei_umiusi_core::robot_strategy

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_MATCH_HPP
