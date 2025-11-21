#ifndef SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_MATCH_HPP
#define SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_MATCH_HPP

#include <behaviortree_cpp/condition_node.h>

#include "sinsei_umiusi_msgs/srv/set_state.hpp"

namespace sinsei_umiusi_core::robot_state
{
class ModeMatch : public BT::ConditionNode
{
  private:
    sinsei_umiusi_msgs::srv::SetState::Request::_state_type target_mode;

  public:
    ModeMatch(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts()
    {
        using StateType = sinsei_umiusi_msgs::srv::SetState::Request::_state_type;
        return {
          BT::InputPort<StateType>("mode"),     // MANUAL, AUTO, DEBUG
          BT::InputPort<StateType>("match_to")  // MANUAL, AUTO, DEBUG
        };
    }

    auto tick() -> BT::NodeStatus override;
};

}  // namespace sinsei_umiusi_core::robot_state

#endif  // SINSEI_UMIUSI_CORE_ROBOT_STATE_MODE_MATCH_HPP
