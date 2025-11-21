#include "sinsei_umiusi_core/robot_state/mode_match.hpp"

sinsei_umiusi_core::robot_state::ModeMatch::ModeMatch(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config), target_mode(sinsei_umiusi_msgs::srv::SetState::Request::STANDBY)
{
}

auto sinsei_umiusi_core::robot_state::ModeMatch::tick() -> BT::NodeStatus
{
    using StateType = sinsei_umiusi_msgs::srv::SetState::Request::_state_type;

    auto mode = this->getInput<StateType>("mode");
    auto match_to = this->getInput<StateType>("match_to");
    if (mode == match_to) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}
