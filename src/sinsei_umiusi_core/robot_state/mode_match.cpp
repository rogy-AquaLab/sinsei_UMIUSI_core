#include "sinsei_umiusi_core/robot_state/mode_match.hpp"

sinsei_umiusi_core::robot_state::ModeMatch::ModeMatch(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode{name, config}, target_mode{MODE_STANDBY}
{
}

auto sinsei_umiusi_core::robot_state::ModeMatch::tick() -> BT::NodeStatus
{
    auto mode = this->getInput<Mode>("mode");
    auto match_to = this->getInput<Mode>("match_to");
    if (mode == match_to) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}
