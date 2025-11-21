#include "sinsei_umiusi_core/robot_state/standby.hpp"

sinsei_umiusi_core::robot_state::Standby::Standby(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

auto sinsei_umiusi_core::robot_state::Standby::onStart() -> BT::NodeStatus
{
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_state::Standby::onRunning() -> BT::NodeStatus
{
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_state::Standby::onHalted() -> void {}
