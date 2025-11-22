#include "sinsei_umiusi_core/robot_strategy/standby.hpp"

sinsei_umiusi_core::robot_strategy::Standby::Standby(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode{name, config}
{
}

auto sinsei_umiusi_core::robot_strategy::Standby::onStart() -> BT::NodeStatus
{
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_strategy::Standby::onRunning() -> BT::NodeStatus
{
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_strategy::Standby::onHalted() -> void {}
