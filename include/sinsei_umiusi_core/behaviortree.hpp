#ifndef SINSEI_UMIUSI_CORE_BEHAVIOR_TREE_HPP
#define SINSEI_UMIUSI_CORE_BEHAVIOR_TREE_HPP

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sinsei_umiusi_core::behavortree::action_node
{

class PublishDisplayInfo : public BT::SyncActionNode
{
  private:
    rclcpp::Node::SharedPtr node;

  public:
    PublishDisplayInfo(const std::string & name, const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
    {
        this->node = rclcpp::Node::make_shared("PublishDisplayInfo");
    }

    inline auto tick() -> BT::NodeStatus override
    {
        RCLCPP_INFO(this->node->get_logger(), "Publish display info");
        return BT::NodeStatus::SUCCESS;
    }

    inline static auto providedPorts() -> BT::PortsList { return {}; }
};

class LowPowerCircuitCheck : public BT::SyncActionNode
{
  private:
    rclcpp::Node::SharedPtr node;

  public:
    LowPowerCircuitCheck(const std::string & name, const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
    {
        this->node = rclcpp::Node::make_shared("LowPowerCircuitCheck");
    }

    inline auto tick() -> BT::NodeStatus override
    {
        RCLCPP_INFO(this->node->get_logger(), "Low power circuit check");
        return BT::NodeStatus::SUCCESS;
    }

    inline static auto providedPorts() -> BT::PortsList { return {}; }
};

class HighPowerCircuitCheck : public BT::SyncActionNode
{
  private:
    rclcpp::Node::SharedPtr node;

  public:
    HighPowerCircuitCheck(const std::string & name, const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
    {
        this->node = rclcpp::Node::make_shared("HighPowerCircuitCheck");
    }

    inline auto tick() -> BT::NodeStatus override
    {
        RCLCPP_INFO(this->node->get_logger(), "High power circuit check");
        return BT::NodeStatus::SUCCESS;
    }

    inline static auto providedPorts() -> BT::PortsList { return {}; }
};

class WaitForPowerOnOrder : public BT::StatefulActionNode
{
  private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Time start_time;

  public:
    WaitForPowerOnOrder(const std::string & name, const BT::NodeConfiguration & config)
    : BT::StatefulActionNode(name, config)
    {
        this->node = rclcpp::Node::make_shared("WaitForPowerOnOrder");
    }

    inline auto onStart() -> BT::NodeStatus override
    {
        this->start_time = this->node->now();
        RCLCPP_INFO(this->node->get_logger(), "Wait for power on order");
        return BT::NodeStatus::RUNNING;
    }

    inline auto onRunning() -> BT::NodeStatus override
    {
        const auto duration = this->node->now() - this->start_time;
        if (duration < rclcpp::Duration::from_seconds(5.0)) {
            RCLCPP_INFO_THROTTLE(
              this->node->get_logger(), *this->node->get_clock(), 1000, "Waiting... (%lf)",
              duration.seconds());
            return BT::NodeStatus::RUNNING;
        } else {
            RCLCPP_INFO(this->node->get_logger(), "Received power on order");
            return BT::NodeStatus::SUCCESS;
        }
    }

    inline auto onHalted() -> void override { RCLCPP_INFO(this->node->get_logger(), "Halted"); }

    inline static auto providedPorts() -> BT::PortsList { return {}; }
};

class HealthCheck : public BT::SyncActionNode
{
  private:
    rclcpp::Node::SharedPtr node;

  public:
    HealthCheck(const std::string & name, const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
    {
        this->node = rclcpp::Node::make_shared("HealthCheck");
    }

    inline auto tick() -> BT::NodeStatus override
    {
        RCLCPP_INFO(this->node->get_logger(), "Health check");
        return BT::NodeStatus::SUCCESS;
    }

    inline static auto providedPorts() -> BT::PortsList { return {}; }
};

}  // namespace sinsei_umiusi_core::behavortree::action_node

#endif  // SINSEI_UMIUSI_CORE_BEHAVIOR_TREE_HPP
