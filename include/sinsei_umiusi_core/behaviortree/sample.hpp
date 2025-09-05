// TODO: 実装サンプル (将来的に消す)

#ifndef SINSEI_UMIUSI_CORE_BEHAVIORTREE_SAMPLE_HPP
#define SINSEI_UMIUSI_CORE_BEHAVIORTREE_SAMPLE_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "sinsei_umiusi_core/behaviortree/ros_stateful_action_node.hpp"
#include "sinsei_umiusi_core/behaviortree/ros_sync_action_node.hpp"

namespace sinsei_umiusi_core::behavortree::sample
{

class PublishDisplayInfo : public RosSyncActionNode
{
  private:
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher;

  public:
    PublishDisplayInfo(
      const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
    : RosSyncActionNode(name, config, node)
    {
        this->publisher =
          this->node->create_publisher<std_msgs::msg::Empty>("display_info", rclcpp::ServicesQoS());
    }
    virtual ~PublishDisplayInfo() = default;

    inline static auto providedPorts() -> BT::PortsList { return {}; }

    inline virtual auto tick() -> BT::NodeStatus override
    {
        this->publisher->publish(std_msgs::msg::Empty());
        RCLCPP_INFO_THROTTLE(
          this->node->get_logger(), *this->node->get_clock(), 1000,
          "Display information published");
        return BT::NodeStatus::SUCCESS;
    }
};

class LowPowerCircuitCheck : public RosSyncActionNode
{
  private:
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher;

  public:
    LowPowerCircuitCheck(
      const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
    : RosSyncActionNode(name, config, node)
    {
        this->publisher = this->node->create_publisher<std_msgs::msg::Empty>(
          "low_power_check", rclcpp::ServicesQoS());
    }
    virtual ~LowPowerCircuitCheck() = default;

    inline static auto providedPorts() -> BT::PortsList { return {}; }

    inline virtual auto tick() -> BT::NodeStatus override
    {
        this->publisher->publish(std_msgs::msg::Empty());
        RCLCPP_INFO_THROTTLE(
          this->node->get_logger(), *this->node->get_clock(), 1000, "Run low power circuit check");
        return BT::NodeStatus::SUCCESS;
    }
};

class HighPowerCircuitCheck : public RosSyncActionNode
{
  private:
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher;

  public:
    HighPowerCircuitCheck(
      const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
    : RosSyncActionNode(name, config, node)
    {
        this->publisher = this->node->create_publisher<std_msgs::msg::Empty>(
          "high_power_check", rclcpp::ServicesQoS());
    }
    virtual ~HighPowerCircuitCheck() = default;

    inline static auto providedPorts() -> BT::PortsList { return {}; }

    inline virtual auto tick() -> BT::NodeStatus override
    {
        this->publisher->publish(std_msgs::msg::Empty());
        RCLCPP_INFO_THROTTLE(
          this->node->get_logger(), *this->node->get_clock(), 1000, "Run high power circuit check");
        return BT::NodeStatus::SUCCESS;
    }
};

class PowerOff : public RosStatefulActionNode
{
  private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    bool recieved;

  public:
    PowerOff(
      const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
    : RosStatefulActionNode(name, config, node), recieved(false)
    {
    }
    virtual ~PowerOff() = default;

    inline static auto providedPorts() -> BT::PortsList { return {}; }

    inline virtual auto onStart() -> BT::NodeStatus override
    {
        this->recieved = false;
        this->subscription = this->node->create_subscription<std_msgs::msg::Empty>(
          "power_on", rclcpp::SystemDefaultsQoS(),
          [this](const std_msgs::msg::Empty & _) { this->recieved = true; });
        this->publisher =
          this->node->create_publisher<std_msgs::msg::String>("state", rclcpp::ServicesQoS());

        return BT::NodeStatus::RUNNING;
    }

    inline virtual auto onRunning() -> BT::NodeStatus override
    {
        if (this->recieved) {
            RCLCPP_INFO_ONCE(this->node->get_logger(), "Power on order received.");
            return BT::NodeStatus::SUCCESS;
        }
        this->publisher->publish(std_msgs::msg::String().set__data("POWER_OFF"));
        RCLCPP_INFO_THROTTLE(
          this->node->get_logger(), *this->node->get_clock(), 1000,
          "Waiting for power on order... (State: POWER_OFF)");
        return BT::NodeStatus::RUNNING;
    }

    inline virtual void onHalted() override
    {
        this->subscription.reset();
        RCLCPP_WARN(this->node->get_logger(), "PowerOff halted.");
    }
};

class HealthCheck : public RosSyncActionNode
{
  public:
    HealthCheck(
      const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
    : RosSyncActionNode(name, config, node)
    {
    }
    virtual ~HealthCheck() = default;

    inline static auto providedPorts() -> BT::PortsList { return {}; }

    inline virtual auto tick() -> BT::NodeStatus override
    {
        RCLCPP_INFO_ONCE(this->node->get_logger(), "Health check passed.");
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace sinsei_umiusi_core::behavortree::sample

#endif  // SINSEI_UMIUSI_CORE_BEHAVIORTREE_SAMPLE_HPP
