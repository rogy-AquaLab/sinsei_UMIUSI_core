#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "sinsei_umiusi_core/behaviortree/sample.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("behaviortree_runner");

    auto factory = BT::BehaviorTreeFactory();
    factory.registerNodeType<sinsei_umiusi_core::behavortree::sample::PublishDisplayInfo>(
      "PublishDisplayInfo", node);
    factory.registerNodeType<sinsei_umiusi_core::behavortree::sample::LowPowerCircuitCheck>(
      "LowPowerCircuitCheck", node);
    factory.registerNodeType<sinsei_umiusi_core::behavortree::sample::HighPowerCircuitCheck>(
      "HighPowerCircuitCheck", node);
    factory.registerNodeType<sinsei_umiusi_core::behavortree::sample::PowerOff>("PowerOff", node);
    factory.registerNodeType<sinsei_umiusi_core::behavortree::sample::HealthCheck>(
      "HealthCheck", node);

    auto tree = factory.createTreeFromFile("/home/ubuntu/ros_ws/src/pkg/behaviortree/main.xml");

    auto _groot2_publisher = BT::Groot2Publisher(tree);  // For Groot monitoring (宣言するだけでOK)

    auto status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING) {
        if (!rclcpp::ok()) {
            RCLCPP_WARN(node->get_logger(), "rclcpp has been shutdown. Exiting...");
            tree.haltTree();
            break;
        }
        rclcpp::spin_some(node);
        status = tree.tickOnce();
    }
    rclcpp::shutdown();

    return 0;
}
