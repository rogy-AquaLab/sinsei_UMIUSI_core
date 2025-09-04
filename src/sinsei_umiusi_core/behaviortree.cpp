#include "sinsei_umiusi_core/behaviortree.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto factory = BT::BehaviorTreeFactory();
    factory.registerNodeType<sinsei_umiusi_core::behavortree::action_node::PublishDisplayInfo>(
      "PublishDisplayInfo");
    factory.registerNodeType<sinsei_umiusi_core::behavortree::action_node::LowPowerCircuitCheck>(
      "LowPowerCircuitCheck");
    factory.registerNodeType<sinsei_umiusi_core::behavortree::action_node::HighPowerCircuitCheck>(
      "HighPowerCircuitCheck");
    factory.registerNodeType<sinsei_umiusi_core::behavortree::action_node::WaitForPowerOnOrder>(
      "WaitForPowerOnOrder");
    factory.registerNodeType<sinsei_umiusi_core::behavortree::action_node::HealthCheck>(
      "HealthCheck");
    auto tree = factory.createTreeFromFile("/home/ubuntu/ros_ws/src/pkg/behaviortree/main.xml");
    auto _groot2_publisher = BT::Groot2Publisher(tree);  // For Groot monitoring (宣言するだけでOK)

    auto status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickOnce();
    }
    tree.haltTree();
    rclcpp::shutdown();
    return 0;
}
