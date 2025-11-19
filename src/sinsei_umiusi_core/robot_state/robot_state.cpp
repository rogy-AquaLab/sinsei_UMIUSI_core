#include "sinsei_umiusi_core/robot_state/robot_state.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <functional>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "sinsei_umiusi_core/robot_state/power_off.hpp"

using namespace std::chrono_literals;

sinsei_umiusi_core::robot_state::RobotState::RobotState()
: rclcpp::Node("robot_state"), timer(nullptr), tree(nullptr)
{
    this->declare_parameter(
      std::string(PARAM_NAME_BEHAVIOR_TREE_FILE), "unknown",
      rcl_interfaces::msg::ParameterDescriptor()
        .set__description("BehaviorTree file (*.xml)")
        .set__read_only(true)
        .set__type(rclcpp::ParameterType::PARAMETER_STRING));

    {
        auto factory = BT::BehaviorTreeFactory();
        // factory.registerNodeType<YourCustomNode>("YourCustomNode");
        factory.registerNodeType<sinsei_umiusi_core::robot_state::PowerOff>("PowerOff");
        auto tree = factory.createTreeFromFile(
          this->get_parameter(std::string(PARAM_NAME_BEHAVIOR_TREE_FILE)).as_string());
        this->tree = std::make_unique<BT::Tree>(std::move(tree));
    }
    this->change_state_manual_target_generator =
      this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/manual_target_generator/change_state");
    this->change_state_auto_target_generator =
      this->create_client<lifecycle_msgs::srv::ChangeState>("/auto_target_generator/change_state");
    {
        // Request to configure managed nodes
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

        this->change_state_manual_target_generator->wait_for_service();
        this->change_state_auto_target_generator->wait_for_service();
        // this->change_state_debug_thruster_output->wait_for_service();

        auto future_manual = this->change_state_manual_target_generator->async_send_request(req);
        auto future_auto = this->change_state_auto_target_generator->async_send_request(req);
        // auto future_debug = this->change_state_debug_thruster_output->async_send_request(req);

        auto result_manual =
          rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_manual);
        auto result_auto =
          rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_auto);
        // auto result_debug =
        //   rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_debug);
        if (
          result_manual != rclcpp::FutureReturnCode::SUCCESS ||
          result_auto != rclcpp::FutureReturnCode::SUCCESS
          // || result_debug != rclcpp::FutureReturnCode::SUCCESS
        ) {
            RCLCPP_ERROR(
              this->get_logger(), "Failed to call service change_state on manual_target_generator");
        }
    }
    this->timer = this->create_wall_timer(1s, std::bind(&RobotState::timer_callback, this));
}

auto sinsei_umiusi_core::robot_state::RobotState::timer_callback() const -> void
{
    if (!tree) {
        RCLCPP_WARN(this->get_logger(), "Behavior tree is not initialized.");
        return;
    }
    BT::NodeStatus status = tree->rootNode()->executeTick();
    switch (status) {
        case BT::NodeStatus::SUCCESS:
            RCLCPP_INFO(this->get_logger(), "Behavior tree finished with SUCCESS.");
            break;
        case BT::NodeStatus::FAILURE:
            RCLCPP_ERROR(this->get_logger(), "Behavior tree finished with FAILURE.");
            break;
        default:
            break;
    }
}
