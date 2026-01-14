#include "sinsei_umiusi_core/robot_strategy/core.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <functional>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "sinsei_umiusi_core/robot_strategy/auto.hpp"
#include "sinsei_umiusi_core/robot_strategy/debug.hpp"
#include "sinsei_umiusi_core/robot_strategy/manual.hpp"
#include "sinsei_umiusi_core/robot_strategy/mode_control.hpp"
#include "sinsei_umiusi_core/robot_strategy/power_off.hpp"
#include "sinsei_umiusi_core/robot_strategy/power_on.hpp"
#include "sinsei_umiusi_core/robot_strategy/should_power_on.hpp"
#include "sinsei_umiusi_core/robot_strategy/standby.hpp"

using namespace std::chrono_literals;

sinsei_umiusi_core::robot_strategy::Core::Core()
: rclcpp::Node{"core"}, timer{nullptr}, tree{nullptr}, groot2_publisher{std::nullopt}
{
    this->declare_parameter(
      std::string(PARAM_NAME_BEHAVIOR_TREE_FILE), "",
      rcl_interfaces::msg::ParameterDescriptor()
        .set__description("BehaviorTree file (*.xml)")
        .set__read_only(true)
        .set__type(rclcpp::ParameterType::PARAMETER_STRING));

    {
        auto factory = BT::BehaviorTreeFactory();
        factory.registerNodeType<sinsei_umiusi_core::robot_strategy::ShouldPowerOn>("ShouldPowerOn");
        factory.registerNodeType<sinsei_umiusi_core::robot_strategy::PowerOn>("PowerOn");
        factory.registerNodeType<sinsei_umiusi_core::robot_strategy::ModeControl>("ModeControl");
        factory.registerNodeType<sinsei_umiusi_core::robot_strategy::Standby>("Standby");
        factory.registerNodeType<sinsei_umiusi_core::robot_strategy::Manual>("Manual");
        factory.registerNodeType<sinsei_umiusi_core::robot_strategy::Auto>("Auto");
        factory.registerNodeType<sinsei_umiusi_core::robot_strategy::Debug>("Debug");
        factory.registerNodeType<sinsei_umiusi_core::robot_strategy::PowerOff>("PowerOff");
        auto tree = factory.createTreeFromFile(
          this->get_parameter(std::string(PARAM_NAME_BEHAVIOR_TREE_FILE)).as_string());
        this->groot2_publisher.emplace(tree, 1667);
        this->tree = std::make_unique<BT::Tree>(std::move(tree));
    }

    this->change_state_manual_clt = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "/manual_target_generator/change_state");
    this->change_state_auto_clt =
      this->create_client<lifecycle_msgs::srv::ChangeState>("/auto_target_generator/change_state");
    {
        // Request to configure managed nodes
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

        this->change_state_manual_clt->wait_for_service();
        this->change_state_auto_clt->wait_for_service();
        // this->change_state_debug_clt->wait_for_service();

        auto future_manual = this->change_state_manual_clt->async_send_request(req);
        auto future_auto = this->change_state_auto_clt->async_send_request(req);
        // auto future_debug = this->change_state_debug_clt->async_send_request(req);

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
    this->timer = this->create_wall_timer(100ms, std::bind(&Core::timer_callback, this));
}

auto sinsei_umiusi_core::robot_strategy::Core::timer_callback() const -> void
{
    if (!this->tree) {
        RCLCPP_WARN(this->get_logger(), "Behavior tree is not initialized.");
        return;
    }
    auto status = this->tree->tickExactlyOnce();
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
