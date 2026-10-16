#include "sinsei_umiusi_core/robot_strategy/auto.hpp"

#include <lifecycle_msgs/msg/state.hpp>

using namespace std::chrono_literals;

namespace
{

// ~/status がこれだけ来なければ「止まった」とみなし、推力 0 を出して立て直す
constexpr auto STATUS_TIMEOUT = 1s;
// 立て直しの要求に答えが来ないとき、次を出すまでの間
constexpr auto RECOVER_RETRY = 3s;
// Standby に戻すとき、auto_target_generator の答えを待つ上限（落ちていても BT を止めない）
constexpr auto HALT_TIMEOUT = 2s;

}  // namespace

sinsei_umiusi_core::robot_strategy::Auto::Auto(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode{name, config},
  ros_node{nullptr},
  change_state_target_generator_clt{nullptr},
  get_state_target_generator_clt{nullptr},
  target_generator_params_clt{nullptr},
  target_generator_status_sub{nullptr},
  thruster_runnable_pub{nullptr},
  robot_state_auto_msg{
    sinsei_umiusi_msgs::msg::RobotState{}.set__state(sinsei_umiusi_msgs::msg::RobotState::AUTO)},
  thruster_all_runnable_msg{
    sinsei_umiusi_msgs::msg::ThrusterRunnableAll{}
      .set__lf(sinsei_umiusi_msgs::msg::ThrusterRunnable{}.set__esc(true).set__servo(true))
      .set__lb(sinsei_umiusi_msgs::msg::ThrusterRunnable{}.set__esc(true).set__servo(true))
      .set__rf(sinsei_umiusi_msgs::msg::ThrusterRunnable{}.set__esc(true).set__servo(true))
      .set__rb(sinsei_umiusi_msgs::msg::ThrusterRunnable{}.set__esc(true).set__servo(true))},
  recovering{false}
{
    this->ros_node = rclcpp::Node::make_shared("_bt_auto");
    this->change_state_target_generator_clt =
      this->ros_node->create_client<lifecycle_msgs::srv::ChangeState>(
        "/auto_target_generator/change_state");
    this->get_state_target_generator_clt =
      this->ros_node->create_client<lifecycle_msgs::srv::GetState>(
        "/auto_target_generator/get_state");
    this->target_generator_params_clt =
      std::make_shared<rclcpp::AsyncParametersClient>(this->ros_node, "/auto_target_generator");
    this->target_generator_status_sub = this->ros_node->create_subscription<std_msgs::msg::String>(
      "/auto_target_generator/status", rclcpp::SystemDefaultsQoS{},
      [this](const std_msgs::msg::String::SharedPtr msg) {
          // Standby の間の認識だけの表示（state PREVIEW）は、走っている証拠にしない
          if (msg->data.find("\"state\": \"PREVIEW\"") == std::string::npos) {
              this->last_status_at = Clock::now();
          }
      });
    this->thruster_runnable_pub =
      this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::ThrusterRunnableAll>(
        "/cmd/thruster_runnable_all", rclcpp::SystemDefaultsQoS{});
    this->robot_state_pub = this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::RobotState>(
      "/robot_state", rclcpp::SystemDefaultsQoS{});
    this->target_pub = this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::Target>(
      "/cmd/target", rclcpp::SystemDefaultsQoS{});
    this->attitude_target_pub =
      this->ros_node->create_publisher<sinsei_umiusi_msgs::msg::AttitudeTarget>(
        "/cmd/attitude_target", rclcpp::SystemDefaultsQoS{});
}

auto sinsei_umiusi_core::robot_strategy::Auto::onStart() -> BT::NodeStatus
{
    this->started_at = Clock::now();
    this->last_status_at = this->started_at;
    this->recovering = false;

    this->thruster_runnable_pub->publish(this->thruster_all_runnable_msg);

    // activate も立て直しと同じ道で行う（落ちていて答えが無くても BT を止めない）
    this->recover();
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_strategy::Auto::onRunning() -> BT::NodeStatus
{
    rclcpp::spin_some(this->ros_node);

    this->robot_state_pub->publish(this->robot_state_auto_msg);
    this->thruster_runnable_pub->publish(this->thruster_all_runnable_msg);

    const auto now = Clock::now();
    if (now - this->last_status_at > STATUS_TIMEOUT) {
        // 最後の指令（突撃中の前進など）が出たままにならないよう、こちらで推力 0 を出し続ける
        this->publish_zero_target();
        RCLCPP_WARN_THROTTLE(
          this->ros_node->get_logger(), *this->ros_node->get_clock(), 2000,
          "auto_target_generator is silent. Publishing zero target and recovering it.");
        if (!this->recovering || now - this->recover_requested_at > RECOVER_RETRY) {
            this->recover();
        }
    }
    return BT::NodeStatus::RUNNING;
}

auto sinsei_umiusi_core::robot_strategy::Auto::onHalted() -> void
{
    this->recovering = false;

    // Request to deactivate auto target generator
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->set__transition(lifecycle_msgs::msg::Transition{}.set__id(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));

    this->thruster_runnable_pub->publish(
      sinsei_umiusi_msgs::msg::ThrusterRunnableAll{}
        .set__lf(sinsei_umiusi_msgs::msg::ThrusterRunnable{}.set__esc(false).set__servo(false))
        .set__lb(sinsei_umiusi_msgs::msg::ThrusterRunnable{}.set__esc(false).set__servo(false))
        .set__rf(sinsei_umiusi_msgs::msg::ThrusterRunnable{}.set__esc(false).set__servo(false))
        .set__rb(sinsei_umiusi_msgs::msg::ThrusterRunnable{}.set__esc(false).set__servo(false)));
    this->publish_zero_target();

    if (!this->change_state_target_generator_clt->wait_for_service(HALT_TIMEOUT)) {
        RCLCPP_ERROR(
          this->ros_node->get_logger(), "auto_target_generator is not available. Skip deactivate.");
        return;
    }
    auto future = this->change_state_target_generator_clt->async_send_request(req);

    // Wait for the result.
    auto result = rclcpp::spin_until_future_complete(this->ros_node, future, HALT_TIMEOUT);
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->ros_node->get_logger(), "Auto target generator deactivated.");
        } else {
            RCLCPP_ERROR(
              this->ros_node->get_logger(), "Failed to deactivate auto target generator.");
        }
    } else {
        RCLCPP_ERROR(
          this->ros_node->get_logger(), "Service call to deactivate auto target generator failed.");
    }
}

auto sinsei_umiusi_core::robot_strategy::Auto::recover() -> void
{
    this->recovering = true;
    this->recover_requested_at = Clock::now();
    if (!this->get_state_target_generator_clt->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
          this->ros_node->get_logger(), *this->ros_node->get_clock(), 2000,
          "auto_target_generator is not available yet.");
        return;  // RECOVER_RETRY 後にもう一度
    }
    const auto requested_at = this->recover_requested_at;
    this->get_state_target_generator_clt->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
      [this, requested_at](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
          if (!this->recovering || this->recover_requested_at != requested_at) {
              return;  // Standby に戻った / 次の立て直しが始まった
          }
          const auto state = future.get()->current_state.id;
          using lifecycle_msgs::msg::State;
          using lifecycle_msgs::msg::Transition;
          // 途中で次の立て直しが始まったら、古い方は手を引く（activate を二重に送らない）
          auto stale = [this, requested_at]() {
              return !this->recovering || this->recover_requested_at != requested_at;
          };
          auto activate = [this, stale](bool ok) {
              if (!ok || stale()) {
                  return;
              }
              this->set_run_started_at([this, stale]() {
                  if (stale()) {
                      return;
                  }
                  this->change_state_async(Transition::TRANSITION_ACTIVATE, [this, stale](bool ok) {
                      RCLCPP_INFO(
                        this->ros_node->get_logger(), "Activate auto target generator: %s",
                        ok ? "ok" : "failed");
                      if (!stale()) {
                          this->recovering = !ok;
                      }
                  });
              });
          };
          if (state == State::PRIMARY_STATE_UNCONFIGURED) {
              RCLCPP_WARN(
                this->ros_node->get_logger(),
                "auto_target_generator was restarted. Configure and activate it again.");
              this->change_state_async(Transition::TRANSITION_CONFIGURE, activate);
          } else if (state == State::PRIMARY_STATE_INACTIVE) {
              activate(true);
          } else {
              // active なのに黙っている（固まっている）。推力 0 を出し続けて待つ
              this->recovering = false;
          }
      });
}

auto sinsei_umiusi_core::robot_strategy::Auto::change_state_async(
  uint8_t transition, std::function<void(bool)> done) -> void
{
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->set__transition(lifecycle_msgs::msg::Transition{}.set__id(transition));
    this->change_state_target_generator_clt->async_send_request(
      req, [done](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
          done(future.get()->success);
      });
}

auto sinsei_umiusi_core::robot_strategy::Auto::set_run_started_at(std::function<void()> then)
  -> void
{
    // activate の前に、この Auto に入った時刻（CLOCK_MONOTONIC の秒。Python の time.monotonic と同じ）を渡す。
    // 立ち上がり直したノードは、ここから持ち時間を数え続ける。渡せなくても activate はする（その時刻から数える）
    const double t = std::chrono::duration<double>(this->started_at.time_since_epoch()).count();
    this->target_generator_params_clt->set_parameters(
      {rclcpp::Parameter("run_started_at", t)},
      [this, then](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
          const auto results = f.get();
          if (results.empty() || !results[0].successful) {
              RCLCPP_WARN(this->ros_node->get_logger(), "Failed to set run_started_at.");
          }
          then();
      });
}

auto sinsei_umiusi_core::robot_strategy::Auto::publish_zero_target() -> void
{
    this->target_pub->publish(sinsei_umiusi_msgs::msg::Target{});
    auto attitude = sinsei_umiusi_msgs::msg::AttitudeTarget{};
    attitude.attitude.w = 1.0;  // roll / pitch は水平、yaw_rate 0
    this->attitude_target_pub->publish(attitude);
}
