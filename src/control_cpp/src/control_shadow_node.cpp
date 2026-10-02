#include "control_cpp/shadow_contract.hpp"

#include <control_interface/msg/control_shadow_decision.hpp>
#include <control_interface/msg/control_shadow_request.hpp>
#include <control_interface/msg/control_shadow_timing.hpp>
#include <control_interface/msg/device_stream.hpp>
#include <control_interface/msg/manager_event.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace control_cpp
{
namespace
{

using namespace std::chrono_literals;

std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

diagnostic_msgs::msg::KeyValue key_value(
  std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

std::string bool_text(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

class ControlShadowNode : public rclcpp::Node
{
public:
  ControlShadowNode()
  : Node("control_shadow"),
    shell_epoch_(static_cast<std::uint64_t>(steady_now_ns()))
  {
    request_rate_hz_ = declare_parameter<double>("request_rate_hz", 15.0);
    heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 100.0);
    diagnostic_rate_hz_ = declare_parameter<double>("diagnostic_rate_hz", 2.0);
    maximum_result_age_s_ = declare_parameter<double>(
      "maximum_result_age_s", 0.20);
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/shape_tracking/markers");
    controller_mode_ = declare_parameter<std::string>(
      "controller_mode", "python_reference");
    configuration_identity_ = declare_parameter<std::string>(
      "configuration_identity", "unknown");
    if (
      request_rate_hz_ <= 0.0 || heartbeat_rate_hz_ <= 0.0 ||
      diagnostic_rate_hz_ <= 0.0 || maximum_result_age_s_ <= 0.0)
    {
      throw std::invalid_argument("shadow rates and result age must be positive");
    }
    maximum_result_age_ns_ = static_cast<std::int64_t>(
      maximum_result_age_s_ * 1.0e9);
    heartbeat_period_ns_ = static_cast<std::int64_t>(
      1.0e9 / heartbeat_rate_hz_);
    next_heartbeat_release_ns_ = steady_now_ns() + heartbeat_period_ns_;

    input_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    request_result_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    heartbeat_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    diagnostic_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    request_pub_ = create_publisher<control_interface::msg::ControlShadowRequest>(
      "/catheter_mppi/shadow_request", rclcpp::QoS(1).reliable());
    timing_pub_ = create_publisher<control_interface::msg::ControlShadowTiming>(
      "/catheter_mppi/shadow_timing", rclcpp::SensorDataQoS());
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/catheter_mppi/shadow_status", rclcpp::QoS(10));

    rclcpp::SubscriptionOptions input_options;
    input_options.callback_group = input_group_;
    device_sub_ = create_subscription<control_interface::msg::DeviceStream>(
      "/device/state", rclcpp::SensorDataQoS(),
      std::bind(&ControlShadowNode::device_callback, this, std::placeholders::_1),
      input_options);
    marker_sub_ = create_subscription<sensor_msgs::msg::PointCloud>(
      marker_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ControlShadowNode::marker_callback, this, std::placeholders::_1),
      input_options);
    manager_sub_ = create_subscription<control_interface::msg::ManagerEvent>(
      "/manager/safety_status", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ControlShadowNode::manager_callback, this, std::placeholders::_1),
      input_options);
    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/catheter_mppi/target_tip", rclcpp::QoS(10),
      std::bind(&ControlShadowNode::target_callback, this, std::placeholders::_1),
      input_options);

    rclcpp::SubscriptionOptions result_options;
    result_options.callback_group = request_result_group_;
    decision_sub_ = create_subscription<
      control_interface::msg::ControlShadowDecision>(
      "/catheter_mppi/shadow_decision", rclcpp::QoS(1).reliable(),
      std::bind(&ControlShadowNode::decision_callback, this, std::placeholders::_1),
      result_options);

    request_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / request_rate_hz_),
      std::bind(&ControlShadowNode::request_tick, this), request_result_group_);
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / heartbeat_rate_hz_),
      std::bind(&ControlShadowNode::heartbeat_tick, this), heartbeat_group_);
    diagnostic_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / diagnostic_rate_hz_),
      std::bind(&ControlShadowNode::diagnostic_tick, this), diagnostic_group_);

    RCLCPP_INFO(
      get_logger(),
      "non-commanding C++ shadow shell started; epoch=%lu; mode=%s",
      static_cast<unsigned long>(shell_epoch_), controller_mode_.c_str());
  }

  rclcpp::CallbackGroup::SharedPtr input_group() const {return input_group_;}
  rclcpp::CallbackGroup::SharedPtr request_result_group() const
  {
    return request_result_group_;
  }
  rclcpp::CallbackGroup::SharedPtr heartbeat_group() const
  {
    return heartbeat_group_;
  }
  rclcpp::CallbackGroup::SharedPtr diagnostic_group() const
  {
    return diagnostic_group_;
  }

private:
  void device_callback(const control_interface::msg::DeviceStream::SharedPtr message)
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    ++device_sequence_;
    device_source_stamp_ns_ = stamp_ns(message->header.stamp);
    device_present_ = true;
  }

  void marker_callback(const sensor_msgs::msg::PointCloud::SharedPtr message)
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    ++marker_sequence_;
    marker_source_stamp_ns_ = stamp_ns(message->header.stamp);
    marker_present_ = true;
  }

  void manager_callback(const control_interface::msg::ManagerEvent::SharedPtr message)
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    ++manager_sequence_;
    manager_source_stamp_ns_ = stamp_ns(message->header.stamp);
    manager_present_ = true;
  }

  void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr)
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    ++target_revision_;
    target_present_ = true;
  }

  void request_tick()
  {
    const auto request_steady_ns = steady_now_ns();
    control_interface::msg::ControlShadowRequest message;
    ShadowRequestRecord record;
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      record.shell_epoch = shell_epoch_;
      record.request_sequence = ++request_sequence_;
      record.target_revision = target_revision_;
      record.device_sequence = device_sequence_;
      record.marker_sequence = marker_sequence_;
      record.manager_sequence = manager_sequence_;
      record.request_steady_ns = request_steady_ns;
      requests_.push_back(record);
      while (requests_.size() > maximum_request_history_) {
        requests_.pop_front();
      }
      message.device_source_stamp_ns = device_source_stamp_ns_;
      message.marker_source_stamp_ns = marker_source_stamp_ns_;
      message.manager_source_stamp_ns = manager_source_stamp_ns_;
      message.device_present = device_present_;
      message.marker_present = marker_present_;
      message.manager_present = manager_present_;
      message.target_present = target_present_;
    }
    message.header.stamp = now();
    message.header.frame_id = "control_shadow";
    message.schema_version = kShadowSchemaVersion;
    message.shell_epoch = record.shell_epoch;
    message.request_sequence = record.request_sequence;
    message.request_steady_ns = record.request_steady_ns;
    message.device_sequence = record.device_sequence;
    message.marker_sequence = record.marker_sequence;
    message.manager_sequence = record.manager_sequence;
    message.target_revision = record.target_revision;
    message.controller_mode = controller_mode_;
    message.configuration_identity = configuration_identity_;
    request_pub_->publish(message);
  }

  void decision_callback(
    const control_interface::msg::ControlShadowDecision::SharedPtr message)
  {
    const auto received_ns = steady_now_ns();
    ShadowDecisionRecord decision;
    decision.schema_version = message->schema_version;
    decision.shell_epoch = message->shell_epoch;
    decision.request_sequence = message->request_sequence;
    decision.target_revision = message->target_revision;
    decision.device_sequence = message->device_sequence;
    decision.marker_sequence = message->marker_sequence;
    decision.manager_sequence = message->manager_sequence;
    decision.computation_start_steady_ns = message->computation_start_steady_ns;
    decision.computation_end_steady_ns = message->computation_end_steady_ns;
    decision.valid = message->valid;
    std::copy(
      message->logical_velocity.begin(), message->logical_velocity.end(),
      decision.logical_velocity.begin());

    std::lock_guard<std::mutex> guard(state_mutex_);
    const auto request_iterator = std::find_if(
      requests_.begin(), requests_.end(),
      [&decision](const ShadowRequestRecord & request) {
        return request.shell_epoch == decision.shell_epoch &&
               request.request_sequence == decision.request_sequence;
      });
    const ShadowRequestRecord * request =
      request_iterator == requests_.end() ? nullptr : &*request_iterator;
    const ShadowValidationContext context{
      shell_epoch_, target_revision_, last_accepted_decision_sequence_,
      received_ns, maximum_result_age_ns_, 1000000};
    decision_reason_ = decision_rejection_reason(decision, request, context);
    latest_result_receive_ns_ = received_ns;
    ++received_decisions_;
    if (decision_reason_ != "accepted") {
      ++rejected_decisions_;
      return;
    }
    last_accepted_decision_sequence_ = decision.request_sequence;
    latest_velocity_ = decision.logical_velocity;
    latest_result_compute_ms_ = 1.0e-6 * static_cast<double>(
      decision.computation_end_steady_ns -
      decision.computation_start_steady_ns);
    latest_request_to_result_ms_ = request == nullptr ? NAN :
      1.0e-6 * static_cast<double>(received_ns - request->request_steady_ns);
    ++accepted_decisions_;
  }

  void heartbeat_tick()
  {
    const auto started_ns = steady_now_ns();
    const auto lateness_ns = std::max<std::int64_t>(
      0, started_ns - next_heartbeat_release_ns_);
    const auto elapsed_periods = std::max<std::int64_t>(
      1, (started_ns - next_heartbeat_release_ns_) / heartbeat_period_ns_ + 1);
    next_heartbeat_release_ns_ += elapsed_periods * heartbeat_period_ns_;

    control_interface::msg::ControlShadowTiming timing;
    timing.header.stamp = now();
    timing.header.frame_id = "control_shadow";
    timing.shell_epoch = shell_epoch_;
    timing.heartbeat_sequence = ++heartbeat_sequence_;
    timing.heartbeat_lateness_ms = 1.0e-6 * lateness_ns;
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      timing.latest_request_sequence = request_sequence_;
      timing.latest_accepted_decision_sequence =
        last_accepted_decision_sequence_;
      timing.latest_result_receive_age_ms = latest_result_receive_ns_ == 0 ?
        NAN : 1.0e-6 * static_cast<double>(started_ns - latest_result_receive_ns_);
      timing.latest_result_compute_ms = latest_result_compute_ms_;
      timing.latest_request_to_result_ms = latest_request_to_result_ms_;
      timing.decision_fresh =
        decision_reason_ == "accepted" && latest_result_receive_ns_ > 0 &&
        started_ns - latest_result_receive_ns_ <= maximum_result_age_ns_;
      timing.decision_reason = timing.decision_fresh ?
        "accepted" : (decision_reason_ == "accepted" ?
        "accepted_result_expired" : decision_reason_);
    }
    timing.heartbeat_callback_ms = 1.0e-6 * static_cast<double>(
      steady_now_ns() - started_ns);
    timing_pub_->publish(timing);
  }

  void diagnostic_tick()
  {
    const auto now_ns = steady_now_ns();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "control/cpp_shadow";
    status.hardware_id = "non_actuating";
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      const bool fresh =
        decision_reason_ == "accepted" && latest_result_receive_ns_ > 0 &&
        now_ns - latest_result_receive_ns_ <= maximum_result_age_ns_;
      status.level = fresh ?
        diagnostic_msgs::msg::DiagnosticStatus::OK :
        diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = fresh ? "SHADOW_TRACKING" : "SHADOW_WAITING";
      status.values = {
        key_value("command_publisher_present", "false"),
        key_value("command_output_enabled", "false"),
        key_value("shell_epoch", std::to_string(shell_epoch_)),
        key_value("request_sequence", std::to_string(request_sequence_)),
        key_value(
          "accepted_decision_sequence",
          std::to_string(last_accepted_decision_sequence_)),
        key_value("decision_reason", decision_reason_),
        key_value("decision_fresh", bool_text(fresh)),
        key_value("received_decisions", std::to_string(received_decisions_)),
        key_value("accepted_decisions", std::to_string(accepted_decisions_)),
        key_value("rejected_decisions", std::to_string(rejected_decisions_)),
        key_value("device_sequence", std::to_string(device_sequence_)),
        key_value("marker_sequence", std::to_string(marker_sequence_)),
        key_value("manager_sequence", std::to_string(manager_sequence_)),
        key_value("target_revision", std::to_string(target_revision_)),
      };
    }
    array.status = {status};
    diagnostic_pub_->publish(array);
  }

  const std::uint64_t shell_epoch_;
  double request_rate_hz_{};
  double heartbeat_rate_hz_{};
  double diagnostic_rate_hz_{};
  double maximum_result_age_s_{};
  std::int64_t maximum_result_age_ns_{};
  std::int64_t heartbeat_period_ns_{};
  std::int64_t next_heartbeat_release_ns_{};
  std::string marker_topic_;
  std::string controller_mode_;
  std::string configuration_identity_;

  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp::CallbackGroup::SharedPtr request_result_group_;
  rclcpp::CallbackGroup::SharedPtr heartbeat_group_;
  rclcpp::CallbackGroup::SharedPtr diagnostic_group_;
  rclcpp::Publisher<control_interface::msg::ControlShadowRequest>::SharedPtr
    request_pub_;
  rclcpp::Publisher<control_interface::msg::ControlShadowTiming>::SharedPtr
    timing_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_pub_;
  rclcpp::Subscription<control_interface::msg::DeviceStream>::SharedPtr
    device_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr marker_sub_;
  rclcpp::Subscription<control_interface::msg::ManagerEvent>::SharedPtr
    manager_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<control_interface::msg::ControlShadowDecision>::SharedPtr
    decision_sub_;
  rclcpp::TimerBase::SharedPtr request_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  std::mutex state_mutex_;
  static constexpr std::size_t maximum_request_history_ = 16;
  std::deque<ShadowRequestRecord> requests_;
  std::uint64_t request_sequence_{};
  std::uint64_t heartbeat_sequence_{};
  std::uint64_t device_sequence_{};
  std::uint64_t marker_sequence_{};
  std::uint64_t manager_sequence_{};
  std::uint64_t target_revision_{};
  std::uint64_t last_accepted_decision_sequence_{};
  std::uint64_t received_decisions_{};
  std::uint64_t accepted_decisions_{};
  std::uint64_t rejected_decisions_{};
  std::int64_t device_source_stamp_ns_{};
  std::int64_t marker_source_stamp_ns_{};
  std::int64_t manager_source_stamp_ns_{};
  std::int64_t latest_result_receive_ns_{};
  double latest_result_compute_ms_{NAN};
  double latest_request_to_result_ms_{NAN};
  bool device_present_{};
  bool marker_present_{};
  bool manager_present_{};
  bool target_present_{};
  std::string decision_reason_{"decision_unavailable"};
  std::array<double, 6> latest_velocity_{};
};

}  // namespace control_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<control_cpp::ControlShadowNode>();
  auto base = node->get_node_base_interface();

  rclcpp::executors::SingleThreadedExecutor heartbeat_executor;
  rclcpp::executors::SingleThreadedExecutor input_executor;
  rclcpp::executors::SingleThreadedExecutor request_result_executor;
  rclcpp::executors::SingleThreadedExecutor diagnostic_executor;
  heartbeat_executor.add_callback_group(node->heartbeat_group(), base);
  input_executor.add_callback_group(node->input_group(), base);
  request_result_executor.add_callback_group(node->request_result_group(), base);
  diagnostic_executor.add_callback_group(node->diagnostic_group(), base);
  diagnostic_executor.add_callback_group(node->get_node_base_interface()->get_default_callback_group(), base);

  std::thread heartbeat_thread([&heartbeat_executor]() {heartbeat_executor.spin();});
  std::thread input_thread([&input_executor]() {input_executor.spin();});
  std::thread request_result_thread(
    [&request_result_executor]() {request_result_executor.spin();});
  diagnostic_executor.spin();

  heartbeat_executor.cancel();
  input_executor.cancel();
  request_result_executor.cancel();
  heartbeat_thread.join();
  input_thread.join();
  request_result_thread.join();
  rclcpp::shutdown();
  return 0;
}
