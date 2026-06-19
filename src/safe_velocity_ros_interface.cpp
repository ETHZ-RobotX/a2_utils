#include "a2_utils/safe_velocity_ros_interface.hpp"

#include <chrono>
#include <rclcpp/logging.hpp>

namespace {
constexpr std::chrono::milliseconds kControlPeriod{20};  // 50 Hz
}  // namespace

namespace a2 {
namespace utils {

SafeVelocityRosInterface::SafeVelocityRosInterface(float max_vel_x, float max_vel_y,
                                                   float max_yaw_rate,
                                                   std::chrono::milliseconds control_period,
                                                   int64_t cmd_vel_max_age_ns)
    : control_period_(control_period),
      cmd_vel_max_age_ns_(cmd_vel_max_age_ns),
      mode_fsm_(max_vel_x, max_vel_y, max_yaw_rate) {}

void SafeVelocityRosInterface::init(rclcpp::Node* node) {
  node_ = node;
  setupSubscribers();
  setupTimers();
}

void SafeVelocityRosInterface::setupSubscribers() {
  mode_sub_ = node_->create_subscription<a2_interfaces::msg::OperatingMode>(
    "/a2/mode", rclcpp::QoS(10),
    [this](const a2_interfaces::msg::OperatingMode::SharedPtr msg) { modeCallback(msg); });

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel", rclcpp::QoS(1),
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { cmdVelCallback(msg); });
}

void SafeVelocityRosInterface::setupTimers() {
  timer_ = node_->create_timer(control_period_, [this]() { controlLoop(); });
}

void SafeVelocityRosInterface::modeCallback(
  const a2_interfaces::msg::OperatingMode::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!mode_fsm_.mode_transition(static_cast<OpMode>(msg->mode))) {
    RCLCPP_WARN(node_->get_logger(), "Invalid mode transition to %d", msg->mode);
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Mode Requested: %d", msg->mode);
}

void SafeVelocityRosInterface::cmdVelCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  int64_t age_ns = (node_->now() - rclcpp::Time(msg->header.stamp)).nanoseconds();
  if (age_ns > cmd_vel_max_age_ns_) {
    RCLCPP_WARN(node_->get_logger(), "Dropping stale cmd_vel (age %.0f ms)", age_ns / 1e6);
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!mode_fsm_.set_cmd_vel(msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z)) {
    RCLCPP_WARN(node_->get_logger(), "Incompatible op mode, zeroing velocity");
  }
}

void SafeVelocityRosInterface::controlLoop() {
  OpMode mode;
  bool mode_changed;
  std::array<float, 3> vel;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::tie(mode, mode_changed) = mode_fsm_.get_mode();
    vel = mode_fsm_.get_cmd_vel();
  }

  if (mode != OpMode::VELOCITY_MOVE && !mode_changed) {
    return;
  }

  onControl(mode, mode_changed, vel);
}

}  // namespace utils
}  // namespace a2
