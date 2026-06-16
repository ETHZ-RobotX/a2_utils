#ifndef A2_UTILS_SAFE_VELOCITY_ROS_INTERFACE_H_
#define A2_UTILS_SAFE_VELOCITY_ROS_INTERFACE_H_

#include <array>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "a2_interfaces/msg/operating_mode.hpp"
#include "a2_utils/mode_fsm.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace a2 {
namespace utils {

/*
 * ROS interface for the mode FSM. Owns /a2/mode and /cmd_vel subscriptions,
 * a 50 Hz control timer, and the underlying ModeFsm + mutex.
 *
 * Subclasses implement onControl() to drive hardware or sim — topic names,
 * FSM logic, and cmd_vel age rejection are shared automatically.
 */
class SafeVelocityRosInterface {
public:
  SafeVelocityRosInterface(float max_vel_x, float max_vel_y, float max_yaw_rate,
                           std::chrono::milliseconds control_period, int64_t cmd_vel_max_age_ns);
  virtual ~SafeVelocityRosInterface() = default;

  void init(rclcpp::Node* node);

protected:
  // Called at 50 Hz with the current FSM state. mode_changed is true on the
  // first tick after a mode transition.
  virtual void onControl(OpMode mode, bool mode_changed, std::array<float, 3> vel) = 0;

  // Exposed so subclasses can use node_ for logging.
  rclcpp::Node* node_{nullptr};

private:
  void setupSubscribers();
  void setupTimers();
  void modeCallback(const a2_interfaces::msg::OperatingMode::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void controlLoop();

private:
  const std::chrono::milliseconds control_period_;
  const int64_t cmd_vel_max_age_ns_;
  ModeFsm mode_fsm_;
  std::mutex state_mutex_;
  rclcpp::Subscription<a2_interfaces::msg::OperatingMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace utils
}  // namespace a2

#endif /* A2_UTILS_SAFE_VELOCITY_ROS_INTERFACE_H_ */
