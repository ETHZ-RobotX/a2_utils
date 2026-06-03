#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

class JointStatesPub : public rclcpp::Node
{
public:
  JointStatesPub()
  : Node("joint_states_pub")
  {
    pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    sub_ = create_subscription<unitree_hg::msg::LowState>(
      "/lowstate", 10,
      std::bind(&JointStatesPub::lowstate_callback, this, std::placeholders::_1));

    joint_names_ = {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
    };
  }

private:
  void lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg)
  {
    // Throttle to 200 Hz — matches robot state update rate.
    if ((this->now() - last_pub_).seconds() < 0.005) return;
    last_pub_ = this->now();

    auto out = sensor_msgs::msg::JointState();
    out.header.stamp = this->now();
    out.name = joint_names_;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      out.position.push_back(msg->motor_state[i].q);
      out.velocity.push_back(msg->motor_state[i].dq);
      out.effort.push_back(msg->motor_state[i].tau_est);
    }
    pub_->publish(out);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr sub_;
  std::vector<std::string> joint_names_;
  rclcpp::Time last_pub_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatesPub>());
  rclcpp::shutdown();
  return 0;
}
