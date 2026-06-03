#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "unitree_hg/msg/low_state.hpp"

class ImuPub : public rclcpp::Node
{
public:
  ImuPub()
  : Node("imu_pub")
  {
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    sub_ = create_subscription<unitree_hg::msg::LowState>(
      "/lowstate", 10,
      std::bind(&ImuPub::lowstate_callback, this, std::placeholders::_1));
  }

private:
  void lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg)
  {
    // Throttle to 200 Hz — fast enough for DLIO IMU deskewing.
    if ((this->now() - last_pub_).seconds() < 0.005) return;
    last_pub_ = this->now();

    auto out = sensor_msgs::msg::Imu();
    out.header.stamp    = this->now();
    out.header.frame_id = "imu_link";
    out.orientation.w = msg->imu_state.quaternion[0];
    out.orientation.x = msg->imu_state.quaternion[1];
    out.orientation.y = msg->imu_state.quaternion[2];
    out.orientation.z = msg->imu_state.quaternion[3];
    out.angular_velocity.x = msg->imu_state.gyroscope[0];
    out.angular_velocity.y = msg->imu_state.gyroscope[1];
    out.angular_velocity.z = msg->imu_state.gyroscope[2];
    out.linear_acceleration.x = msg->imu_state.accelerometer[0];
    out.linear_acceleration.y = msg->imu_state.accelerometer[1];
    out.linear_acceleration.z = msg->imu_state.accelerometer[2];
    pub_->publish(out);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr sub_;
  rclcpp::Time last_pub_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPub>());
  rclcpp::shutdown();
  return 0;
}
