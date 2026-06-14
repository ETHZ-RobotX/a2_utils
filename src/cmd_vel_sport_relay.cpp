#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"

static constexpr int32_t kSportApiMove = 1008;

class CmdVelSportRelay : public rclcpp::Node
{
public:
  CmdVelSportRelay()
  : Node("cmd_vel_sport_relay")
  {
    pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelSportRelay::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "cmd_vel_sport_relay ready: /cmd_vel -> /api/sport/request");
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}",
      msg->linear.x, msg->linear.y, msg->angular.z);

    unitree_api::msg::Request req;
    req.header.identity.api_id = kSportApiMove;
    req.parameter = buf;
    pub_->publish(req);
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSportRelay>());
  rclcpp::shutdown();
  return 0;
}
