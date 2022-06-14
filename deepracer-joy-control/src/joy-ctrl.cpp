#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DeepRacerJoyControl : public rclcpp::Node
{
  public:
    DeepRacerJoyControl()
    : Node("joy_ctrl"), count_(0)
    {
      publisher_ = this->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>("/ctrl_pkg/servo_msg", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&DeepRacerJoyControl::timer_callback, this));
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"/joy", 10, std::bind(&DeepRacerJoyControl::get_joy_ctrl, this, _1));
    }

  private:
    void get_joy_ctrl(const sensor_msgs::msg::Joy::SharedPtr joy_ctrl)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", joy_ctrl->axes[1]);
    }
    
    void timer_callback()
    {
      count_++;
      auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
      servoMsg.angle = (((count_%2) == 0)? -1.0:1.0);
      servoMsg.throttle = 0.1;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", servoMsg.angle);
      publisher_->publish(servoMsg);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeepRacerJoyControl>());
  rclcpp::shutdown();
  return 0;
}
