#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

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
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
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
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeepRacerJoyControl>());
  rclcpp::shutdown();
  return 0;
}
