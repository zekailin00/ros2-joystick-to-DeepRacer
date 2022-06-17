#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>
#include <iostream>
#include <iomanip>

#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

struct ControllerData {
  std::vector<char> joy_button;
  std::vector<int> joy_axis;
}

class DeepRacerJoyControl : public rclcpp::Node
{
  public:
    DeepRacerJoyControl()
    : Node("joy_ctrl"), count_(0), sensitivity(0.2)
    {
      joystick_fd = open(JOYSTICK_DEV.c_str(), O_RDONLY | O_NONBLOCK);
      if (joystick_fd < 0) 
      {
        RCLCPP_INFO(this->get_logger(), "Joystick error: %d", joystick_fd); 
        exit(1);
      }

      ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
      ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
      ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

      RCLCPP_INFO(this->get_logger(), "Joystick Connected: %s", name_of_joystick); 
      data.joy_button.resize(num_of_buttons, 0);
      data.joy_axis.resize(num_of_axis, 0);

      publisher_ = this->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>("/ctrl_pkg/servo_msg", 10);
      timer_ = this->create_wall_timer(100ms, std::bind(&DeepRacerJoyControl::timer_callback, this));
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"/joy", 10, std::bind(&DeepRacerJoyControl::get_joy_ctrl, this, _1));
    }

  private:
    void get_joy_ctrl(const sensor_msgs::msg::Joy::SharedPtr joy_ctrl)
    {
	    if (joy_ctrl->axes[1] > sensitivity || joy_ctrl->axes[1] < -sensitivity ||
		joy_ctrl->axes[0] > sensitivity || joy_ctrl->axes[0] < -sensitivity)
	    {
		    auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();  
		    servoMsg.angle = joy_ctrl->axes[0];
		    servoMsg.throttle = joy_ctrl->axes[1]; 
		    RCLCPP_INFO(this->get_logger(), "Active Angle: '%f'", servoMsg.angle); 
		    RCLCPP_INFO(this->get_logger(), "Active Throttle: '%f'", servoMsg.throttle); 
		    publisher_->publish(servoMsg); 
	    }
	    else
	    {
		   auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg(); 
       		   servoMsg.angle = 0.0f;  		   
		   servoMsg.throttle = 0.0f;    
		   publisher_->publish(servoMsg);
	    }
    }
    
    void timer_callback()
    {
      count_++;
      js_event js;
      read(joy_fd, &js, sizeof(js_event));

      switch (js.type & ~JS_EVENT_INIT)
      {
      case JS_EVENT_AXIS:
        if((int)js.number>=data.joy_axis.size()) continue;
        data.joy_axis[(int)js.number]= js.value;
        break;
      case JS_EVENT_BUTTON:
        if((int)js.number>=data.joy_button.size()) continue;
        data.joy_button[(int)js.number]= js.value;
        break;
      }

    std::cout << "axis/10000: ";
    for(size_t i(0); i<data.joy_axis.size(); ++i)
      std::cout<<" "<<setw(2)<<data.joy_axis[i]/10000;
    std::cout <<  std::endl;

    std::cout << "  button: ";
    for(size_t i(0); i<data.joy_button.size(); ++i)
      std::cout<<" "<<(int)data.joy_button[i];
    std::cout << std::endl;

    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr publisher_;
    size_t count_;
    float sensitivity;

    std::string JOYSTICK_DEV = "/dev/input/js0";
    ControllerData data;
    int joystick_fd;
    int num_of_axis;
    int  num_of_buttons;
    char name_of_joystick[80];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeepRacerJoyControl>());
  rclcpp::shutdown();
  return 0;
}
