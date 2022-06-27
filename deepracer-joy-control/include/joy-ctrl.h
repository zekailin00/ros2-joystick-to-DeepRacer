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
#include <sstream>

#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <fcntl.h>

#ifndef INT16_MAX
#define INT16_MAX 32767
#endif

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

struct ControllerData {
  std::vector<char> joy_button;
  std::vector<int> joy_axis;
};

class DeepRacerJoyControl : public rclcpp::Node
{
  public:
    DeepRacerJoyControl();
    void get_joy_ctrl(const sensor_msgs::msg::Joy::SharedPtr joy_ctrl);
    void timer_callback();
private:
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