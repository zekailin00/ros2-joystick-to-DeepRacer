#include "joy-ctrl.h"

    DeepRacerJoyControl::DeepRacerJoyControl()
    : Node("joy_ctrl"), count_(0), sensitivity(0.2)
    {
      joystick_fd = open(JOYSTICK_DEV.c_str(), O_RDONLY | O_NONBLOCK);
      if (joystick_fd < 0) 
      {
        RCLCPP_INFO(this->get_logger(), "Joystick error: %d", joystick_fd); 
        exit(1);
      }

      ioctl(joystick_fd, JSIOCGAXES, &num_of_axis);
      ioctl(joystick_fd, JSIOCGBUTTONS, &num_of_buttons);
      ioctl(joystick_fd, JSIOCGNAME(80), &name_of_joystick);
 
      data.joy_button.resize(num_of_buttons, 0);
      data.joy_axis.resize(num_of_axis, 0);

      RCLCPP_INFO(this->get_logger(), "Joystick Connected: %s", name_of_joystick);
      RCLCPP_INFO(this->get_logger(), "Joystick Number of Axes: %d", num_of_axis);
      RCLCPP_INFO(this->get_logger(), "Joystick Number of Buttons: %d", num_of_buttons);

      publisher_ = this->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>("/ctrl_pkg/servo_msg", 10);
      timer_ = this->create_wall_timer(5ms, std::bind(&DeepRacerJoyControl::timer_callback, this));
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"/joy", 10, std::bind(&DeepRacerJoyControl::get_joy_ctrl, this, _1));
    }

    void DeepRacerJoyControl::get_joy_ctrl(const sensor_msgs::msg::Joy::SharedPtr joy_ctrl)
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
    
    void DeepRacerJoyControl::timer_callback()
    {
      count_++;
      js_event js;
      read(joystick_fd, &js, sizeof(js_event));

      switch (js.type & ~JS_EVENT_INIT)
      {
      case JS_EVENT_AXIS:
        if(!((int)js.number>=data.joy_axis.size()))
          data.joy_axis[(int)js.number]= js.value;
        break;
      case JS_EVENT_BUTTON:
        if(!((int)js.number>=data.joy_button.size()))
          data.joy_button[(int)js.number]= js.value;
        break;
      }

      std::stringstream buffer;

      buffer << "axis/10000: ";
      for(size_t i(0); i<data.joy_axis.size(); ++i)
        buffer << " " 
               << std::setw(2)
               << static_cast<float>(data.joy_axis[i])/INT16_MAX;
               
      buffer <<  std::endl;

      buffer << "  button: ";
      for(size_t i(0); i<data.joy_button.size(); ++i)
        buffer << " " 
               << (int)data.joy_button[i];

      buffer << std::endl;
      
      //RCLCPP_INFO(this->get_logger(), "Message %d: " + buffer.str(), count_);

      auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();  
		  servoMsg.angle = -(static_cast<float>(data.joy_axis[0])/INT16_MAX);
		  servoMsg.throttle = -(static_cast<float>(data.joy_axis[1])/INT16_MAX); 
		  RCLCPP_INFO(this->get_logger(), "Active Angle: '%f'", servoMsg.angle); 
		  RCLCPP_INFO(this->get_logger(), "Active Throttle: '%f'", servoMsg.throttle); 
		  publisher_->publish(servoMsg);

    }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeepRacerJoyControl>());
  rclcpp::shutdown();
  return 0;
}
