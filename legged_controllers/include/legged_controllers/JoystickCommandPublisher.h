//
// Created by chenzheng on 23-6-9.
//

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace legged {

class JoystickCommandPublisher {
 public:
  explicit JoystickCommandPublisher(ros::NodeHandle nodeHandle);

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  bool risingEdgeTrigger(int button, double value);

  double linear_vel_scale_, angular_vel_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher status_pub_;
  ros::Subscriber joy_sub_;
  double last_joy_axes_[8];
  double last_joy_buttons_[11];
  bool locomotion_enable_{};
  int stage_{};
  std::string gait_{};

};
}