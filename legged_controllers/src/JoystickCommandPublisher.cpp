//
// Created by chenzheng on 2023/6/9.
//

#include "legged_controllers/JoystickCommandPublisher.h"
#include "legged_controllers/status_command.h"

#include <geometry_msgs/Twist.h>

using namespace legged;

JoystickCommandPublisher::JoystickCommandPublisher(ros::NodeHandle nodeHandle) {
  nodeHandle.param("linear_vel_scale", linear_vel_scale_, 1.0);
  nodeHandle.param("angular_vel_scale", angular_vel_scale_, 1.0);

  locomotion_enable_ = false;
  stage_ = 0;
  gait_ = "stance";

  vel_pub_ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  status_pub_ = nodeHandle.advertise<legged_controllers::status_command>("/status_command", 1);
  joy_sub_ = nodeHandle.subscribe<sensor_msgs::Joy>("/joy", 10, &JoystickCommandPublisher::joyCallback, this);
}

// axes: [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]
// 0 Left/Right Axis stick left
// 1 Up/Down Axis stick left
// 2 LT
// 3 Left/Right Axis stick right
// 4 Up/Down Axis stick right
// 5 RT
// 6 cross key left/right
// 7 cross key up/down

// buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
// 0 A
// 1 B
// 2 X
// 3 Y
// 4 LB
// 5 RB
// 6 back
// 7 start
// 8 power
// 9 Button stick left
// 10 Button stick right

void JoystickCommandPublisher::joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
  geometry_msgs::Twist twist;
  twist.linear.x = linear_vel_scale_ * joy->axes[1];
  twist.linear.y = linear_vel_scale_ * joy->axes[0];
  twist.angular.z = angular_vel_scale_ * joy->axes[3];
  vel_pub_.publish(twist);

  legged_controllers::status_command status_command;
  status_command.locomotion_enable = locomotion_enable_;
  status_command.gait = gait_;
  status_command.stage = stage_;

  if (risingEdgeTrigger(5, joy->buttons[5])) {
    if (!locomotion_enable_ && stage_ == 2)
      locomotion_enable_ = true;
    else if (locomotion_enable_)
      locomotion_enable_ = false;
    status_command.locomotion_enable = locomotion_enable_;
    status_pub_.publish(status_command);
  }
  if (locomotion_enable_) {
    if (risingEdgeTrigger(0, joy->buttons[0])) {
      gait_ = "trot";
      status_command.gait = gait_;
      status_pub_.publish(status_command);
    } else if (risingEdgeTrigger(1, joy->buttons[1])) {
      gait_ = "flying_trot";
      status_command.gait = gait_;
      status_pub_.publish(status_command);
    } else if (risingEdgeTrigger(2, joy->buttons[2])) {
      gait_ = "pace";
      status_command.gait = gait_;
      status_pub_.publish(status_command);
    } else if (risingEdgeTrigger(3, joy->buttons[3])) {
      gait_ = "stance";
      status_command.gait = gait_;
      status_pub_.publish(status_command);
    }
  } else {
    if (risingEdgeTrigger(4, joy->buttons[4])) {
      if (stage_ < 2)
        stage_++;
      status_command.stage = stage_;
      status_pub_.publish(status_command);
    } else if (last_joy_axes_[2] != -1.0 && joy->axes[2] == -1.0) {
      if (stage_ > 0)
        stage_--;
      status_command.stage = stage_;
      status_pub_.publish(status_command);
    }
  }

  for (int i = 0; i < joy->axes.size(); ++i)
    last_joy_axes_[i] = joy->axes[i];
  for (int i = 0; i < joy->buttons.size(); ++i)
    last_joy_buttons_[i] = joy->buttons[i];
}

bool JoystickCommandPublisher::risingEdgeTrigger(const int button, const double value) {
  if (last_joy_buttons_[button] == 0.0 && value == 1.0)
    return true;
  else
    return false;
}

int main(int argc, char **argv) {
  std::string node_name = "legged_joystick_command_publisher";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nodeHandle("~");

  JoystickCommandPublisher joystickCommand(nodeHandle);

  ros::spin();
}