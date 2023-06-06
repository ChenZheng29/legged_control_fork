//
// Created by chenzheng on 2023/6/5.
//

#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>

#include "legged_controllers/gait_command.h"

namespace legged {

/** This class implements ModeSequence communication using ROS. */
class GaitJoystickPublisher {
 public:
  GaitJoystickPublisher(ros::NodeHandle nodeHandle,
                        const std::string &gaitFile,
                        const std::string &robotName,
                        bool verbose = false);

 private:
  /** Prints the list of available gaits. */
  static void printGaitList(const std::vector<std::string> &gaitList);

  std::vector<std::string> gaitList_;
  std::map<std::string, ocs2::legged_robot::ModeSequenceTemplate> gaitMap_;
  std::string currentGait_;

  ros::Publisher modeSequenceTemplatePublisher_;
  ros::Subscriber joySubscriber_;
};

}  // namespace legged
