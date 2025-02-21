//
// Created by chenzheng on 2023/6/5.
//

#include "legged_controllers/GaitPublisher.h"
#include "legged_controllers/status_command.h"

#include <algorithm>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>

#include <ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h>

using namespace legged;
using namespace ocs2;

GaitPublisher::GaitPublisher(ros::NodeHandle nodeHandle,
                             const std::string &gaitFile,
                             const std::string &robotName,
                             bool verbose) {
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
  loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ =
      nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

  gaitMap_.clear();
  for (const auto &gaitName : gaitList_) {
    gaitMap_.insert({gaitName, legged_robot::loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }

  auto statusCommandCallback = [this](const legged_controllers::status_command::ConstPtr &msg) {
    std::string gaitCommand;
    gaitCommand = msg->gait;

    try {
      if (!gaitCommand.empty() && gaitCommand != currentGait_) {
        legged_robot::ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
        modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
        currentGait_ = gaitCommand;
      }
    } catch (const std::out_of_range &e) {
      std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
      printGaitList(gaitList_);
    }
  };
  statusSubscriber_ = nodeHandle.subscribe<legged_controllers::status_command>("/status_command", 1, statusCommandCallback);

  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");
}

void GaitPublisher::printGaitList(const std::vector<std::string> &gaitList) {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto &s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

int main(int argc, char *argv[]) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc_mode_schedule");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string gaitCommandFile;
  nodeHandle.getParam("/gaitCommandFile", gaitCommandFile);
  std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;

  GaitPublisher gaitPublisher(nodeHandle, gaitCommandFile, robotName, true);

  ros::spin();
  
  return 0;
}