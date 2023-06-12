//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "legged_controllers/status_command.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace legged {
using namespace ocs2;

class TargetTrajectoriesPublisher final {
 public:
  TargetTrajectoriesPublisher(::ros::NodeHandle &nh, const std::string &topicPrefix);

 private:
  static TargetTrajectories cmdVelToTargetTrajectories(const vector_t &cmd, const SystemObservation &observation);
  static TargetTrajectories goalToTargetTrajectories(const vector_t &cmd, const SystemObservation &observation);
  static TargetTrajectories targetPoseToTargetTrajectories(const vector_t &targetPose,
                                                           const SystemObservation &observation,
                                                           const scalar_t &targetReachingTime);
  static scalar_t estimateTimeToTarget(const vector_t &desiredBaseDisplacement);
  void statusCommandCallback(const legged_controllers::status_command::ConstPtr &msg);

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_, statusSub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace legged
