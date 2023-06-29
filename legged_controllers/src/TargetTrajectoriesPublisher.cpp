//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/TargetTrajectoriesPublisher.h"
#include "legged_controllers/target_trajectories_data.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace legged;

namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
}  // namespace

TargetTrajectoriesPublisher::TargetTrajectoriesPublisher(::ros::NodeHandle &nh, const std::string &topicPrefix) : tf2_(buffer_) {
  // Trajectories publisher
  targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

  // goal subscriber
  auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (latestObservation_.time == 0.0) {
      return;
    }
    geometry_msgs::PoseStamped pose = *msg;
    try {
      buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Failure %s\n", ex.what());
      return;
    }

    vector_t cmdGoal = vector_t::Zero(6);
    cmdGoal[0] = pose.pose.position.x;
    cmdGoal[1] = pose.pose.position.y;
    cmdGoal[2] = pose.pose.position.z;
    Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
    cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
    cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

    const auto trajectories = goalToTargetTrajectories(cmdGoal, latestObservation_);
    targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
  };

  // cmd_vel subscriber
  auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr &msg) {
    if (latestObservation_.time == 0.0) {
      return;
    }

    vector_t cmdVel = vector_t::Zero(4);
    cmdVel[0] = msg->linear.x;
    cmdVel[1] = msg->linear.y;
    cmdVel[2] = msg->linear.z;
    cmdVel[3] = msg->angular.z;

    const auto trajectories = cmdVelToTargetTrajectories(cmdVel, latestObservation_);
    targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
  };

  goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
  cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);

  // status subscriber
  auto targetTrajectoriesDataCallback = [this](const legged_controllers::target_trajectories_data::ConstPtr &msg) {
    COM_HEIGHT = msg->com_height;
    for (int i = 0; i < 12; ++i)
      DEFAULT_JOINT_STATE[i] = msg->default_joint_state[i];
  };

  targetTrajectoriesDataSub_ =
      nh.subscribe<legged_controllers::target_trajectories_data>("/target_trajectories_data", 1, targetTrajectoriesDataCallback);
};

scalar_t TargetTrajectoriesPublisher::estimateTimeToTarget(const vector_t &desiredBaseDisplacement) {
  const scalar_t &dx = desiredBaseDisplacement(0);
  const scalar_t &dy = desiredBaseDisplacement(1);
  const scalar_t &dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

TargetTrajectories TargetTrajectoriesPublisher::targetPoseToTargetTrajectories(const vector_t &targetPose, const SystemObservation &observation,
                                                                               const scalar_t &targetReachingTime) {
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  currentPose(2) = COM_HEIGHT;
  currentPose(4) = 0;
  currentPose(5) = 0;
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

TargetTrajectories TargetTrajectoriesPublisher::goalToTargetTrajectories(const vector_t &goal, const SystemObservation &observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = COM_HEIGHT;
    target(3) = goal(3);
    target(4) = 0;
    target(5) = 0;
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}

TargetTrajectories TargetTrajectoriesPublisher::cmdVelToTargetTrajectories(const vector_t &cmdVel, const SystemObservation &observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = COM_HEIGHT;
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}

int main(int argc, char **argv) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string referenceFile;
  std::string taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/taskFile", taskFile);

  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName);

  ros::spin();
  // Successful exit
  return 0;
}
