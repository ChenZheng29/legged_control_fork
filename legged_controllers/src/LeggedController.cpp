//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"
#include "legged_controllers/target_trajectories_data.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace legged {
bool LeggedController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) {
  // Initialize OCS2
  std::string urdfFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile_);
  controller_nh.getParam("/referenceFile", referenceFile_);
  bool verbose = false;
  loadData::loadCppDataType(taskFile_, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile_, urdfFile, referenceFile_, verbose);

  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  // Hardware interface
  auto *hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto &joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  auto *contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto &name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  // State estimation
  setupStateEstimate(taskFile_, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile_, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  defaultJointState_.setZero(12);
  defaultFootPos_.setZero(2);
  squatJointState_.setZero(12);
  squatFootPos_.setZero(3);
  loadData::loadEigenMatrix(referenceFile_, "defaultJointState", defaultJointState_);
  loadData::loadEigenMatrix(referenceFile_, "defaultFootPos", defaultFootPos_);
  loadData::loadEigenMatrix(referenceFile_, "squatJointState", squatJointState_);
  loadData::loadEigenMatrix(referenceFile_, "squatFootPos", squatFootPos_);
  loadData::loadCppDataType(referenceFile_, "comHeight", comHeight_);
  loadData::loadCppDataType(referenceFile_, "position_control_parameter.kp", kp_);
  loadData::loadCppDataType(referenceFile_, "position_control_parameter.kd", kd_);

  // Init solve inverse kinematics
  vector_t footPos(12), jointDes(12);
  footPos << squatFootPos_[0], squatFootPos_[1], squatFootPos_[2], squatFootPos_[0], squatFootPos_[1], squatFootPos_[2],
      squatFootPos_[0], -squatFootPos_[1], squatFootPos_[2], squatFootPos_[0], -squatFootPos_[1], squatFootPos_[2];
  if (getJointPos(footPos, jointDes))
    squatJointState_ = jointDes;
  footPos << defaultFootPos_[0], defaultFootPos_[1], -comHeight_, defaultFootPos_[0], defaultFootPos_[1], -comHeight_,
      defaultFootPos_[0], -defaultFootPos_[1], -comHeight_, defaultFootPos_[0], -defaultFootPos_[1], -comHeight_;
  if (getJointPos(footPos, jointDes))
    defaultJointState_ = jointDes;

  statusSubscriber_ = nh.subscribe<legged_controllers::status_command>("/status_command", 1, &LeggedController::statusCommandCallback, this);
  targetTrajectoriesDataPublisher_ = nh.advertise<legged_controllers::target_trajectories_data>("/target_trajectories_data", 1, true);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>("legged_robot_mpc_observation", 1);

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception &e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  if (locomotionEnable_) {
    if (!initLocomotionSwitch_) {
      leggedInterface_->resetOcp(taskFile_, false);
      setupMpc();
      setupMrt();
      TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
      mpcMrtInterface_->resetMpcNode(target_trajectories);
      mpcMrtInterface_->setCurrentObservation(currentObservation_);
      mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
      mpcMrtInterface_->advanceMpc();
      if (mpcMrtInterface_->initialPolicyReceived()) {
        mpcRunning_ = true;
        initLocomotionSwitch_ = true;
        ROS_INFO("[Legged Controller] mpc and wbc control");
      } else
        ROS_INFO("[Legged Controller] mpc initial policy received fail");
      return;
    }

    // Update the current state of the system
    mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    vector_t optimizedState, optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    // Whole body control
    currentObservation_.input = optimizedInput;

    wbcTimer_.startTimer();
    vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
    wbcTimer_.endTimer();

    vector_t torque = x.tail(12);
    vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

    // Safety check, if failed, stop the controller
    if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
      ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
      stopRequest(time);
    }

    for (int j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
      hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
    }

    // Visualization
    robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
    selfCollisionVisualization_->update(currentObservation_);
  } else {
    if (!initLocomotionSwitch_) {
      mpcRunning_ = false;
      initLocomotionSwitch_ = true;
      ROS_INFO("[Legged Controller] position control");
      return;
    }

    if (stage_ == 0) {
      for (int j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j)
        hybridJointHandles_[j].setCommand(0, 0, 0, 20, 0);
    } else {
      vector_t jointDes(12);
      if (currentObservation_.time > timeSequence_.back())
        jointDes << jointDesSequence_.back();
      else
        jointDes << LinearInterpolation::interpolate(currentObservation_.time, timeSequence_, jointDesSequence_);
      for (int j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j)
        hybridJointHandles_[j].setCommand(jointDes[j], 0, kp_, kd_, 0);
    }

    // Visualization
    robotVisualizer_->update(currentObservation_, PrimalSolution(), CommandData());
    selfCollisionVisualization_->update(currentObservation_);
  }

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (size_t i = 0; i < contacts.size(); ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
  }
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_.reset();
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_.reset();
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();
}

bool LeggedController::eeInverseKinematics(const std::string &leg,
                                           const int hipIndex,
                                           const Eigen::Vector3d &footPosDes,
                                           Eigen::Vector3d &jointPos,
                                           bool verbose) {
  // get pinocchioInterface
  PinocchioInterface pinocchioInterface = leggedInterface_->getPinocchioInterface();
  const auto &model = pinocchioInterface.getModel();
  auto &data = pinocchioInterface.getData();
  // set target frameId and get transform from base to thigh
  const size_t frameId = model.getFrameId(leg + "_FOOT");
  Eigen::VectorXd q = pinocchio::neutral(model);
  // Under this initial configuration, it is possible to find a solution faster and prevent obtaining a solution that exceeds the range of joint motion
  q[hipIndex + 1] = 0.7;
  q[hipIndex + 2] = -1.4;
  pinocchio::forwardKinematics(model, data, q);
  Eigen::Vector3d base2thigh = data.oMi[model.getJointId(leg + "_HFE")].translation();
  // parameter
  const double eps = 0.01;
  const int itMax = 1000;
  const double dt = 100.0;
  const double damp = 50.0;

  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();
  bool success;
  Eigen::Matrix<double, 6, 1> err;
  Eigen::VectorXd v(model.nv);
  for (int i = 0;; i++) {
    // update data
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacement(model, data, frameId);
    // set des
    const pinocchio::SE3 oMdes(data.oMf[frameId].rotation(), footPosDes + base2thigh);
    // compute rotation and position error (des - current)
    const pinocchio::SE3 iMd = data.oMf[frameId].actInv(oMdes);
    err = pinocchio::log6(iMd).toVector();
    // exit conditions
    if (err.norm() < eps) {
      if ((q[hipIndex] >= model.lowerPositionLimit[hipIndex] && q[hipIndex] <= model.upperPositionLimit[hipIndex])
          && (q[hipIndex + 1] >= model.lowerPositionLimit[hipIndex + 1] && q[hipIndex + 1] <= model.upperPositionLimit[hipIndex + 1])
          && (q[hipIndex + 2] >= model.lowerPositionLimit[hipIndex + 2] && q[hipIndex + 2] <= model.upperPositionLimit[hipIndex + 2])) {
        success = true;
        jointPos << q[hipIndex], q[hipIndex + 1], q[hipIndex + 2];
      } else {
        success = false;
        ROS_WARN("%s foot inverse kinematics solve fail! (Beyond joint range)", leg.c_str());
      }
      break;
    }
    if (i >= itMax) {
      success = false;
      ROS_WARN("%s foot inverse kinematics solve fail! (Reached maximum number of iterations)", leg.c_str());
      break;
    }
    // jacobian iteration
    pinocchio::computeFrameJacobian(model, data, q, frameId, J);
    pinocchio::Data::Matrix6x Jlog(6, 6);
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * dt);
    // Ensure that the base coordinate system coincides with the world coordinate system, equivalent to adding joint constraints
    q.segment<6>(0).setZero();
  }

  if (verbose) {
    std::cout << "###############################################################################" << std::endl;
    std::cout << "#### leg: " << leg << std::endl;
    if (success)
      std::cout << "#### success!!!" << std::endl;
    else
      std::cout << "#### fail!!!" << std::endl;
    std::cout << "#### final foot position: " << data.oMf[frameId].translation().transpose() << std::endl;
    std::cout << "#### final foot rpy: " << data.oMf[frameId].rotation().eulerAngles(0, 1, 2).transpose() << std::endl;
    std::cout << "#### final foot position error: " << err.transpose()[0] << " " << err.transpose()[1] << " " << err.transpose()[2] << std::endl;
    std::cout << "#### final joint position: " << q.transpose()[6] << " " << q.transpose()[7] << " " << q.transpose()[8] << std::endl;
    std::cout << "###############################################################################" << std::endl;
  }

  return success;
}

void LeggedController::statusCommandCallback(const legged_controllers::status_command::ConstPtr &msg) {
  // switch mpc or position control
  if (locomotionEnable_ != msg->locomotion_enable) {
    locomotionEnable_ = msg->locomotion_enable;
    initLocomotionSwitch_ = false;
    isUpdateJointDesSequence_ = true;
  }

  // update centroid height
  if (comHeight_ != msg->com_height) {
    comHeight_ = msg->com_height;
    vector_t footPos(12), jointDes(12);
    footPos << defaultFootPos_[0], defaultFootPos_[1], -comHeight_, defaultFootPos_[0], defaultFootPos_[1], -comHeight_,
        defaultFootPos_[0], -defaultFootPos_[1], -comHeight_, defaultFootPos_[0], -defaultFootPos_[1], -comHeight_;
    if (getJointPos(footPos, jointDes)) {
      defaultJointState_ = jointDes; // defaultJointState_ need to update when modifying the height
      isUpdateJointDesSequence_ = true;
    }
    legged_controllers::target_trajectories_data targetTrajectoriesData;
    targetTrajectoriesData.com_height = comHeight_;
    for (int i = 0; i < defaultJointState_.size(); ++i)
      targetTrajectoriesData.default_joint_state.push_back(defaultJointState_[i]);
    targetTrajectoriesDataPublisher_.publish(targetTrajectoriesData);
  }

  // update stage
  if (stage_ != msg->stage || isUpdateJointDesSequence_) {
    isUpdateJointDesSequence_ = false;
    stage_ = msg->stage;
    if (stage_ == 0) {
      jointDesSequence_.clear();
      timeSequence_.clear();
    } else {
      vector_t jointInit(12), jointDes(12), jointError(12);
      // update joint sequence init pos
      if (jointDesSequence_.empty())
        for (int i = 0; i < 12; ++i)
          jointInit[i] = hybridJointHandles_[i].getPosition();
      else {
        if (currentObservation_.time > timeSequence_.back())
          jointInit << jointDesSequence_.back();
        else
          jointInit << LinearInterpolation::interpolate(currentObservation_.time, timeSequence_, jointDesSequence_);
      }
      // update joint sequence final pos
      if (stage_ == 1)
        jointDes = squatJointState_;
      else if (stage_ == 2)
        jointDes = defaultJointState_;
      // update sequence time horizon
      jointError = jointDes - jointInit;
      double maxJointError{}, timeHorizon{}, maxJointVel = 1.0;
      for (int i = 0; i < jointError.size(); ++i)
        maxJointError = std::max(maxJointError, std::abs(jointError[i]));
      timeHorizon = maxJointError / maxJointVel;
      // update sequence
      jointDesSequence_.clear();
      jointDesSequence_.push_back(jointInit);
      jointDesSequence_.push_back(jointDes);
      timeSequence_.clear();
      timeSequence_.push_back(currentObservation_.time);
      timeSequence_.push_back(currentObservation_.time + timeHorizon);
    }
  }
}

bool LeggedController::getJointPos(const vector_t &footPos, vector_t &jointPos) {
  bool solveFlag[4];
  Eigen::Vector3d LFJointPos, LHJointPos, RFJointPos, RHJointPos;
  solveFlag[0] = eeInverseKinematics("LF", 6, footPos.segment<3>(0), LFJointPos);
  solveFlag[1] = eeInverseKinematics("LH", 9, footPos.segment<3>(3), LHJointPos);
  solveFlag[2] = eeInverseKinematics("RF", 12, footPos.segment<3>(6), RFJointPos);
  solveFlag[3] = eeInverseKinematics("RH", 15, footPos.segment<3>(9), RHJointPos);
  jointPos << LFJointPos, LHJointPos, RFJointPos, RHJointPos;

  return solveFlag[0] && solveFlag[1] && solveFlag[2] && solveFlag[3];
}

void LeggedController::setupStateEstimate(const std::string &taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string & /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
