//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"
#include "legged_controllers/status_command.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  LeggedController() = default;
  ~LeggedController() override;
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void starting(const ros::Time &time) override;
  void stopping(const ros::Time & /*time*/) override { mpcRunning_ = false; }

 protected:
  virtual void updateStateEstimation(const ros::Time &time, const ros::Duration &period);

  virtual void setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string &taskFile, bool verbose);
  bool eeInverseKinematics(const std::string &leg,
                           int hipIndex,
                           const Eigen::Vector3d &footPosDes,
                           Eigen::Vector3d &jointPos,
                           bool verbose = false); // ref: https://github.com/stack-of-tasks/pinocchio/pull/1963/files/b8ff8069f20b15f887af24ae65f03bff32ad3708#diff-632c271335e29edd654d2b482e4ee20ea8173a644cb21c368eb59b3a90be3be3

  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;

 private:
  void statusCommandCallback(const legged_controllers::status_command::ConstPtr &msg);
  bool getJointPos(const vector_t &footPos, vector_t &jointPos);

  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;

  ros::Subscriber statusSubscriber_;
  vector_t defaultJointState_;
  vector_t squatJointState_;
  vector_array_t jointDesSequence_;
  int sequenceIndex_{};
  bool locomotionEnable_{};
  bool initLocomotionSwitch_{};
  int stage_{};
};

class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
