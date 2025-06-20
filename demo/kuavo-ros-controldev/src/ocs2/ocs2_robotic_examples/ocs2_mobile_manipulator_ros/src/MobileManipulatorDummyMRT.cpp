/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

#include <ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ros/init.h>
#include <ros/package.h>

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv) {
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string taskFile, libFolder, urdfFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;
  // Robot Interface
  mobile_manipulator::MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);

  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  auto dummyVisualization = std::make_shared<mobile_manipulator::MobileManipulatorDummyVisualization>(nodeHandle, interface);

  // Dummy MRT
  MRT_ROS_Dummy_Loop dummy(mrt, interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
  dummy.subscribeObservers({dummyVisualization});

  // initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);
  initObservation.time = 0.0;

  // initial command
  const size_t baseDim = interface.getManipulatorModelInfo().stateDim - interface.getManipulatorModelInfo().armDim;
  vector_t initTarget(baseDim + 7*2);
  initTarget.head(baseDim).setZero();
  vector_t hand_pose(7);

  hand_pose.head(3) << 1, 0, 0.4;
  auto q = Eigen::Quaternion<scalar_t>(0.707, 0, -0.707, 0);
  q.normalize();
  hand_pose.tail(4) << q.coeffs();
  initTarget.segment<7>(baseDim) = hand_pose;
  initTarget.segment<7>(baseDim+7) = hand_pose;
  const vector_t zeroInput = vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
  const TargetTrajectories initTargetTrajectories({initObservation.time}, {initTarget}, {zeroInput});

  // Run dummy (loops while ros is ok)
  dummy.run(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}
