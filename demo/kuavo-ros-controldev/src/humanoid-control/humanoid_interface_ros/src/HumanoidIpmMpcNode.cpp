/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ros/init.h>
#include <ros/package.h>

// #include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ipm/IpmMpc.h>
#include <humanoid_interface/HumanoidInterface.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/synchronized_module/SolverObserverRosCallbacks.h>

#include "humanoid_interface_ros/gait/GaitReceiver.h"

using namespace ocs2;
using namespace humanoid;

int main(int argc, char **argv)
{
  // std::cout << "argc=" << argc << std::endl;
  // for (int i=0; i<argc; i++) {
  //   std::cout << "argv["<<i<<"]: " << std::string(*(argv+i)) << std::endl;
  // }
  const std::string robotName = "humanoid";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_mpc_sqp");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  bool multiplot = false;
  std::string taskFile, urdfFile, referenceFile,gaitCommandFile;
  nodeHandle.getParam("/multiplot", multiplot);
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/gaitCommandFile", gaitCommandFile);



  // Robot interface
  HumanoidInterface interface(taskFile, urdfFile, referenceFile,gaitCommandFile);

  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // MPC
  IpmMpc mpc(interface.mpcSettings(), interface.ipmSettings(), interface.getOptimalControlProblem(),
             interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

  // ocs2::vector_t init_state = interface.getInitialState();
  // const vector_array_t inputTrajectory(2, vector_t::Zero(24));
  // auto ocp_ptr = interface.getOptimalControlProblem();
  // vector_t my_left_f = ocp_ptr.equalityConstraintPtr->get("l_foot_roll_zeroFootPitch")
  //                                                   .getValue(0, init_state, inputTrajectory[0],
  //                                                   *(ocp_ptr.preComputationPtr));

  // std::cout << "!!!!!!!!!!!!my left f:" << "\n";
  //       std::cout << my_left_f << "\n" ;

  // vector_t my_right_f = ocp_ptr.equalityConstraintPtr->get("r_foot_roll_zeroFootPitch")
  //                                                   .getValue(0, init_state, inputTrajectory[0],
  //                                                   *(ocp_ptr.preComputationPtr));

  // std::cout << "!!!!!!!!!!!!my right f:" << "\n";
  //       std::cout << my_right_f << "\n" ;

  // TODO 暂时不管multiplot
  // observer for zero velocity constraints (only add this for debugging as it slows down the solver)
  // if (multiplot) {
  //   auto createStateInputBoundsObserver = [&](const std::string& termName) {
  //     const ocs2::scalar_array_t observingTimePoints{0.0};
  //     const std::vector<std::string> topicNames{"metrics/" + termName + "/0MsLookAhead"};
  //     auto callback = ocs2::ros::createConstraintCallback(nodeHandle, {0.0}, topicNames,
  //                                                         ocs2::ros::CallbackInterpolationStrategy::linear_interpolation);
  //     return ocs2::SolverObserver::ConstraintTermObserver(ocs2::SolverObserver::Type::Intermediate, termName, std::move(callback));
  //   };
  //   for (size_t i = 0; i < interface.getCentroidalModelInfo().numThreeDofContacts; i++) {
  //     const std::string& footName = interface.modelSettings().contactNames3DoF[i];
  //     mpc.getSolverPtr()->addSolverObserver(createStateInputBoundsObserver(footName + "_zeroVelocity"));
  //   }
  // }

  // Launch MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);

  std::cout << "!!!!!!!!!!!!!!!!!!!end!!!!!!!!!!!!!!" << std::endl;

  // Successful exit
  return 0;
}
