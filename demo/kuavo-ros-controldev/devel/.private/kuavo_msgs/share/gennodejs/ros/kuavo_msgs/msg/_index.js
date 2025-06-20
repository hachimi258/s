
"use strict";

let robotArmQVVD = require('./robotArmQVVD.js');
let lejuClawState = require('./lejuClawState.js');
let imuData = require('./imuData.js');
let jointCmd = require('./jointCmd.js');
let footPoses = require('./footPoses.js');
let armHandPose = require('./armHandPose.js');
let recordArmHandPose = require('./recordArmHandPose.js');
let robotHandPosition = require('./robotHandPosition.js');
let footPoseTargetTrajectories = require('./footPoseTargetTrajectories.js');
let Metadata = require('./Metadata.js');
let twoArmHandPose = require('./twoArmHandPose.js');
let dexhandTouchState = require('./dexhandTouchState.js');
let robotHeadMotionData = require('./robotHeadMotionData.js');
let AprilTagDetectionArray = require('./AprilTagDetectionArray.js');
let footPose = require('./footPose.js');
let jointData = require('./jointData.js');
let handPose = require('./handPose.js');
let gestureTask = require('./gestureTask.js');
let ikSolveParam = require('./ikSolveParam.js');
let gestureInfo = require('./gestureInfo.js');
let endEffectorData = require('./endEffectorData.js');
let ikSolveError = require('./ikSolveError.js');
let robotState = require('./robotState.js');
let armPoseWithTimeStamp = require('./armPoseWithTimeStamp.js');
let headBodyPose = require('./headBodyPose.js');
let switchGaitByName = require('./switchGaitByName.js');
let gaitTimeName = require('./gaitTimeName.js');
let dexhandCommand = require('./dexhandCommand.js');
let questJoySticks = require('./questJoySticks.js');
let AprilTagDetection = require('./AprilTagDetection.js');
let touchSensorStatus = require('./touchSensorStatus.js');
let yoloDetection = require('./yoloDetection.js');
let sensorsData = require('./sensorsData.js');
let yoloOutputData = require('./yoloOutputData.js');
let armTargetPoses = require('./armTargetPoses.js');
let lejuClawCommand = require('./lejuClawCommand.js');
let twoArmHandPoseCmd = require('./twoArmHandPoseCmd.js');

module.exports = {
  robotArmQVVD: robotArmQVVD,
  lejuClawState: lejuClawState,
  imuData: imuData,
  jointCmd: jointCmd,
  footPoses: footPoses,
  armHandPose: armHandPose,
  recordArmHandPose: recordArmHandPose,
  robotHandPosition: robotHandPosition,
  footPoseTargetTrajectories: footPoseTargetTrajectories,
  Metadata: Metadata,
  twoArmHandPose: twoArmHandPose,
  dexhandTouchState: dexhandTouchState,
  robotHeadMotionData: robotHeadMotionData,
  AprilTagDetectionArray: AprilTagDetectionArray,
  footPose: footPose,
  jointData: jointData,
  handPose: handPose,
  gestureTask: gestureTask,
  ikSolveParam: ikSolveParam,
  gestureInfo: gestureInfo,
  endEffectorData: endEffectorData,
  ikSolveError: ikSolveError,
  robotState: robotState,
  armPoseWithTimeStamp: armPoseWithTimeStamp,
  headBodyPose: headBodyPose,
  switchGaitByName: switchGaitByName,
  gaitTimeName: gaitTimeName,
  dexhandCommand: dexhandCommand,
  questJoySticks: questJoySticks,
  AprilTagDetection: AprilTagDetection,
  touchSensorStatus: touchSensorStatus,
  yoloDetection: yoloDetection,
  sensorsData: sensorsData,
  yoloOutputData: yoloOutputData,
  armTargetPoses: armTargetPoses,
  lejuClawCommand: lejuClawCommand,
  twoArmHandPoseCmd: twoArmHandPoseCmd,
};
