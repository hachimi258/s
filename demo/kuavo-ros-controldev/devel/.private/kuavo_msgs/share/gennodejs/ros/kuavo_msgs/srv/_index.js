
"use strict";

let setTagId = require('./setTagId.js')
let gestureList = require('./gestureList.js')
let gestureExecuteState = require('./gestureExecuteState.js')
let twoArmHandPoseCmdSrv = require('./twoArmHandPoseCmdSrv.js')
let handForceLevel = require('./handForceLevel.js')
let enableHandTouchSensor = require('./enableHandTouchSensor.js')
let changeTorsoCtrlMode = require('./changeTorsoCtrlMode.js')
let setHwIntialState = require('./setHwIntialState.js')
let gestureExecute = require('./gestureExecute.js')
let changeArmCtrlModeKuavo = require('./changeArmCtrlModeKuavo.js')
let jointMoveTo = require('./jointMoveTo.js')
let SpeechSynthesis = require('./SpeechSynthesis.js')
let singleStepControl = require('./singleStepControl.js')
let setMotorEncoderRoundService = require('./setMotorEncoderRoundService.js')
let controlLejuClaw = require('./controlLejuClaw.js')
let SetTagPose = require('./SetTagPose.js')
let recordmusic = require('./recordmusic.js')
let changeArmCtrlMode = require('./changeArmCtrlMode.js')
let playmusic = require('./playmusic.js')
let footPoseTargetTrajectoriesSrv = require('./footPoseTargetTrajectoriesSrv.js')
let setMmCtrlFrame = require('./setMmCtrlFrame.js')
let fkSrv = require('./fkSrv.js')
let getCurrentGaitName = require('./getCurrentGaitName.js')

module.exports = {
  setTagId: setTagId,
  gestureList: gestureList,
  gestureExecuteState: gestureExecuteState,
  twoArmHandPoseCmdSrv: twoArmHandPoseCmdSrv,
  handForceLevel: handForceLevel,
  enableHandTouchSensor: enableHandTouchSensor,
  changeTorsoCtrlMode: changeTorsoCtrlMode,
  setHwIntialState: setHwIntialState,
  gestureExecute: gestureExecute,
  changeArmCtrlModeKuavo: changeArmCtrlModeKuavo,
  jointMoveTo: jointMoveTo,
  SpeechSynthesis: SpeechSynthesis,
  singleStepControl: singleStepControl,
  setMotorEncoderRoundService: setMotorEncoderRoundService,
  controlLejuClaw: controlLejuClaw,
  SetTagPose: SetTagPose,
  recordmusic: recordmusic,
  changeArmCtrlMode: changeArmCtrlMode,
  playmusic: playmusic,
  footPoseTargetTrajectoriesSrv: footPoseTargetTrajectoriesSrv,
  setMmCtrlFrame: setMmCtrlFrame,
  fkSrv: fkSrv,
  getCurrentGaitName: getCurrentGaitName,
};
