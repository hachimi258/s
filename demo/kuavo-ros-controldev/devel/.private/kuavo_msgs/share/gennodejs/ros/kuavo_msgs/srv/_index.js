
"use strict";

let SetTagPose = require('./SetTagPose.js')
let gestureExecute = require('./gestureExecute.js')
let jointMoveTo = require('./jointMoveTo.js')
let setTagId = require('./setTagId.js')
let footPoseTargetTrajectoriesSrv = require('./footPoseTargetTrajectoriesSrv.js')
let getCurrentGaitName = require('./getCurrentGaitName.js')
let controlLejuClaw = require('./controlLejuClaw.js')
let gestureExecuteState = require('./gestureExecuteState.js')
let setHwIntialState = require('./setHwIntialState.js')
let singleStepControl = require('./singleStepControl.js')
let gestureList = require('./gestureList.js')
let setMotorEncoderRoundService = require('./setMotorEncoderRoundService.js')
let changeTorsoCtrlMode = require('./changeTorsoCtrlMode.js')
let changeArmCtrlMode = require('./changeArmCtrlMode.js')

module.exports = {
  SetTagPose: SetTagPose,
  gestureExecute: gestureExecute,
  jointMoveTo: jointMoveTo,
  setTagId: setTagId,
  footPoseTargetTrajectoriesSrv: footPoseTargetTrajectoriesSrv,
  getCurrentGaitName: getCurrentGaitName,
  controlLejuClaw: controlLejuClaw,
  gestureExecuteState: gestureExecuteState,
  setHwIntialState: setHwIntialState,
  singleStepControl: singleStepControl,
  gestureList: gestureList,
  setMotorEncoderRoundService: setMotorEncoderRoundService,
  changeTorsoCtrlMode: changeTorsoCtrlMode,
  changeArmCtrlMode: changeArmCtrlMode,
};
