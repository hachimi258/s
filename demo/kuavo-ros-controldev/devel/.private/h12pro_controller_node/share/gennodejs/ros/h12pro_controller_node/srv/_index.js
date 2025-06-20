
"use strict";

let changeHandArmPosesByConfigName = require('./changeHandArmPosesByConfigName.js')
let changeAMBACCtrlMode = require('./changeAMBACCtrlMode.js')
let srvchangeCtlMode = require('./srvchangeCtlMode.js')
let changeArmCtrlMode = require('./changeArmCtrlMode.js')
let playmusic = require('./playmusic.js')
let ExecuteArmAction = require('./ExecuteArmAction.js')
let srvClearPositionCMD = require('./srvClearPositionCMD.js')
let srvChangePhases = require('./srvChangePhases.js')

module.exports = {
  changeHandArmPosesByConfigName: changeHandArmPosesByConfigName,
  changeAMBACCtrlMode: changeAMBACCtrlMode,
  srvchangeCtlMode: srvchangeCtlMode,
  changeArmCtrlMode: changeArmCtrlMode,
  playmusic: playmusic,
  ExecuteArmAction: ExecuteArmAction,
  srvClearPositionCMD: srvClearPositionCMD,
  srvChangePhases: srvChangePhases,
};
