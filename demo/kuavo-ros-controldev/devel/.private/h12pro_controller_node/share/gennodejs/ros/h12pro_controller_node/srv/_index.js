
"use strict";

let ExecuteArmAction = require('./ExecuteArmAction.js')
let srvChangePhases = require('./srvChangePhases.js')
let changeAMBACCtrlMode = require('./changeAMBACCtrlMode.js')
let playmusic = require('./playmusic.js')
let srvClearPositionCMD = require('./srvClearPositionCMD.js')
let changeHandArmPosesByConfigName = require('./changeHandArmPosesByConfigName.js')
let changeArmCtrlMode = require('./changeArmCtrlMode.js')
let srvchangeCtlMode = require('./srvchangeCtlMode.js')

module.exports = {
  ExecuteArmAction: ExecuteArmAction,
  srvChangePhases: srvChangePhases,
  changeAMBACCtrlMode: changeAMBACCtrlMode,
  playmusic: playmusic,
  srvClearPositionCMD: srvClearPositionCMD,
  changeHandArmPosesByConfigName: changeHandArmPosesByConfigName,
  changeArmCtrlMode: changeArmCtrlMode,
  srvchangeCtlMode: srvchangeCtlMode,
};
