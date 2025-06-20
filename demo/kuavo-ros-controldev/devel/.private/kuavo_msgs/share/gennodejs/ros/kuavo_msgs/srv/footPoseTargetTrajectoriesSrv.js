// Auto-generated. Do not edit!

// (in-package kuavo_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let footPoseTargetTrajectories = require('../msg/footPoseTargetTrajectories.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class footPoseTargetTrajectoriesSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.foot_pose_target_trajectories = null;
    }
    else {
      if (initObj.hasOwnProperty('foot_pose_target_trajectories')) {
        this.foot_pose_target_trajectories = initObj.foot_pose_target_trajectories
      }
      else {
        this.foot_pose_target_trajectories = new footPoseTargetTrajectories();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type footPoseTargetTrajectoriesSrvRequest
    // Serialize message field [foot_pose_target_trajectories]
    bufferOffset = footPoseTargetTrajectories.serialize(obj.foot_pose_target_trajectories, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type footPoseTargetTrajectoriesSrvRequest
    let len;
    let data = new footPoseTargetTrajectoriesSrvRequest(null);
    // Deserialize message field [foot_pose_target_trajectories]
    data.foot_pose_target_trajectories = footPoseTargetTrajectories.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += footPoseTargetTrajectories.getMessageSize(object.foot_pose_target_trajectories);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/footPoseTargetTrajectoriesSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '074a67ab229c3a1e68e2d35d1ecee61f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    footPoseTargetTrajectories foot_pose_target_trajectories
    
    ================================================================================
    MSG: kuavo_msgs/footPoseTargetTrajectories
    float64[]    timeTrajectory
    int32[]      footIndexTrajectory
    footPose[]   footPoseTrajectory
    footPoses[]  additionalFootPoseTrajectory  # 可选字段，用于存储额外的轨迹点规划值
    
    ================================================================================
    MSG: kuavo_msgs/footPose
    float64[4] footPose # x, y, z, yaw
    float64[4] torsoPose # x, y, z, yaw
    
    ================================================================================
    MSG: kuavo_msgs/footPoses
    footPose[] data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new footPoseTargetTrajectoriesSrvRequest(null);
    if (msg.foot_pose_target_trajectories !== undefined) {
      resolved.foot_pose_target_trajectories = footPoseTargetTrajectories.Resolve(msg.foot_pose_target_trajectories)
    }
    else {
      resolved.foot_pose_target_trajectories = new footPoseTargetTrajectories()
    }

    return resolved;
    }
};

class footPoseTargetTrajectoriesSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type footPoseTargetTrajectoriesSrvResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type footPoseTargetTrajectoriesSrvResponse
    let len;
    let data = new footPoseTargetTrajectoriesSrvResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/footPoseTargetTrajectoriesSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new footPoseTargetTrajectoriesSrvResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: footPoseTargetTrajectoriesSrvRequest,
  Response: footPoseTargetTrajectoriesSrvResponse,
  md5sum() { return '86a3a0586a4bc1d73a3be321ecdc3a15'; },
  datatype() { return 'kuavo_msgs/footPoseTargetTrajectoriesSrv'; }
};
