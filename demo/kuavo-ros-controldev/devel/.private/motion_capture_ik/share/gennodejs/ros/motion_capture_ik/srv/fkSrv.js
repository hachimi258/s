// Auto-generated. Do not edit!

// (in-package motion_capture_ik.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let twoArmHandPose = require('../msg/twoArmHandPose.js');

//-----------------------------------------------------------

class fkSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.q = null;
    }
    else {
      if (initObj.hasOwnProperty('q')) {
        this.q = initObj.q
      }
      else {
        this.q = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type fkSrvRequest
    // Serialize message field [q]
    bufferOffset = _arraySerializer.float64(obj.q, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type fkSrvRequest
    let len;
    let data = new fkSrvRequest(null);
    // Deserialize message field [q]
    data.q = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.q.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_capture_ik/fkSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ab94b9bcaaa12f74def43e4b33992df1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] q # 广义关节角度，如果是虚拟关节ik，则前4维度为躯干虚拟关节的角度，后14维度为手臂关节的角度
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new fkSrvRequest(null);
    if (msg.q !== undefined) {
      resolved.q = msg.q;
    }
    else {
      resolved.q = []
    }

    return resolved;
    }
};

class fkSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.hand_poses = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('hand_poses')) {
        this.hand_poses = initObj.hand_poses
      }
      else {
        this.hand_poses = new twoArmHandPose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type fkSrvResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [hand_poses]
    bufferOffset = twoArmHandPose.serialize(obj.hand_poses, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type fkSrvResponse
    let len;
    let data = new fkSrvResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [hand_poses]
    data.hand_poses = twoArmHandPose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += twoArmHandPose.getMessageSize(object.hand_poses);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_capture_ik/fkSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0989ef9ed6b7b2e5e8a37c79ae6916d9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    twoArmHandPose  hand_poses
    
    ================================================================================
    MSG: motion_capture_ik/twoArmHandPose
    Header header
    armHandPose  left_pose
    armHandPose  right_pose
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: motion_capture_ik/armHandPose
    float64[3] pos_xyz
    float64[4] quat_xyzw
    
    float64[3] elbow_pos_xyz
    
    float64[7] joint_angles
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new fkSrvResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.hand_poses !== undefined) {
      resolved.hand_poses = twoArmHandPose.Resolve(msg.hand_poses)
    }
    else {
      resolved.hand_poses = new twoArmHandPose()
    }

    return resolved;
    }
};

module.exports = {
  Request: fkSrvRequest,
  Response: fkSrvResponse,
  md5sum() { return 'b89cc987a02b6d1c2a1588d5659bf064'; },
  datatype() { return 'motion_capture_ik/fkSrv'; }
};
