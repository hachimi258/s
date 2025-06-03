// Auto-generated. Do not edit!

// (in-package kuavo_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let endEffectorData = require('./endEffectorData.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class lejuClawState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = [];
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new endEffectorData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lejuClawState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _arraySerializer.int8(obj.state, buffer, bufferOffset, null);
    // Serialize message field [data]
    bufferOffset = endEffectorData.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lejuClawState
    let len;
    let data = new lejuClawState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [data]
    data.data = endEffectorData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.state.length;
    length += endEffectorData.getMessageSize(object.data);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kuavo_msgs/lejuClawState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '71c0eb8f4803a00f5667de51a2f70aac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    int8 kError = -1                
    int8 kUnknown = 0              
    int8 kMoving = 1              
    int8 kReached = 2            
    int8 kGrabbed = 3         
    
    int8[] state # 0:; left; 1: right
    kuavo_msgs/endEffectorData data
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
    MSG: kuavo_msgs/endEffectorData
    string[] name  
    float64[] position
    float64[] velocity  
    float64[] effort
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lejuClawState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = []
    }

    if (msg.data !== undefined) {
      resolved.data = endEffectorData.Resolve(msg.data)
    }
    else {
      resolved.data = new endEffectorData()
    }

    return resolved;
    }
};

// Constants for message
lejuClawState.Constants = {
  KERROR: -1,
  KUNKNOWN: 0,
  KMOVING: 1,
  KREACHED: 2,
  KGRABBED: 3,
}

module.exports = lejuClawState;
