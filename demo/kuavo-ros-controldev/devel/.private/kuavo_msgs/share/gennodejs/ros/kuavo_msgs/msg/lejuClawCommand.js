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

class lejuClawCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
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
    // Serializes a message object of type lejuClawCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = endEffectorData.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lejuClawCommand
    let len;
    let data = new lejuClawCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = endEffectorData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += endEffectorData.getMessageSize(object.data);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kuavo_msgs/lejuClawCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57c40e80f90e7a289ae0b2488552e043';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # kuavo_msgs/endEffectorData:
    # string[] name  
    # float64[] position
    # float64[] velocity  
    # float64[] effort
    # 
    # ** For the Topic Notes **
    # 
    # name     : 'left_claw' , 'right_claw'
    # position : 0 ~ 100, the percentage of the claw's opening angle
    #            0: closed, 100: open   
    # velocity : 0 ~ 100, if size is 0, will use default `50.0`.
    # effort   : torque/current, better 1A ~ 2A, if size is 0, will use default `1.0`.
    # 
    # ** Example **
    # data:
    #   - name: ['left_claw', 'right_claw']
    #     position: [20.0, 20.0]
    #     velocity: [50.0, 50.0]
    #     effort: [1.0, 1.0]
    
    std_msgs/Header header
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
    const resolved = new lejuClawCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
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

module.exports = lejuClawCommand;
