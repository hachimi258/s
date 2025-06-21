// Auto-generated. Do not edit!

// (in-package kuavo_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let touchSensorStatus = require('./touchSensorStatus.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class dexhandTouchState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.left_hand = null;
      this.right_hand = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('left_hand')) {
        this.left_hand = initObj.left_hand
      }
      else {
        this.left_hand = new Array(5).fill(new touchSensorStatus());
      }
      if (initObj.hasOwnProperty('right_hand')) {
        this.right_hand = initObj.right_hand
      }
      else {
        this.right_hand = new Array(5).fill(new touchSensorStatus());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type dexhandTouchState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [left_hand] has the right length
    if (obj.left_hand.length !== 5) {
      throw new Error('Unable to serialize array field left_hand - length must be 5')
    }
    // Serialize message field [left_hand]
    obj.left_hand.forEach((val) => {
      bufferOffset = touchSensorStatus.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [right_hand] has the right length
    if (obj.right_hand.length !== 5) {
      throw new Error('Unable to serialize array field right_hand - length must be 5')
    }
    // Serialize message field [right_hand]
    obj.right_hand.forEach((val) => {
      bufferOffset = touchSensorStatus.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type dexhandTouchState
    let len;
    let data = new dexhandTouchState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [left_hand]
    len = 5;
    data.left_hand = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.left_hand[i] = touchSensorStatus.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [right_hand]
    len = 5;
    data.right_hand = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.right_hand[i] = touchSensorStatus.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 320;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kuavo_msgs/dexhandTouchState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ce777577e1167705dca90d1f63037a05';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    kuavo_msgs/touchSensorStatus[5] left_hand
    kuavo_msgs/touchSensorStatus[5] right_hand
    
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
    MSG: kuavo_msgs/touchSensorStatus
    uint16 normal_force1  # 法向力1
    uint16 normal_force2  # 法向力2
    uint16 normal_force3  # 法向力3
    uint16 tangential_force1  # 切向力1
    uint16 tangential_force2  # 切向力2
    uint16 tangential_force3  # 切向力3
    uint16 tangential_direction1  # 切向力方向1
    uint16 tangential_direction2  # 切向力方向2
    uint16 tangential_direction3  # 切向力方向3
    uint32 self_proximity1  # 自电容接近传感器1
    uint32 self_proximity2  # 自电容接近传感器2
    uint32 mutual_proximity  # 互电容接近传感器
    uint16 status  # 传感器状态
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new dexhandTouchState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.left_hand !== undefined) {
      resolved.left_hand = new Array(5)
      for (let i = 0; i < resolved.left_hand.length; ++i) {
        if (msg.left_hand.length > i) {
          resolved.left_hand[i] = touchSensorStatus.Resolve(msg.left_hand[i]);
        }
        else {
          resolved.left_hand[i] = new touchSensorStatus();
        }
      }
    }
    else {
      resolved.left_hand = new Array(5).fill(new touchSensorStatus())
    }

    if (msg.right_hand !== undefined) {
      resolved.right_hand = new Array(5)
      for (let i = 0; i < resolved.right_hand.length; ++i) {
        if (msg.right_hand.length > i) {
          resolved.right_hand[i] = touchSensorStatus.Resolve(msg.right_hand[i]);
        }
        else {
          resolved.right_hand[i] = new touchSensorStatus();
        }
      }
    }
    else {
      resolved.right_hand = new Array(5).fill(new touchSensorStatus())
    }

    return resolved;
    }
};

module.exports = dexhandTouchState;
