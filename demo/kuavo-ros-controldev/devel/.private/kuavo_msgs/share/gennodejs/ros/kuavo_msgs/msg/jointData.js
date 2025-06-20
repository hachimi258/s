// Auto-generated. Do not edit!

// (in-package kuavo_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class jointData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_q = null;
      this.joint_v = null;
      this.joint_vd = null;
      this.joint_torque = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_q')) {
        this.joint_q = initObj.joint_q
      }
      else {
        this.joint_q = [];
      }
      if (initObj.hasOwnProperty('joint_v')) {
        this.joint_v = initObj.joint_v
      }
      else {
        this.joint_v = [];
      }
      if (initObj.hasOwnProperty('joint_vd')) {
        this.joint_vd = initObj.joint_vd
      }
      else {
        this.joint_vd = [];
      }
      if (initObj.hasOwnProperty('joint_torque')) {
        this.joint_torque = initObj.joint_torque
      }
      else {
        this.joint_torque = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type jointData
    // Serialize message field [joint_q]
    bufferOffset = _arraySerializer.float64(obj.joint_q, buffer, bufferOffset, null);
    // Serialize message field [joint_v]
    bufferOffset = _arraySerializer.float64(obj.joint_v, buffer, bufferOffset, null);
    // Serialize message field [joint_vd]
    bufferOffset = _arraySerializer.float64(obj.joint_vd, buffer, bufferOffset, null);
    // Serialize message field [joint_torque]
    bufferOffset = _arraySerializer.float64(obj.joint_torque, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type jointData
    let len;
    let data = new jointData(null);
    // Deserialize message field [joint_q]
    data.joint_q = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_v]
    data.joint_v = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_vd]
    data.joint_vd = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_torque]
    data.joint_torque = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_q.length;
    length += 8 * object.joint_v.length;
    length += 8 * object.joint_vd.length;
    length += 8 * object.joint_torque.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kuavo_msgs/jointData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2e01436cbc40e94e8fe8f54a2c4ea282';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] joint_q  
    float64[] joint_v  
    float64[] joint_vd    
    float64[] joint_torque  
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new jointData(null);
    if (msg.joint_q !== undefined) {
      resolved.joint_q = msg.joint_q;
    }
    else {
      resolved.joint_q = []
    }

    if (msg.joint_v !== undefined) {
      resolved.joint_v = msg.joint_v;
    }
    else {
      resolved.joint_v = []
    }

    if (msg.joint_vd !== undefined) {
      resolved.joint_vd = msg.joint_vd;
    }
    else {
      resolved.joint_vd = []
    }

    if (msg.joint_torque !== undefined) {
      resolved.joint_torque = msg.joint_torque;
    }
    else {
      resolved.joint_torque = []
    }

    return resolved;
    }
};

module.exports = jointData;
