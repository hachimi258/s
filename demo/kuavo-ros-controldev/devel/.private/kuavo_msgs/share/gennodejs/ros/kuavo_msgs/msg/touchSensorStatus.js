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

class touchSensorStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.normal_force1 = null;
      this.normal_force2 = null;
      this.normal_force3 = null;
      this.tangential_force1 = null;
      this.tangential_force2 = null;
      this.tangential_force3 = null;
      this.tangential_direction1 = null;
      this.tangential_direction2 = null;
      this.tangential_direction3 = null;
      this.self_proximity1 = null;
      this.self_proximity2 = null;
      this.mutual_proximity = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('normal_force1')) {
        this.normal_force1 = initObj.normal_force1
      }
      else {
        this.normal_force1 = 0;
      }
      if (initObj.hasOwnProperty('normal_force2')) {
        this.normal_force2 = initObj.normal_force2
      }
      else {
        this.normal_force2 = 0;
      }
      if (initObj.hasOwnProperty('normal_force3')) {
        this.normal_force3 = initObj.normal_force3
      }
      else {
        this.normal_force3 = 0;
      }
      if (initObj.hasOwnProperty('tangential_force1')) {
        this.tangential_force1 = initObj.tangential_force1
      }
      else {
        this.tangential_force1 = 0;
      }
      if (initObj.hasOwnProperty('tangential_force2')) {
        this.tangential_force2 = initObj.tangential_force2
      }
      else {
        this.tangential_force2 = 0;
      }
      if (initObj.hasOwnProperty('tangential_force3')) {
        this.tangential_force3 = initObj.tangential_force3
      }
      else {
        this.tangential_force3 = 0;
      }
      if (initObj.hasOwnProperty('tangential_direction1')) {
        this.tangential_direction1 = initObj.tangential_direction1
      }
      else {
        this.tangential_direction1 = 0;
      }
      if (initObj.hasOwnProperty('tangential_direction2')) {
        this.tangential_direction2 = initObj.tangential_direction2
      }
      else {
        this.tangential_direction2 = 0;
      }
      if (initObj.hasOwnProperty('tangential_direction3')) {
        this.tangential_direction3 = initObj.tangential_direction3
      }
      else {
        this.tangential_direction3 = 0;
      }
      if (initObj.hasOwnProperty('self_proximity1')) {
        this.self_proximity1 = initObj.self_proximity1
      }
      else {
        this.self_proximity1 = 0;
      }
      if (initObj.hasOwnProperty('self_proximity2')) {
        this.self_proximity2 = initObj.self_proximity2
      }
      else {
        this.self_proximity2 = 0;
      }
      if (initObj.hasOwnProperty('mutual_proximity')) {
        this.mutual_proximity = initObj.mutual_proximity
      }
      else {
        this.mutual_proximity = 0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type touchSensorStatus
    // Serialize message field [normal_force1]
    bufferOffset = _serializer.uint16(obj.normal_force1, buffer, bufferOffset);
    // Serialize message field [normal_force2]
    bufferOffset = _serializer.uint16(obj.normal_force2, buffer, bufferOffset);
    // Serialize message field [normal_force3]
    bufferOffset = _serializer.uint16(obj.normal_force3, buffer, bufferOffset);
    // Serialize message field [tangential_force1]
    bufferOffset = _serializer.uint16(obj.tangential_force1, buffer, bufferOffset);
    // Serialize message field [tangential_force2]
    bufferOffset = _serializer.uint16(obj.tangential_force2, buffer, bufferOffset);
    // Serialize message field [tangential_force3]
    bufferOffset = _serializer.uint16(obj.tangential_force3, buffer, bufferOffset);
    // Serialize message field [tangential_direction1]
    bufferOffset = _serializer.uint16(obj.tangential_direction1, buffer, bufferOffset);
    // Serialize message field [tangential_direction2]
    bufferOffset = _serializer.uint16(obj.tangential_direction2, buffer, bufferOffset);
    // Serialize message field [tangential_direction3]
    bufferOffset = _serializer.uint16(obj.tangential_direction3, buffer, bufferOffset);
    // Serialize message field [self_proximity1]
    bufferOffset = _serializer.uint32(obj.self_proximity1, buffer, bufferOffset);
    // Serialize message field [self_proximity2]
    bufferOffset = _serializer.uint32(obj.self_proximity2, buffer, bufferOffset);
    // Serialize message field [mutual_proximity]
    bufferOffset = _serializer.uint32(obj.mutual_proximity, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint16(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type touchSensorStatus
    let len;
    let data = new touchSensorStatus(null);
    // Deserialize message field [normal_force1]
    data.normal_force1 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [normal_force2]
    data.normal_force2 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [normal_force3]
    data.normal_force3 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tangential_force1]
    data.tangential_force1 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tangential_force2]
    data.tangential_force2 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tangential_force3]
    data.tangential_force3 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tangential_direction1]
    data.tangential_direction1 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tangential_direction2]
    data.tangential_direction2 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tangential_direction3]
    data.tangential_direction3 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [self_proximity1]
    data.self_proximity1 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [self_proximity2]
    data.self_proximity2 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [mutual_proximity]
    data.mutual_proximity = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kuavo_msgs/touchSensorStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '08cd59dc396363cba4d4f01df99ec86c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new touchSensorStatus(null);
    if (msg.normal_force1 !== undefined) {
      resolved.normal_force1 = msg.normal_force1;
    }
    else {
      resolved.normal_force1 = 0
    }

    if (msg.normal_force2 !== undefined) {
      resolved.normal_force2 = msg.normal_force2;
    }
    else {
      resolved.normal_force2 = 0
    }

    if (msg.normal_force3 !== undefined) {
      resolved.normal_force3 = msg.normal_force3;
    }
    else {
      resolved.normal_force3 = 0
    }

    if (msg.tangential_force1 !== undefined) {
      resolved.tangential_force1 = msg.tangential_force1;
    }
    else {
      resolved.tangential_force1 = 0
    }

    if (msg.tangential_force2 !== undefined) {
      resolved.tangential_force2 = msg.tangential_force2;
    }
    else {
      resolved.tangential_force2 = 0
    }

    if (msg.tangential_force3 !== undefined) {
      resolved.tangential_force3 = msg.tangential_force3;
    }
    else {
      resolved.tangential_force3 = 0
    }

    if (msg.tangential_direction1 !== undefined) {
      resolved.tangential_direction1 = msg.tangential_direction1;
    }
    else {
      resolved.tangential_direction1 = 0
    }

    if (msg.tangential_direction2 !== undefined) {
      resolved.tangential_direction2 = msg.tangential_direction2;
    }
    else {
      resolved.tangential_direction2 = 0
    }

    if (msg.tangential_direction3 !== undefined) {
      resolved.tangential_direction3 = msg.tangential_direction3;
    }
    else {
      resolved.tangential_direction3 = 0
    }

    if (msg.self_proximity1 !== undefined) {
      resolved.self_proximity1 = msg.self_proximity1;
    }
    else {
      resolved.self_proximity1 = 0
    }

    if (msg.self_proximity2 !== undefined) {
      resolved.self_proximity2 = msg.self_proximity2;
    }
    else {
      resolved.self_proximity2 = 0
    }

    if (msg.mutual_proximity !== undefined) {
      resolved.mutual_proximity = msg.mutual_proximity;
    }
    else {
      resolved.mutual_proximity = 0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

module.exports = touchSensorStatus;
