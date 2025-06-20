// Auto-generated. Do not edit!

// (in-package kuavo_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class setMotorEncoderRoundServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.motor_id = null;
      this.direction = null;
      this.save_offset = null;
    }
    else {
      if (initObj.hasOwnProperty('motor_id')) {
        this.motor_id = initObj.motor_id
      }
      else {
        this.motor_id = 0;
      }
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = 0;
      }
      if (initObj.hasOwnProperty('save_offset')) {
        this.save_offset = initObj.save_offset
      }
      else {
        this.save_offset = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setMotorEncoderRoundServiceRequest
    // Serialize message field [motor_id]
    bufferOffset = _serializer.int32(obj.motor_id, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = _serializer.int32(obj.direction, buffer, bufferOffset);
    // Serialize message field [save_offset]
    bufferOffset = _serializer.bool(obj.save_offset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setMotorEncoderRoundServiceRequest
    let len;
    let data = new setMotorEncoderRoundServiceRequest(null);
    // Deserialize message field [motor_id]
    data.motor_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [save_offset]
    data.save_offset = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/setMotorEncoderRoundServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5104e77e6b69ebe8cb1f06d5c2f1d52a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 设置电机编码器圈数，用于单编码器版本的校准
    # 传入参数
    int32 motor_id     # 电机 ID
    int32 direction    # 方向，1 表示顺时针+1圈，-1 表示逆时针-1圈
    bool save_offset   # 是否保存偏移量到文件
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setMotorEncoderRoundServiceRequest(null);
    if (msg.motor_id !== undefined) {
      resolved.motor_id = msg.motor_id;
    }
    else {
      resolved.motor_id = 0
    }

    if (msg.direction !== undefined) {
      resolved.direction = msg.direction;
    }
    else {
      resolved.direction = 0
    }

    if (msg.save_offset !== undefined) {
      resolved.save_offset = msg.save_offset;
    }
    else {
      resolved.save_offset = false
    }

    return resolved;
    }
};

class setMotorEncoderRoundServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setMotorEncoderRoundServiceResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setMotorEncoderRoundServiceResponse
    let len;
    let data = new setMotorEncoderRoundServiceResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/setMotorEncoderRoundServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 返回结果
    bool success       # 成功与否
    string message     # 返回信息
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setMotorEncoderRoundServiceResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: setMotorEncoderRoundServiceRequest,
  Response: setMotorEncoderRoundServiceResponse,
  md5sum() { return 'c8e2df10122ff9a180b403f54a3f8831'; },
  datatype() { return 'kuavo_msgs/setMotorEncoderRoundService'; }
};
