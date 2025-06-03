// Auto-generated. Do not edit!

// (in-package kuavo_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let endEffectorData = require('../msg/endEffectorData.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class controlLejuClawRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new endEffectorData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type controlLejuClawRequest
    // Serialize message field [data]
    bufferOffset = endEffectorData.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type controlLejuClawRequest
    let len;
    let data = new controlLejuClawRequest(null);
    // Deserialize message field [data]
    data.data = endEffectorData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += endEffectorData.getMessageSize(object.data);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/controlLejuClawRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4255ea8183b49bda4b8fed5d1dd8d5b9';
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
    # ** For the Service Notes **
    # 
    # name     : 'left_claw' , 'right_claw'
    # position : 0 ~ 100, the percentage of the claw's opening angle
    #            0: closed, 100: open   
    # velocity : 0 ~ 100, if size is 0, will use default `50.0`.
    # effort   : torque/current, better 1A ~ 2A, if size is 0, will use default `1.0`.
    # 
    # ** Example **
    # Request:
    # data:
    #   - name: ['left_claw', 'right_claw']
    #     position: [20.0, 20.0]
    #     velocity: [50.0, 50.0]
    #     effort: [1.0, 1.0]
    #
    # Response:
    # success: True/False, call service success or not.
    # message: 'success'
    kuavo_msgs/endEffectorData data
    
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
    const resolved = new controlLejuClawRequest(null);
    if (msg.data !== undefined) {
      resolved.data = endEffectorData.Resolve(msg.data)
    }
    else {
      resolved.data = new endEffectorData()
    }

    return resolved;
    }
};

class controlLejuClawResponse {
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
    // Serializes a message object of type controlLejuClawResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type controlLejuClawResponse
    let len;
    let data = new controlLejuClawResponse(null);
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
    return 'kuavo_msgs/controlLejuClawResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new controlLejuClawResponse(null);
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
  Request: controlLejuClawRequest,
  Response: controlLejuClawResponse,
  md5sum() { return '674277f611b34c602b5afcc4b45849d1'; },
  datatype() { return 'kuavo_msgs/controlLejuClaw'; }
};
