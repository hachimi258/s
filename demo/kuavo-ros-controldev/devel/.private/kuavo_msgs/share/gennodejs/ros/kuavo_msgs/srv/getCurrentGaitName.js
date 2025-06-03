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

class getCurrentGaitNameRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getCurrentGaitNameRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getCurrentGaitNameRequest
    let len;
    let data = new getCurrentGaitNameRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/getCurrentGaitNameRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request: Empty message, no fields needed
    
    # Response:
    # success: Whether the service call was successful
    # gait_name: Name of the current gait being executed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getCurrentGaitNameRequest(null);
    return resolved;
    }
};

class getCurrentGaitNameResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.gait_name = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('gait_name')) {
        this.gait_name = initObj.gait_name
      }
      else {
        this.gait_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getCurrentGaitNameResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [gait_name]
    bufferOffset = _serializer.string(obj.gait_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getCurrentGaitNameResponse
    let len;
    let data = new getCurrentGaitNameResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gait_name]
    data.gait_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.gait_name);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/getCurrentGaitNameResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '657f58245742fcfc53dd6ab5bfa3063e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string gait_name
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getCurrentGaitNameResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.gait_name !== undefined) {
      resolved.gait_name = msg.gait_name;
    }
    else {
      resolved.gait_name = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: getCurrentGaitNameRequest,
  Response: getCurrentGaitNameResponse,
  md5sum() { return '657f58245742fcfc53dd6ab5bfa3063e'; },
  datatype() { return 'kuavo_msgs/getCurrentGaitName'; }
};
