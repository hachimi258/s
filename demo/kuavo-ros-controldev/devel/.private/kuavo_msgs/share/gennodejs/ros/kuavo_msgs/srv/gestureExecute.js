// Auto-generated. Do not edit!

// (in-package kuavo_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let gestureTask = require('../msg/gestureTask.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class gestureExecuteRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gestures = null;
    }
    else {
      if (initObj.hasOwnProperty('gestures')) {
        this.gestures = initObj.gestures
      }
      else {
        this.gestures = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gestureExecuteRequest
    // Serialize message field [gestures]
    // Serialize the length for message field [gestures]
    bufferOffset = _serializer.uint32(obj.gestures.length, buffer, bufferOffset);
    obj.gestures.forEach((val) => {
      bufferOffset = gestureTask.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gestureExecuteRequest
    let len;
    let data = new gestureExecuteRequest(null);
    // Deserialize message field [gestures]
    // Deserialize array length for message field [gestures]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.gestures = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.gestures[i] = gestureTask.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.gestures.forEach((val) => {
      length += gestureTask.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/gestureExecuteRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '686baa778a78aa01b0a6cc9824ebcd9f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This service executes a specified gesture.
    # It is used to trigger a gesture by providing its name and the side of the hand(s) to use.
    #
    # Request:
    # kuavo_msgs/gestureTask[] gestures # An array of gestures to execute, each with a name and hand side
    #
    # Response:
    # bool success         # Indicates whether the gesture execution was successful.
    # string message       # A message providing additional information (e.g., error details if the gesture failed).
    
    kuavo_msgs/gestureTask[] gestures
    
    ================================================================================
    MSG: kuavo_msgs/gestureTask
    # This message is used to specify a gesture to execute.
    # The gesture is triggered by providing its name and the side of the hand(s) to use.
    
    string gesture_name  # Name of the gesture to execute
    int8   hand_side    # Side of the hand to use (e.g., 0 for left, 1 for right, 2 for both)
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gestureExecuteRequest(null);
    if (msg.gestures !== undefined) {
      resolved.gestures = new Array(msg.gestures.length);
      for (let i = 0; i < resolved.gestures.length; ++i) {
        resolved.gestures[i] = gestureTask.Resolve(msg.gestures[i]);
      }
    }
    else {
      resolved.gestures = []
    }

    return resolved;
    }
};

class gestureExecuteResponse {
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
    // Serializes a message object of type gestureExecuteResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gestureExecuteResponse
    let len;
    let data = new gestureExecuteResponse(null);
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
    return 'kuavo_msgs/gestureExecuteResponse';
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
    const resolved = new gestureExecuteResponse(null);
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
  Request: gestureExecuteRequest,
  Response: gestureExecuteResponse,
  md5sum() { return 'b599da36839d439975fbac8d4bfbeb7e'; },
  datatype() { return 'kuavo_msgs/gestureExecute'; }
};
