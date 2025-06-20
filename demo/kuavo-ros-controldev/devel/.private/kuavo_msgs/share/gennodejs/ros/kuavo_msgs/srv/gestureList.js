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

let gestureInfo = require('../msg/gestureInfo.js');

//-----------------------------------------------------------

class gestureListRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gestureListRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gestureListRequest
    let len;
    let data = new gestureListRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/gestureListRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This service returns a list of available gestures.
    # It is used to query the system for all gestures that can be recognized or performed.
    #
    # Request:
    # No input parameters are required.
    #
    # Response:
    # bool success                # Indicates whether the request was successful.
    # int32 gesture_count         # The number of gestures returned in the list.
    # string message              # A message indicating the result of the request.
    # kuavo_msgs/gestureInfo[] gesture_infos # A list of gesture information, each containing the name, alias, and description of a gesture.
    
    # Define the GestureInfo message
    # string gesture_name        # The name of the gesture.
    # string[] alias             # A list of aliases for the gesture.
    # string description         # A description of the gesture.
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gestureListRequest(null);
    return resolved;
    }
};

class gestureListResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.gesture_count = null;
      this.message = null;
      this.gesture_infos = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('gesture_count')) {
        this.gesture_count = initObj.gesture_count
      }
      else {
        this.gesture_count = 0;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('gesture_infos')) {
        this.gesture_infos = initObj.gesture_infos
      }
      else {
        this.gesture_infos = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gestureListResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [gesture_count]
    bufferOffset = _serializer.int32(obj.gesture_count, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [gesture_infos]
    // Serialize the length for message field [gesture_infos]
    bufferOffset = _serializer.uint32(obj.gesture_infos.length, buffer, bufferOffset);
    obj.gesture_infos.forEach((val) => {
      bufferOffset = gestureInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gestureListResponse
    let len;
    let data = new gestureListResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gesture_count]
    data.gesture_count = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [gesture_infos]
    // Deserialize array length for message field [gesture_infos]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.gesture_infos = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.gesture_infos[i] = gestureInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    object.gesture_infos.forEach((val) => {
      length += gestureInfo.getMessageSize(val);
    });
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/gestureListResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ee839a0568f441526fe05bf2e2f25572';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    int32 gesture_count
    string message
    kuavo_msgs/gestureInfo[] gesture_infos
    
    ================================================================================
    MSG: kuavo_msgs/gestureInfo
    # This message defines the information for a single gesture.
    # It includes the name, a list of aliases, and a description of the gesture.
    
    # The name of the gesture.
    string gesture_name
    
    # A list of aliases for the gesture. These can be alternative names or shortcuts.
    string[] alias
    
    # A description of the gesture, providing more detailed information about its purpose and usage.
    string description
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gestureListResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.gesture_count !== undefined) {
      resolved.gesture_count = msg.gesture_count;
    }
    else {
      resolved.gesture_count = 0
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.gesture_infos !== undefined) {
      resolved.gesture_infos = new Array(msg.gesture_infos.length);
      for (let i = 0; i < resolved.gesture_infos.length; ++i) {
        resolved.gesture_infos[i] = gestureInfo.Resolve(msg.gesture_infos[i]);
      }
    }
    else {
      resolved.gesture_infos = []
    }

    return resolved;
    }
};

module.exports = {
  Request: gestureListRequest,
  Response: gestureListResponse,
  md5sum() { return 'ee839a0568f441526fe05bf2e2f25572'; },
  datatype() { return 'kuavo_msgs/gestureList'; }
};
