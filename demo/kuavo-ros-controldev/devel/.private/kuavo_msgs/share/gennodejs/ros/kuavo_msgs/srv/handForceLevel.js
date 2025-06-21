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

class handForceLevelRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.force_level = null;
      this.hand_side = null;
    }
    else {
      if (initObj.hasOwnProperty('force_level')) {
        this.force_level = initObj.force_level
      }
      else {
        this.force_level = 0;
      }
      if (initObj.hasOwnProperty('hand_side')) {
        this.hand_side = initObj.hand_side
      }
      else {
        this.hand_side = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type handForceLevelRequest
    // Serialize message field [force_level]
    bufferOffset = _serializer.int8(obj.force_level, buffer, bufferOffset);
    // Serialize message field [hand_side]
    bufferOffset = _serializer.int8(obj.hand_side, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type handForceLevelRequest
    let len;
    let data = new handForceLevelRequest(null);
    // Deserialize message field [force_level]
    data.force_level = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [hand_side]
    data.hand_side = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/handForceLevelRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5311779bc3f9aed0d69c0597dc982fd7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This service sets the force level for the Dexhand.
    # It is used to control the force applied by the Dexhand.
    #
    # Request:
    # int8 SMALL = 0          # Small force level.
    # int8 NORMAL = 1         # Normal force level.
    # int8 FULL = 2           # Full force level.
    # int8 force_level        # The desired force level to set.
    #
    # Response:
    # bool success            # Indicates whether the request was successful.
    # string message          # A message indicating the result of the request.
    
    int8 SMALL = 0
    int8 NORMAL = 1
    int8 FULL = 2
    
    int8 LEFT_HAND=0
    int8 RIGHT_HAND=1
    int8 BOTH_HANDS=2
    
    int8 force_level
    int8 hand_side
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new handForceLevelRequest(null);
    if (msg.force_level !== undefined) {
      resolved.force_level = msg.force_level;
    }
    else {
      resolved.force_level = 0
    }

    if (msg.hand_side !== undefined) {
      resolved.hand_side = msg.hand_side;
    }
    else {
      resolved.hand_side = 0
    }

    return resolved;
    }
};

// Constants for message
handForceLevelRequest.Constants = {
  SMALL: 0,
  NORMAL: 1,
  FULL: 2,
  LEFT_HAND: 0,
  RIGHT_HAND: 1,
  BOTH_HANDS: 2,
}

class handForceLevelResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
      this.force_level = null;
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
      if (initObj.hasOwnProperty('force_level')) {
        this.force_level = initObj.force_level
      }
      else {
        this.force_level = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type handForceLevelResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [force_level]
    bufferOffset = _arraySerializer.int8(obj.force_level, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type handForceLevelResponse
    let len;
    let data = new handForceLevelResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [force_level]
    data.force_level = _arrayDeserializer.int8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    length += object.force_level.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/handForceLevelResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '77e365aa913b429c04069f41b0a0fea7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string message
    int8[] force_level
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new handForceLevelResponse(null);
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

    if (msg.force_level !== undefined) {
      resolved.force_level = msg.force_level;
    }
    else {
      resolved.force_level = []
    }

    return resolved;
    }
};

module.exports = {
  Request: handForceLevelRequest,
  Response: handForceLevelResponse,
  md5sum() { return '0f6c4fd291557ab445334f0487fbaf78'; },
  datatype() { return 'kuavo_msgs/handForceLevel'; }
};
