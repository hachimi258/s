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

class enableHandTouchSensorRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mask = null;
    }
    else {
      if (initObj.hasOwnProperty('mask')) {
        this.mask = initObj.mask
      }
      else {
        this.mask = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type enableHandTouchSensorRequest
    // Serialize message field [mask]
    bufferOffset = _serializer.uint8(obj.mask, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type enableHandTouchSensorRequest
    let len;
    let data = new enableHandTouchSensorRequest(null);
    // Deserialize message field [mask]
    data.mask = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_msgs/enableHandTouchSensorRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0543438552cc194d7406fa44a80edec7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Bit masks for enabling individual touch sensors on the robotic hand
    # Each sensor can be enabled by setting the corresponding bit in the mask
    
    # Examples: 0b00000011 enables thumb and index sensors, 0b00000000 disables all sensors
    #  ``` 
    #    mask_value = THUMB_SENSOR | INDEX_SENSOR
    #    req = enableHandTouchSensorRequest()
    #    req.mask = mask_value
    #  ``` 
    
    # Thumb finger touch sensor (bit 0)
    uint8 THUMB_SENSOR = 1
    
    # Index finger touch sensor (bit 1)
    uint8 INDEX_SENSOR = 2
    
    # Middle finger touch sensor (bit 2)
    uint8 MIDDLE_SENSOR = 4
    
    # Ring finger touch sensor (bit 3)
    uint8 RING_SENSOR = 8
    
    # Pinky finger touch sensor (bit 4)
    uint8 PINKY_SENSOR = 16
    
    # Bitmask indicating which sensors to enable
    # Multiple sensors can be enabled by combining masks with bitwise OR
    uint8 mask
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new enableHandTouchSensorRequest(null);
    if (msg.mask !== undefined) {
      resolved.mask = msg.mask;
    }
    else {
      resolved.mask = 0
    }

    return resolved;
    }
};

// Constants for message
enableHandTouchSensorRequest.Constants = {
  THUMB_SENSOR: 1,
  INDEX_SENSOR: 2,
  MIDDLE_SENSOR: 4,
  RING_SENSOR: 8,
  PINKY_SENSOR: 16,
}

class enableHandTouchSensorResponse {
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
    // Serializes a message object of type enableHandTouchSensorResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type enableHandTouchSensorResponse
    let len;
    let data = new enableHandTouchSensorResponse(null);
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
    return 'kuavo_msgs/enableHandTouchSensorResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Whether the operation was successful
    bool success
    
    # Additional status or error message
    string message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new enableHandTouchSensorResponse(null);
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
  Request: enableHandTouchSensorRequest,
  Response: enableHandTouchSensorResponse,
  md5sum() { return '710f3d70d245856e41b01f7ffef21580'; },
  datatype() { return 'kuavo_msgs/enableHandTouchSensor'; }
};
