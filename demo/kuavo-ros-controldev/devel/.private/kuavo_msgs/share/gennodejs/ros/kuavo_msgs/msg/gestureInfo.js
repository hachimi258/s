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

class gestureInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gesture_name = null;
      this.alias = null;
      this.description = null;
    }
    else {
      if (initObj.hasOwnProperty('gesture_name')) {
        this.gesture_name = initObj.gesture_name
      }
      else {
        this.gesture_name = '';
      }
      if (initObj.hasOwnProperty('alias')) {
        this.alias = initObj.alias
      }
      else {
        this.alias = [];
      }
      if (initObj.hasOwnProperty('description')) {
        this.description = initObj.description
      }
      else {
        this.description = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gestureInfo
    // Serialize message field [gesture_name]
    bufferOffset = _serializer.string(obj.gesture_name, buffer, bufferOffset);
    // Serialize message field [alias]
    bufferOffset = _arraySerializer.string(obj.alias, buffer, bufferOffset, null);
    // Serialize message field [description]
    bufferOffset = _serializer.string(obj.description, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gestureInfo
    let len;
    let data = new gestureInfo(null);
    // Deserialize message field [gesture_name]
    data.gesture_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [alias]
    data.alias = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [description]
    data.description = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.gesture_name);
    object.alias.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += _getByteLength(object.description);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kuavo_msgs/gestureInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '65efb896db2f0292354e0a9098b39b97';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new gestureInfo(null);
    if (msg.gesture_name !== undefined) {
      resolved.gesture_name = msg.gesture_name;
    }
    else {
      resolved.gesture_name = ''
    }

    if (msg.alias !== undefined) {
      resolved.alias = msg.alias;
    }
    else {
      resolved.alias = []
    }

    if (msg.description !== undefined) {
      resolved.description = msg.description;
    }
    else {
      resolved.description = ''
    }

    return resolved;
    }
};

module.exports = gestureInfo;
