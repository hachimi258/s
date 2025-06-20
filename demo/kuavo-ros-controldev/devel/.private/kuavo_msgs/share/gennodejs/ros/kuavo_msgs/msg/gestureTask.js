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

class gestureTask {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gesture_name = null;
      this.hand_side = null;
    }
    else {
      if (initObj.hasOwnProperty('gesture_name')) {
        this.gesture_name = initObj.gesture_name
      }
      else {
        this.gesture_name = '';
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
    // Serializes a message object of type gestureTask
    // Serialize message field [gesture_name]
    bufferOffset = _serializer.string(obj.gesture_name, buffer, bufferOffset);
    // Serialize message field [hand_side]
    bufferOffset = _serializer.int8(obj.hand_side, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gestureTask
    let len;
    let data = new gestureTask(null);
    // Deserialize message field [gesture_name]
    data.gesture_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [hand_side]
    data.hand_side = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.gesture_name);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kuavo_msgs/gestureTask';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'be7fe1eba1df13c392c3a5d13b9f3dae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new gestureTask(null);
    if (msg.gesture_name !== undefined) {
      resolved.gesture_name = msg.gesture_name;
    }
    else {
      resolved.gesture_name = ''
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

module.exports = gestureTask;
