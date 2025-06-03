// Auto-generated. Do not edit!

// (in-package kuavo_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class tagDataArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tag_ids = null;
      this.tag_poses = null;
    }
    else {
      if (initObj.hasOwnProperty('tag_ids')) {
        this.tag_ids = initObj.tag_ids
      }
      else {
        this.tag_ids = [];
      }
      if (initObj.hasOwnProperty('tag_poses')) {
        this.tag_poses = initObj.tag_poses
      }
      else {
        this.tag_poses = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type tagDataArray
    // Serialize message field [tag_ids]
    bufferOffset = _arraySerializer.int32(obj.tag_ids, buffer, bufferOffset, null);
    // Serialize message field [tag_poses]
    // Serialize the length for message field [tag_poses]
    bufferOffset = _serializer.uint32(obj.tag_poses.length, buffer, bufferOffset);
    obj.tag_poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type tagDataArray
    let len;
    let data = new tagDataArray(null);
    // Deserialize message field [tag_ids]
    data.tag_ids = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [tag_poses]
    // Deserialize array length for message field [tag_poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tag_poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tag_poses[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.tag_ids.length;
    length += 56 * object.tag_poses.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kuavo_msgs/tagDataArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '454da6edf551b421dda595d3272ef7ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] tag_ids
    geometry_msgs/Pose[] tag_poses
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new tagDataArray(null);
    if (msg.tag_ids !== undefined) {
      resolved.tag_ids = msg.tag_ids;
    }
    else {
      resolved.tag_ids = []
    }

    if (msg.tag_poses !== undefined) {
      resolved.tag_poses = new Array(msg.tag_poses.length);
      for (let i = 0; i < resolved.tag_poses.length; ++i) {
        resolved.tag_poses[i] = geometry_msgs.msg.Pose.Resolve(msg.tag_poses[i]);
      }
    }
    else {
      resolved.tag_poses = []
    }

    return resolved;
    }
};

module.exports = tagDataArray;
