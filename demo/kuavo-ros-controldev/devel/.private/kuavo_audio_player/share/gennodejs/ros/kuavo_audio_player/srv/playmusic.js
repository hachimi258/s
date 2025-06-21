// Auto-generated. Do not edit!

// (in-package kuavo_audio_player.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class playmusicRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.music_number = null;
      this.volume = null;
      this.speed = null;
    }
    else {
      if (initObj.hasOwnProperty('music_number')) {
        this.music_number = initObj.music_number
      }
      else {
        this.music_number = '';
      }
      if (initObj.hasOwnProperty('volume')) {
        this.volume = initObj.volume
      }
      else {
        this.volume = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type playmusicRequest
    // Serialize message field [music_number]
    bufferOffset = _serializer.string(obj.music_number, buffer, bufferOffset);
    // Serialize message field [volume]
    bufferOffset = _serializer.float32(obj.volume, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type playmusicRequest
    let len;
    let data = new playmusicRequest(null);
    // Deserialize message field [music_number]
    data.music_number = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [volume]
    data.volume = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.music_number);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_audio_player/playmusicRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '24067736323ff98005697d24ca92721f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string music_number
    float32 volume
    float32 speed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new playmusicRequest(null);
    if (msg.music_number !== undefined) {
      resolved.music_number = msg.music_number;
    }
    else {
      resolved.music_number = ''
    }

    if (msg.volume !== undefined) {
      resolved.volume = msg.volume;
    }
    else {
      resolved.volume = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    return resolved;
    }
};

class playmusicResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('success_flag')) {
        this.success_flag = initObj.success_flag
      }
      else {
        this.success_flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type playmusicResponse
    // Serialize message field [success_flag]
    bufferOffset = _serializer.bool(obj.success_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type playmusicResponse
    let len;
    let data = new playmusicResponse(null);
    // Deserialize message field [success_flag]
    data.success_flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_audio_player/playmusicResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '51d4deda6e3cbea57b8c79590b6cd9bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success_flag
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new playmusicResponse(null);
    if (msg.success_flag !== undefined) {
      resolved.success_flag = msg.success_flag;
    }
    else {
      resolved.success_flag = false
    }

    return resolved;
    }
};

module.exports = {
  Request: playmusicRequest,
  Response: playmusicResponse,
  md5sum() { return '348f2445d1e569f5dc4e72a19fb11934'; },
  datatype() { return 'kuavo_audio_player/playmusic'; }
};
