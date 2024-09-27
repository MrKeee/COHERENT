// Auto-generated. Do not edit!

// (in-package hademo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ResultInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.has_result = null;
      this.success = null;
      this.info = null;
    }
    else {
      if (initObj.hasOwnProperty('has_result')) {
        this.has_result = initObj.has_result
      }
      else {
        this.has_result = false;
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('info')) {
        this.info = initObj.info
      }
      else {
        this.info = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResultInfo
    // Serialize message field [has_result]
    bufferOffset = _serializer.bool(obj.has_result, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [info]
    bufferOffset = _serializer.string(obj.info, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResultInfo
    let len;
    let data = new ResultInfo(null);
    // Deserialize message field [has_result]
    data.has_result = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [info]
    data.info = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.info);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hademo/ResultInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc965c583533320bba311cfc20577920';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool has_result
    bool success
    string info
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResultInfo(null);
    if (msg.has_result !== undefined) {
      resolved.has_result = msg.has_result;
    }
    else {
      resolved.has_result = false
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.info !== undefined) {
      resolved.info = msg.info;
    }
    else {
      resolved.info = ''
    }

    return resolved;
    }
};

module.exports = ResultInfo;
