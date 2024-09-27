// Auto-generated. Do not edit!

// (in-package hademo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ResultInfo = require('./ResultInfo.js');

//-----------------------------------------------------------

class Result {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.franka_0 = null;
      this.franka_1 = null;
      this.franka_2 = null;
      this.aliengo_0 = null;
      this.aliengo_1 = null;
      this.aliengo_2 = null;
      this.quadrotor_0 = null;
      this.quadrotor_1 = null;
      this.quadrotor_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('franka_0')) {
        this.franka_0 = initObj.franka_0
      }
      else {
        this.franka_0 = new ResultInfo();
      }
      if (initObj.hasOwnProperty('franka_1')) {
        this.franka_1 = initObj.franka_1
      }
      else {
        this.franka_1 = new ResultInfo();
      }
      if (initObj.hasOwnProperty('franka_2')) {
        this.franka_2 = initObj.franka_2
      }
      else {
        this.franka_2 = new ResultInfo();
      }
      if (initObj.hasOwnProperty('aliengo_0')) {
        this.aliengo_0 = initObj.aliengo_0
      }
      else {
        this.aliengo_0 = new ResultInfo();
      }
      if (initObj.hasOwnProperty('aliengo_1')) {
        this.aliengo_1 = initObj.aliengo_1
      }
      else {
        this.aliengo_1 = new ResultInfo();
      }
      if (initObj.hasOwnProperty('aliengo_2')) {
        this.aliengo_2 = initObj.aliengo_2
      }
      else {
        this.aliengo_2 = new ResultInfo();
      }
      if (initObj.hasOwnProperty('quadrotor_0')) {
        this.quadrotor_0 = initObj.quadrotor_0
      }
      else {
        this.quadrotor_0 = new ResultInfo();
      }
      if (initObj.hasOwnProperty('quadrotor_1')) {
        this.quadrotor_1 = initObj.quadrotor_1
      }
      else {
        this.quadrotor_1 = new ResultInfo();
      }
      if (initObj.hasOwnProperty('quadrotor_2')) {
        this.quadrotor_2 = initObj.quadrotor_2
      }
      else {
        this.quadrotor_2 = new ResultInfo();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Result
    // Serialize message field [franka_0]
    bufferOffset = ResultInfo.serialize(obj.franka_0, buffer, bufferOffset);
    // Serialize message field [franka_1]
    bufferOffset = ResultInfo.serialize(obj.franka_1, buffer, bufferOffset);
    // Serialize message field [franka_2]
    bufferOffset = ResultInfo.serialize(obj.franka_2, buffer, bufferOffset);
    // Serialize message field [aliengo_0]
    bufferOffset = ResultInfo.serialize(obj.aliengo_0, buffer, bufferOffset);
    // Serialize message field [aliengo_1]
    bufferOffset = ResultInfo.serialize(obj.aliengo_1, buffer, bufferOffset);
    // Serialize message field [aliengo_2]
    bufferOffset = ResultInfo.serialize(obj.aliengo_2, buffer, bufferOffset);
    // Serialize message field [quadrotor_0]
    bufferOffset = ResultInfo.serialize(obj.quadrotor_0, buffer, bufferOffset);
    // Serialize message field [quadrotor_1]
    bufferOffset = ResultInfo.serialize(obj.quadrotor_1, buffer, bufferOffset);
    // Serialize message field [quadrotor_2]
    bufferOffset = ResultInfo.serialize(obj.quadrotor_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Result
    let len;
    let data = new Result(null);
    // Deserialize message field [franka_0]
    data.franka_0 = ResultInfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [franka_1]
    data.franka_1 = ResultInfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [franka_2]
    data.franka_2 = ResultInfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [aliengo_0]
    data.aliengo_0 = ResultInfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [aliengo_1]
    data.aliengo_1 = ResultInfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [aliengo_2]
    data.aliengo_2 = ResultInfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [quadrotor_0]
    data.quadrotor_0 = ResultInfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [quadrotor_1]
    data.quadrotor_1 = ResultInfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [quadrotor_2]
    data.quadrotor_2 = ResultInfo.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ResultInfo.getMessageSize(object.franka_0);
    length += ResultInfo.getMessageSize(object.franka_1);
    length += ResultInfo.getMessageSize(object.franka_2);
    length += ResultInfo.getMessageSize(object.aliengo_0);
    length += ResultInfo.getMessageSize(object.aliengo_1);
    length += ResultInfo.getMessageSize(object.aliengo_2);
    length += ResultInfo.getMessageSize(object.quadrotor_0);
    length += ResultInfo.getMessageSize(object.quadrotor_1);
    length += ResultInfo.getMessageSize(object.quadrotor_2);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hademo/Result';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a97bcb6cbe98951369cc3755518f34c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    hademo/ResultInfo franka_0
    hademo/ResultInfo franka_1
    hademo/ResultInfo franka_2
    hademo/ResultInfo aliengo_0
    hademo/ResultInfo aliengo_1
    hademo/ResultInfo aliengo_2
    hademo/ResultInfo quadrotor_0
    hademo/ResultInfo quadrotor_1
    hademo/ResultInfo quadrotor_2
    
    ================================================================================
    MSG: hademo/ResultInfo
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
    const resolved = new Result(null);
    if (msg.franka_0 !== undefined) {
      resolved.franka_0 = ResultInfo.Resolve(msg.franka_0)
    }
    else {
      resolved.franka_0 = new ResultInfo()
    }

    if (msg.franka_1 !== undefined) {
      resolved.franka_1 = ResultInfo.Resolve(msg.franka_1)
    }
    else {
      resolved.franka_1 = new ResultInfo()
    }

    if (msg.franka_2 !== undefined) {
      resolved.franka_2 = ResultInfo.Resolve(msg.franka_2)
    }
    else {
      resolved.franka_2 = new ResultInfo()
    }

    if (msg.aliengo_0 !== undefined) {
      resolved.aliengo_0 = ResultInfo.Resolve(msg.aliengo_0)
    }
    else {
      resolved.aliengo_0 = new ResultInfo()
    }

    if (msg.aliengo_1 !== undefined) {
      resolved.aliengo_1 = ResultInfo.Resolve(msg.aliengo_1)
    }
    else {
      resolved.aliengo_1 = new ResultInfo()
    }

    if (msg.aliengo_2 !== undefined) {
      resolved.aliengo_2 = ResultInfo.Resolve(msg.aliengo_2)
    }
    else {
      resolved.aliengo_2 = new ResultInfo()
    }

    if (msg.quadrotor_0 !== undefined) {
      resolved.quadrotor_0 = ResultInfo.Resolve(msg.quadrotor_0)
    }
    else {
      resolved.quadrotor_0 = new ResultInfo()
    }

    if (msg.quadrotor_1 !== undefined) {
      resolved.quadrotor_1 = ResultInfo.Resolve(msg.quadrotor_1)
    }
    else {
      resolved.quadrotor_1 = new ResultInfo()
    }

    if (msg.quadrotor_2 !== undefined) {
      resolved.quadrotor_2 = ResultInfo.Resolve(msg.quadrotor_2)
    }
    else {
      resolved.quadrotor_2 = new ResultInfo()
    }

    return resolved;
    }
};

module.exports = Result;
