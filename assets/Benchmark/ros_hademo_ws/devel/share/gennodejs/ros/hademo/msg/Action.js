// Auto-generated. Do not edit!

// (in-package hademo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Func_and_Args = require('./Func_and_Args.js');

//-----------------------------------------------------------

class Action {
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
        this.franka_0 = new Func_and_Args();
      }
      if (initObj.hasOwnProperty('franka_1')) {
        this.franka_1 = initObj.franka_1
      }
      else {
        this.franka_1 = new Func_and_Args();
      }
      if (initObj.hasOwnProperty('franka_2')) {
        this.franka_2 = initObj.franka_2
      }
      else {
        this.franka_2 = new Func_and_Args();
      }
      if (initObj.hasOwnProperty('aliengo_0')) {
        this.aliengo_0 = initObj.aliengo_0
      }
      else {
        this.aliengo_0 = new Func_and_Args();
      }
      if (initObj.hasOwnProperty('aliengo_1')) {
        this.aliengo_1 = initObj.aliengo_1
      }
      else {
        this.aliengo_1 = new Func_and_Args();
      }
      if (initObj.hasOwnProperty('aliengo_2')) {
        this.aliengo_2 = initObj.aliengo_2
      }
      else {
        this.aliengo_2 = new Func_and_Args();
      }
      if (initObj.hasOwnProperty('quadrotor_0')) {
        this.quadrotor_0 = initObj.quadrotor_0
      }
      else {
        this.quadrotor_0 = new Func_and_Args();
      }
      if (initObj.hasOwnProperty('quadrotor_1')) {
        this.quadrotor_1 = initObj.quadrotor_1
      }
      else {
        this.quadrotor_1 = new Func_and_Args();
      }
      if (initObj.hasOwnProperty('quadrotor_2')) {
        this.quadrotor_2 = initObj.quadrotor_2
      }
      else {
        this.quadrotor_2 = new Func_and_Args();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Action
    // Serialize message field [franka_0]
    bufferOffset = Func_and_Args.serialize(obj.franka_0, buffer, bufferOffset);
    // Serialize message field [franka_1]
    bufferOffset = Func_and_Args.serialize(obj.franka_1, buffer, bufferOffset);
    // Serialize message field [franka_2]
    bufferOffset = Func_and_Args.serialize(obj.franka_2, buffer, bufferOffset);
    // Serialize message field [aliengo_0]
    bufferOffset = Func_and_Args.serialize(obj.aliengo_0, buffer, bufferOffset);
    // Serialize message field [aliengo_1]
    bufferOffset = Func_and_Args.serialize(obj.aliengo_1, buffer, bufferOffset);
    // Serialize message field [aliengo_2]
    bufferOffset = Func_and_Args.serialize(obj.aliengo_2, buffer, bufferOffset);
    // Serialize message field [quadrotor_0]
    bufferOffset = Func_and_Args.serialize(obj.quadrotor_0, buffer, bufferOffset);
    // Serialize message field [quadrotor_1]
    bufferOffset = Func_and_Args.serialize(obj.quadrotor_1, buffer, bufferOffset);
    // Serialize message field [quadrotor_2]
    bufferOffset = Func_and_Args.serialize(obj.quadrotor_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Action
    let len;
    let data = new Action(null);
    // Deserialize message field [franka_0]
    data.franka_0 = Func_and_Args.deserialize(buffer, bufferOffset);
    // Deserialize message field [franka_1]
    data.franka_1 = Func_and_Args.deserialize(buffer, bufferOffset);
    // Deserialize message field [franka_2]
    data.franka_2 = Func_and_Args.deserialize(buffer, bufferOffset);
    // Deserialize message field [aliengo_0]
    data.aliengo_0 = Func_and_Args.deserialize(buffer, bufferOffset);
    // Deserialize message field [aliengo_1]
    data.aliengo_1 = Func_and_Args.deserialize(buffer, bufferOffset);
    // Deserialize message field [aliengo_2]
    data.aliengo_2 = Func_and_Args.deserialize(buffer, bufferOffset);
    // Deserialize message field [quadrotor_0]
    data.quadrotor_0 = Func_and_Args.deserialize(buffer, bufferOffset);
    // Deserialize message field [quadrotor_1]
    data.quadrotor_1 = Func_and_Args.deserialize(buffer, bufferOffset);
    // Deserialize message field [quadrotor_2]
    data.quadrotor_2 = Func_and_Args.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Func_and_Args.getMessageSize(object.franka_0);
    length += Func_and_Args.getMessageSize(object.franka_1);
    length += Func_and_Args.getMessageSize(object.franka_2);
    length += Func_and_Args.getMessageSize(object.aliengo_0);
    length += Func_and_Args.getMessageSize(object.aliengo_1);
    length += Func_and_Args.getMessageSize(object.aliengo_2);
    length += Func_and_Args.getMessageSize(object.quadrotor_0);
    length += Func_and_Args.getMessageSize(object.quadrotor_1);
    length += Func_and_Args.getMessageSize(object.quadrotor_2);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hademo/Action';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd976dee5559eb23c2568d9c0e79066c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    hademo/Func_and_Args franka_0
    hademo/Func_and_Args franka_1
    hademo/Func_and_Args franka_2
    hademo/Func_and_Args aliengo_0
    hademo/Func_and_Args aliengo_1
    hademo/Func_and_Args aliengo_2
    hademo/Func_and_Args quadrotor_0
    hademo/Func_and_Args quadrotor_1
    hademo/Func_and_Args quadrotor_2
    
    ================================================================================
    MSG: hademo/Func_and_Args
    bool has_func
    string func_name
    hademo/Args args
    
    ================================================================================
    MSG: hademo/Args
    bool has_args
    string attached_prim_path
    std_msgs/Float64MultiArray waypoint_pos
    std_msgs/Float64MultiArray waypoint_ori
    int32 waypoint_ind
    
    ================================================================================
    MSG: std_msgs/Float64MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float64[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Action(null);
    if (msg.franka_0 !== undefined) {
      resolved.franka_0 = Func_and_Args.Resolve(msg.franka_0)
    }
    else {
      resolved.franka_0 = new Func_and_Args()
    }

    if (msg.franka_1 !== undefined) {
      resolved.franka_1 = Func_and_Args.Resolve(msg.franka_1)
    }
    else {
      resolved.franka_1 = new Func_and_Args()
    }

    if (msg.franka_2 !== undefined) {
      resolved.franka_2 = Func_and_Args.Resolve(msg.franka_2)
    }
    else {
      resolved.franka_2 = new Func_and_Args()
    }

    if (msg.aliengo_0 !== undefined) {
      resolved.aliengo_0 = Func_and_Args.Resolve(msg.aliengo_0)
    }
    else {
      resolved.aliengo_0 = new Func_and_Args()
    }

    if (msg.aliengo_1 !== undefined) {
      resolved.aliengo_1 = Func_and_Args.Resolve(msg.aliengo_1)
    }
    else {
      resolved.aliengo_1 = new Func_and_Args()
    }

    if (msg.aliengo_2 !== undefined) {
      resolved.aliengo_2 = Func_and_Args.Resolve(msg.aliengo_2)
    }
    else {
      resolved.aliengo_2 = new Func_and_Args()
    }

    if (msg.quadrotor_0 !== undefined) {
      resolved.quadrotor_0 = Func_and_Args.Resolve(msg.quadrotor_0)
    }
    else {
      resolved.quadrotor_0 = new Func_and_Args()
    }

    if (msg.quadrotor_1 !== undefined) {
      resolved.quadrotor_1 = Func_and_Args.Resolve(msg.quadrotor_1)
    }
    else {
      resolved.quadrotor_1 = new Func_and_Args()
    }

    if (msg.quadrotor_2 !== undefined) {
      resolved.quadrotor_2 = Func_and_Args.Resolve(msg.quadrotor_2)
    }
    else {
      resolved.quadrotor_2 = new Func_and_Args()
    }

    return resolved;
    }
};

module.exports = Action;
