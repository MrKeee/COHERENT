// Auto-generated. Do not edit!

// (in-package hademo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Args = require('./Args.js');

//-----------------------------------------------------------

class Func_and_Args {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.has_func = null;
      this.func_name = null;
      this.args = null;
    }
    else {
      if (initObj.hasOwnProperty('has_func')) {
        this.has_func = initObj.has_func
      }
      else {
        this.has_func = false;
      }
      if (initObj.hasOwnProperty('func_name')) {
        this.func_name = initObj.func_name
      }
      else {
        this.func_name = '';
      }
      if (initObj.hasOwnProperty('args')) {
        this.args = initObj.args
      }
      else {
        this.args = new Args();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Func_and_Args
    // Serialize message field [has_func]
    bufferOffset = _serializer.bool(obj.has_func, buffer, bufferOffset);
    // Serialize message field [func_name]
    bufferOffset = _serializer.string(obj.func_name, buffer, bufferOffset);
    // Serialize message field [args]
    bufferOffset = Args.serialize(obj.args, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Func_and_Args
    let len;
    let data = new Func_and_Args(null);
    // Deserialize message field [has_func]
    data.has_func = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [func_name]
    data.func_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [args]
    data.args = Args.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.func_name);
    length += Args.getMessageSize(object.args);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hademo/Func_and_Args';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2cf85b88f235eeb7b23bd9784805fafa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Func_and_Args(null);
    if (msg.has_func !== undefined) {
      resolved.has_func = msg.has_func;
    }
    else {
      resolved.has_func = false
    }

    if (msg.func_name !== undefined) {
      resolved.func_name = msg.func_name;
    }
    else {
      resolved.func_name = ''
    }

    if (msg.args !== undefined) {
      resolved.args = Args.Resolve(msg.args)
    }
    else {
      resolved.args = new Args()
    }

    return resolved;
    }
};

module.exports = Func_and_Args;
