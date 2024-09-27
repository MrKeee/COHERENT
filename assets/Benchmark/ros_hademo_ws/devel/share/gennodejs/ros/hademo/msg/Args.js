// Auto-generated. Do not edit!

// (in-package hademo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Args {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.has_args = null;
      this.attached_prim_path = null;
      this.waypoint_pos = null;
      this.waypoint_ori = null;
      this.waypoint_ind = null;
    }
    else {
      if (initObj.hasOwnProperty('has_args')) {
        this.has_args = initObj.has_args
      }
      else {
        this.has_args = false;
      }
      if (initObj.hasOwnProperty('attached_prim_path')) {
        this.attached_prim_path = initObj.attached_prim_path
      }
      else {
        this.attached_prim_path = '';
      }
      if (initObj.hasOwnProperty('waypoint_pos')) {
        this.waypoint_pos = initObj.waypoint_pos
      }
      else {
        this.waypoint_pos = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('waypoint_ori')) {
        this.waypoint_ori = initObj.waypoint_ori
      }
      else {
        this.waypoint_ori = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('waypoint_ind')) {
        this.waypoint_ind = initObj.waypoint_ind
      }
      else {
        this.waypoint_ind = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Args
    // Serialize message field [has_args]
    bufferOffset = _serializer.bool(obj.has_args, buffer, bufferOffset);
    // Serialize message field [attached_prim_path]
    bufferOffset = _serializer.string(obj.attached_prim_path, buffer, bufferOffset);
    // Serialize message field [waypoint_pos]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.waypoint_pos, buffer, bufferOffset);
    // Serialize message field [waypoint_ori]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.waypoint_ori, buffer, bufferOffset);
    // Serialize message field [waypoint_ind]
    bufferOffset = _serializer.int32(obj.waypoint_ind, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Args
    let len;
    let data = new Args(null);
    // Deserialize message field [has_args]
    data.has_args = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [attached_prim_path]
    data.attached_prim_path = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [waypoint_pos]
    data.waypoint_pos = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [waypoint_ori]
    data.waypoint_ori = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [waypoint_ind]
    data.waypoint_ind = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.attached_prim_path);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.waypoint_pos);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.waypoint_ori);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hademo/Args';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '482e2fefd3fc2e17f27a0aa075e67ef3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Args(null);
    if (msg.has_args !== undefined) {
      resolved.has_args = msg.has_args;
    }
    else {
      resolved.has_args = false
    }

    if (msg.attached_prim_path !== undefined) {
      resolved.attached_prim_path = msg.attached_prim_path;
    }
    else {
      resolved.attached_prim_path = ''
    }

    if (msg.waypoint_pos !== undefined) {
      resolved.waypoint_pos = std_msgs.msg.Float64MultiArray.Resolve(msg.waypoint_pos)
    }
    else {
      resolved.waypoint_pos = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.waypoint_ori !== undefined) {
      resolved.waypoint_ori = std_msgs.msg.Float64MultiArray.Resolve(msg.waypoint_ori)
    }
    else {
      resolved.waypoint_ori = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.waypoint_ind !== undefined) {
      resolved.waypoint_ind = msg.waypoint_ind;
    }
    else {
      resolved.waypoint_ind = 0
    }

    return resolved;
    }
};

module.exports = Args;
