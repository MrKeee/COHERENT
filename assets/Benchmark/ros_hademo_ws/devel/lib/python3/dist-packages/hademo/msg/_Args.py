# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hademo/Args.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Args(genpy.Message):
  _md5sum = "482e2fefd3fc2e17f27a0aa075e67ef3"
  _type = "hademo/Args"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool has_args
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
uint32 stride  # stride of given dimension"""
  __slots__ = ['has_args','attached_prim_path','waypoint_pos','waypoint_ori','waypoint_ind']
  _slot_types = ['bool','string','std_msgs/Float64MultiArray','std_msgs/Float64MultiArray','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       has_args,attached_prim_path,waypoint_pos,waypoint_ori,waypoint_ind

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Args, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.has_args is None:
        self.has_args = False
      if self.attached_prim_path is None:
        self.attached_prim_path = ''
      if self.waypoint_pos is None:
        self.waypoint_pos = std_msgs.msg.Float64MultiArray()
      if self.waypoint_ori is None:
        self.waypoint_ori = std_msgs.msg.Float64MultiArray()
      if self.waypoint_ind is None:
        self.waypoint_ind = 0
    else:
      self.has_args = False
      self.attached_prim_path = ''
      self.waypoint_pos = std_msgs.msg.Float64MultiArray()
      self.waypoint_ori = std_msgs.msg.Float64MultiArray()
      self.waypoint_ind = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.has_args
      buff.write(_get_struct_B().pack(_x))
      _x = self.attached_prim_path
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.waypoint_pos.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.waypoint_pos.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      _x = self.waypoint_pos.layout.data_offset
      buff.write(_get_struct_I().pack(_x))
      length = len(self.waypoint_pos.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.waypoint_pos.data))
      length = len(self.waypoint_ori.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.waypoint_ori.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      _x = self.waypoint_ori.layout.data_offset
      buff.write(_get_struct_I().pack(_x))
      length = len(self.waypoint_ori.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.waypoint_ori.data))
      _x = self.waypoint_ind
      buff.write(_get_struct_i().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.waypoint_pos is None:
        self.waypoint_pos = std_msgs.msg.Float64MultiArray()
      if self.waypoint_ori is None:
        self.waypoint_ori = std_msgs.msg.Float64MultiArray()
      end = 0
      start = end
      end += 1
      (self.has_args,) = _get_struct_B().unpack(str[start:end])
      self.has_args = bool(self.has_args)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.attached_prim_path = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.attached_prim_path = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.waypoint_pos.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.waypoint_pos.layout.dim.append(val1)
      start = end
      end += 4
      (self.waypoint_pos.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.waypoint_pos.data = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.waypoint_ori.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.waypoint_ori.layout.dim.append(val1)
      start = end
      end += 4
      (self.waypoint_ori.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.waypoint_ori.data = s.unpack(str[start:end])
      start = end
      end += 4
      (self.waypoint_ind,) = _get_struct_i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.has_args
      buff.write(_get_struct_B().pack(_x))
      _x = self.attached_prim_path
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.waypoint_pos.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.waypoint_pos.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      _x = self.waypoint_pos.layout.data_offset
      buff.write(_get_struct_I().pack(_x))
      length = len(self.waypoint_pos.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.waypoint_pos.data.tostring())
      length = len(self.waypoint_ori.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.waypoint_ori.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      _x = self.waypoint_ori.layout.data_offset
      buff.write(_get_struct_I().pack(_x))
      length = len(self.waypoint_ori.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.waypoint_ori.data.tostring())
      _x = self.waypoint_ind
      buff.write(_get_struct_i().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.waypoint_pos is None:
        self.waypoint_pos = std_msgs.msg.Float64MultiArray()
      if self.waypoint_ori is None:
        self.waypoint_ori = std_msgs.msg.Float64MultiArray()
      end = 0
      start = end
      end += 1
      (self.has_args,) = _get_struct_B().unpack(str[start:end])
      self.has_args = bool(self.has_args)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.attached_prim_path = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.attached_prim_path = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.waypoint_pos.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.waypoint_pos.layout.dim.append(val1)
      start = end
      end += 4
      (self.waypoint_pos.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.waypoint_pos.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.waypoint_ori.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.waypoint_ori.layout.dim.append(val1)
      start = end
      end += 4
      (self.waypoint_ori.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.waypoint_ori.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (self.waypoint_ind,) = _get_struct_i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
