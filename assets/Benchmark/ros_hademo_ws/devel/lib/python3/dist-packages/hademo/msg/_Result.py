# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hademo/Result.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import hademo.msg

class Result(genpy.Message):
  _md5sum = "6a97bcb6cbe98951369cc3755518f34c"
  _type = "hademo/Result"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """hademo/ResultInfo franka_0
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
"""
  __slots__ = ['franka_0','franka_1','franka_2','aliengo_0','aliengo_1','aliengo_2','quadrotor_0','quadrotor_1','quadrotor_2']
  _slot_types = ['hademo/ResultInfo','hademo/ResultInfo','hademo/ResultInfo','hademo/ResultInfo','hademo/ResultInfo','hademo/ResultInfo','hademo/ResultInfo','hademo/ResultInfo','hademo/ResultInfo']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       franka_0,franka_1,franka_2,aliengo_0,aliengo_1,aliengo_2,quadrotor_0,quadrotor_1,quadrotor_2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Result, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.franka_0 is None:
        self.franka_0 = hademo.msg.ResultInfo()
      if self.franka_1 is None:
        self.franka_1 = hademo.msg.ResultInfo()
      if self.franka_2 is None:
        self.franka_2 = hademo.msg.ResultInfo()
      if self.aliengo_0 is None:
        self.aliengo_0 = hademo.msg.ResultInfo()
      if self.aliengo_1 is None:
        self.aliengo_1 = hademo.msg.ResultInfo()
      if self.aliengo_2 is None:
        self.aliengo_2 = hademo.msg.ResultInfo()
      if self.quadrotor_0 is None:
        self.quadrotor_0 = hademo.msg.ResultInfo()
      if self.quadrotor_1 is None:
        self.quadrotor_1 = hademo.msg.ResultInfo()
      if self.quadrotor_2 is None:
        self.quadrotor_2 = hademo.msg.ResultInfo()
    else:
      self.franka_0 = hademo.msg.ResultInfo()
      self.franka_1 = hademo.msg.ResultInfo()
      self.franka_2 = hademo.msg.ResultInfo()
      self.aliengo_0 = hademo.msg.ResultInfo()
      self.aliengo_1 = hademo.msg.ResultInfo()
      self.aliengo_2 = hademo.msg.ResultInfo()
      self.quadrotor_0 = hademo.msg.ResultInfo()
      self.quadrotor_1 = hademo.msg.ResultInfo()
      self.quadrotor_2 = hademo.msg.ResultInfo()

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
      _x = self
      buff.write(_get_struct_2B().pack(_x.franka_0.has_result, _x.franka_0.success))
      _x = self.franka_0.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.franka_1.has_result, _x.franka_1.success))
      _x = self.franka_1.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.franka_2.has_result, _x.franka_2.success))
      _x = self.franka_2.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.aliengo_0.has_result, _x.aliengo_0.success))
      _x = self.aliengo_0.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.aliengo_1.has_result, _x.aliengo_1.success))
      _x = self.aliengo_1.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.aliengo_2.has_result, _x.aliengo_2.success))
      _x = self.aliengo_2.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.quadrotor_0.has_result, _x.quadrotor_0.success))
      _x = self.quadrotor_0.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.quadrotor_1.has_result, _x.quadrotor_1.success))
      _x = self.quadrotor_1.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.quadrotor_2.has_result, _x.quadrotor_2.success))
      _x = self.quadrotor_2.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      if self.franka_0 is None:
        self.franka_0 = hademo.msg.ResultInfo()
      if self.franka_1 is None:
        self.franka_1 = hademo.msg.ResultInfo()
      if self.franka_2 is None:
        self.franka_2 = hademo.msg.ResultInfo()
      if self.aliengo_0 is None:
        self.aliengo_0 = hademo.msg.ResultInfo()
      if self.aliengo_1 is None:
        self.aliengo_1 = hademo.msg.ResultInfo()
      if self.aliengo_2 is None:
        self.aliengo_2 = hademo.msg.ResultInfo()
      if self.quadrotor_0 is None:
        self.quadrotor_0 = hademo.msg.ResultInfo()
      if self.quadrotor_1 is None:
        self.quadrotor_1 = hademo.msg.ResultInfo()
      if self.quadrotor_2 is None:
        self.quadrotor_2 = hademo.msg.ResultInfo()
      end = 0
      _x = self
      start = end
      end += 2
      (_x.franka_0.has_result, _x.franka_0.success,) = _get_struct_2B().unpack(str[start:end])
      self.franka_0.has_result = bool(self.franka_0.has_result)
      self.franka_0.success = bool(self.franka_0.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.franka_0.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.franka_0.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.franka_1.has_result, _x.franka_1.success,) = _get_struct_2B().unpack(str[start:end])
      self.franka_1.has_result = bool(self.franka_1.has_result)
      self.franka_1.success = bool(self.franka_1.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.franka_1.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.franka_1.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.franka_2.has_result, _x.franka_2.success,) = _get_struct_2B().unpack(str[start:end])
      self.franka_2.has_result = bool(self.franka_2.has_result)
      self.franka_2.success = bool(self.franka_2.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.franka_2.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.franka_2.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.aliengo_0.has_result, _x.aliengo_0.success,) = _get_struct_2B().unpack(str[start:end])
      self.aliengo_0.has_result = bool(self.aliengo_0.has_result)
      self.aliengo_0.success = bool(self.aliengo_0.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.aliengo_0.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.aliengo_0.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.aliengo_1.has_result, _x.aliengo_1.success,) = _get_struct_2B().unpack(str[start:end])
      self.aliengo_1.has_result = bool(self.aliengo_1.has_result)
      self.aliengo_1.success = bool(self.aliengo_1.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.aliengo_1.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.aliengo_1.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.aliengo_2.has_result, _x.aliengo_2.success,) = _get_struct_2B().unpack(str[start:end])
      self.aliengo_2.has_result = bool(self.aliengo_2.has_result)
      self.aliengo_2.success = bool(self.aliengo_2.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.aliengo_2.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.aliengo_2.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.quadrotor_0.has_result, _x.quadrotor_0.success,) = _get_struct_2B().unpack(str[start:end])
      self.quadrotor_0.has_result = bool(self.quadrotor_0.has_result)
      self.quadrotor_0.success = bool(self.quadrotor_0.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.quadrotor_0.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.quadrotor_0.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.quadrotor_1.has_result, _x.quadrotor_1.success,) = _get_struct_2B().unpack(str[start:end])
      self.quadrotor_1.has_result = bool(self.quadrotor_1.has_result)
      self.quadrotor_1.success = bool(self.quadrotor_1.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.quadrotor_1.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.quadrotor_1.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.quadrotor_2.has_result, _x.quadrotor_2.success,) = _get_struct_2B().unpack(str[start:end])
      self.quadrotor_2.has_result = bool(self.quadrotor_2.has_result)
      self.quadrotor_2.success = bool(self.quadrotor_2.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.quadrotor_2.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.quadrotor_2.info = str[start:end]
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
      _x = self
      buff.write(_get_struct_2B().pack(_x.franka_0.has_result, _x.franka_0.success))
      _x = self.franka_0.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.franka_1.has_result, _x.franka_1.success))
      _x = self.franka_1.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.franka_2.has_result, _x.franka_2.success))
      _x = self.franka_2.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.aliengo_0.has_result, _x.aliengo_0.success))
      _x = self.aliengo_0.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.aliengo_1.has_result, _x.aliengo_1.success))
      _x = self.aliengo_1.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.aliengo_2.has_result, _x.aliengo_2.success))
      _x = self.aliengo_2.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.quadrotor_0.has_result, _x.quadrotor_0.success))
      _x = self.quadrotor_0.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.quadrotor_1.has_result, _x.quadrotor_1.success))
      _x = self.quadrotor_1.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.quadrotor_2.has_result, _x.quadrotor_2.success))
      _x = self.quadrotor_2.info
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      if self.franka_0 is None:
        self.franka_0 = hademo.msg.ResultInfo()
      if self.franka_1 is None:
        self.franka_1 = hademo.msg.ResultInfo()
      if self.franka_2 is None:
        self.franka_2 = hademo.msg.ResultInfo()
      if self.aliengo_0 is None:
        self.aliengo_0 = hademo.msg.ResultInfo()
      if self.aliengo_1 is None:
        self.aliengo_1 = hademo.msg.ResultInfo()
      if self.aliengo_2 is None:
        self.aliengo_2 = hademo.msg.ResultInfo()
      if self.quadrotor_0 is None:
        self.quadrotor_0 = hademo.msg.ResultInfo()
      if self.quadrotor_1 is None:
        self.quadrotor_1 = hademo.msg.ResultInfo()
      if self.quadrotor_2 is None:
        self.quadrotor_2 = hademo.msg.ResultInfo()
      end = 0
      _x = self
      start = end
      end += 2
      (_x.franka_0.has_result, _x.franka_0.success,) = _get_struct_2B().unpack(str[start:end])
      self.franka_0.has_result = bool(self.franka_0.has_result)
      self.franka_0.success = bool(self.franka_0.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.franka_0.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.franka_0.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.franka_1.has_result, _x.franka_1.success,) = _get_struct_2B().unpack(str[start:end])
      self.franka_1.has_result = bool(self.franka_1.has_result)
      self.franka_1.success = bool(self.franka_1.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.franka_1.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.franka_1.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.franka_2.has_result, _x.franka_2.success,) = _get_struct_2B().unpack(str[start:end])
      self.franka_2.has_result = bool(self.franka_2.has_result)
      self.franka_2.success = bool(self.franka_2.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.franka_2.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.franka_2.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.aliengo_0.has_result, _x.aliengo_0.success,) = _get_struct_2B().unpack(str[start:end])
      self.aliengo_0.has_result = bool(self.aliengo_0.has_result)
      self.aliengo_0.success = bool(self.aliengo_0.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.aliengo_0.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.aliengo_0.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.aliengo_1.has_result, _x.aliengo_1.success,) = _get_struct_2B().unpack(str[start:end])
      self.aliengo_1.has_result = bool(self.aliengo_1.has_result)
      self.aliengo_1.success = bool(self.aliengo_1.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.aliengo_1.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.aliengo_1.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.aliengo_2.has_result, _x.aliengo_2.success,) = _get_struct_2B().unpack(str[start:end])
      self.aliengo_2.has_result = bool(self.aliengo_2.has_result)
      self.aliengo_2.success = bool(self.aliengo_2.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.aliengo_2.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.aliengo_2.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.quadrotor_0.has_result, _x.quadrotor_0.success,) = _get_struct_2B().unpack(str[start:end])
      self.quadrotor_0.has_result = bool(self.quadrotor_0.has_result)
      self.quadrotor_0.success = bool(self.quadrotor_0.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.quadrotor_0.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.quadrotor_0.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.quadrotor_1.has_result, _x.quadrotor_1.success,) = _get_struct_2B().unpack(str[start:end])
      self.quadrotor_1.has_result = bool(self.quadrotor_1.has_result)
      self.quadrotor_1.success = bool(self.quadrotor_1.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.quadrotor_1.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.quadrotor_1.info = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.quadrotor_2.has_result, _x.quadrotor_2.success,) = _get_struct_2B().unpack(str[start:end])
      self.quadrotor_2.has_result = bool(self.quadrotor_2.has_result)
      self.quadrotor_2.success = bool(self.quadrotor_2.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.quadrotor_2.info = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.quadrotor_2.info = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2B = None
def _get_struct_2B():
    global _struct_2B
    if _struct_2B is None:
        _struct_2B = struct.Struct("<2B")
    return _struct_2B
