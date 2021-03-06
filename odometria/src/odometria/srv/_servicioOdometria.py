"""autogenerated by genpy from odometria/servicioOdometriaRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class servicioOdometriaRequest(genpy.Message):
  _md5sum = "d7be0bb39af8fb9129d5a76e6b63a290"
  _type = "odometria/servicioOdometriaRequest"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header']
  _slot_types = ['std_msgs/Header']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(servicioOdometriaRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
    else:
      self.header = std_msgs.msg.Header()

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
"""autogenerated by genpy from odometria/servicioOdometriaResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg

class servicioOdometriaResponse(genpy.Message):
  _md5sum = "d02ae0a68c66ec7e1d3b7cc2edd9c4d6"
  _type = "odometria/servicioOdometriaResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """nav_msgs/Odometry msjOdometria


================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
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

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
"""
  __slots__ = ['msjOdometria']
  _slot_types = ['nav_msgs/Odometry']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       msjOdometria

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(servicioOdometriaResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.msjOdometria is None:
        self.msjOdometria = nav_msgs.msg.Odometry()
    else:
      self.msjOdometria = nav_msgs.msg.Odometry()

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
      buff.write(_struct_3I.pack(_x.msjOdometria.header.seq, _x.msjOdometria.header.stamp.secs, _x.msjOdometria.header.stamp.nsecs))
      _x = self.msjOdometria.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.msjOdometria.child_frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.msjOdometria.pose.pose.position.x, _x.msjOdometria.pose.pose.position.y, _x.msjOdometria.pose.pose.position.z, _x.msjOdometria.pose.pose.orientation.x, _x.msjOdometria.pose.pose.orientation.y, _x.msjOdometria.pose.pose.orientation.z, _x.msjOdometria.pose.pose.orientation.w))
      buff.write(_struct_36d.pack(*self.msjOdometria.pose.covariance))
      _x = self
      buff.write(_struct_6d.pack(_x.msjOdometria.twist.twist.linear.x, _x.msjOdometria.twist.twist.linear.y, _x.msjOdometria.twist.twist.linear.z, _x.msjOdometria.twist.twist.angular.x, _x.msjOdometria.twist.twist.angular.y, _x.msjOdometria.twist.twist.angular.z))
      buff.write(_struct_36d.pack(*self.msjOdometria.twist.covariance))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.msjOdometria is None:
        self.msjOdometria = nav_msgs.msg.Odometry()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.msjOdometria.header.seq, _x.msjOdometria.header.stamp.secs, _x.msjOdometria.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.msjOdometria.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.msjOdometria.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.msjOdometria.child_frame_id = str[start:end].decode('utf-8')
      else:
        self.msjOdometria.child_frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.msjOdometria.pose.pose.position.x, _x.msjOdometria.pose.pose.position.y, _x.msjOdometria.pose.pose.position.z, _x.msjOdometria.pose.pose.orientation.x, _x.msjOdometria.pose.pose.orientation.y, _x.msjOdometria.pose.pose.orientation.z, _x.msjOdometria.pose.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 288
      self.msjOdometria.pose.covariance = _struct_36d.unpack(str[start:end])
      _x = self
      start = end
      end += 48
      (_x.msjOdometria.twist.twist.linear.x, _x.msjOdometria.twist.twist.linear.y, _x.msjOdometria.twist.twist.linear.z, _x.msjOdometria.twist.twist.angular.x, _x.msjOdometria.twist.twist.angular.y, _x.msjOdometria.twist.twist.angular.z,) = _struct_6d.unpack(str[start:end])
      start = end
      end += 288
      self.msjOdometria.twist.covariance = _struct_36d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.msjOdometria.header.seq, _x.msjOdometria.header.stamp.secs, _x.msjOdometria.header.stamp.nsecs))
      _x = self.msjOdometria.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.msjOdometria.child_frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.msjOdometria.pose.pose.position.x, _x.msjOdometria.pose.pose.position.y, _x.msjOdometria.pose.pose.position.z, _x.msjOdometria.pose.pose.orientation.x, _x.msjOdometria.pose.pose.orientation.y, _x.msjOdometria.pose.pose.orientation.z, _x.msjOdometria.pose.pose.orientation.w))
      buff.write(self.msjOdometria.pose.covariance.tostring())
      _x = self
      buff.write(_struct_6d.pack(_x.msjOdometria.twist.twist.linear.x, _x.msjOdometria.twist.twist.linear.y, _x.msjOdometria.twist.twist.linear.z, _x.msjOdometria.twist.twist.angular.x, _x.msjOdometria.twist.twist.angular.y, _x.msjOdometria.twist.twist.angular.z))
      buff.write(self.msjOdometria.twist.covariance.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.msjOdometria is None:
        self.msjOdometria = nav_msgs.msg.Odometry()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.msjOdometria.header.seq, _x.msjOdometria.header.stamp.secs, _x.msjOdometria.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.msjOdometria.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.msjOdometria.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.msjOdometria.child_frame_id = str[start:end].decode('utf-8')
      else:
        self.msjOdometria.child_frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.msjOdometria.pose.pose.position.x, _x.msjOdometria.pose.pose.position.y, _x.msjOdometria.pose.pose.position.z, _x.msjOdometria.pose.pose.orientation.x, _x.msjOdometria.pose.pose.orientation.y, _x.msjOdometria.pose.pose.orientation.z, _x.msjOdometria.pose.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 288
      self.msjOdometria.pose.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      _x = self
      start = end
      end += 48
      (_x.msjOdometria.twist.twist.linear.x, _x.msjOdometria.twist.twist.linear.y, _x.msjOdometria.twist.twist.linear.z, _x.msjOdometria.twist.twist.angular.x, _x.msjOdometria.twist.twist.angular.y, _x.msjOdometria.twist.twist.angular.z,) = _struct_6d.unpack(str[start:end])
      start = end
      end += 288
      self.msjOdometria.twist.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_7d = struct.Struct("<7d")
_struct_6d = struct.Struct("<6d")
_struct_36d = struct.Struct("<36d")
class servicioOdometria(object):
  _type          = 'odometria/servicioOdometria'
  _md5sum = 'f7ae249dd74d9001173047d4595ab5af'
  _request_class  = servicioOdometriaRequest
  _response_class = servicioOdometriaResponse
