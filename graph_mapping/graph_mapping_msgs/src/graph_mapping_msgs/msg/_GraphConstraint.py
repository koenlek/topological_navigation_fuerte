"""autogenerated by genpy from graph_mapping_msgs/GraphConstraint.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import graph_mapping_msgs.msg

class GraphConstraint(genpy.Message):
  _md5sum = "9373a29b959ebfb34f1271bdd2b8a95f"
  _type = "graph_mapping_msgs/GraphConstraint"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint32 src
uint32 dest
PoseWithPrecision constraint
================================================================================
MSG: graph_mapping_msgs/PoseWithPrecision
geometry_msgs/Pose pose
float64[36] precision
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

"""
  __slots__ = ['src','dest','constraint']
  _slot_types = ['uint32','uint32','graph_mapping_msgs/PoseWithPrecision']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       src,dest,constraint

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GraphConstraint, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.src is None:
        self.src = 0
      if self.dest is None:
        self.dest = 0
      if self.constraint is None:
        self.constraint = graph_mapping_msgs.msg.PoseWithPrecision()
    else:
      self.src = 0
      self.dest = 0
      self.constraint = graph_mapping_msgs.msg.PoseWithPrecision()

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
      buff.write(_struct_2I7d.pack(_x.src, _x.dest, _x.constraint.pose.position.x, _x.constraint.pose.position.y, _x.constraint.pose.position.z, _x.constraint.pose.orientation.x, _x.constraint.pose.orientation.y, _x.constraint.pose.orientation.z, _x.constraint.pose.orientation.w))
      buff.write(_struct_36d.pack(*self.constraint.precision))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.constraint is None:
        self.constraint = graph_mapping_msgs.msg.PoseWithPrecision()
      end = 0
      _x = self
      start = end
      end += 64
      (_x.src, _x.dest, _x.constraint.pose.position.x, _x.constraint.pose.position.y, _x.constraint.pose.position.z, _x.constraint.pose.orientation.x, _x.constraint.pose.orientation.y, _x.constraint.pose.orientation.z, _x.constraint.pose.orientation.w,) = _struct_2I7d.unpack(str[start:end])
      start = end
      end += 288
      self.constraint.precision = _struct_36d.unpack(str[start:end])
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
      buff.write(_struct_2I7d.pack(_x.src, _x.dest, _x.constraint.pose.position.x, _x.constraint.pose.position.y, _x.constraint.pose.position.z, _x.constraint.pose.orientation.x, _x.constraint.pose.orientation.y, _x.constraint.pose.orientation.z, _x.constraint.pose.orientation.w))
      buff.write(self.constraint.precision.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.constraint is None:
        self.constraint = graph_mapping_msgs.msg.PoseWithPrecision()
      end = 0
      _x = self
      start = end
      end += 64
      (_x.src, _x.dest, _x.constraint.pose.position.x, _x.constraint.pose.position.y, _x.constraint.pose.position.z, _x.constraint.pose.orientation.x, _x.constraint.pose.orientation.y, _x.constraint.pose.orientation.z, _x.constraint.pose.orientation.w,) = _struct_2I7d.unpack(str[start:end])
      start = end
      end += 288
      self.constraint.precision = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_36d = struct.Struct("<36d")
_struct_2I7d = struct.Struct("<2I7d")