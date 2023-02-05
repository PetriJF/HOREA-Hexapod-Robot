# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hexapod_interfaces:msg/TargetAngles.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'shoulder_angle'
# Member 'hip_angle'
# Member 'knee_angle'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TargetAngles(type):
    """Metaclass of message 'TargetAngles'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('hexapod_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hexapod_interfaces.msg.TargetAngles')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__target_angles
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__target_angles
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__target_angles
            cls._TYPE_SUPPORT = module.type_support_msg__msg__target_angles
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__target_angles

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SHOULDER_ANGLE__DEFAULT': numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32),
            'HIP_ANGLE__DEFAULT': numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32),
            'KNEE_ANGLE__DEFAULT': numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32),
        }

    @property
    def SHOULDER_ANGLE__DEFAULT(cls):
        """Return default value for message field 'shoulder_angle'."""
        return numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32)

    @property
    def HIP_ANGLE__DEFAULT(cls):
        """Return default value for message field 'hip_angle'."""
        return numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32)

    @property
    def KNEE_ANGLE__DEFAULT(cls):
        """Return default value for message field 'knee_angle'."""
        return numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32)


class TargetAngles(metaclass=Metaclass_TargetAngles):
    """Message class 'TargetAngles'."""

    __slots__ = [
        '_shoulder_angle',
        '_hip_angle',
        '_knee_angle',
    ]

    _fields_and_field_types = {
        'shoulder_angle': 'float[7]',
        'hip_angle': 'float[7]',
        'knee_angle': 'float[7]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 7),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 7),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 7),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.shoulder_angle = kwargs.get(
            'shoulder_angle', TargetAngles.SHOULDER_ANGLE__DEFAULT)
        self.hip_angle = kwargs.get(
            'hip_angle', TargetAngles.HIP_ANGLE__DEFAULT)
        self.knee_angle = kwargs.get(
            'knee_angle', TargetAngles.KNEE_ANGLE__DEFAULT)

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if all(self.shoulder_angle != other.shoulder_angle):
            return False
        if all(self.hip_angle != other.hip_angle):
            return False
        if all(self.knee_angle != other.knee_angle):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def shoulder_angle(self):
        """Message field 'shoulder_angle'."""
        return self._shoulder_angle

    @shoulder_angle.setter
    def shoulder_angle(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'shoulder_angle' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 7, \
                "The 'shoulder_angle' numpy.ndarray() must have a size of 7"
            self._shoulder_angle = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'shoulder_angle' field must be a set or sequence with length 7 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._shoulder_angle = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def hip_angle(self):
        """Message field 'hip_angle'."""
        return self._hip_angle

    @hip_angle.setter
    def hip_angle(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'hip_angle' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 7, \
                "The 'hip_angle' numpy.ndarray() must have a size of 7"
            self._hip_angle = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'hip_angle' field must be a set or sequence with length 7 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._hip_angle = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def knee_angle(self):
        """Message field 'knee_angle'."""
        return self._knee_angle

    @knee_angle.setter
    def knee_angle(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'knee_angle' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 7, \
                "The 'knee_angle' numpy.ndarray() must have a size of 7"
            self._knee_angle = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'knee_angle' field must be a set or sequence with length 7 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._knee_angle = numpy.array(value, dtype=numpy.float32)
