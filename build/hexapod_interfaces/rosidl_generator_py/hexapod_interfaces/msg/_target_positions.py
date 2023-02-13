# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hexapod_interfaces:msg/TargetPositions.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'x_pos'
# Member 'y_pos'
# Member 'z_pos'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TargetPositions(type):
    """Metaclass of message 'TargetPositions'."""

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
                'hexapod_interfaces.msg.TargetPositions')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__target_positions
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__target_positions
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__target_positions
            cls._TYPE_SUPPORT = module.type_support_msg__msg__target_positions
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__target_positions

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'X_POS__DEFAULT': numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32),
            'Y_POS__DEFAULT': numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32),
            'Z_POS__DEFAULT': numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32),
        }

    @property
    def X_POS__DEFAULT(cls):
        """Return default value for message field 'x_pos'."""
        return numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32)

    @property
    def Y_POS__DEFAULT(cls):
        """Return default value for message field 'y_pos'."""
        return numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32)

    @property
    def Z_POS__DEFAULT(cls):
        """Return default value for message field 'z_pos'."""
        return numpy.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ), dtype=numpy.float32)


class TargetPositions(metaclass=Metaclass_TargetPositions):
    """Message class 'TargetPositions'."""

    __slots__ = [
        '_x_pos',
        '_y_pos',
        '_z_pos',
    ]

    _fields_and_field_types = {
        'x_pos': 'float[7]',
        'y_pos': 'float[7]',
        'z_pos': 'float[7]',
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
        self.x_pos = kwargs.get(
            'x_pos', TargetPositions.X_POS__DEFAULT)
        self.y_pos = kwargs.get(
            'y_pos', TargetPositions.Y_POS__DEFAULT)
        self.z_pos = kwargs.get(
            'z_pos', TargetPositions.Z_POS__DEFAULT)

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
        if all(self.x_pos != other.x_pos):
            return False
        if all(self.y_pos != other.y_pos):
            return False
        if all(self.z_pos != other.z_pos):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x_pos(self):
        """Message field 'x_pos'."""
        return self._x_pos

    @x_pos.setter
    def x_pos(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'x_pos' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 7, \
                "The 'x_pos' numpy.ndarray() must have a size of 7"
            self._x_pos = value
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
                "The 'x_pos' field must be a set or sequence with length 7 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._x_pos = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def y_pos(self):
        """Message field 'y_pos'."""
        return self._y_pos

    @y_pos.setter
    def y_pos(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'y_pos' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 7, \
                "The 'y_pos' numpy.ndarray() must have a size of 7"
            self._y_pos = value
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
                "The 'y_pos' field must be a set or sequence with length 7 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._y_pos = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def z_pos(self):
        """Message field 'z_pos'."""
        return self._z_pos

    @z_pos.setter
    def z_pos(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'z_pos' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 7, \
                "The 'z_pos' numpy.ndarray() must have a size of 7"
            self._z_pos = value
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
                "The 'z_pos' field must be a set or sequence with length 7 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._z_pos = numpy.array(value, dtype=numpy.float32)
