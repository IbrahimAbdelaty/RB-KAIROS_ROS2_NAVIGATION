# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robotnik_battery_msgs:msg/BatteryStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'cell_voltages'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BatteryStatus(type):
    """Metaclass of message 'BatteryStatus'."""

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
            module = import_type_support('robotnik_battery_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robotnik_battery_msgs.msg.BatteryStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__battery_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__battery_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__battery_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__battery_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__battery_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BatteryStatus(metaclass=Metaclass_BatteryStatus):
    """Message class 'BatteryStatus'."""

    __slots__ = [
        '_voltage',
        '_current',
        '_level',
        '_time_remaining',
        '_time_charging',
        '_is_charging',
        '_cell_voltages',
    ]

    _fields_and_field_types = {
        'voltage': 'float',
        'current': 'float',
        'level': 'float',
        'time_remaining': 'uint32',
        'time_charging': 'uint32',
        'is_charging': 'boolean',
        'cell_voltages': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.voltage = kwargs.get('voltage', float())
        self.current = kwargs.get('current', float())
        self.level = kwargs.get('level', float())
        self.time_remaining = kwargs.get('time_remaining', int())
        self.time_charging = kwargs.get('time_charging', int())
        self.is_charging = kwargs.get('is_charging', bool())
        self.cell_voltages = array.array('f', kwargs.get('cell_voltages', []))

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
        if self.voltage != other.voltage:
            return False
        if self.current != other.current:
            return False
        if self.level != other.level:
            return False
        if self.time_remaining != other.time_remaining:
            return False
        if self.time_charging != other.time_charging:
            return False
        if self.is_charging != other.is_charging:
            return False
        if self.cell_voltages != other.cell_voltages:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def voltage(self):
        """Message field 'voltage'."""
        return self._voltage

    @voltage.setter
    def voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._voltage = value

    @builtins.property
    def current(self):
        """Message field 'current'."""
        return self._current

    @current.setter
    def current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current = value

    @builtins.property
    def level(self):
        """Message field 'level'."""
        return self._level

    @level.setter
    def level(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'level' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'level' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._level = value

    @builtins.property
    def time_remaining(self):
        """Message field 'time_remaining'."""
        return self._time_remaining

    @time_remaining.setter
    def time_remaining(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_remaining' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'time_remaining' field must be an unsigned integer in [0, 4294967295]"
        self._time_remaining = value

    @builtins.property
    def time_charging(self):
        """Message field 'time_charging'."""
        return self._time_charging

    @time_charging.setter
    def time_charging(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_charging' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'time_charging' field must be an unsigned integer in [0, 4294967295]"
        self._time_charging = value

    @builtins.property
    def is_charging(self):
        """Message field 'is_charging'."""
        return self._is_charging

    @is_charging.setter
    def is_charging(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_charging' field must be of type 'bool'"
        self._is_charging = value

    @builtins.property
    def cell_voltages(self):
        """Message field 'cell_voltages'."""
        return self._cell_voltages

    @cell_voltages.setter
    def cell_voltages(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'cell_voltages' array.array() must have the type code of 'f'"
            self._cell_voltages = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'cell_voltages' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._cell_voltages = array.array('f', value)
