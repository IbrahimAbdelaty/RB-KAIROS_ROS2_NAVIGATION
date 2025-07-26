# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robotnik_battery_msgs:msg/InverterStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_InverterStatus(type):
    """Metaclass of message 'InverterStatus'."""

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
                'robotnik_battery_msgs.msg.InverterStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__inverter_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__inverter_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__inverter_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__inverter_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__inverter_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InverterStatus(metaclass=Metaclass_InverterStatus):
    """Message class 'InverterStatus'."""

    __slots__ = [
        '_ac_voltage',
        '_dc_voltage',
        '_load',
        '_percentage',
        '_temperature',
        '_on',
        '_serial_number',
    ]

    _fields_and_field_types = {
        'ac_voltage': 'float',
        'dc_voltage': 'float',
        'load': 'float',
        'percentage': 'float',
        'temperature': 'float',
        'on': 'boolean',
        'serial_number': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ac_voltage = kwargs.get('ac_voltage', float())
        self.dc_voltage = kwargs.get('dc_voltage', float())
        self.load = kwargs.get('load', float())
        self.percentage = kwargs.get('percentage', float())
        self.temperature = kwargs.get('temperature', float())
        self.on = kwargs.get('on', bool())
        self.serial_number = kwargs.get('serial_number', str())

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
        if self.ac_voltage != other.ac_voltage:
            return False
        if self.dc_voltage != other.dc_voltage:
            return False
        if self.load != other.load:
            return False
        if self.percentage != other.percentage:
            return False
        if self.temperature != other.temperature:
            return False
        if self.on != other.on:
            return False
        if self.serial_number != other.serial_number:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ac_voltage(self):
        """Message field 'ac_voltage'."""
        return self._ac_voltage

    @ac_voltage.setter
    def ac_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ac_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ac_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ac_voltage = value

    @builtins.property
    def dc_voltage(self):
        """Message field 'dc_voltage'."""
        return self._dc_voltage

    @dc_voltage.setter
    def dc_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dc_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'dc_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._dc_voltage = value

    @builtins.property
    def load(self):
        """Message field 'load'."""
        return self._load

    @load.setter
    def load(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'load' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'load' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._load = value

    @builtins.property
    def percentage(self):
        """Message field 'percentage'."""
        return self._percentage

    @percentage.setter
    def percentage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'percentage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'percentage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._percentage = value

    @builtins.property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._temperature = value

    @builtins.property
    def on(self):
        """Message field 'on'."""
        return self._on

    @on.setter
    def on(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'on' field must be of type 'bool'"
        self._on = value

    @builtins.property
    def serial_number(self):
        """Message field 'serial_number'."""
        return self._serial_number

    @serial_number.setter
    def serial_number(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'serial_number' field must be of type 'str'"
        self._serial_number = value
