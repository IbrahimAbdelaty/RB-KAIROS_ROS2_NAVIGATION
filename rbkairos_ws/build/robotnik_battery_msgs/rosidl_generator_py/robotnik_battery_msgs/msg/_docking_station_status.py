# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robotnik_battery_msgs:msg/DockingStationStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DockingStationStatus(type):
    """Metaclass of message 'DockingStationStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MODE_DISABLED': 'disabled',
        'MODE_AUTO_HW': 'automatic_hw',
        'MODE_AUTO_SW': 'automatic_sw',
        'MODE_MANUAL_SW': 'manual_sw',
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
                'robotnik_battery_msgs.msg.DockingStationStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__docking_station_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__docking_station_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__docking_station_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__docking_station_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__docking_station_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MODE_DISABLED': cls.__constants['MODE_DISABLED'],
            'MODE_AUTO_HW': cls.__constants['MODE_AUTO_HW'],
            'MODE_AUTO_SW': cls.__constants['MODE_AUTO_SW'],
            'MODE_MANUAL_SW': cls.__constants['MODE_MANUAL_SW'],
        }

    @property
    def MODE_DISABLED(self):
        """Message constant 'MODE_DISABLED'."""
        return Metaclass_DockingStationStatus.__constants['MODE_DISABLED']

    @property
    def MODE_AUTO_HW(self):
        """Message constant 'MODE_AUTO_HW'."""
        return Metaclass_DockingStationStatus.__constants['MODE_AUTO_HW']

    @property
    def MODE_AUTO_SW(self):
        """Message constant 'MODE_AUTO_SW'."""
        return Metaclass_DockingStationStatus.__constants['MODE_AUTO_SW']

    @property
    def MODE_MANUAL_SW(self):
        """Message constant 'MODE_MANUAL_SW'."""
        return Metaclass_DockingStationStatus.__constants['MODE_MANUAL_SW']


class DockingStationStatus(metaclass=Metaclass_DockingStationStatus):
    """
    Message class 'DockingStationStatus'.

    Constants:
      MODE_DISABLED
      MODE_AUTO_HW
      MODE_AUTO_SW
      MODE_MANUAL_SW
    """

    __slots__ = [
        '_operation_mode',
        '_contact_relay_status',
        '_charger_relay_status',
    ]

    _fields_and_field_types = {
        'operation_mode': 'string',
        'contact_relay_status': 'boolean',
        'charger_relay_status': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.operation_mode = kwargs.get('operation_mode', str())
        self.contact_relay_status = kwargs.get('contact_relay_status', bool())
        self.charger_relay_status = kwargs.get('charger_relay_status', bool())

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
        if self.operation_mode != other.operation_mode:
            return False
        if self.contact_relay_status != other.contact_relay_status:
            return False
        if self.charger_relay_status != other.charger_relay_status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def operation_mode(self):
        """Message field 'operation_mode'."""
        return self._operation_mode

    @operation_mode.setter
    def operation_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'operation_mode' field must be of type 'str'"
        self._operation_mode = value

    @builtins.property
    def contact_relay_status(self):
        """Message field 'contact_relay_status'."""
        return self._contact_relay_status

    @contact_relay_status.setter
    def contact_relay_status(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'contact_relay_status' field must be of type 'bool'"
        self._contact_relay_status = value

    @builtins.property
    def charger_relay_status(self):
        """Message field 'charger_relay_status'."""
        return self._charger_relay_status

    @charger_relay_status.setter
    def charger_relay_status(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'charger_relay_status' field must be of type 'bool'"
        self._charger_relay_status = value
