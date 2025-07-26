# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robotnik_io_msgs:msg/InputsOutputs.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_InputsOutputs(type):
    """Metaclass of message 'InputsOutputs'."""

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
            module = import_type_support('robotnik_io_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robotnik_io_msgs.msg.InputsOutputs')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__inputs_outputs
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__inputs_outputs
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__inputs_outputs
            cls._TYPE_SUPPORT = module.type_support_msg__msg__inputs_outputs
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__inputs_outputs

            from robotnik_io_msgs.msg import AnalogIO
            if AnalogIO.__class__._TYPE_SUPPORT is None:
                AnalogIO.__class__.__import_type_support__()

            from robotnik_io_msgs.msg import DigitalIO
            if DigitalIO.__class__._TYPE_SUPPORT is None:
                DigitalIO.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InputsOutputs(metaclass=Metaclass_InputsOutputs):
    """Message class 'InputsOutputs'."""

    __slots__ = [
        '_digital_inputs',
        '_digital_outputs',
        '_analog_inputs',
        '_analog_outputs',
    ]

    _fields_and_field_types = {
        'digital_inputs': 'sequence<robotnik_io_msgs/DigitalIO>',
        'digital_outputs': 'sequence<robotnik_io_msgs/DigitalIO>',
        'analog_inputs': 'sequence<robotnik_io_msgs/AnalogIO>',
        'analog_outputs': 'sequence<robotnik_io_msgs/AnalogIO>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['robotnik_io_msgs', 'msg'], 'DigitalIO')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['robotnik_io_msgs', 'msg'], 'DigitalIO')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['robotnik_io_msgs', 'msg'], 'AnalogIO')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['robotnik_io_msgs', 'msg'], 'AnalogIO')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.digital_inputs = kwargs.get('digital_inputs', [])
        self.digital_outputs = kwargs.get('digital_outputs', [])
        self.analog_inputs = kwargs.get('analog_inputs', [])
        self.analog_outputs = kwargs.get('analog_outputs', [])

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
        if self.digital_inputs != other.digital_inputs:
            return False
        if self.digital_outputs != other.digital_outputs:
            return False
        if self.analog_inputs != other.analog_inputs:
            return False
        if self.analog_outputs != other.analog_outputs:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def digital_inputs(self):
        """Message field 'digital_inputs'."""
        return self._digital_inputs

    @digital_inputs.setter
    def digital_inputs(self, value):
        if __debug__:
            from robotnik_io_msgs.msg import DigitalIO
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
                 all(isinstance(v, DigitalIO) for v in value) and
                 True), \
                "The 'digital_inputs' field must be a set or sequence and each value of type 'DigitalIO'"
        self._digital_inputs = value

    @builtins.property
    def digital_outputs(self):
        """Message field 'digital_outputs'."""
        return self._digital_outputs

    @digital_outputs.setter
    def digital_outputs(self, value):
        if __debug__:
            from robotnik_io_msgs.msg import DigitalIO
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
                 all(isinstance(v, DigitalIO) for v in value) and
                 True), \
                "The 'digital_outputs' field must be a set or sequence and each value of type 'DigitalIO'"
        self._digital_outputs = value

    @builtins.property
    def analog_inputs(self):
        """Message field 'analog_inputs'."""
        return self._analog_inputs

    @analog_inputs.setter
    def analog_inputs(self, value):
        if __debug__:
            from robotnik_io_msgs.msg import AnalogIO
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
                 all(isinstance(v, AnalogIO) for v in value) and
                 True), \
                "The 'analog_inputs' field must be a set or sequence and each value of type 'AnalogIO'"
        self._analog_inputs = value

    @builtins.property
    def analog_outputs(self):
        """Message field 'analog_outputs'."""
        return self._analog_outputs

    @analog_outputs.setter
    def analog_outputs(self, value):
        if __debug__:
            from robotnik_io_msgs.msg import AnalogIO
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
                 all(isinstance(v, AnalogIO) for v in value) and
                 True), \
                "The 'analog_outputs' field must be a set or sequence and each value of type 'AnalogIO'"
        self._analog_outputs = value
