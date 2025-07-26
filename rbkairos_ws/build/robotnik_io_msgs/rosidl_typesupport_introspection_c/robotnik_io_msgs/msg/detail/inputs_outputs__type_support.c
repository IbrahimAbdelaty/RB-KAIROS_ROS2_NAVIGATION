// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robotnik_io_msgs:msg/InputsOutputs.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robotnik_io_msgs/msg/detail/inputs_outputs__rosidl_typesupport_introspection_c.h"
#include "robotnik_io_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robotnik_io_msgs/msg/detail/inputs_outputs__functions.h"
#include "robotnik_io_msgs/msg/detail/inputs_outputs__struct.h"


// Include directives for member types
// Member `digital_inputs`
// Member `digital_outputs`
#include "robotnik_io_msgs/msg/digital_io.h"
// Member `digital_inputs`
// Member `digital_outputs`
#include "robotnik_io_msgs/msg/detail/digital_io__rosidl_typesupport_introspection_c.h"
// Member `analog_inputs`
// Member `analog_outputs`
#include "robotnik_io_msgs/msg/analog_io.h"
// Member `analog_inputs`
// Member `analog_outputs`
#include "robotnik_io_msgs/msg/detail/analog_io__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotnik_io_msgs__msg__InputsOutputs__init(message_memory);
}

void robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_fini_function(void * message_memory)
{
  robotnik_io_msgs__msg__InputsOutputs__fini(message_memory);
}

size_t robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__size_function__InputsOutputs__digital_inputs(
  const void * untyped_member)
{
  const robotnik_io_msgs__msg__DigitalIO__Sequence * member =
    (const robotnik_io_msgs__msg__DigitalIO__Sequence *)(untyped_member);
  return member->size;
}

const void * robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__digital_inputs(
  const void * untyped_member, size_t index)
{
  const robotnik_io_msgs__msg__DigitalIO__Sequence * member =
    (const robotnik_io_msgs__msg__DigitalIO__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__digital_inputs(
  void * untyped_member, size_t index)
{
  robotnik_io_msgs__msg__DigitalIO__Sequence * member =
    (robotnik_io_msgs__msg__DigitalIO__Sequence *)(untyped_member);
  return &member->data[index];
}

void robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__fetch_function__InputsOutputs__digital_inputs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robotnik_io_msgs__msg__DigitalIO * item =
    ((const robotnik_io_msgs__msg__DigitalIO *)
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__digital_inputs(untyped_member, index));
  robotnik_io_msgs__msg__DigitalIO * value =
    (robotnik_io_msgs__msg__DigitalIO *)(untyped_value);
  *value = *item;
}

void robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__assign_function__InputsOutputs__digital_inputs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robotnik_io_msgs__msg__DigitalIO * item =
    ((robotnik_io_msgs__msg__DigitalIO *)
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__digital_inputs(untyped_member, index));
  const robotnik_io_msgs__msg__DigitalIO * value =
    (const robotnik_io_msgs__msg__DigitalIO *)(untyped_value);
  *item = *value;
}

bool robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__resize_function__InputsOutputs__digital_inputs(
  void * untyped_member, size_t size)
{
  robotnik_io_msgs__msg__DigitalIO__Sequence * member =
    (robotnik_io_msgs__msg__DigitalIO__Sequence *)(untyped_member);
  robotnik_io_msgs__msg__DigitalIO__Sequence__fini(member);
  return robotnik_io_msgs__msg__DigitalIO__Sequence__init(member, size);
}

size_t robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__size_function__InputsOutputs__digital_outputs(
  const void * untyped_member)
{
  const robotnik_io_msgs__msg__DigitalIO__Sequence * member =
    (const robotnik_io_msgs__msg__DigitalIO__Sequence *)(untyped_member);
  return member->size;
}

const void * robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__digital_outputs(
  const void * untyped_member, size_t index)
{
  const robotnik_io_msgs__msg__DigitalIO__Sequence * member =
    (const robotnik_io_msgs__msg__DigitalIO__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__digital_outputs(
  void * untyped_member, size_t index)
{
  robotnik_io_msgs__msg__DigitalIO__Sequence * member =
    (robotnik_io_msgs__msg__DigitalIO__Sequence *)(untyped_member);
  return &member->data[index];
}

void robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__fetch_function__InputsOutputs__digital_outputs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robotnik_io_msgs__msg__DigitalIO * item =
    ((const robotnik_io_msgs__msg__DigitalIO *)
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__digital_outputs(untyped_member, index));
  robotnik_io_msgs__msg__DigitalIO * value =
    (robotnik_io_msgs__msg__DigitalIO *)(untyped_value);
  *value = *item;
}

void robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__assign_function__InputsOutputs__digital_outputs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robotnik_io_msgs__msg__DigitalIO * item =
    ((robotnik_io_msgs__msg__DigitalIO *)
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__digital_outputs(untyped_member, index));
  const robotnik_io_msgs__msg__DigitalIO * value =
    (const robotnik_io_msgs__msg__DigitalIO *)(untyped_value);
  *item = *value;
}

bool robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__resize_function__InputsOutputs__digital_outputs(
  void * untyped_member, size_t size)
{
  robotnik_io_msgs__msg__DigitalIO__Sequence * member =
    (robotnik_io_msgs__msg__DigitalIO__Sequence *)(untyped_member);
  robotnik_io_msgs__msg__DigitalIO__Sequence__fini(member);
  return robotnik_io_msgs__msg__DigitalIO__Sequence__init(member, size);
}

size_t robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__size_function__InputsOutputs__analog_inputs(
  const void * untyped_member)
{
  const robotnik_io_msgs__msg__AnalogIO__Sequence * member =
    (const robotnik_io_msgs__msg__AnalogIO__Sequence *)(untyped_member);
  return member->size;
}

const void * robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__analog_inputs(
  const void * untyped_member, size_t index)
{
  const robotnik_io_msgs__msg__AnalogIO__Sequence * member =
    (const robotnik_io_msgs__msg__AnalogIO__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__analog_inputs(
  void * untyped_member, size_t index)
{
  robotnik_io_msgs__msg__AnalogIO__Sequence * member =
    (robotnik_io_msgs__msg__AnalogIO__Sequence *)(untyped_member);
  return &member->data[index];
}

void robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__fetch_function__InputsOutputs__analog_inputs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robotnik_io_msgs__msg__AnalogIO * item =
    ((const robotnik_io_msgs__msg__AnalogIO *)
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__analog_inputs(untyped_member, index));
  robotnik_io_msgs__msg__AnalogIO * value =
    (robotnik_io_msgs__msg__AnalogIO *)(untyped_value);
  *value = *item;
}

void robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__assign_function__InputsOutputs__analog_inputs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robotnik_io_msgs__msg__AnalogIO * item =
    ((robotnik_io_msgs__msg__AnalogIO *)
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__analog_inputs(untyped_member, index));
  const robotnik_io_msgs__msg__AnalogIO * value =
    (const robotnik_io_msgs__msg__AnalogIO *)(untyped_value);
  *item = *value;
}

bool robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__resize_function__InputsOutputs__analog_inputs(
  void * untyped_member, size_t size)
{
  robotnik_io_msgs__msg__AnalogIO__Sequence * member =
    (robotnik_io_msgs__msg__AnalogIO__Sequence *)(untyped_member);
  robotnik_io_msgs__msg__AnalogIO__Sequence__fini(member);
  return robotnik_io_msgs__msg__AnalogIO__Sequence__init(member, size);
}

size_t robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__size_function__InputsOutputs__analog_outputs(
  const void * untyped_member)
{
  const robotnik_io_msgs__msg__AnalogIO__Sequence * member =
    (const robotnik_io_msgs__msg__AnalogIO__Sequence *)(untyped_member);
  return member->size;
}

const void * robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__analog_outputs(
  const void * untyped_member, size_t index)
{
  const robotnik_io_msgs__msg__AnalogIO__Sequence * member =
    (const robotnik_io_msgs__msg__AnalogIO__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__analog_outputs(
  void * untyped_member, size_t index)
{
  robotnik_io_msgs__msg__AnalogIO__Sequence * member =
    (robotnik_io_msgs__msg__AnalogIO__Sequence *)(untyped_member);
  return &member->data[index];
}

void robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__fetch_function__InputsOutputs__analog_outputs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robotnik_io_msgs__msg__AnalogIO * item =
    ((const robotnik_io_msgs__msg__AnalogIO *)
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__analog_outputs(untyped_member, index));
  robotnik_io_msgs__msg__AnalogIO * value =
    (robotnik_io_msgs__msg__AnalogIO *)(untyped_value);
  *value = *item;
}

void robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__assign_function__InputsOutputs__analog_outputs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robotnik_io_msgs__msg__AnalogIO * item =
    ((robotnik_io_msgs__msg__AnalogIO *)
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__analog_outputs(untyped_member, index));
  const robotnik_io_msgs__msg__AnalogIO * value =
    (const robotnik_io_msgs__msg__AnalogIO *)(untyped_value);
  *item = *value;
}

bool robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__resize_function__InputsOutputs__analog_outputs(
  void * untyped_member, size_t size)
{
  robotnik_io_msgs__msg__AnalogIO__Sequence * member =
    (robotnik_io_msgs__msg__AnalogIO__Sequence *)(untyped_member);
  robotnik_io_msgs__msg__AnalogIO__Sequence__fini(member);
  return robotnik_io_msgs__msg__AnalogIO__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_member_array[4] = {
  {
    "digital_inputs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs__msg__InputsOutputs, digital_inputs),  // bytes offset in struct
    NULL,  // default value
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__size_function__InputsOutputs__digital_inputs,  // size() function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__digital_inputs,  // get_const(index) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__digital_inputs,  // get(index) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__fetch_function__InputsOutputs__digital_inputs,  // fetch(index, &value) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__assign_function__InputsOutputs__digital_inputs,  // assign(index, value) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__resize_function__InputsOutputs__digital_inputs  // resize(index) function pointer
  },
  {
    "digital_outputs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs__msg__InputsOutputs, digital_outputs),  // bytes offset in struct
    NULL,  // default value
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__size_function__InputsOutputs__digital_outputs,  // size() function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__digital_outputs,  // get_const(index) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__digital_outputs,  // get(index) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__fetch_function__InputsOutputs__digital_outputs,  // fetch(index, &value) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__assign_function__InputsOutputs__digital_outputs,  // assign(index, value) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__resize_function__InputsOutputs__digital_outputs  // resize(index) function pointer
  },
  {
    "analog_inputs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs__msg__InputsOutputs, analog_inputs),  // bytes offset in struct
    NULL,  // default value
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__size_function__InputsOutputs__analog_inputs,  // size() function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__analog_inputs,  // get_const(index) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__analog_inputs,  // get(index) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__fetch_function__InputsOutputs__analog_inputs,  // fetch(index, &value) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__assign_function__InputsOutputs__analog_inputs,  // assign(index, value) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__resize_function__InputsOutputs__analog_inputs  // resize(index) function pointer
  },
  {
    "analog_outputs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs__msg__InputsOutputs, analog_outputs),  // bytes offset in struct
    NULL,  // default value
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__size_function__InputsOutputs__analog_outputs,  // size() function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_const_function__InputsOutputs__analog_outputs,  // get_const(index) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__get_function__InputsOutputs__analog_outputs,  // get(index) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__fetch_function__InputsOutputs__analog_outputs,  // fetch(index, &value) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__assign_function__InputsOutputs__analog_outputs,  // assign(index, value) function pointer
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__resize_function__InputsOutputs__analog_outputs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_members = {
  "robotnik_io_msgs__msg",  // message namespace
  "InputsOutputs",  // message name
  4,  // number of fields
  sizeof(robotnik_io_msgs__msg__InputsOutputs),
  robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_member_array,  // message members
  robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_init_function,  // function to initialize message memory (memory has to be allocated)
  robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_type_support_handle = {
  0,
  &robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotnik_io_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, msg, InputsOutputs)() {
  robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, msg, DigitalIO)();
  robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, msg, DigitalIO)();
  robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, msg, AnalogIO)();
  robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, msg, AnalogIO)();
  if (!robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_type_support_handle.typesupport_identifier) {
    robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robotnik_io_msgs__msg__InputsOutputs__rosidl_typesupport_introspection_c__InputsOutputs_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
