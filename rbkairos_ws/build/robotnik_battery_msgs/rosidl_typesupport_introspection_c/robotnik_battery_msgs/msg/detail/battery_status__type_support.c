// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robotnik_battery_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robotnik_battery_msgs/msg/detail/battery_status__rosidl_typesupport_introspection_c.h"
#include "robotnik_battery_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robotnik_battery_msgs/msg/detail/battery_status__functions.h"
#include "robotnik_battery_msgs/msg/detail/battery_status__struct.h"


// Include directives for member types
// Member `cell_voltages`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotnik_battery_msgs__msg__BatteryStatus__init(message_memory);
}

void robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_fini_function(void * message_memory)
{
  robotnik_battery_msgs__msg__BatteryStatus__fini(message_memory);
}

size_t robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__size_function__BatteryStatus__cell_voltages(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__get_const_function__BatteryStatus__cell_voltages(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__get_function__BatteryStatus__cell_voltages(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__fetch_function__BatteryStatus__cell_voltages(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__get_const_function__BatteryStatus__cell_voltages(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__assign_function__BatteryStatus__cell_voltages(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__get_function__BatteryStatus__cell_voltages(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__resize_function__BatteryStatus__cell_voltages(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_member_array[7] = {
  {
    "voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__BatteryStatus, voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__BatteryStatus, current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__BatteryStatus, level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_remaining",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__BatteryStatus, time_remaining),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_charging",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__BatteryStatus, time_charging),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_charging",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__BatteryStatus, is_charging),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cell_voltages",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__BatteryStatus, cell_voltages),  // bytes offset in struct
    NULL,  // default value
    robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__size_function__BatteryStatus__cell_voltages,  // size() function pointer
    robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__get_const_function__BatteryStatus__cell_voltages,  // get_const(index) function pointer
    robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__get_function__BatteryStatus__cell_voltages,  // get(index) function pointer
    robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__fetch_function__BatteryStatus__cell_voltages,  // fetch(index, &value) function pointer
    robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__assign_function__BatteryStatus__cell_voltages,  // assign(index, value) function pointer
    robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__resize_function__BatteryStatus__cell_voltages  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_members = {
  "robotnik_battery_msgs__msg",  // message namespace
  "BatteryStatus",  // message name
  7,  // number of fields
  sizeof(robotnik_battery_msgs__msg__BatteryStatus),
  robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_member_array,  // message members
  robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_type_support_handle = {
  0,
  &robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotnik_battery_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_battery_msgs, msg, BatteryStatus)() {
  if (!robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_type_support_handle.typesupport_identifier) {
    robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robotnik_battery_msgs__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
