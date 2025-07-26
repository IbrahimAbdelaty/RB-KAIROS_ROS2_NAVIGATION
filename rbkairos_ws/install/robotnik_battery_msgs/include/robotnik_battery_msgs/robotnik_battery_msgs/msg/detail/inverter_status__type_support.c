// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robotnik_battery_msgs:msg/InverterStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robotnik_battery_msgs/msg/detail/inverter_status__rosidl_typesupport_introspection_c.h"
#include "robotnik_battery_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robotnik_battery_msgs/msg/detail/inverter_status__functions.h"
#include "robotnik_battery_msgs/msg/detail/inverter_status__struct.h"


// Include directives for member types
// Member `serial_number`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotnik_battery_msgs__msg__InverterStatus__init(message_memory);
}

void robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_fini_function(void * message_memory)
{
  robotnik_battery_msgs__msg__InverterStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_message_member_array[7] = {
  {
    "ac_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__InverterStatus, ac_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dc_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__InverterStatus, dc_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "load",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__InverterStatus, load),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "percentage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__InverterStatus, percentage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__InverterStatus, temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "on",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__InverterStatus, on),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "serial_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs__msg__InverterStatus, serial_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_message_members = {
  "robotnik_battery_msgs__msg",  // message namespace
  "InverterStatus",  // message name
  7,  // number of fields
  sizeof(robotnik_battery_msgs__msg__InverterStatus),
  robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_message_member_array,  // message members
  robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_message_type_support_handle = {
  0,
  &robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotnik_battery_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_battery_msgs, msg, InverterStatus)() {
  if (!robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_message_type_support_handle.typesupport_identifier) {
    robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robotnik_battery_msgs__msg__InverterStatus__rosidl_typesupport_introspection_c__InverterStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
