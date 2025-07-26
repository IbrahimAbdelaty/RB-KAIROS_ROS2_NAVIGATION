// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_controllers_msgs:msg/ControllerState.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__MSG__DETAIL__CONTROLLER_STATE__STRUCT_H_
#define ROBOTNIK_CONTROLLERS_MSGS__MSG__DETAIL__CONTROLLER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'state'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ControllerState in the package robotnik_controllers_msgs.
typedef struct robotnik_controllers_msgs__msg__ControllerState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String state;
  uint8_t state_number;
} robotnik_controllers_msgs__msg__ControllerState;

// Struct for a sequence of robotnik_controllers_msgs__msg__ControllerState.
typedef struct robotnik_controllers_msgs__msg__ControllerState__Sequence
{
  robotnik_controllers_msgs__msg__ControllerState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_controllers_msgs__msg__ControllerState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_CONTROLLERS_MSGS__MSG__DETAIL__CONTROLLER_STATE__STRUCT_H_
