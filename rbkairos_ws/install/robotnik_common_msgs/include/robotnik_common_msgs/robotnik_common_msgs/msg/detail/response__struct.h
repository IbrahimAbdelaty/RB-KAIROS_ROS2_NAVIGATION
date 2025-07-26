// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_common_msgs:msg/Response.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__STRUCT_H_
#define ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Response in the package robotnik_common_msgs.
typedef struct robotnik_common_msgs__msg__Response
{
  bool success;
  rosidl_runtime_c__String message;
} robotnik_common_msgs__msg__Response;

// Struct for a sequence of robotnik_common_msgs__msg__Response.
typedef struct robotnik_common_msgs__msg__Response__Sequence
{
  robotnik_common_msgs__msg__Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_common_msgs__msg__Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__STRUCT_H_
