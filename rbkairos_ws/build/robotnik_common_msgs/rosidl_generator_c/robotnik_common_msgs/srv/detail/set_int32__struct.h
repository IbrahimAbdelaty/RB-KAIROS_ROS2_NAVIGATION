// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_common_msgs:srv/SetInt32.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_INT32__STRUCT_H_
#define ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_INT32__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetInt32 in the package robotnik_common_msgs.
typedef struct robotnik_common_msgs__srv__SetInt32_Request
{
  int32_t data;
} robotnik_common_msgs__srv__SetInt32_Request;

// Struct for a sequence of robotnik_common_msgs__srv__SetInt32_Request.
typedef struct robotnik_common_msgs__srv__SetInt32_Request__Sequence
{
  robotnik_common_msgs__srv__SetInt32_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_common_msgs__srv__SetInt32_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.h"

/// Struct defined in srv/SetInt32 in the package robotnik_common_msgs.
typedef struct robotnik_common_msgs__srv__SetInt32_Response
{
  robotnik_common_msgs__msg__Response response;
} robotnik_common_msgs__srv__SetInt32_Response;

// Struct for a sequence of robotnik_common_msgs__srv__SetInt32_Response.
typedef struct robotnik_common_msgs__srv__SetInt32_Response__Sequence
{
  robotnik_common_msgs__srv__SetInt32_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_common_msgs__srv__SetInt32_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_INT32__STRUCT_H_
