// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_common_msgs:srv/SetBool.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_BOOL__STRUCT_H_
#define ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_BOOL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetBool in the package robotnik_common_msgs.
typedef struct robotnik_common_msgs__srv__SetBool_Request
{
  bool data;
} robotnik_common_msgs__srv__SetBool_Request;

// Struct for a sequence of robotnik_common_msgs__srv__SetBool_Request.
typedef struct robotnik_common_msgs__srv__SetBool_Request__Sequence
{
  robotnik_common_msgs__srv__SetBool_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_common_msgs__srv__SetBool_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.h"

/// Struct defined in srv/SetBool in the package robotnik_common_msgs.
typedef struct robotnik_common_msgs__srv__SetBool_Response
{
  robotnik_common_msgs__msg__Response response;
} robotnik_common_msgs__srv__SetBool_Response;

// Struct for a sequence of robotnik_common_msgs__srv__SetBool_Response.
typedef struct robotnik_common_msgs__srv__SetBool_Response__Sequence
{
  robotnik_common_msgs__srv__SetBool_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_common_msgs__srv__SetBool_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_BOOL__STRUCT_H_
