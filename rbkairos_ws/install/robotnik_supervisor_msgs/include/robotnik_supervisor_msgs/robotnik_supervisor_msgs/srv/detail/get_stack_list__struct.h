// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_supervisor_msgs:srv/GetStackList.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_STACK_LIST__STRUCT_H_
#define ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_STACK_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetStackList in the package robotnik_supervisor_msgs.
typedef struct robotnik_supervisor_msgs__srv__GetStackList_Request
{
  uint8_t structure_needs_at_least_one_member;
} robotnik_supervisor_msgs__srv__GetStackList_Request;

// Struct for a sequence of robotnik_supervisor_msgs__srv__GetStackList_Request.
typedef struct robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence
{
  robotnik_supervisor_msgs__srv__GetStackList_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.h"
// Member 'stacks'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetStackList in the package robotnik_supervisor_msgs.
typedef struct robotnik_supervisor_msgs__srv__GetStackList_Response
{
  robotnik_common_msgs__msg__Response response;
  rosidl_runtime_c__String__Sequence stacks;
} robotnik_supervisor_msgs__srv__GetStackList_Response;

// Struct for a sequence of robotnik_supervisor_msgs__srv__GetStackList_Response.
typedef struct robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence
{
  robotnik_supervisor_msgs__srv__GetStackList_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_STACK_LIST__STRUCT_H_
