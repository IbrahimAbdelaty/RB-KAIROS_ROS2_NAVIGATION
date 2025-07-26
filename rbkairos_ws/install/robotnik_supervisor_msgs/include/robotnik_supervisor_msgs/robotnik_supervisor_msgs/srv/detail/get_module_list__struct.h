// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_supervisor_msgs:srv/GetModuleList.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_MODULE_LIST__STRUCT_H_
#define ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_MODULE_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stack'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetModuleList in the package robotnik_supervisor_msgs.
typedef struct robotnik_supervisor_msgs__srv__GetModuleList_Request
{
  rosidl_runtime_c__String stack;
} robotnik_supervisor_msgs__srv__GetModuleList_Request;

// Struct for a sequence of robotnik_supervisor_msgs__srv__GetModuleList_Request.
typedef struct robotnik_supervisor_msgs__srv__GetModuleList_Request__Sequence
{
  robotnik_supervisor_msgs__srv__GetModuleList_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_supervisor_msgs__srv__GetModuleList_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.h"
// Member 'modules'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetModuleList in the package robotnik_supervisor_msgs.
typedef struct robotnik_supervisor_msgs__srv__GetModuleList_Response
{
  robotnik_common_msgs__msg__Response response;
  rosidl_runtime_c__String__Sequence modules;
} robotnik_supervisor_msgs__srv__GetModuleList_Response;

// Struct for a sequence of robotnik_supervisor_msgs__srv__GetModuleList_Response.
typedef struct robotnik_supervisor_msgs__srv__GetModuleList_Response__Sequence
{
  robotnik_supervisor_msgs__srv__GetModuleList_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_supervisor_msgs__srv__GetModuleList_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_MODULE_LIST__STRUCT_H_
