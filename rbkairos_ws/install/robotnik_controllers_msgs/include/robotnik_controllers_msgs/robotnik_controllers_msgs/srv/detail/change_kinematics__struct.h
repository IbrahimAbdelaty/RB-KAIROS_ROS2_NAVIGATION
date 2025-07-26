// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_controllers_msgs:srv/ChangeKinematics.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__CHANGE_KINEMATICS__STRUCT_H_
#define ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__CHANGE_KINEMATICS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ChangeKinematics in the package robotnik_controllers_msgs.
typedef struct robotnik_controllers_msgs__srv__ChangeKinematics_Request
{
  rosidl_runtime_c__String name;
} robotnik_controllers_msgs__srv__ChangeKinematics_Request;

// Struct for a sequence of robotnik_controllers_msgs__srv__ChangeKinematics_Request.
typedef struct robotnik_controllers_msgs__srv__ChangeKinematics_Request__Sequence
{
  robotnik_controllers_msgs__srv__ChangeKinematics_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_controllers_msgs__srv__ChangeKinematics_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.h"

/// Struct defined in srv/ChangeKinematics in the package robotnik_controllers_msgs.
typedef struct robotnik_controllers_msgs__srv__ChangeKinematics_Response
{
  robotnik_common_msgs__msg__Response response;
} robotnik_controllers_msgs__srv__ChangeKinematics_Response;

// Struct for a sequence of robotnik_controllers_msgs__srv__ChangeKinematics_Response.
typedef struct robotnik_controllers_msgs__srv__ChangeKinematics_Response__Sequence
{
  robotnik_controllers_msgs__srv__ChangeKinematics_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_controllers_msgs__srv__ChangeKinematics_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__CHANGE_KINEMATICS__STRUCT_H_
