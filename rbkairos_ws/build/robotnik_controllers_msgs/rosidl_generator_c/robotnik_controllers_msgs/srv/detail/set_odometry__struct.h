// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_controllers_msgs:srv/SetOdometry.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__SET_ODOMETRY__STRUCT_H_
#define ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__SET_ODOMETRY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetOdometry in the package robotnik_controllers_msgs.
typedef struct robotnik_controllers_msgs__srv__SetOdometry_Request
{
  float x;
  /// New robot Y position (m)
  float y;
  /// New robot Orientation (rads)
  float yaw;
} robotnik_controllers_msgs__srv__SetOdometry_Request;

// Struct for a sequence of robotnik_controllers_msgs__srv__SetOdometry_Request.
typedef struct robotnik_controllers_msgs__srv__SetOdometry_Request__Sequence
{
  robotnik_controllers_msgs__srv__SetOdometry_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_controllers_msgs__srv__SetOdometry_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.h"

/// Struct defined in srv/SetOdometry in the package robotnik_controllers_msgs.
typedef struct robotnik_controllers_msgs__srv__SetOdometry_Response
{
  robotnik_common_msgs__msg__Response response;
} robotnik_controllers_msgs__srv__SetOdometry_Response;

// Struct for a sequence of robotnik_controllers_msgs__srv__SetOdometry_Response.
typedef struct robotnik_controllers_msgs__srv__SetOdometry_Response__Sequence
{
  robotnik_controllers_msgs__srv__SetOdometry_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_controllers_msgs__srv__SetOdometry_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__SET_ODOMETRY__STRUCT_H_
