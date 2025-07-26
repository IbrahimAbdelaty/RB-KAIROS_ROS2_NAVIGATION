// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_perception_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_
#define ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_

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
// Member 'object'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in msg/Detection in the package robotnik_perception_msgs.
typedef struct robotnik_perception_msgs__msg__Detection
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__PoseStamped object;
} robotnik_perception_msgs__msg__Detection;

// Struct for a sequence of robotnik_perception_msgs__msg__Detection.
typedef struct robotnik_perception_msgs__msg__Detection__Sequence
{
  robotnik_perception_msgs__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_perception_msgs__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_
