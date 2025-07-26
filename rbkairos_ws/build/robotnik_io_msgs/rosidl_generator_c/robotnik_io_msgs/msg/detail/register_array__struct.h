// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_io_msgs:msg/RegisterArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__STRUCT_H_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'registers'
#include "robotnik_io_msgs/msg/detail/register__struct.h"

/// Struct defined in msg/RegisterArray in the package robotnik_io_msgs.
typedef struct robotnik_io_msgs__msg__RegisterArray
{
  robotnik_io_msgs__msg__Register__Sequence registers;
} robotnik_io_msgs__msg__RegisterArray;

// Struct for a sequence of robotnik_io_msgs__msg__RegisterArray.
typedef struct robotnik_io_msgs__msg__RegisterArray__Sequence
{
  robotnik_io_msgs__msg__RegisterArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_io_msgs__msg__RegisterArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__STRUCT_H_
