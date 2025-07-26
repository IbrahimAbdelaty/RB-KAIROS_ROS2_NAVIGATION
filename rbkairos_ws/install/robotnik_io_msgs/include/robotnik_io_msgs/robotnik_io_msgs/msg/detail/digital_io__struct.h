// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_io_msgs:msg/DigitalIO.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__DIGITAL_IO__STRUCT_H_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__DIGITAL_IO__STRUCT_H_

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

/// Struct defined in msg/DigitalIO in the package robotnik_io_msgs.
typedef struct robotnik_io_msgs__msg__DigitalIO
{
  int32_t id;
  rosidl_runtime_c__String name;
  bool value;
} robotnik_io_msgs__msg__DigitalIO;

// Struct for a sequence of robotnik_io_msgs__msg__DigitalIO.
typedef struct robotnik_io_msgs__msg__DigitalIO__Sequence
{
  robotnik_io_msgs__msg__DigitalIO * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_io_msgs__msg__DigitalIO__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__DIGITAL_IO__STRUCT_H_
